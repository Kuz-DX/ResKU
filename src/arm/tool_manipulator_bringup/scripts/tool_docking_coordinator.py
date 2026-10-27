#!/usr/bin/env python3
"""Optional L2 path -> measured tag alignment -> Arduino transaction coordinator."""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tool_trajectory_joy import ToolTrajectoryJoy, ARM_JOINTS
from tool_docking_core import (load_settings, transform, pose_errors, bounded_target,
                               tag_pose, camera_normal_target)


class ToolDockingCoordinator(ToolTrajectoryJoy):
    def __init__(self):
        super().__init__()
        self.settings = load_settings(self.declare_parameter('docking_config', '').value)
        # Must match the inherited ArmKinematics root, not an arbitrary TF frame.
        self.base_frame = 'base_actuator'
        self.camera_frame = self.declare_parameter('docking_camera_frame', '').value
        self.motion_enabled = bool(self.declare_parameter(
            'docking_motion_enabled', False).value)
        if self.settings and not self.camera_frame:
            raise ValueError('docking_camera_frame must name the calibrated optical frame')
        self.camera = None
        self.velocity = None
        self.feedback_sequence = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.declare_parameter(
            'docking_camera_info_topic', '/arm/camera/color/camera_info').value,
            self.on_camera, qos_profile_sensor_data)
        self.create_subscription(String, self.declare_parameter(
            'docking_centers_topic', '/arm/apriltag/centers').value, self.on_tag, 1)
        self.deadman_timeout = float(self.declare_parameter(
            'docking_deadman_timeout_s', 0.3).value)
        if not 0.1 <= self.deadman_timeout <= 1.0:
            raise ValueError('docking_deadman_timeout_s must be in [0.1, 1.0]')
        self.create_service(Trigger, '/arm/tool_docking/cancel', self.on_cancel)
        self.get_logger().info(f'Optional docking paths: {sorted(self.settings)}')

    def reset(self):
        super().reset()
        self.active = False
        self.docking_started = None
        self.expected_docked = None
        self.actuation_deadline = None
        self.observation = None
        self.last_image_stamp = -1
        self.dock_stage = 0
        self.stable = 0
        self.observed_image = -1
        self.observed_feedback = -1
        self.hold_epoch = None
        self.lock_goal = None
        self.last_base_tag = None
        self.last_joy_time = -math.inf
        self.deadman_held = False
        self.last_preview_report = -math.inf
        self.camera_alignment_distance = None
        self.camera_reference_x = None
        self.tcp_camera = None
        self.advance_locked = False
        self.advance_target = None
        self.advance_plan = None

    def on_joy(self, msg: Joy):
        self.last_joy_time = time.monotonic()
        self.deadman_held = (
            len(msg.axes) > self.l2_axis
            and math.isfinite(msg.axes[self.l2_axis])
            and msg.axes[self.l2_axis] < self.l2_threshold)
        super().on_joy(msg)

    def on_state(self, msg):
        super().on_state(msg)
        velocities = dict(zip(msg.name, msg.velocity))
        self.velocity = (np.asarray([velocities[j] for j in ARM_JOINTS])
                         if len(msg.name) == len(msg.velocity)
                         and all(j in velocities for j in ARM_JOINTS)
                         and all(math.isfinite(v) for v in msg.velocity) else None)
        self.feedback_sequence += 1

    def on_camera(self, msg):
        self.camera = None
        matrix, distortion = np.asarray(msg.k).reshape(3, 3), np.asarray(msg.d)
        if (msg.header.frame_id == self.camera_frame and matrix[0, 0] > 0
                and matrix[1, 1] > 0 and np.all(np.isfinite(matrix))
                and np.all(np.isfinite(distortion))
                and msg.distortion_model in ('plumb_bob', 'rational_polynomial')):
            self.camera = (matrix, distortion)

    def on_tag(self, msg):
        if not self.active or getattr(self, 'advance_locked', False):
            return
        self.observation = None
        try:
            data = json.loads(msg.data)
            stamp = Time(seconds=int(data['stamp']['sec']), nanoseconds=int(data['stamp']['nanosec']))
            stamp_ns = stamp.nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            if (self.camera is None or data['frame_id'] != self.camera_frame
                    or not 0 <= age < self.cfg['tag_max_age_s']
                    or stamp_ns <= self.last_image_stamp):
                raise ValueError('stale tag image or camera calibration unavailable')
            self.last_image_stamp = stamp_ns
            detections = [d for d in data['detections'] if d['id'] == self.cfg['tag_id']
                          and ('tag_family' not in self.cfg
                               or d.get('family') == self.cfg['tag_family'])]
            if len(detections) != 1:
                raise ValueError('expected exactly one matching tag')
            camera_tag = tag_pose(detections[0], *self.camera, self.cfg)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, stamp,
                timeout=Duration(seconds=self.cfg['tf_timeout_s'])).transform
            p, q = tf.translation, tf.rotation
            base_tag = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]) @ camera_tag
            if self.last_base_tag is not None:
                jump, turn = pose_errors(self.last_base_tag, base_tag)
                if (jump > self.cfg['max_tag_position_jump_m']
                        or turn > self.cfg['max_tag_angle_jump_rad']):
                    raise ValueError('tag pose discontinuity rejected')
            self.last_base_tag = base_tag
            self.observation = (stamp_ns, time.monotonic(), base_tag)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError,
                TransformException, cv2.error):
            # An invalid/absent target invalidates the previous observation too.
            self.stable = 0

    def begin(self, path_id, planned_path=None):
        if (planned_path is None and self.requested_docked_state is None
                and path_id in self.settings):
            self.active = True
            self.cfg = self.settings[path_id]
            self.docking_started = None
            if (self.cfg['actuation_enabled']
                    and (self.docking_command.get_subscription_count() == 0
                         or self.count_publishers('/arm/docked') == 0)):
                self.fail('Arduino command/status connection unavailable before docking')
                return
        super().begin(path_id, planned_path)

    def begin_failed(self, reason):
        if self.active:
            self.fail(reason)
        else:
            super().begin_failed(reason)

    def fail(self, reason):
        if self.active:
            if self.cancel.service_is_ready():
                self.cancel.call_async(Trigger.Request())
        super().fail(reason)

    def on_cancel(self, request, response):
        if self.stage == 'idle':
            response.success, response.message = False, 'no active path or docking'
        else:
            self.fail('operator cancelled docking/path')
            response.success, response.message = True, 'cancel requested; no automatic retreat'
        return response

    def path_completed(self):
        if not self.active:
            super().path_completed()
            return
        if getattr(self, 'advance_locked', False):
            q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
            distance, angle = pose_errors(self.model.fk(q), self.advance_target)
            if (distance > self.cfg['position_tolerance_m']
                    or angle > self.cfg['angle_tolerance_rad']
                    or np.max(np.abs(self.velocity)) > self.cfg['stopped_speed_rad_s']):
                raise RuntimeError('advance endpoint feedback outside tolerance; remain HOLD')
            self.report('completed', 'tag-normal advance %.1f mm completed; HOLD; no actuation' %
                        (self.cfg['advance_distance_m']*1000.))
            self.reset()
            return
        if self.docking_started is None:
            self.docking_started = time.monotonic()
            self.stage = 'dock_acquire'
        else:
            self.stage = 'dock_observe'
        self.execution_id = None
        self.hold_epoch = self.status['epoch']
        self.observe_after_ns = self.get_clock().now().nanoseconds
        self.observed_feedback = self.feedback_sequence
        self.stable = 0
        self.report(self.stage, f'waiting for stopped feedback and fresh tag; stage {self.dock_stage}')

    def fresh_observation(self, now):
        if self.observation is None:
            raise RuntimeError('target tag missing or rejected')
        stamp, received, base_tag = self.observation
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age < self.cfg['tag_max_age_s'] or now-received >= self.cfg['tag_max_age_s']:
            raise RuntimeError('target tag stale')
        return stamp, base_tag

    def guard(self, now):
        if now-self.last_joy_time >= self.deadman_timeout or not self.deadman_held:
            raise RuntimeError('docking deadman released or joystick stream stale')
        if now-self.docking_started > self.cfg['sequence_timeout_s']:
            raise RuntimeError('docking sequence timeout')
        s = self.status
        if (now-self.status_time >= .3 or now-self.state_time >= .3
                or self.full_state is None or self.velocity is None
                or not s.get('ready') or not s.get('startup_complete')
                or s.get('focus') != 'arm' or s.get('mode') == 'fault'):
            raise RuntimeError('docking feedback/ownership unavailable')
        if self.stage.startswith('dock_'):
            self.healthy_hold(now)
            if s.get('epoch') != self.hold_epoch:
                raise RuntimeError('docking HOLD ownership changed/cancelled')

    def tick(self):
        try:
            if self.active and self.docking_started is None:
                if (time.monotonic()-self.last_joy_time >= self.deadman_timeout
                        or not self.deadman_held):
                    raise RuntimeError('docking deadman released or joystick stream stale')
            if self.active and self.docking_started is not None:
                now = time.monotonic()
                self.guard(now)
                if self.stage == 'dock_acquire':
                    if now-self.docking_started > self.cfg['acquire_timeout_s']:
                        raise RuntimeError('target tag acquisition timed out')
                    if self.observation is None:
                        return
                    self.fresh_observation(now)
                    self.stage = 'dock_observe'
                elif not getattr(self, 'advance_locked', False):
                    # Tag loss also cancels a correction while it is being checked/executed.
                    self.fresh_observation(now)
                if self.stage == 'dock_advance_plan':
                    self.plan_advance_step()
                    return
                if self.stage in ('dock_observe', 'dock_actuate'):
                    self.observe(now)
                    return
            super().tick()
        except (ValueError, RuntimeError, KeyError, TransformException, np.linalg.LinAlgError) as exc:
            self.fail(str(exc))

    def camera_target(self, base_tag, current):
        if self.tcp_camera is None:
            # Camera is rigidly mounted on the wrist; get the optical-frame
            # extrinsic, not the mounting/body frame. No saved endpoint is used.
            tf = self.tf_buffer.lookup_transform(
                'tcp_link', self.camera_frame, Time(),
                timeout=Duration(seconds=self.cfg['tf_timeout_s'])).transform
            p, q = tf.translation, tf.rotation
            self.tcp_camera = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        camera = current @ self.tcp_camera
        if self.camera_reference_x is None:
            self.camera_reference_x = camera[:3, 0].copy()
        target, distance = camera_normal_target(
            base_tag, camera, self.camera_reference_x, self.camera_alignment_distance)
        if distance <= self.cfg['advance_distance_m']+self.cfg['position_tolerance_m']:
            raise RuntimeError('tag stand-off too short for the requested fixed advance')
        self.camera_alignment_distance = distance
        return target @ np.linalg.inv(self.tcp_camera), target

    def start_advance(self, q, current, camera_target):
        """Latch one relative line after visual alignment; never re-anchor it."""
        self.advance_locked = True
        self.advance_origin = current.copy()
        self.advance_target = current.copy()
        self.advance_target[:3, 3] += (
            camera_target[:3, 2]*self.cfg['advance_distance_m'])
        self.advance_seed = q.copy()
        self.advance_count = math.ceil(self.cfg['advance_distance_m']/self.cfg['step_m'])
        self.advance_plan = dict(waypoints=[], durations=[], total=0., start_error=0.,
                                 cartesian_line=True)
        self.advance_snapshot = dict(self.full_state)
        self.stage = 'dock_advance_plan'
        self.report('advance_planning',
                    'alignment confirmed; planning fixed %.1f mm tag-normal advance; '
                    'tag visibility no longer required, L2 remains required' %
                    (self.cfg['advance_distance_m']*1000.))

    def plan_advance_step(self):
        # One bounded IK solve per timer tick keeps feedback/cancel callbacks live.
        tolerances = self.snapshot_tolerances(validation=True)
        if (self.full_state.keys() != self.advance_snapshot.keys()
                or any(abs(value-self.advance_snapshot[j]) > tolerances.get(j, .001)
                       for j, value in self.full_state.items())):
            raise RuntimeError('robot moved during fixed advance planning')
        index = len(self.advance_plan['waypoints'])+1
        target = self.advance_origin.copy()
        target[:3, 3] += ((self.advance_target[:3, 3]-self.advance_origin[:3, 3])
                         * index/self.advance_count)
        solved = self.model.solve(target, self.advance_seed, self.margin,
                                  max_iterations=100, max_time_s=.03)
        delta = np.abs(solved-self.advance_seed)
        if (not self.model.within_limits(solved, self.margin)
                or np.max(delta) > self.cfg['joint_step_rad']):
            raise RuntimeError('advance IK exceeds joint step/limits')
        residual = self.model.pose_error(solved, target)
        if np.linalg.norm(residual[:3]) > 1e-4 or np.linalg.norm(residual[3:]) > 1e-3:
            raise RuntimeError('advance IK residual too large')
        duration = max(.02, float(np.max(delta))/self.cfg['joint_speed_rad_s'],
                       self.cfg['advance_distance_m']/self.advance_count
                       / self.cfg['linear_speed_m_s'])
        # Preserve the Cartesian line instead of isolating shoulder motion;
        # retain its conservative speed cap on every segment.
        if self.shoulder_isolation_enabled:
            duration = max(duration, float(delta[1])/self.shoulder_isolation_speed)
        self.advance_plan['waypoints'].append(solved.copy())
        self.advance_plan['durations'].append(duration)
        self.advance_plan['total'] += duration
        self.advance_seed = solved
        if index == self.advance_count:
            self.advance_plan['stop_at_waypoints'] = [False]*(index-1)+[True]
            self.begin(self.path_id, self.advance_plan)

    def observe(self, now):
        stamp, base_tag = self.fresh_observation(now)
        q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
        current = self.model.fk(q)
        camera_mode = self.cfg.get('alignment_mode') == 'camera_normal_advance'
        if camera_mode:
            target, camera_target = self.camera_target(base_tag, current)
        else:
            target = base_tag @ self.cfg['targets'][self.dock_stage]
        if camera_mode:
            distance, angle = pose_errors(current @ self.tcp_camera, camera_target)
            if pose_errors(current, target)[0] > self.cfg['max_position_error_m']:
                raise RuntimeError('camera alignment TCP target beyond permitted correction envelope')
        else:
            distance, angle = pose_errors(current, target)
        stopped = np.max(np.abs(self.velocity)) <= self.cfg['stopped_speed_rad_s']
        aligned = (distance <= self.cfg['position_tolerance_m']
                   and angle <= self.cfg['angle_tolerance_rad'])
        if self.stage == 'dock_actuate':
            if (not stopped or not aligned
                    or np.max(np.abs(q-self.lock_goal)) > self.cfg['joint_tolerance_rad']):
                raise RuntimeError('arm departed measured docking pose during Arduino actuation')
            if now >= self.actuation_deadline:
                raise RuntimeError('Arduino docking completion timeout')
            if (self.docked_feedback_sequence > self.command_feedback_sequence
                    and self.docked_state == self.expected_docked):
                state = 'docked' if self.expected_docked else 'released'
                self.report(state, f'Arduino {state} state confirmed; remain HOLD')
                self.reset()
            return
        if (stamp <= self.observe_after_ns or stamp == self.observed_image
                or self.feedback_sequence == self.observed_feedback):
            return
        self.observed_image, self.observed_feedback = stamp, self.feedback_sequence
        if not stopped:
            self.stable = 0
            return
        if distance > self.cfg['max_position_error_m'] or angle > self.cfg['max_angle_error_rad']:
            raise RuntimeError('tag target beyond permitted correction envelope')
        if not self.motion_enabled:
            if now-self.last_preview_report >= 1.0:
                self.last_preview_report = now
                self.report('preview',
                            f'stage {self.dock_stage}: position_error={distance:.6f}m, '
                            f'angle_error={angle:.6f}rad; alignment motion disarmed')
            return
        if aligned:
            self.stable += 1
            if self.stable < self.cfg['stable_samples']:
                return
            self.stable = 0
            if camera_mode:
                self.start_advance(q, current, camera_target)
                return
            if self.dock_stage < 2:
                self.dock_stage += 1
                self.observe_after_ns = self.get_clock().now().nanoseconds
                self.report('aligning', f'confirmed alignment; advance to stage {self.dock_stage}')
                return
            if not self.cfg['actuation_enabled']:
                self.report('docked', 'visual docking settled; Arduino actuation disabled; remain HOLD')
                self.reset()
                return
            if self.docking_command.get_subscription_count() == 0:
                raise RuntimeError('Arduino command subscriber unavailable')
            self.lock_goal = q.copy()
            self.expected_docked = self.cfg['desired_docked']
            self.command_feedback_sequence = self.docked_feedback_sequence
            self.actuation_deadline = now+self.cfg['actuation_timeout_s']
            self.send_docking_pulse(self.expected_docked)
            self.stage = 'dock_actuate'
            action = 'dock' if self.expected_docked else 'release'
            self.report('actuating', f'Arduino {action} pulse sent; waiting for /arm/docked')
            return
        self.stable = 0
        increment = bounded_target(current, target, self.cfg['step_m'], self.cfg['step_rad'])
        solved = self.model.solve(increment, q, self.margin, max_iterations=60, max_time_s=.03)
        if (not self.model.within_limits(solved, self.margin)
                or np.max(np.abs(solved-q)) > self.cfg['joint_step_rad']):
            raise RuntimeError('docking IK exceeds joint step/limits')
        residual = self.model.pose_error(solved, increment)
        if np.linalg.norm(residual[:3]) > 1e-4 or np.linalg.norm(residual[3:]) > 1e-3:
            raise RuntimeError('docking IK residual too large')
        travel, rotation = pose_errors(current, increment)
        duration = max(.5, float(np.max(np.abs(solved-q)))/self.cfg['joint_speed_rad_s'],
                       travel/self.cfg['linear_speed_m_s'], rotation/self.cfg['angular_speed_rad_s'])
        self.begin(self.path_id, dict(waypoints=[solved], durations=[duration],
                                     total=duration, start_error=0.))


def main():
    rclpy.init()
    node = None
    try:
        node = ToolDockingCoordinator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node.stage != 'idle':
                node.fail('docking coordinator shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
