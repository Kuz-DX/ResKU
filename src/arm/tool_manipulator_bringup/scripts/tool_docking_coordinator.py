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
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tool_trajectory_joy import ToolTrajectoryJoy, ARM_JOINTS
from tool_docking_core import load_settings, transform, pose_errors, bounded_target, tag_pose
try:
    from tool_change_min.changer_protocol import ChangerTransaction
except ImportError:
    from arm_changer_protocol import ChangerTransaction


class ToolDockingCoordinator(ToolTrajectoryJoy):
    def __init__(self):
        super().__init__()
        self.settings = load_settings(self.declare_parameter('docking_config', '').value)
        # Must match the inherited ArmKinematics root, not an arbitrary TF frame.
        self.base_frame = 'base_actuator'
        self.camera_frame = self.declare_parameter('docking_camera_frame', '').value
        if self.settings and not self.camera_frame:
            raise ValueError('docking_camera_frame must name the calibrated optical frame')
        self.camera = None
        self.velocity = None
        self.feedback_sequence = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.changer = self.create_publisher(String, '/tool_changer/command', 10)
        self.create_subscription(String, '/tool_changer/status', self.on_changer, 10)
        self.create_subscription(CameraInfo, self.declare_parameter(
            'docking_camera_info_topic', '/arm/camera/color/camera_info').value,
            self.on_camera, qos_profile_sensor_data)
        self.create_subscription(String, self.declare_parameter(
            'docking_centers_topic', '/arm/apriltag/centers').value, self.on_tag, 1)
        self.create_service(Trigger, '/arm/tool_docking/cancel', self.on_cancel)
        self.get_logger().info(f'Optional docking paths: {sorted(self.settings)}')

    def reset(self):
        super().reset()
        self.active = False
        self.docking_started = None
        self.transaction = None
        self.observation = None
        self.last_image_stamp = -1
        self.dock_stage = 0
        self.stable = 0
        self.observed_image = -1
        self.observed_feedback = -1
        self.hold_epoch = None
        self.lock_goal = None

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
        if not self.active:
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
            detections = [d for d in data['detections'] if d['id'] == self.cfg['tag_id']]
            if len(detections) != 1:
                raise ValueError('expected exactly one matching tag')
            camera_tag = tag_pose(detections[0], *self.camera, self.cfg)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, stamp, timeout=Duration(seconds=0.)).transform
            p, q = tf.translation, tf.rotation
            base_tag = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]) @ camera_tag
            self.observation = (stamp_ns, time.monotonic(), base_tag)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError,
                TransformException, cv2.error):
            # An invalid/absent target invalidates the previous observation too.
            self.stable = 0

    def begin(self, path_id, planned_path=None):
        if planned_path is None and path_id in self.settings:
            self.active = True
            self.cfg = self.settings[path_id]
            self.docking_started = None
        super().begin(path_id, planned_path)

    def begin_failed(self, reason):
        if self.active:
            self.fail(reason)
        else:
            super().begin_failed(reason)

    def fail(self, reason):
        if self.active:
            if self.transaction is not None:
                stop = self.transaction.abort(reason)
                if stop is not None:
                    self.changer.publish(String(data=stop))
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

    def on_changer(self, msg):
        if self.transaction is not None:
            try:
                self.transaction.receive(msg.data, time.monotonic())
            except RuntimeError as exc:
                self.fail(str(exc))

    def path_completed(self):
        if not self.active:
            super().path_completed()
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
                else:
                    # Tag loss also cancels a correction while it is being checked/executed.
                    self.fresh_observation(now)
                if self.stage in ('dock_observe', 'dock_lock'):
                    self.observe(now)
                    return
            super().tick()
        except (ValueError, RuntimeError, KeyError, np.linalg.LinAlgError) as exc:
            self.fail(str(exc))

    def observe(self, now):
        stamp, base_tag = self.fresh_observation(now)
        q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
        target = base_tag @ self.cfg['targets'][self.dock_stage]
        current = self.model.fk(q)
        distance, angle = pose_errors(current, target)
        stopped = np.max(np.abs(self.velocity)) <= self.cfg['stopped_speed_rad_s']
        aligned = (distance <= self.cfg['position_tolerance_m']
                   and angle <= self.cfg['angle_tolerance_rad'])
        if self.stage == 'dock_lock':
            if (not stopped or not aligned
                    or np.max(np.abs(q-self.lock_goal)) > self.cfg['joint_tolerance_rad']):
                raise RuntimeError('arm departed measured docking pose during LOCK')
            if self.transaction.check(now):
                self.report('locked', 'Arduino LOCKED confirmed; remain HOLD (no automatic retreat)')
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
        if aligned:
            self.stable += 1
            if self.stable < self.cfg['stable_samples']:
                return
            self.stable = 0
            if self.dock_stage < 2:
                self.dock_stage += 1
                self.observe_after_ns = self.get_clock().now().nanoseconds
                self.report('aligning', f'confirmed alignment; advance to stage {self.dock_stage}')
                return
            if not self.cfg['lock_enabled']:
                self.report('docked', 'visual docking settled; LOCK disabled; remain HOLD')
                self.reset()
                return
            if self.changer.get_subscription_count() == 0:
                raise RuntimeError('Arduino command subscriber unavailable')
            self.lock_goal = q.copy()
            self.transaction = ChangerTransaction('LOCK', self.cfg['lock_timeout_s'], now)
            self.changer.publish(String(data=self.transaction.request()))
            self.stage = 'dock_lock'
            self.report('locking', 'LOCK sent once; waiting for matching LOCKED')
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
