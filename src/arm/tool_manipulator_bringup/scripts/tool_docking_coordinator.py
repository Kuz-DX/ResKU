#!/usr/bin/env python3
"""Optional tool path -> tag-guided continuation coordinator."""
import json
import math
import time
import uuid

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from tool_trajectory_joy import ToolTrajectoryJoy, ARM_JOINTS
from tool_docking_core import (load_settings, transform, pose_errors, bounded_target,
                               tag_pose, tag_normal_from_camera_side,
                               direction_angle, interpolate_direction)
from tool_ids import tool_label


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
        self.camera_reason = 'CameraInfo not received'
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
        self.get_logger().info(
            f'Optional docking paths: {sorted(self.settings)}; motion_enabled={self.motion_enabled}; '
            'waiting for an explicit tool path request')

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
        self.tcp_camera = None
        self.advance_locked = False
        self.advance_target = None
        self.advance_plan = None
        self.tool0_removal_complete = False
        self.reset_tag_acquisition()

    def reset_tag_acquisition(self):
        """Acquire a fresh anchor after the taught approach has stopped."""
        self.observation = None
        self.last_image_stamp = -1
        self.last_base_tag = None
        self.tag_received = self.tag_accepted = 0
        self.tag_reason = 'no tag messages received since acquisition started'
        self.last_acquire_report = -math.inf

    def tag_diagnostic(self):
        return ('received=%d accepted=%d; %s; camera=%s' % (
            self.tag_received, self.tag_accepted, self.tag_reason,
            self.camera_reason))

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
        if msg.header.frame_id != self.camera_frame:
            self.camera_reason = (
                f'CameraInfo frame mismatch: received={msg.header.frame_id!r}, '
                f'expected={self.camera_frame!r}')
        elif (matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion))):
            self.camera_reason = 'CameraInfo has invalid intrinsics/distortion'
        elif msg.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            self.camera_reason = f'unsupported CameraInfo distortion model: {msg.distortion_model!r}'
        else:
            self.camera = (matrix, distortion)
            self.camera_reason = f'ready ({self.camera_frame})'

    def on_tag(self, msg):
        if not self.active or getattr(self, 'advance_locked', False):
            return
        self.tag_received += 1
        try:
            data = json.loads(msg.data)
            stamp = Time(seconds=int(data['stamp']['sec']), nanoseconds=int(data['stamp']['nanosec']))
            stamp_ns = stamp.nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            if self.camera is None:
                raise ValueError(self.camera_reason)
            if data['frame_id'] != self.camera_frame:
                raise ValueError(
                    f'tag frame mismatch: received={data["frame_id"]!r}, '
                    f'expected={self.camera_frame!r}')
            if not 0 <= age < self.cfg['tag_max_age_s']:
                raise ValueError(
                    f'tag image age={age:.3f}s outside [0, {self.cfg["tag_max_age_s"]:.3f}s)')
            if stamp_ns <= self.last_image_stamp:
                raise ValueError('tag image timestamp did not advance')
            self.last_image_stamp = stamp_ns
            detections = [d for d in data['detections'] if d['id'] == self.cfg['tag_id']
                          and ('tag_family' not in self.cfg
                               or d.get('family') == self.cfg['tag_family'])]
            if len(detections) != 1:
                seen = [(d.get('id'), d.get('family')) for d in data['detections']]
                raise ValueError(
                    f'expected one tag id={self.cfg["tag_id"]} '
                    f'family={self.cfg.get("tag_family", "any")}; received={seen}')
            camera_tag = tag_pose(detections[0], *self.camera, self.cfg)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, stamp,
                timeout=Duration(seconds=self.cfg['tf_timeout_s'])).transform
            p, q = tf.translation, tf.rotation
            base_camera = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            base_tag = base_camera @ camera_tag
            if self.last_base_tag is not None:
                jump, turn = pose_errors(self.last_base_tag, base_tag)
                if (jump > self.cfg['max_tag_position_jump_m']
                        or turn > self.cfg['max_tag_angle_jump_rad']):
                    # Planar IPPE candidates can swap reprojection-error order
                    # from sub-pixel noise. Retry against the previously
                    # accepted stationary rack pose before rejecting the frame.
                    camera_reference = np.linalg.inv(base_camera) @ self.last_base_tag
                    camera_tag = tag_pose(
                        detections[0], *self.camera, self.cfg, reference=camera_reference)
                    base_tag = base_camera @ camera_tag
                    jump, turn = pose_errors(self.last_base_tag, base_tag)
                    if (jump > self.cfg['max_tag_position_jump_m']
                            or turn > self.cfg['max_tag_angle_jump_rad']):
                        raise ValueError(
                            f'tag pose discontinuity rejected: translation={jump:.4f}m '
                            f'angle={turn:.4f}rad')
            self.last_base_tag = base_tag
            self.observation = (stamp_ns, time.monotonic(), base_tag)
            self.tag_accepted += 1
            self.tag_reason = 'valid tag pose accepted'
        except TransformException as exc:
            self.tag_reason = f'TF {self.base_frame} <- {self.camera_frame}: {exc}'
        except (ValueError, TypeError, KeyError, IndexError, OverflowError,
                cv2.error) as exc:
            # Keep the last accepted pose only until its strict freshness
            # deadline. A rejected image is not a stability sample, but it must
            # not erase already confirmed, distinct inliers.
            self.tag_reason = str(exc)

    def begin(self, path_id, planned_path=None):
        if planned_path is None and path_id in self.settings:
            self.active = True
            self.cfg = self.settings[path_id]
            self.docking_started = None
            self.path_id = path_id
            mode = self.cfg.get('alignment_mode')
            continuation = (
                'checked final docking joint goal'
                if mode == 'fixed_joint_goal' else
                'checked tagless TCP-axis advance'
                if mode == 'fixed_tcp_axis_advance' else
                'stopped tag confirmation and fixed advance'
                if mode == 'camera_normal_advance' else 'tag alignment')
            self.report('docking_armed',
                        f'{tool_label(path_id)} path includes {continuation} after approach completion; '
                        f'motion_enabled={self.motion_enabled}')
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
            actual = self.model.fk(q)
            distance = np.linalg.norm(actual[:3, 3]-self.advance_target[:3, 3])
            angle = (pose_errors(actual, self.advance_target)[1]
                     if self.cfg.get('maintain_entry_orientation', False) else
                     direction_angle(actual[:3, :3] @ self.advance_local_axis,
                                     self.advance_direction))
            speed = float(np.max(np.abs(self.velocity)))
            fixed_joint = self.cfg.get('alignment_mode') == 'fixed_joint_goal'
            joint_error = (float(np.max(np.abs(q-self.advance_plan['waypoints'][-1])))
                           if fixed_joint else 0.)
            invalid = ((joint_error > self.cfg['joint_tolerance_rad']) if fixed_joint else
                       (distance > self.cfg['position_tolerance_m']
                        or angle > self.cfg['angle_tolerance_rad']))
            if invalid or speed > self.cfg['stopped_speed_rad_s']:
                raise RuntimeError(
                    'advance endpoint feedback outside tolerance; remain HOLD: '
                    f'joint_error={joint_error:.6f}rad '
                    f'limit={self.cfg["joint_tolerance_rad"]:.6f}rad; '
                    f'position_error={distance:.6f}m '
                    f'limit={self.cfg["position_tolerance_m"]:.6f}m; '
                    f'angle_error={angle:.6f}rad '
                    f'limit={self.cfg["angle_tolerance_rad"]:.6f}rad; '
                    f'max_speed={speed:.6f}rad/s '
                    f'limit={self.cfg["stopped_speed_rad_s"]:.6f}rad/s')
            if self.cfg['actuation_enabled']:
                if self.path_id == 0 and fixed_joint:
                    self.lock_goal = q.copy()
                    self.hold_epoch = self.status['epoch']
                    self.command_feedback_sequence = self.docked_feedback_sequence
                    self.send_docking_pulse(True)
                    self.actuation_deadline = time.monotonic()+self.cfg['actuation_timeout_s']
                    self.stage = 'dock_attach_wait'
                    self.report('actuating', 'final dock reached; /arm/docking=true; waiting for Arduino completion')
                    return
                self.lock_goal = q.copy()
                self.expected_docked = self.cfg['desired_docked']
                self.command_feedback_sequence = self.docked_feedback_sequence
                self.actuation_deadline = time.monotonic()+self.cfg['actuation_timeout_s']
                self.snapshot = dict(self.full_state)
                self.send_docking_pulse(self.expected_docked)
                self.stage = 'dock_actuate'
                action = 'dock' if self.expected_docked else 'release'
                self.report('actuating',
                            'fixed advance completed; Arduino %s pulse sent; '
                            'waiting for /arm/docked' % action)
            else:
                self.report('completed', 'tag-normal advance %.1f mm completed; HOLD; no actuation' %
                            (self.cfg['advance_distance_m']*1000.))
                self.reset()
            return
        if (self.docking_started is None
                and self.cfg.get('alignment_mode') == 'fixed_joint_goal'):
            if not self.motion_enabled:
                self.report('preview', 'final docking motion disabled; remain HOLD')
                self.reset()
                return
            q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
            self.execution_id = None
            self.hold_epoch = self.status['epoch']
            self.docking_started = time.monotonic()
            self.start_final_joint_goal(q)
            return
        if (self.docking_started is None
                and self.cfg.get('alignment_mode') == 'fixed_tcp_axis_advance'):
            q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
            current = self.model.fk(q)
            local_axis = np.asarray(self.cfg['advance_axis_tcp'], dtype=float)
            direction = current[:3, :3] @ local_axis
            self.execution_id = None
            self.hold_epoch = self.status['epoch']
            self.docking_started = time.monotonic()
            self.start_advance(q, current, direction)
            return
        if self.docking_started is None:
            # Do not let a noisy pose observed during approach permanently
            # reject every stopped observation against an old jump anchor.
            self.reset_tag_acquisition()
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
            raise RuntimeError('target tag missing or rejected: '+self.tag_diagnostic())
        stamp, received, base_tag = self.observation
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age < self.cfg['tag_max_age_s'] or now-received >= self.cfg['tag_max_age_s']:
            raise RuntimeError('target tag stale: '+self.tag_diagnostic())
        return stamp, base_tag

    def check_deadman(self, now):
        # Tool 1 is a single requested sequence, including its docking tail.
        # The trigger authorizes the sequence; subsequent Joy messages are not
        # its heartbeat. Feedback, ownership and explicit cancel still guard it.
        if not self.cfg.get('require_l2_hold', True):
            return
        age = now-self.last_joy_time
        detail = f'stage={self.stage}, joy_age={age:.3f}s'
        if self.stage == 'checking':
            detail += f', checked={self.request_index}/{len(self.requests)}'
        if age >= self.deadman_timeout:
            raise RuntimeError(
                f'docking joystick stream stale ({detail}, limit={self.deadman_timeout:.3f}s)')
        if not self.deadman_held:
            raise RuntimeError(f'docking deadman L2 released ({detail}); hold L2 through checking and motion')

    def guard(self, now):
        self.check_deadman(now)
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
            if self.active and self.stage in ('dock_release_wait', 'dock_attach_wait',
                                              'ee_mode_wait', 'ee_removal_running'):
                self.tick_tool0_finish(time.monotonic())
                return
            if self.active and self.docking_started is None:
                self.check_deadman(time.monotonic())
            if self.active and self.docking_started is not None:
                now = time.monotonic()
                self.guard(now)
                if self.stage == 'dock_acquire':
                    if now-self.docking_started > self.cfg['acquire_timeout_s']:
                        raise RuntimeError('target tag acquisition timed out: '+self.tag_diagnostic())
                    if self.observation is None:
                        if now-self.last_acquire_report >= 1.:
                            self.report('dock_acquire', 'waiting for valid tag: '+self.tag_diagnostic())
                            self.last_acquire_report = now
                        return
                    try:
                        self.fresh_observation(now)
                    except RuntimeError as exc:
                        if not str(exc).startswith('target tag stale:'):
                            raise
                        # A frame accepted close to the age boundary can turn
                        # stale between its callback and this timer tick. During
                        # acquisition, discard it and wait for the next image;
                        # the acquisition timeout remains the hard deadline.
                        self.observation = None
                        self.last_base_tag = None
                        self.stable = 0
                        self.tag_reason = 'stale acquisition sample discarded; waiting for fresh tag'
                        return
                    self.stage = 'dock_observe'
                elif not getattr(self, 'advance_locked', False):
                    # Tag loss also cancels a correction while it is being checked/executed.
                    try:
                        self.fresh_observation(now)
                    except RuntimeError as exc:
                        if (self.cfg.get('alignment_mode') != 'camera_normal_advance'
                                or not str(exc).startswith('target tag stale:')):
                            raise
                        # Camera-normal docking has not moved before the fixed
                        # advance is locked. Return to acquisition instead of
                        # aborting because the first accepted frame aged out
                        # while stable samples were still being collected.
                        self.observation = None
                        self.last_base_tag = None
                        self.stable = 0
                        self.stage = 'dock_acquire'
                        self.tag_reason = 'stale pre-advance sample discarded; waiting for fresh tag'
                        return
                if self.stage == 'dock_advance_plan':
                    self.plan_advance_step()
                    return
                if self.stage in ('dock_observe', 'dock_actuate'):
                    self.observe(now)
                    return
            super().tick()
        except (ValueError, RuntimeError, KeyError, TransformException, np.linalg.LinAlgError) as exc:
            self.fail(str(exc))

    def tick_tool0_finish(self, now):
        self.guard(now)
        q = np.asarray([self.full_state[j] for j in ARM_JOINTS])
        if np.max(np.abs(q-self.lock_goal)) > self.cfg['joint_tolerance_rad']:
            raise RuntimeError('arm departed final dock during completion sequence')
        if self.stage == 'ee_mode_wait':
            if now-self.ee_mode_started > 3.:
                raise RuntimeError('EE mode transition timeout')
            if not self.future.done():
                return
            if self.future.result() is None or not self.future.result().success:
                raise RuntimeError('EE mode request rejected')
            if self.status.get('mode') != 'tool_auto' or self.status.get('transitioning'):
                return
            self.execution_id = 'tool0-ee-'+uuid.uuid4().hex
            self.publisher.publish(String(data=json.dumps(dict(
                epoch=self.status['epoch'], execution_id=self.execution_id,
                ee_velocity_rad_s=5.0, ee_duration_sec=2.0))))
            self.stage = 'ee_removal_running'
            self.ee_mode_started = now
            self.report('ee_removal', 'CCW +5 rad/s for 2 seconds; waiting for stopped feedback')
            return
        if self.stage in ('dock_release_wait', 'dock_attach_wait'):
            if getattr(self, 'docking_status', None) in ('ERROR', 'UNKNOWN'):
                raise RuntimeError('Arduino unavailable: '+self.docking_status)
            if np.max(np.abs(self.velocity)) > self.cfg['stopped_speed_rad_s']:
                raise RuntimeError('arm not stopped during Arduino completion')
            if now >= self.actuation_deadline:
                raise RuntimeError('Arduino completion timeout; EE not started')
            expected = self.stage == 'dock_attach_wait'
            if (self.docked_feedback_sequence <= self.command_feedback_sequence
                    or self.docked_state != expected):
                return
            if expected:
                self.command_feedback_sequence = self.docked_feedback_sequence
                self.send_docking_pulse(False)
                self.actuation_deadline = now+self.cfg['actuation_timeout_s']
                self.stage = 'dock_release_wait'
                self.report('actuating', '/arm/docking=false; waiting for Arduino completion')
            else:
                self.future = self.mode.call_async(Trigger.Request())
                self.stage = 'ee_mode_wait'
                self.ee_mode_started = now
            return
        if self.stage == 'ee_removal_running':
            if now-self.ee_mode_started > 10.:
                raise RuntimeError('EE removal completion timeout')
            if self.status.get('last_execution_id') == self.execution_id:
                if self.status.get('last_execution_result') != 'succeeded':
                    raise RuntimeError('EE removal failed: '+self.status.get('reason', 'unknown'))
                if self.status.get('mode') == 'hold' and not self.status.get('transitioning'):
                    self.active = False
                    self.advance_locked = False
                    self.tool0_removal_complete = True
                    self.begin_docking_retreat()
            elif self.status.get('mode') != 'tool_auto':
                raise RuntimeError('EE removal ownership lost')

    def camera_advance_direction(self, base_tag, current):
        if self.tcp_camera is None:
            # Camera is rigidly mounted on the wrist; get the optical-frame
            # extrinsic, not the mounting/body frame. No saved endpoint is used.
            tf = self.tf_buffer.lookup_transform(
                'tcp_link', self.camera_frame, Time(),
                timeout=Duration(seconds=self.cfg['tf_timeout_s'])).transform
            p, q = tf.translation, tf.rotation
            self.tcp_camera = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        camera = current @ self.tcp_camera
        normal, distance = tag_normal_from_camera_side(base_tag, camera)
        if distance <= self.cfg['advance_distance_m']+self.cfg['position_tolerance_m']:
            raise RuntimeError('tag stand-off too short for the requested fixed advance')
        return normal, distance

    def start_advance(self, q, current, tag_normal):
        """Latch one relative line after stopped tag confirmation."""
        if self.cfg.get('alignment_mode') == 'fixed_tcp_axis_advance':
            self.advance_local_axis = np.asarray(self.cfg['advance_axis_tcp'], dtype=float)
        else:
            self.advance_local_axis = self.tcp_camera[:3, 2].copy()
        self.advance_start_direction = current[:3, :3] @ self.advance_local_axis
        self.advance_direction = tag_normal.copy()
        tilt = direction_angle(self.advance_start_direction, tag_normal)
        if tilt > self.cfg['max_angle_error_rad']:
            raise RuntimeError(f'entry-axis tilt {tilt:.4f}rad exceeds permitted correction envelope')
        self.advance_locked = True
        self.advance_origin = current.copy()
        self.advance_target = current.copy()
        self.advance_target[:3, 3] += tag_normal*self.cfg['advance_distance_m']
        self.advance_seed = q.copy()
        maintain = self.cfg.get('maintain_entry_orientation', False)
        self.advance_count = max(math.ceil(self.cfg['advance_distance_m']/self.cfg['step_m']),
                                 1 if maintain else math.ceil(tilt/self.cfg['step_rad']))
        self.advance_plan = dict(start=q.copy(), waypoints=[], durations=[], total=0., start_error=0.,
                                 cartesian_line=True)
        self.advance_snapshot = dict(self.full_state)
        self.stage = 'dock_advance_plan'
        source = ('confirmed TCP local axis'
                  if self.cfg.get('alignment_mode') == 'fixed_tcp_axis_advance'
                  else 'confirmed tag normal')
        self.report('advance_planning',
                    'stopped %s endpoint; planning fixed %.1f mm advance along %s; '
                    'normal=[%.4f, %.4f, %.4f] seed=%s; %s; '
                    'entry tilt %.2f deg; tag visibility no longer required; %s' %
                    (tool_label(self.path_id), self.cfg['advance_distance_m']*1000., source,
                     *map(float, tag_normal),
                     '['+', '.join(f'{value:.4f}' for value in q)+']',
                     ('holding the completed pre-dock TCP orientation'
                      if maintain else 'axial rotation free; tilt changes gradually'),
                     math.degrees(tilt),
                     'L2 remains required' if self.cfg.get('require_l2_hold', True)
                     else 'continuing automatic tool sequence'))

    def start_final_joint_goal(self, q):
        """Build one fully checked joint-space tail to the measured dock pose."""
        goals = list(self.cfg.get('intermediate_joint_goals', [])) + [self.cfg['final_joint_goal']]
        waypoints, stops, seed = [], [], q.copy()
        for captured in goals:
            goal = np.asarray(captured, dtype=float).copy()
            goal[0] += 2.*math.pi*round((seed[0]-goal[0])/(2.*math.pi))
            if not self.model.within_limits(goal, self.margin):
                raise RuntimeError('docking joint waypoint violates planning limits')
            count = max(1, math.ceil(float(np.max(np.abs(goal-seed)))
                                     / self.cfg['joint_step_rad']))
            waypoints.extend(seed+(goal-seed)*(i/count) for i in range(1, count+1))
            stops.extend([False]*(count-1)+[True])
            seed = goal
        axial = self.cfg.get('axial_sampled_path', False)
        if axial:
            # Reject an off-axis measured entry rather than silently dragging it
            # into the insertion line. All samples still go through collision checks.
            final_pose = self.model.fk(goal)
            entry_pose = self.model.fk(q)
            expected_entry = final_pose.copy()
            expected_entry[:3, 3] -= final_pose[:3, 2] * .030
            position_error, angle_error = pose_errors(entry_pose, expected_entry)
            if position_error > .002 or angle_error > .02:
                raise RuntimeError('axial docking entry mismatch: '
                                   f'{position_error:.6f}m, {angle_error:.6f}rad')
            stops = [False] * (len(waypoints)-1) + [True]
        count = len(waypoints)
        durations, previous = [], q.copy()
        for point in waypoints:
            durations.append(max(.02, float(np.max(np.abs(point-previous)))
                                 / self.cfg['joint_speed_rad_s']))
            if axial:
                distance = np.linalg.norm(self.model.fk(point)[:3, 3] -
                                          self.model.fk(previous)[:3, 3])
                durations[-1] = max(durations[-1], distance/self.cfg['linear_speed_m_s'])
            previous = point
        self.advance_locked = True
        self.advance_origin = self.model.fk(q)
        self.advance_target = self.model.fk(goal)
        self.advance_local_axis = np.array([0., 0., 1.])
        self.advance_direction = self.advance_target[:3, 2].copy()
        self.advance_plan = dict(
            start=q.copy(), waypoints=waypoints, durations=durations,
            total=sum(durations), start_error=0., cartesian_line=axial,
            stop_at_waypoints=stops,
            allowed_collision_pairs=frozenset((
                tuple(sorted(('tool_case_1', 'wrist_link_yaw'))),)))
        self.report('advance_planning',
                    'stopped %s pre-dock; checking measured final joint goal; '
                    'joint_delta_max=%.4f rad, samples=%d; tag not required' %
                    (tool_label(self.path_id), float(np.max(np.abs(goal-q))), count))
        self.begin(self.path_id, self.advance_plan)

    def plan_advance_step(self):
        # One bounded IK solve per timer tick keeps feedback/cancel callbacks live.
        tolerances = self.snapshot_tolerances(validation=True)
        current = {j: v for j, v in self.full_state.items() if j != 'ee_joint'}
        snapshot = {j: v for j, v in self.advance_snapshot.items() if j != 'ee_joint'}
        if (current.keys() != snapshot.keys()
                or any(abs(value-snapshot[j]) > tolerances.get(j, .001)
                       for j, value in current.items())):
            raise RuntimeError('robot moved during fixed advance planning')
        index = len(self.advance_plan['waypoints'])+1
        target = self.advance_origin.copy()
        target[:3, 3] += ((self.advance_target[:3, 3]-self.advance_origin[:3, 3])
                         * index/self.advance_count)
        maintain = self.cfg.get('maintain_entry_orientation', False)
        direction = (self.advance_start_direction if maintain else interpolate_direction(
            self.advance_start_direction, self.advance_direction, index/self.advance_count))
        try:
            if maintain:
                # target retains advance_origin's complete rotation, so every
                # point is translation-only after pre-dock alignment.
                solved = self.model.solve(
                    target, self.advance_seed, self.margin,
                    max_iterations=100, max_time_s=.03)
            else:
                solved = self.model.solve_axis(
                    target[:3, 3], direction, self.advance_local_axis,
                    self.advance_seed, self.margin, max_iterations=100,
                    max_time_s=.03, joint_step_rad=self.cfg['joint_step_rad'])
        except ValueError as exc:
            raise ValueError(f'advance IK point {index}/{self.advance_count}: {exc}') from exc
        delta = np.abs(solved-self.advance_seed)
        if (not self.model.within_limits(solved, self.margin)
                or np.max(delta) > self.cfg['joint_step_rad']):
            raise RuntimeError('advance IK exceeds joint step/limits')
        actual = self.model.fk(solved)
        rotation_error = (pose_errors(actual, target)[1] if maintain else
                          direction_angle(actual[:3, :3] @ self.advance_local_axis, direction))
        if (np.linalg.norm(actual[:3, 3]-target[:3, 3]) > 1e-4
                or rotation_error > 1e-3):
            raise RuntimeError('advance IK residual too large')
        duration = max(.02, float(np.max(delta))/self.cfg['joint_speed_rad_s'],
                       self.cfg['advance_distance_m']/self.advance_count
                       / self.cfg['linear_speed_m_s'],
                       pose_errors(self.model.fk(self.advance_seed), actual)[1]
                       / self.cfg['angular_speed_rad_s'])
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
            tag_normal, tag_distance = self.camera_advance_direction(base_tag, current)
            if self.cfg.get('measured_pre_dock_alignment_enabled', False):
                final_target = base_tag @ self.cfg['measured_tag_tcp_target']
                target = current.copy()
                target[:3, 3] = final_target[:3, 3]
                if self.stage != 'dock_actuate':
                    target[:3, 3] -= tag_normal*self.cfg['advance_distance_m']
                distance = float(np.linalg.norm(target[:3, 3]-current[:3, 3]))
                angle = 0.0  # Axis tilt is corrected gradually by the fixed advance.
                aligned = distance <= self.cfg['position_tolerance_m']
            else:
                target = current
                distance = angle = 0.0
                aligned = True
        else:
            target = base_tag @ self.cfg['targets'][self.dock_stage]
            distance, angle = pose_errors(current, target)
            aligned = (distance <= self.cfg['position_tolerance_m']
                       and angle <= self.cfg['angle_tolerance_rad'])
        stopped = np.max(np.abs(self.velocity)) <= self.cfg['stopped_speed_rad_s']
        if self.stage == 'dock_actuate':
            if (not stopped or not aligned
                    or np.max(np.abs(q-self.lock_goal)) > self.cfg['joint_tolerance_rad']):
                raise RuntimeError('arm departed measured docking pose during Arduino actuation')
            if now >= self.actuation_deadline:
                raise RuntimeError('Arduino docking completion timeout')
            if (self.docked_feedback_sequence > self.command_feedback_sequence
                    and self.docked_state == self.expected_docked):
                state = 'docked' if self.expected_docked else 'released'
                self.report(state, f'Arduino {state} state confirmed; starting retreat')
                self.active = False
                self.advance_locked = False
                self.begin_docking_retreat()
            return
        if (stamp <= self.observe_after_ns or stamp == self.observed_image
                or self.feedback_sequence == self.observed_feedback):
            return
        self.observed_image, self.observed_feedback = stamp, self.feedback_sequence
        if not stopped:
            self.stable = 0
            return
        if (distance > self.cfg['max_position_error_m']
                or angle > self.cfg['max_angle_error_rad']):
            delta = target[:3, 3]-current[:3, 3]
            raise RuntimeError(
                'tag target beyond permitted correction envelope: '
                f'position_error={distance:.6f}m '
                f'delta=[{delta[0]:.6f}, {delta[1]:.6f}, {delta[2]:.6f}]m '
                f'limit={self.cfg["max_position_error_m"]:.6f}m; '
                f'angle_error={angle:.6f}rad '
                f'limit={self.cfg["max_angle_error_rad"]:.6f}rad')
        if not self.motion_enabled:
            if now-self.last_preview_report >= 1.0:
                self.last_preview_report = now
                detail = (f'tag stand-off={tag_distance:.3f}m; '
                          f'pre-dock position_error={distance:.6f}m; motion disarmed'
                          if camera_mode else
                          f'stage {self.dock_stage}: position_error={distance:.6f}m, '
                          f'angle_error={angle:.6f}rad; alignment motion disarmed')
                self.report('preview', detail)
            return
        if aligned:
            self.stable += 1
            if self.stable < self.cfg['stable_samples']:
                return
            self.stable = 0
            if camera_mode:
                self.start_advance(q, current, tag_normal)
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
        if camera_mode and now-self.last_preview_report >= 1.0:
            self.last_preview_report = now
            delta = target[:3, 3]-current[:3, 3]
            self.report(
                'aligning',
                'tag-derived pre-dock target: remaining=%.1f mm '
                'delta=[%.1f, %.1f, %.1f] mm; tag stand-off=%.1f mm; '
                'final checked advance=%.1f mm' %
                (distance*1000., *(delta*1000.), tag_distance*1000.,
                 self.cfg['advance_distance_m']*1000.))
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
        self.begin(self.path_id, dict(start=q.copy(), waypoints=[solved], durations=[duration],
                                     total=duration, start_error=0.))


def main():
    rclpy.init()
    node = None
    executor = None
    try:
        node = ToolDockingCoordinator()
        # The default mutually-exclusive group still serializes path/tag/state
        # callbacks. TransformListener has its own reentrant group: a second
        # worker can receive TF while on_tag waits for the image-time transform.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            if node.stage != 'idle':
                node.fail('docking coordinator shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
