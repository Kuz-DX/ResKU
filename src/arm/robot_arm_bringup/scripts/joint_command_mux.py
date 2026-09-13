#!/usr/bin/env python3
"""Validate and combine manual arm and gripper position targets."""

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String


class JointCommandMux(Node):
    """Sole command source for position_controller outside AUTO."""

    # Manual modes hold the measured pose on entry.  MANUAL_EE homes only
    # when the operator explicitly presses the configured gamepad button.
    HOME_ENTRY_MODES = ()

    def __init__(self):
        super().__init__('joint_command_mux')
        self.declare_parameter('joint_names', ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint', 'gripper_joint'])
        self.declare_parameter('manual_joint_names', ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint'])
        self.declare_parameter('gripper_joint_name', 'gripper_joint')
        self.declare_parameter('lower_limits', [-1.46955, -1.4818, -1.63541, -1.78041, 0.0])
        self.declare_parameter('upper_limits', [1.72113, 1.71548, 1.6383, 1.78041, 2.59396])
        # Order: base, shoulder, elbow, wrist, gripper.  Elbow hardware is
        # configured for 5 deg/s (0.0873 rad/s), so retain tracking margin.
        self.declare_parameter(
            'velocity_limits', [0.1, 0.1, 0.07, 0.1, 0.2])
        self.declare_parameter(
            'following_error_limits', [0.2, 0.2, 0.2, 0.2, 0.4])
        self.declare_parameter('following_error_timeout', 1.0)
        # MANUAL_EE-only collision-avoidance return sequence.
        self.declare_parameter(
            'home_joint_names', ['shoulder_joint', 'elbow_joint', 'wrist_joint'])
        self.declare_parameter('home_positions', [-0.55851, 1.58825, 1.51844])
        self.declare_parameter('home_base_position', 0.0)
        # Separate MANUAL_EE carrying pose. Excluding gripper_joint preserves
        # the operator's last grasp command while the arm moves to this pose.
        self.declare_parameter(
            'hold_joint_names',
            ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint'])
        self.declare_parameter(
            'hold_positions', [0.0, -0.76044, 1.63830, 1.53290])
        # Linux PlayStation mapping: Square = button 3.
        self.declare_parameter('hold_button', 3)
        self.declare_parameter(
            'home_shoulder_clearance_position', math.radians(30.0))
        self.declare_parameter(
            'home_wrist_stage1_position', math.radians(75.0))
        self.declare_parameter('home_branch_deadband', 0.03)
        self.declare_parameter('home_move_threshold', 0.03)
        self.declare_parameter('home_tolerance', 0.02)
        self.declare_parameter('home_base_tolerance', 0.05)
        self.declare_parameter('home_dwell_time', 0.3)
        self.declare_parameter('home_phase_timeout', 60.0)
        # Linux PlayStation mapping: Circle = button 1.
        self.declare_parameter('home_button', 1)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('manual_timeout', 0.75)
        self.declare_parameter('gripper_timeout', 0.75)
        self.declare_parameter('feedback_wait_warn_after', 1.0)
        self.declare_parameter('feedback_wait_log_interval', 5.0)
        self.declare_parameter('manual_command_topic', '/manual_joint_commands')
        self.declare_parameter(
            'manual_ee_command_topic', '/manual_ee_joint_commands')
        self.declare_parameter('manual_ee_ready_topic', '/control/manual_ee_ready')
        self.declare_parameter('gripper_command_topic', '/manual_gripper_command')
        self.declare_parameter('controller_command_topic', '/position_controller/commands')
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.manual_joint_names = list(self.get_parameter('manual_joint_names').value)
        self.gripper_joint = str(self.get_parameter('gripper_joint_name').value)
        self.lower = self.float_array('lower_limits')
        self.upper = self.float_array('upper_limits')
        self.velocity_limits = self.float_array('velocity_limits')
        self.following_error_limits = self.float_array(
            'following_error_limits')
        self.following_error_timeout = float(
            self.get_parameter('following_error_timeout').value)
        self.home_joint_names = list(
            self.get_parameter('home_joint_names').value)
        self.home_positions = self.float_array('home_positions')
        self.home_values = dict(zip(self.home_joint_names, self.home_positions))
        self.home_base_position = float(
            self.get_parameter('home_base_position').value)
        self.hold_joint_names = list(
            self.get_parameter('hold_joint_names').value)
        self.hold_positions = self.float_array('hold_positions')
        self.hold_values = dict(zip(self.hold_joint_names, self.hold_positions))
        self.hold_button = int(self.get_parameter('hold_button').value)
        self.home_shoulder_clearance_position = float(
            self.get_parameter('home_shoulder_clearance_position').value)
        self.home_wrist_stage1_position = float(
            self.get_parameter('home_wrist_stage1_position').value)
        self.home_branch_deadband = float(
            self.get_parameter('home_branch_deadband').value)
        self.home_move_threshold = float(
            self.get_parameter('home_move_threshold').value)
        self.home_tolerance = float(self.get_parameter('home_tolerance').value)
        self.home_base_tolerance = float(
            self.get_parameter('home_base_tolerance').value)
        self.home_dwell_time = float(
            self.get_parameter('home_dwell_time').value)
        self.home_phase_timeout = float(
            self.get_parameter('home_phase_timeout').value)
        self.home_button = int(self.get_parameter('home_button').value)
        self.validate_parameters()
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.gripper_timeout = float(self.get_parameter('gripper_timeout').value)
        self.feedback_wait_warn_after = float(
            self.get_parameter('feedback_wait_warn_after').value)
        self.feedback_wait_log_interval = float(
            self.get_parameter('feedback_wait_log_interval').value)
        if (not math.isfinite(self.feedback_wait_warn_after) or
                self.feedback_wait_warn_after < 0.0):
            raise ValueError(
                'feedback_wait_warn_after must be finite and non-negative')
        if (not math.isfinite(self.feedback_wait_log_interval) or
                self.feedback_wait_log_interval <= 0.0):
            raise ValueError(
                'feedback_wait_log_interval must be finite and positive')
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        self.mode = 'OFF'
        self.protective_stop = False
        self.hardware_fault = None
        self.hold_target: Optional[list] = None
        self.off_hold_target: Optional[list] = None
        self.manual_ee_pause_hold_target: Optional[list] = None
        self.tracking_fault = False
        self.following_error_started = {}
        self.positions: Dict[str, float] = {}
        self.manual_command: Optional[Dict[str, float]] = None
        self.ee_command: Optional[Dict[str, float]] = None
        self.gripper_command: Optional[float] = None
        self.manual_stamp = None
        self.ee_stamp = None
        self.gripper_stamp = None
        self.output: Optional[list] = None
        self.home_phase = None
        self.sequence_kind = None
        self.sequence_gripper_hold = None
        self.home_hold_target: Optional[list] = None
        self.home_phase_target = {}
        self.home_pending_phases = []
        self.home_phase_started = None
        self.home_in_tolerance_since = None
        self.last_joy_buttons = []
        self.last_timer_time = self.get_clock().now()
        self.feedback_wait_started = self.last_timer_time
        self.last_feedback_wait_log = None
        self.feedback_ready_logged = False
        self.publisher = self.create_publisher(Float64MultiArray, self.get_parameter('controller_command_topic').value, 10)
        ready_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.manual_ee_ready_pub = self.create_publisher(
            Bool, self.get_parameter('manual_ee_ready_topic').value,
            ready_qos)
        self.create_subscription(JointState, self.get_parameter('manual_command_topic').value, self.on_manual_command, 10)
        self.create_subscription(
            JointState,
            self.get_parameter('manual_ee_command_topic').value,
            self.on_ee_command,
            10)
        self.create_subscription(Float64, self.get_parameter('gripper_command_topic').value, self.on_gripper_command, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/control/mode', self.on_mode, qos)
        self.create_subscription(Bool, '/control/protective_stop', self.on_protective_stop, qos)
        self.create_subscription(
            String, '/control/hardware_fault', self.on_hardware_fault, qos)
        self.create_timer(1.0 / publish_rate, self.on_timer)
        self.publish_manual_ee_ready(False)

    def publish_manual_ee_ready(self, ready):
        self.manual_ee_ready_pub.publish(Bool(data=bool(ready)))

    def float_array(self, name):
        return list(map(float, self.get_parameter(name).value))

    def validate_parameters(self):
        count = len(self.joint_names)
        if count == 0 or len(set(self.joint_names)) != count:
            raise ValueError('joint_names must be non-empty and unique')
        if (count != len(self.lower) or count != len(self.upper) or
                count != len(self.velocity_limits) or
                count != len(self.following_error_limits)):
            raise ValueError('joint parameter array lengths must match')
        if set(self.manual_joint_names) != set(self.joint_names) - {self.gripper_joint}:
            raise ValueError('manual joints must contain every joint except gripper')
        if (not self.home_joint_names or
                len(self.home_joint_names) != len(self.home_positions) or
                len(set(self.home_joint_names)) != len(self.home_joint_names)):
            raise ValueError('home joint names/positions must be non-empty, unique, and equal length')
        if not set(self.home_joint_names).issubset(self.manual_joint_names):
            raise ValueError('home joints must be arm joints')
        if (not self.hold_joint_names or
                len(self.hold_joint_names) != len(self.hold_positions) or
                len(set(self.hold_joint_names)) != len(self.hold_joint_names)):
            raise ValueError(
                'hold joint names/positions must be non-empty, unique, and equal length')
        if set(self.hold_joint_names) != set(self.manual_joint_names):
            raise ValueError(
                'hold pose must contain every arm joint and exclude the gripper')
        if 'shoulder_joint' not in self.home_joint_names:
            raise ValueError('home joints must include shoulder_joint')
        if self.home_tolerance <= 0.0 or not math.isfinite(self.home_tolerance):
            raise ValueError('home_tolerance must be finite and positive')
        if (self.home_base_tolerance <= 0.0 or
                not math.isfinite(self.home_base_tolerance)):
            raise ValueError(
                'home_base_tolerance must be finite and positive')
        if not all(math.isfinite(value) for value in self.home_positions):
            raise ValueError('home positions must be finite')
        if not all(math.isfinite(value) for value in self.hold_positions):
            raise ValueError('hold positions must be finite')
        home_scalars = (
            self.home_base_position,
            self.home_shoulder_clearance_position,
            self.home_wrist_stage1_position,
            self.home_branch_deadband,
            self.home_move_threshold,
            self.home_dwell_time,
            self.home_phase_timeout,
        )
        if not all(math.isfinite(value) for value in home_scalars):
            raise ValueError('MANUAL_EE home sequence parameters must be finite')
        if (self.home_branch_deadband < 0.0 or
                self.home_move_threshold < 0.0 or
                self.home_dwell_time < 0.0 or
                self.home_phase_timeout <= 0.0):
            raise ValueError('MANUAL_EE home sequence timing/thresholds are invalid')
        for i, name in enumerate(self.joint_names):
            if not all(math.isfinite(v) for v in (
                    self.lower[i], self.upper[i], self.velocity_limits[i],
                    self.following_error_limits[i])):
                raise ValueError(f'{name} limits must be finite')
            if (self.lower[i] >= self.upper[i] or
                    self.velocity_limits[i] <= 0.0 or
                    self.following_error_limits[i] <= 0.0):
                raise ValueError(f'{name} limits are invalid')
        if (not math.isfinite(self.following_error_timeout) or
                self.following_error_timeout <= 0.0):
            raise ValueError(
                'following_error_timeout must be finite and positive')

    def on_joint_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        for name, position in zip(msg.name, msg.position):
            if math.isfinite(position):
                self.positions[name] = float(position)
        if self.output is None and self.have_all_positions():
            self.output = self.measured_positions()
        if (self.mode == 'OFF' and self.off_hold_target is None and
                self.have_all_positions()):
            self.capture_off_hold_target()
            self.get_logger().info(
                'OFF hold captured from initial joint feedback.')
        if self.have_all_positions() and not self.feedback_ready_logged:
            self.feedback_ready_logged = True
            self.get_logger().info(
                'Initial joint feedback ready for all joints: ' +
                ', '.join(self.joint_names))

    def on_manual_command(self, msg):
        self.accept_arm_command(msg, 'manual')

    def on_ee_command(self, msg):
        self.accept_arm_command(msg, 'ee')

    def accept_arm_command(self, msg, source):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        values = dict(zip(msg.name, msg.position))
        if set(values) != set(self.manual_joint_names):
            return
        parsed = {name: float(values[name]) for name in self.manual_joint_names}
        if all(math.isfinite(value) for value in parsed.values()):
            setattr(self, source + '_command', parsed)
            setattr(self, source + '_stamp', self.get_clock().now())

    def on_gripper_command(self, msg):
        if math.isfinite(msg.data):
            self.gripper_command = float(msg.data)
            self.gripper_stamp = self.get_clock().now()

    def on_joy(self, msg):
        home_pressed = (
            0 <= self.home_button < len(msg.buttons) and
            msg.buttons[self.home_button] != 0)
        home_was_pressed = (
            0 <= self.home_button < len(self.last_joy_buttons) and
            self.last_joy_buttons[self.home_button] != 0)
        hold_pressed = (
            0 <= self.hold_button < len(msg.buttons) and
            msg.buttons[self.hold_button] != 0)
        hold_was_pressed = (
            0 <= self.hold_button < len(self.last_joy_buttons) and
            self.last_joy_buttons[self.hold_button] != 0)
        can_start_pose = (
            self.mode == 'MANUAL_EE' and self.home_phase is None and
            not self.protective_stop and self.hardware_fault is None)
        if (home_pressed and not home_was_pressed and can_start_pose):
            self.publish_manual_ee_ready(False)
            self.ee_command = None
            self.ee_stamp = None
            self.gripper_command = None
            self.gripper_stamp = None
            self.start_home_sequence(self.get_clock().now())
        elif (hold_pressed and not hold_was_pressed and can_start_pose):
            self.publish_manual_ee_ready(False)
            self.ee_command = None
            self.ee_stamp = None
            self.start_hold_sequence(self.get_clock().now())
        self.last_joy_buttons = list(msg.buttons)

    def on_mode(self, msg):
        requested = msg.data.strip().upper()
        if requested not in ('OFF', 'MANUAL_100', 'MANUAL_EE',
                             'MANUAL_EE_PAUSED', 'AUTO',
                             'ESTOP_LATCHED') or requested == self.mode:
            return
        self.mode = requested
        self.publish_manual_ee_ready(False)
        self.manual_command = None
        self.manual_stamp = None
        self.ee_command = None
        self.ee_stamp = None
        self.following_error_started.clear()
        self.clear_home_sequence()
        if requested == 'OFF':
            self.manual_ee_pause_hold_target = None
            self.off_hold_target = None
            if self.have_all_positions():
                self.capture_off_hold_target()
                self.get_logger().info(
                    'OFF hold captured from current joint feedback.')
        elif requested == 'MANUAL_EE_PAUSED':
            self.off_hold_target = None
            if self.have_all_positions():
                self.manual_ee_pause_hold_target = self.clamp(
                    self.measured_positions())
                self.output = list(self.manual_ee_pause_hold_target)
                self.get_logger().info(
                    'MANUAL_EE paused: holding current measured pose.')
        else:
            self.off_hold_target = None
            self.manual_ee_pause_hold_target = None
            if self.have_all_positions():
                self.output = self.measured_positions()
        self.home_phase = 'shoulder' if requested in self.HOME_ENTRY_MODES else None
        if self.home_phase is not None:
            self.get_logger().info(
                f'{requested} entry: moving to the manual camera-clearance pose.')
        elif requested == 'MANUAL_EE':
            # Enter differential-IK control from exactly the measured pose;
            # no automatic motion is allowed merely by selecting the mode.
            self.publish_manual_ee_ready(True)
            self.get_logger().info(
                'MANUAL_EE entry: holding measured pose; press Circle to '
                'move to home_pose.')

    def on_protective_stop(self, msg):
        if msg.data:
            self.publish_manual_ee_ready(False)
            self.protective_stop = True
            self.clear_home_sequence()
            self.manual_command = None
            self.ee_command = None
            self.gripper_command = None
            self.latch_hold_target('protective stop')

    def on_hardware_fault(self, msg):
        reason = msg.data.strip() or 'unspecified hardware fault'
        self.publish_manual_ee_ready(False)
        if self.hardware_fault is None:
            self.hardware_fault = reason
            self.get_logger().error(
                f'HARDWARE FAULT LATCHED; commanding five-joint hold: {reason}')
        self.protective_stop = True
        self.clear_home_sequence()
        self.manual_command = None
        self.manual_stamp = None
        self.ee_command = None
        self.ee_stamp = None
        self.gripper_command = None
        self.gripper_stamp = None
        self.latch_hold_target('hardware fault')

    def latch_hold_target(self, source):
        if self.hold_target is not None or not self.have_all_positions():
            return
        self.hold_target = self.clamp(self.measured_positions())
        self.output = list(self.hold_target)
        self.get_logger().error(
            f'Five-joint hold latched from measured positions ({source}).')

    def on_timer(self):
        now = self.get_clock().now()
        if not self.have_all_positions():
            self.report_missing_initial_feedback(now)
            return
        if self.mode == 'AUTO' or self.output is None:
            return
        if self.protective_stop or self.hardware_fault is not None:
            self.latch_hold_target('protective stop')
            if self.hold_target is None:
                return
            self.output = list(self.hold_target)
            self.publisher.publish(Float64MultiArray(data=self.output))
            return
        if self.mode == 'OFF':
            if self.off_hold_target is None:
                self.capture_off_hold_target()
            if self.off_hold_target is None:
                return
            # OFF is a fixed snapshot hold.  Never chase live feedback here:
            # doing so ratchets a gravity-loaded joint downward as it sags.
            self.output = list(self.off_hold_target)
            self.publisher.publish(Float64MultiArray(data=self.output))
            return
        if self.mode == 'MANUAL_EE_PAUSED':
            if self.manual_ee_pause_hold_target is None:
                self.manual_ee_pause_hold_target = self.clamp(
                    self.measured_positions())
            self.output = list(self.manual_ee_pause_hold_target)
            self.publisher.publish(Float64MultiArray(data=self.output))
            return
        dt = min(max((now - self.last_timer_time).nanoseconds * 1e-9, 0.0), 0.1)
        self.last_timer_time = now
        desired = self.measured_positions()
        if (self.mode in ('MANUAL_100', 'MANUAL_EE') and
                not self.protective_stop and not self.tracking_fault):
            if self.home_phase is not None:
                desired = self.home_target(desired)
            else:
                is_manual_100 = self.mode == 'MANUAL_100'
                command = self.manual_command if is_manual_100 else self.ee_command
                stamp = self.manual_stamp if is_manual_100 else self.ee_stamp
                if self.is_fresh(stamp, self.manual_timeout, now):
                    for name, value in command.items():
                        desired[self.joint_names.index(name)] = value
                if self.is_fresh(self.gripper_stamp, self.gripper_timeout, now):
                    desired[self.joint_names.index(self.gripper_joint)] = self.gripper_command
        candidate = self.rate_limit(self.clamp(desired), dt)
        if self.update_following_error(candidate, now):
            # Hold the encoder positions immediately. The fault remains
            # latched until bringup is restarted and the mechanism inspected.
            self.output = self.clamp(self.measured_positions())
            self.publisher.publish(Float64MultiArray(data=self.output))
            return
        self.output = candidate
        self.publisher.publish(Float64MultiArray(data=self.output))
        if self.home_phase is not None:
            self.update_home_sequence(now)

    def report_missing_initial_feedback(self, now):
        wait_seconds = (
            now - self.feedback_wait_started).nanoseconds * 1e-9
        if wait_seconds < self.feedback_wait_warn_after:
            return
        if self.last_feedback_wait_log is not None:
            since_log = (
                now - self.last_feedback_wait_log).nanoseconds * 1e-9
            if since_log < self.feedback_wait_log_interval:
                return
        missing = [name for name in self.joint_names
                   if name not in self.positions]
        self.last_feedback_wait_log = now
        self.get_logger().warn(
            'Position commands blocked: waiting for initial /joint_states '
            f'feedback from [{", ".join(missing)}].')

    def update_following_error(self, command, now):
        if self.tracking_fault:
            return True
        if self.mode not in ('MANUAL_100', 'MANUAL_EE') or self.protective_stop:
            self.following_error_started.clear()
            return False

        measured = self.measured_positions()
        expired = []
        for i, name in enumerate(self.joint_names):
            # A gripper can legitimately retain position error while it is
            # holding an object or resting against its mechanical end stop.
            # Its command is already clamped to the commissioned range, so do
            # not escalate gripper-only following error into a whole-arm latch.
            if name == self.gripper_joint:
                self.following_error_started.pop(name, None)
                continue
            error = abs(command[i] - measured[i])
            if error <= self.following_error_limits[i]:
                self.following_error_started.pop(name, None)
                continue
            started = self.following_error_started.setdefault(name, now)
            duration = (now - started).nanoseconds * 1e-9
            if duration >= self.following_error_timeout:
                expired.append((name, error, self.following_error_limits[i]))

        if not expired:
            return False
        self.tracking_fault = True
        self.publish_manual_ee_ready(False)
        self.clear_home_sequence()
        self.manual_command = None
        self.manual_stamp = None
        self.ee_command = None
        self.ee_stamp = None
        self.gripper_command = None
        self.gripper_stamp = None
        details = ', '.join(
            f'{name}: error={error:.3f} rad > limit={limit:.3f} rad'
            for name, error, limit in expired)
        self.latch_hold_target('following error')
        self.get_logger().error(
            'FOLLOWING ERROR LATCHED; holding measured joint positions. ' +
            details)
        return True

    @staticmethod
    def is_fresh(stamp, timeout, now):
        return stamp is not None and (timeout < 0.0 or (now - stamp).nanoseconds * 1e-9 <= timeout)

    def rate_limit(self, desired, dt):
        result = []
        for i, value in enumerate(desired):
            step = self.velocity_limits[i] * dt
            result.append(self.output[i] + min(max(value - self.output[i], -step), step))
        return result

    def measured_positions(self):
        return [self.positions[name] for name in self.joint_names]

    def capture_off_hold_target(self):
        if not self.have_all_positions():
            return
        # Encoder feedback can start slightly outside a newly commissioned
        # software limit.  Publishing that raw snapshot forever makes the
        # hardware interface clamp the same command on every control cycle.
        self.off_hold_target = self.clamp(self.measured_positions())
        self.output = list(self.off_hold_target)

    def clear_home_sequence(self):
        self.home_phase = None
        self.sequence_kind = None
        self.sequence_gripper_hold = None
        self.home_hold_target = None
        self.home_phase_target = {}
        self.home_pending_phases = []
        self.home_phase_started = None
        self.home_in_tolerance_since = None

    def start_home_sequence(self, now):
        if not self.have_all_positions():
            self.get_logger().error(
                'Cannot start MANUAL_EE home: joint feedback is incomplete.')
            return
        self.sequence_kind = 'home'
        shoulder = self.positions['shoulder_joint']
        if shoulder > self.home_branch_deadband:
            branch = 'shoulder-positive'
            phases = [
                ('shoulder_clearance', {
                    'shoulder_joint': self.home_shoulder_clearance_position}),
            ]
            # Never pull the wrist backward to the stage-1 angle. If it is
            # already at or beyond +75 degrees, hold that measured position
            # after the shoulder reaches its collision-clearance angle.
            if self.positions['wrist_joint'] < self.home_wrist_stage1_position:
                phases.append(('wrist_stage1', {
                    'wrist_joint': self.home_wrist_stage1_position}))
            phases.extend([
                ('elbow_home', {
                    'elbow_joint': self.home_values['elbow_joint']}),
                ('base_home', {'base_joint': self.home_base_position}),
                ('shoulder_home', {
                    'shoulder_joint': self.home_values['shoulder_joint']}),
                ('wrist_home', {
                    'wrist_joint': self.home_values['wrist_joint']}),
            ])
        else:
            branch = 'shoulder-nonpositive'
            phases = [
                ('wrist_home', {
                    'wrist_joint': self.home_values['wrist_joint']}),
                ('elbow_home', {
                    'elbow_joint': self.home_values['elbow_joint']}),
                ('shoulder_home', {
                    'shoulder_joint': self.home_values['shoulder_joint']}),
                ('base_home', {'base_joint': self.home_base_position}),
            ]
        self.home_pending_phases = phases
        self.get_logger().info(
            f'MANUAL_EE home requested: branch={branch}, '
            f'shoulder={shoulder:.5f} rad.')
        self.start_next_home_phase(now)

    def start_hold_sequence(self, now):
        if not self.have_all_positions():
            self.get_logger().error(
                'Cannot start MANUAL_EE hold pose: joint feedback is incomplete.')
            return
        self.sequence_kind = 'hold'
        gripper_index = self.joint_names.index(self.gripper_joint)
        if self.gripper_command is not None:
            self.sequence_gripper_hold = self.gripper_command
        elif self.output is not None:
            self.sequence_gripper_hold = self.output[gripper_index]
        else:
            self.sequence_gripper_hold = self.positions[self.gripper_joint]
        self.gripper_command = None
        self.gripper_stamp = None
        self.home_pending_phases = [('hold_pose', dict(self.hold_values))]
        self.get_logger().info(
            'MANUAL_EE hold pose requested; preserving current gripper position.')
        self.start_next_home_phase(now)

    def start_next_home_phase(self, now):
        sequence_kind = self.sequence_kind or 'home'
        while self.home_pending_phases:
            phase, targets = self.home_pending_phases.pop(0)
            moving_targets = {
                name: target for name, target in targets.items()
                if abs(self.positions[name] - target) > self.home_move_threshold
            }
            if not moving_targets:
                self.get_logger().info(
                    f'MANUAL_EE {sequence_kind} phase {phase} skipped: already within '
                    f'{self.home_move_threshold:.3f} rad.')
                continue
            # Every joint not named in targets holds this fixed stage-start
            # snapshot. Never chase live feedback while another joint moves.
            self.home_hold_target = self.clamp(self.measured_positions())
            if (sequence_kind == 'hold' and
                    self.sequence_gripper_hold is not None):
                gripper_index = self.joint_names.index(self.gripper_joint)
                self.home_hold_target[gripper_index] = min(max(
                    self.sequence_gripper_hold,
                    self.lower[gripper_index]), self.upper[gripper_index])
            self.output = list(self.home_hold_target)
            self.home_phase = phase
            self.home_phase_target = moving_targets
            self.home_phase_started = now
            self.home_in_tolerance_since = None
            details = ', '.join(
                f'{name}={target:.5f}' for name, target in targets.items())
            self.get_logger().info(
                f'MANUAL_EE {sequence_kind} phase {phase} started: {details}.')
            return
        self.finish_home_sequence()

    def finish_home_sequence(self):
        sequence_kind = self.sequence_kind or 'home'
        self.clear_home_sequence()
        # Discard commands received while the recovery sequence owned output.
        self.manual_command = None
        self.manual_stamp = None
        self.ee_command = None
        self.ee_stamp = None
        self.gripper_command = None
        self.gripper_stamp = None
        if self.mode == 'MANUAL_EE':
            self.publish_manual_ee_ready(True)
        self.get_logger().info(
            f'MANUAL_EE {sequence_kind}_pose reached; operator commands enabled.')

    def abort_home_sequence(self, reason):
        sequence_kind = self.sequence_kind or 'home'
        if self.have_all_positions():
            self.output = self.clamp(self.measured_positions())
        self.clear_home_sequence()
        self.publish_manual_ee_ready(False)
        self.get_logger().error(
            f'MANUAL_EE {sequence_kind} sequence aborted; '
            f'holding measured pose: {reason}')

    def update_home_sequence(self, now):
        if self.home_phase_started is None:
            return
        elapsed = (now - self.home_phase_started).nanoseconds * 1e-9
        if elapsed > self.home_phase_timeout:
            self.abort_home_sequence(
                f'phase {self.home_phase} exceeded '
                f'{self.home_phase_timeout:.1f}s timeout')
            return
        within = all(
            abs(self.positions[name] - target) <= (
                self.home_base_tolerance
                if name == 'base_joint' else self.home_tolerance)
            for name, target in self.home_phase_target.items())
        if not within:
            self.home_in_tolerance_since = None
            return
        if self.home_in_tolerance_since is None:
            self.home_in_tolerance_since = now
            return
        dwell = (now - self.home_in_tolerance_since).nanoseconds * 1e-9
        if dwell < self.home_dwell_time:
            return
        completed = self.home_phase
        sequence_kind = self.sequence_kind or 'home'
        self.get_logger().info(
            f'MANUAL_EE {sequence_kind} phase {completed} completed.')
        self.start_next_home_phase(now)

    def home_target(self, measured):
        target = list(
            self.home_hold_target
            if self.home_hold_target is not None else measured)
        for name, position in self.home_phase_target.items():
            target[self.joint_names.index(name)] = position
        return target

    def have_all_positions(self):
        return all(name in self.positions for name in self.joint_names)

    def clamp(self, values):
        return [min(max(v, self.lower[i]), self.upper[i]) for i, v in enumerate(values)]


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
