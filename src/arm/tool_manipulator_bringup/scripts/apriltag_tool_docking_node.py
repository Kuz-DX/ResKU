#!/usr/bin/env python3
"""Two-stage AprilTag tool docking controller for ROS 2 Humble.

The visual-servo frame must be the same frame used by the AprilTag detector's
poses.  In the supplied configuration this is the RealSense optical frame.
"""
import json
import math
import threading
import time
from enum import Enum, auto

import rclpy
import yaml
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from arm_pose_safety import joints_stopped
from tool_ids import NO_TOOL_ID


class DockState(Enum):
    IDLE = auto()
    COARSE_MOVING = auto()
    FINE_XY_ALIGNING = auto()
    DESCENDING = auto()
    LOCK_ROTATING = auto()
    RETREAT = auto()
    ATTACHING_SCENE = auto()
    DETACHING_SCENE = auto()
    DETACH_RETREAT = auto()
    RETREATING = auto()
    RETURNING_TO_WAIT = auto()
    MISSION_WAIT = auto()
    POST_LOCK_HOLD = auto()
    DOCKED = auto()
    CANCELLED_HOLD = auto()


class AprilTagToolDocking(Node):
    """Plan once in joint space, then close the last few cm with MoveIt Servo."""
    NONE = NO_TOOL_ID

    def __init__(self):
        super().__init__('apriltag_tool_docking')
        self.cb_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self.goals = self._read_goals()
        self._validate_motion_params()
        self.state = DockState.IDLE
        self.target_tag_id = None
        self.operation = None
        self.phase_started = self._now_sec()
        self.last_detection_time = None
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_state_sequence = 0
        self.joint_state_received_at = 0.0
        self.joint_state_event = threading.Event()
        self.motion_lock = threading.Lock()
        self.trajectory_active = False
        self.trajectory_finished = threading.Event()
        self.trajectory_finished.set()
        # Latched feedback from tool_scene_manager.  No retreat with a
        # newly attached physical tool is allowed until its collision object is live.
        self.active_tool_id = self.NONE

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=list(self.get_parameter('joint_names').value),
            base_link_name=self.get_parameter('base_link').value,
            end_effector_name=self.get_parameter('end_effector_link').value,
            group_name=self.get_parameter('planning_group').value,
            callback_group=self.cb_group,
        )
        self.servo_client = self.create_client(SetBool, self.get_parameter('servo_enable_service').value)
        self.servo_aligned = False
        self.twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter('servo_twist_topic').value, 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, status_qos)
        # These are one-shot physical-motion events, never latched state.
        self.complete_pub = self.create_publisher(Bool, '/docking_complete', 10)
        self.undocking_complete_pub = self.create_publisher(Bool, '/undocking_complete', 10)
        self.yaw_complete_pub = self.create_publisher(Bool, '/wrist_yaw_rotation_complete', 10)
        detection_topic = self.get_parameter('detection_topic').value
        if self.get_parameter('detection_message_type').value == 'vision_json':
            self.create_subscription(String, detection_topic, self._json_detections_cb, 10, callback_group=self.cb_group)
        else:
            self.create_subscription(AprilTagDetectionArray, detection_topic, self._detections_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            JointState, self.get_parameter('joint_state_topic').value, self._joint_state_cb, 20)
        self.create_subscription(
            Int32, self.get_parameter('active_tool_id_topic').value, self._active_tool_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('cancel_topic').value, self._cancel_cb, 10)
        self.create_subscription(
            String, self.get_parameter('hardware_fault_topic').value, self._hardware_fault_cb, 10)
        self.create_subscription(Bool, self.get_parameter('servo_aligned_topic').value, self._servo_aligned_cb, 10)
        self.create_subscription(
            String, self.get_parameter('motion_command_topic').value,
            self._motion_command_cb, 10, callback_group=self.cb_group)
        rate = float(self.get_parameter('servo_rate_hz').value)
        self.create_timer(1.0 / rate, self._control_tick, callback_group=self.cb_group)
        self.create_service(Trigger, '~/operator_reset', self._operator_reset)
        self._publish_status('idle')

    def _declare_parameters(self):
        p = self.declare_parameter
        p('tool_id_topic', '/selected_tool_id')
        p('tool_id_message_type', 'int32')
        p('motion_command_topic', '/docking_command')
        p('active_tool_id_topic', '/active_tool_id')
        p('cancel_topic', '/tool_change/cancel')
        p('hardware_fault_topic', '/control/hardware_fault')
        p('status_topic', '/docking_motion_status')
        p('servo_enable_service', '/visual_servo_node/enable')
        p('servo_aligned_topic', '/servo_aligned')
        p('detection_topic', '/tag_pose_valid')
        p('detection_message_type', 'vision_json')
        # Dotted fields accommodate apriltag_msgs variants/bridge message layouts.
        p('tag_id_field', 'id')
        p('tag_pose_field', 'pose.pose.pose')
        p('servo_twist_topic', '/servo_node/delta_twist_cmds')
        p('servo_command_frame', '')
        p('planning_group', 'arm')
        p('base_link', 'base_actuator')
        p('end_effector_link', 'tcp_link')
        p('joint_state_topic', '/joint_states')
        p('joint_names', ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint'])
        # Preferred registry.  Each ID owns its taught coarse joint goal and
        # later also owns TCP, collision and actuator metadata.
        p('tools_config_file', '')
        # ROS parameters cannot reliably carry a heterogeneous YAML map, so use a
        # YAML string: '{10: [q1, q2, q3, q4], 11: [...]}' in docking.yaml.
        p('tool_joint_goals_yaml', '{}')
        # Named common posture executed before every tag-specific approach.
        # Direct invocation without the fail-closed real launch must also refuse
        # motion: no taught posture or timeout has a production default.
        p('docking_wait_joint_goal', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        p('servo_rate_hz', -1.0)
        p('coarse_timeout_sec', -1.0)
        p('xy_align_timeout_sec', -1.0)
        p('detection_timeout_sec', -1.0)
        p('lock_timeout_sec', -1.0)
        # A successful action result alone is not enough to claim a mechanical
        # lock: require fresh encoder-backed joint feedback in the commanded
        # yaw direction before publishing the lock-complete event.
        p('lock_feedback_min_delta_rad', -1.0)
        p('lock_feedback_timeout_sec', -1.0)
        p('cancel_stop_timeout_sec', -1.0)
        p('stopped_velocity_rad_s', -1.0)
        p('attachment_timeout_sec', -1.0)
        p('post_lock_motion_timeout_sec', -1.0)
        # A planned joint trajectory gives a completion-confirmed yaw rotation.
        p('lock_joint_name', 'wrist_yaw_joint')
        # The final mission posture is supplied only by docking.yaml.
        p('mission_wait_joint_goal_yaml', '[]')

    def _validate_motion_params(self):
        positive = ('servo_rate_hz', 'coarse_timeout_sec', 'xy_align_timeout_sec',
                    'detection_timeout_sec', 'lock_timeout_sec',
                    'lock_feedback_min_delta_rad', 'lock_feedback_timeout_sec',
                    'cancel_stop_timeout_sec', 'stopped_velocity_rad_s',
                    'attachment_timeout_sec', 'post_lock_motion_timeout_sec')
        if any(not isinstance(self.get_parameter(name).value, (int, float)) or
               self.get_parameter(name).value <= 0.0 for name in positive):
            raise RuntimeError('real docking timing parameters must be positive; run tool_change.launch.py preflight')
        if self.get_parameter('lock_joint_name').value not in self.get_parameter('joint_names').value:
            raise RuntimeError('lock_joint_name must be one of joint_names')
        joint_count = len(self.get_parameter('joint_names').value)
        goal = self.get_parameter('docking_wait_joint_goal').value
        if len(goal) != joint_count or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in goal):
            raise RuntimeError('docking_wait_joint_goal must contain one measured radian per joint_names entry')
        mission_goal = self._mission_wait_goal()
        if len(mission_goal) != joint_count:
            raise RuntimeError('mission_wait_joint_goal must contain one measured radian per joint_names entry')

    def _mission_wait_goal(self):
        """Return an optional taught mission pose from a ROS-safe YAML string."""
        try:
            values = yaml.safe_load(self.get_parameter('mission_wait_joint_goal_yaml').value)
            return [] if values is None else [float(value) for value in values]
        except (TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError('mission_wait_joint_goal_yaml must be a YAML list') from exc

    def _read_goals(self):
        tools_file = self.get_parameter('tools_config_file').value
        if tools_file:
            try:
                with open(tools_file, encoding='utf-8') as stream:
                    registry = yaml.safe_load(stream) or {}
                result = registry.get('tools', {})
                self.tool_specs = {int(tool_id): spec for tool_id, spec in result.items() if isinstance(spec, dict)}
                # An unfinished tool entry must never become a plausible
                # zero-joint command.  It stays visible in tools.yaml but is
                # intentionally unavailable until both enabled and taught.
                goals = {
                    int(tool_id): [float(q) for q in spec['coarse_joint_goal']]
                    for tool_id, spec in result.items()
                    if spec.get('enabled', True)
                }
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
                raise RuntimeError('tools_config_file must contain tools.<id>.coarse_joint_goal') from exc
            expected = len(self.get_parameter('joint_names').value)
            if any(len(goal) != expected for goal in goals.values()):
                raise RuntimeError('every tools.<id>.coarse_joint_goal must match joint_names')
            return goals
        self.tool_specs = {}
        try:
            result = yaml.safe_load(self.get_parameter('tool_joint_goals_yaml').value) or {}
            goals = {int(tag): [float(q) for q in values] for tag, values in result.items()}
        except (yaml.YAMLError, TypeError, ValueError) as exc:
            raise RuntimeError('tool_joint_goals_yaml must be a YAML map of tag id to joint list') from exc
        expected = len(self.get_parameter('joint_names').value)
        if any(len(goal) != expected for goal in goals.values()):
            raise RuntimeError('every tool joint goal must have one value per joint_names entry')
        return goals

    def _motion_command_cb(self, msg):
        try:
            operation, raw_tool_id = msg.data.split(':', 1)
            tool_id = int(raw_tool_id)
        except (AttributeError, TypeError, ValueError):
            self.get_logger().warning('Ignoring malformed docking command: %r' % msg.data)
            return
        if operation not in ('attach', 'detach'):
            self.get_logger().warning('Ignoring unsupported docking operation: %r' % operation)
            return
        self._start_request(tool_id, operation)

    def _start_request(self, tag_id, operation):
        if tag_id == self.NONE:
            if self.state == DockState.IDLE:
                self._publish_status('idle:no_tool_selected')
            return

        allowed_states = (DockState.IDLE,) if operation == 'attach' else (DockState.IDLE, DockState.DOCKED)
        if self.state not in allowed_states:
            self.get_logger().warning('Docking request ignored: controller is busy, docked, or awaiting operator recovery')
            return
        if operation == 'attach' and self.active_tool_id != self.NONE:
            self._publish_status(f'failed:active_tool_{self.active_tool_id}_requires_release')
            return
        if operation == 'detach' and self.active_tool_id != tag_id:
            self._publish_status(f'failed:active_tool_{self.active_tool_id}_does_not_match_{tag_id}')
            return
        if tag_id not in self.goals:
            self.get_logger().error(f'No taught joint goal for requested tag {tag_id}')
            self._publish_status('failed:no_taught_goal')
            return
        self._stop_servo()
        self.target_tag_id = tag_id
        self.operation = operation
        self._enter(DockState.COARSE_MOVING)
        # Planning/execution waits must not block perception or Servo publishing.
        threading.Thread(target=self._coarse_worker, args=(tag_id,), daemon=True).start()

    def _coarse_worker(self, tag_id):
        try:
            # docking_wait is the shared rack-facing joint posture. It is kept
            # separate from the tag goal so every UI request begins identically.
            self._publish_status('coarse_moving:docking_wait')
            wait_result = self._execute_joint_goal(
                list(self.get_parameter('docking_wait_joint_goal').value))
            if wait_result is False:
                raise RuntimeError('docking_wait action returned failure')
            if self.state != DockState.COARSE_MOVING or self.target_tag_id != tag_id:
                return
            self._publish_status(f'coarse_moving:tool_{tag_id}')
            # pymoveit2 waits for the FollowJointTrajectory action result here;
            # transition only after it reports completion (or raises on failure).
            result = self._execute_joint_goal(self.goals[tag_id])
            success = result is not False
        except Exception as exc:  # Planning/action failures are returned to a safe state.
            self.get_logger().error(f'Coarse move or docking_wait failed: {exc}')
            success = False
        if self.state != DockState.COARSE_MOVING or self.target_tag_id != tag_id:
            return
        if success:
            self.last_detection_time = None
            self._enter(DockState.FINE_XY_ALIGNING)
        else:
            self._fail('docking_wait_or_coarse_execution')

    def _json_detections_cb(self, msg):
        """Use the existing vision.apriltag PnP JSON without inventing a pose type."""
        if self.state != DockState.FINE_XY_ALIGNING or self.target_tag_id is None:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('Ignoring malformed AprilTag JSON')
            return
        frame_id = payload.get('frame_id', '')
        match = next((item for item in payload.get('detections', [])
                      if int(item.get('id', -1)) == self.target_tag_id), None)
        position_cm = match.get('position_camera_cm') if match else None
        if not frame_id or not position_cm:
            return
        try:
            position = [float(position_cm[axis]) / 100.0 for axis in ('x', 'y', 'z')]
        except (KeyError, TypeError, ValueError):
            return
        self.detection_frame = frame_id
        self._accept_detection_position(position)
    def _detections_cb(self, msg):
        if self.state != DockState.FINE_XY_ALIGNING or self.target_tag_id is None:
            return
        match = next((d for d in msg.detections if self._tag_id(d) == self.target_tag_id), None)
        if match is None:  # Other tags are intentionally irrelevant to this docking run.
            return
        pose = self._pose_from_detection(match)
        if pose is None:
            self.get_logger().warning('Target tag detection has no usable pose')
            return
        position, _quaternion = pose
        self._accept_detection_position(position)

    def _accept_detection_position(self, position):
        # Alignment is computed exclusively by visual_servo_node from its
        # measured desired_tag_position_m.  Keep this executor's local record
        # only for loss/timeout supervision; do not duplicate an XY target.
        self.last_detection_time = self._now_sec()

    def _tag_id(self, detection):
        value = self._field(detection, self.get_parameter('tag_id_field').value)
        if isinstance(value, (list, tuple)):
            return int(value[0]) if value else None
        return int(value) if value is not None else None

    def _pose_from_detection(self, detection):
        obj = self._field(detection, self.get_parameter('tag_pose_field').value)
        if obj is None or not hasattr(obj, 'position') or not hasattr(obj, 'orientation'):
            return None
        p, q = obj.position, obj.orientation
        return ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    @staticmethod
    def _field(message, dotted_path):
        """Resolve an explicitly configured ROS message field path safely."""
        value = message
        for name in dotted_path.split('.'):
            value = getattr(value, name, None)
            if value is None:
                return None
        return value

    def _joint_state_cb(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))
        self.joint_velocities.update(zip(msg.name, msg.velocity))
        self.joint_state_sequence += 1
        self.joint_state_received_at = time.monotonic()
        self.joint_state_event.set()

    def _active_tool_cb(self, msg):
        self.active_tool_id = int(msg.data)

    def _cancel_cb(self, msg):
        if msg.data and self.state not in (DockState.IDLE, DockState.DOCKED, DockState.CANCELLED_HOLD):
            self._fail('cancel')

    def _hardware_fault_cb(self, msg: String):
        # A communication, stale-state, or limit fault is hold-only: do not
        # attempt retreat and never emit either completion event.
        if msg.data and self.state not in (DockState.IDLE, DockState.DOCKED, DockState.CANCELLED_HOLD):
            self._fail('hardware_fault')

    def _servo_aligned_cb(self, msg):
        self.servo_aligned = bool(msg.data)
        if self.servo_aligned and self.state == DockState.FINE_XY_ALIGNING:
            self._enter(DockState.DESCENDING)

    def _set_servo_enabled(self, enabled):
        self.servo_aligned = False
        if not self.servo_client.service_is_ready():
            self.get_logger().warning("visual_servo enable service unavailable; docking remains held")
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self.servo_client.call_async(request)

    def _control_tick(self):
        now = self._now_sec()
        if self.state == DockState.COARSE_MOVING and now - self.phase_started > self.get_parameter('coarse_timeout_sec').value:
            self._fail('coarse_timeout')
            return
        if self.state == DockState.DESCENDING:
            self._descent_tick(now)
            return
        if self.state == DockState.LOCK_ROTATING:
            if now - self.phase_started > self.get_parameter('lock_timeout_sec').value:
                self._fail('lock_rotation_timeout')
            return
        if self.state == DockState.RETREAT:
            self._retreat_tick(now)
            return
        if self.state == DockState.DETACHING_SCENE:
            if self.active_tool_id == self.NONE:
                self._enter(DockState.DETACH_RETREAT)
            elif now - self.phase_started > self.get_parameter('attachment_timeout_sec').value:
                self._fail('planning_scene_detach_timeout')
            return
        if self.state == DockState.DETACH_RETREAT:
            self._retreat_tick(now)
            return
        if self.state == DockState.ATTACHING_SCENE:
            if self.active_tool_id == self.target_tag_id:
                self._enter(DockState.RETREATING)
                threading.Thread(target=self._post_lock_worker, args=(self.target_tag_id,), daemon=True).start()
            elif now - self.phase_started > self.get_parameter('attachment_timeout_sec').value:
                self._fail('planning_scene_attach_timeout')
            return
        if self.state in (DockState.RETREATING, DockState.RETURNING_TO_WAIT, DockState.MISSION_WAIT):
            if now - self.phase_started > self.get_parameter('post_lock_motion_timeout_sec').value:
                self._fail('post_lock_motion_timeout')
            return
        if self.state != DockState.FINE_XY_ALIGNING:
            return
        if self.last_detection_time is None or now - self.last_detection_time > self.get_parameter('detection_timeout_sec').value:
            self._fail('tag_lost')
            return
        if now - self.phase_started > self.get_parameter('xy_align_timeout_sec').value:
            self._fail('xy_alignment_timeout')
            return
        if not self.servo_client.service_is_ready():
            self._fail('visual_servo_service_lost')

    def _tool_motion_value(self, name):
        docking = self.tool_specs.get(self.target_tag_id, {}).get('docking', {})
        return docking.get(name) if isinstance(docking, dict) else None

    @staticmethod
    def _valid_motion(distance, speed, timeout, direction):
        return (isinstance(distance, (int, float)) and math.isfinite(distance) and distance >= 0.0 and
                isinstance(speed, (int, float)) and math.isfinite(speed) and speed > 0.0 and
                isinstance(timeout, (int, float)) and math.isfinite(timeout) and timeout > 0.0 and
                isinstance(direction, list) and len(direction) == 3 and
                all(isinstance(value, (int, float)) and math.isfinite(value) for value in direction) and
                AprilTagToolDocking._norm(direction) > 1e-9)

    def _descent_tick(self, now):
        distance = self._tool_motion_value('descent_distance_m')
        speed = self._tool_motion_value('descent_speed_mps')
        timeout = self._tool_motion_value('descent_timeout_sec')
        direction = self._tool_motion_value('descent_direction')
        frame = self._tool_motion_value('descent_frame')
        if not self._valid_motion(distance, speed, timeout, direction) or not isinstance(frame, str) or not frame:
            self._terminal_failure('descent_unconfigured')
            return
        duration = distance / speed
        if now - self.phase_started > timeout:
            self._fail('descent_timeout')
        elif now - self.phase_started >= duration:
            self._stop_servo()
            self._enter(DockState.LOCK_ROTATING)
            threading.Thread(target=self._lock_worker, daemon=True).start()
        else:
            unit = self._unit(direction)
            self._publish_twist([speed * value for value in unit], [0.0, 0.0, 0.0], frame=frame)

    def _lock_worker(self):
        lock_joint = self.get_parameter('lock_joint_name').value
        joints = list(self.get_parameter('joint_names').value)
        if lock_joint not in joints or any(name not in self.joint_positions for name in joints):
            self.get_logger().error('Cannot lock: joint_states lacks the configured arm joints')
            self._fail('lock_joint_state_unavailable')
            return
        lock_spec = self.tool_specs.get(self.target_tag_id, {}).get('lock', {})
        joint_path = lock_spec.get('joint_path') if isinstance(lock_spec, dict) else None
        yaw_delta = lock_spec.get('attach_yaw_delta_rad') if isinstance(lock_spec, dict) else None
        lock_start_yaw = self.joint_positions[lock_joint]
        joint_state_sequence = self.joint_state_sequence
        if joint_path:
            goals = [(str(step.get('name', index)), step.get('positions_rad'))
                     for index, step in enumerate(joint_path)]
            if any(not isinstance(goal, list) or len(goal) != len(joints) or
                   any(not isinstance(value, (int, float)) or not math.isfinite(value)
                       for value in goal) for _, goal in goals):
                self._terminal_failure('lock_joint_path_invalid')
                return
            if self.operation == 'detach':
                goals.reverse()
        elif isinstance(yaw_delta, (int, float)) and math.isfinite(yaw_delta):
            goal = [self.joint_positions[name] for name in joints]
            signed_delta = float(yaw_delta) if self.operation == 'attach' else -float(yaw_delta)
            goal[joints.index(lock_joint)] += signed_delta
            goals = [('yaw_delta', goal)]
        else:
            self._terminal_failure('attach_yaw_delta_unconfigured')
            return
        final_yaw_delta = goals[-1][1][joints.index(lock_joint)] - lock_start_yaw
        minimum_delta = float(self.get_parameter('lock_feedback_min_delta_rad').value)
        if abs(final_yaw_delta) < minimum_delta:
            self._terminal_failure('lock_feedback_delta_exceeds_command')
            return
        try:
            success = True
            for step_name, goal in goals:
                if self.state != DockState.LOCK_ROTATING:
                    return
                prefix = 'lock_rotating' if self.operation == 'attach' else 'unlock_rotating'
                self._publish_status(f'{prefix}:{step_name}')
                if self._execute_joint_goal(goal) is False:
                    success = False
                    break
        except Exception as exc:
            self.get_logger().error(f'Lock rotation failed: {exc}')
            success = False
        if self.state != DockState.LOCK_ROTATING:
            return
        if not success:
            self._fail('lock_rotation')
            return
        if not self._wait_for_lock_encoder_motion(
                lock_joint, lock_start_yaw, final_yaw_delta, joint_state_sequence):
            self._fail('lock_encoder_feedback_not_confirmed')
            return
        # This means only that the configured yaw trajectory succeeded.  It is
        # not a sensor-confirmed mechanical engagement or a Planning Scene update.
        # It does prove that the live joint-state feedback moved through the
        # configured minimum in the commanded direction.
        self._publish_status('wrist_yaw_encoder_motion_confirmed')
        self.yaw_complete_pub.publish(Bool(data=True))
        if self.operation == 'attach':
            self._publish_status('wrist_yaw_rotation_complete')
            self._enter(DockState.RETREAT)
        else:
            # The reverse taught path is the configured physical release
            # procedure. Remove the attached collision before retreating with
            # an empty flange, and wait for the scene acknowledgement.
            self._publish_status('wrist_yaw_unlock_complete:awaiting_scene_detach')
            self._enter(DockState.DETACHING_SCENE)
            self.undocking_complete_pub.publish(Bool(data=True))

    def _wait_for_lock_encoder_motion(self, joint, start, expected_delta, start_sequence):
        """Require one fresh encoder feedback sample with commanded yaw motion."""
        deadline = time.monotonic() + float(self.get_parameter('lock_feedback_timeout_sec').value)
        minimum = float(self.get_parameter('lock_feedback_min_delta_rad').value)
        while time.monotonic() <= deadline:
            measured = self.joint_positions.get(joint)
            fresh = self.joint_state_sequence > start_sequence
            if isinstance(measured, (int, float)) and math.isfinite(measured) and fresh:
                delta = measured - start
                if delta * expected_delta > 0.0 and abs(delta) >= minimum:
                    return True
            if self.state != DockState.LOCK_ROTATING:
                return False
            time.sleep(0.01)
        return False

    def _retreat_tick(self, now):
        distance = self._tool_motion_value('retreat_distance_m')
        speed = self._tool_motion_value('retreat_speed_mps')
        timeout = self._tool_motion_value('retreat_timeout_sec')
        direction = self._tool_motion_value('retreat_direction')
        frame = self._tool_motion_value('retreat_frame')
        if not self._valid_motion(distance, speed, timeout, direction) or not isinstance(frame, str) or not frame:
            self._terminal_failure('retreat_unconfigured')
            return
        duration = distance / speed
        if now - self.phase_started > timeout:
            self._terminal_failure('retreat_timeout')
        elif now - self.phase_started >= duration:
            self._stop_servo()
            if self.state == DockState.DETACH_RETREAT:
                self._enter(DockState.RETREATING)
                self._publish_status('detach_retreat_complete')
                threading.Thread(
                    target=self._post_lock_worker,
                    args=(self.target_tag_id, 'detach'), daemon=True).start()
                return
            # Only this successful post-yaw retreat is allowed to trigger attach.
            self._enter(DockState.ATTACHING_SCENE)
            self._publish_status('retreat_complete:awaiting_scene_attach')
            self.complete_pub.publish(Bool(data=True))
        else:
            unit = self._unit(direction)
            self._publish_twist([speed * value for value in unit], [0.0, 0.0, 0.0], frame=frame)

    def _post_lock_worker(self, tag_id, operation='attach'):
        try:
            # Planning begins only after the newly attached collision object is live.
            if self._execute_joint_goal(self.goals[tag_id]) is False:
                raise RuntimeError('tag retreat execution failed')
            if self.state != DockState.RETREATING:
                return
            self._enter(DockState.RETURNING_TO_WAIT)
            if self._execute_joint_goal(list(self.get_parameter('docking_wait_joint_goal').value)) is False:
                raise RuntimeError('docking_wait return execution failed')
            if self.state != DockState.RETURNING_TO_WAIT:
                return
            mission_goal = self._mission_wait_goal()
            if not mission_goal:
                self._finish_post_motion(operation, 'mission_wait_pose_unconfigured')
                return
            self._enter(DockState.MISSION_WAIT)
            if self._execute_joint_goal(mission_goal) is False:
                raise RuntimeError('mission_wait execution failed')
            if self.state == DockState.MISSION_WAIT:
                self._finish_post_motion(operation, 'mission_wait')
        except Exception as exc:
            self.get_logger().error(f'Post-attach motion failed: {exc}')
            self._fail('post_attach_motion')

    def _finish_post_motion(self, operation, detail):
        if operation == 'detach':
            self.operation, self.target_tag_id = None, None
            self._enter(DockState.IDLE)
            self._publish_status(f'undocked:{detail}')
        else:
            self._enter(DockState.DOCKED)
            self._publish_status(f'ready:{detail}')

    def _execute_joint_goal(self, goal):
        """Track one trajectory so cancel/fault can stop the real goal."""
        with self.motion_lock:
            self.trajectory_active = True
            self.trajectory_finished.clear()
        try:
            self.moveit2.move_to_configuration(goal)
            return self.moveit2.wait_until_executed()
        finally:
            with self.motion_lock:
                self.trajectory_active = False
                self.trajectory_finished.set()

    def _confirm_motion_stopped_after_cancel(self, had_trajectory, cancel_accepted):
        """Require a terminal action and fresh stopped joint feedback after HOLD."""
        timeout = float(self.get_parameter('cancel_stop_timeout_sec').value)
        if had_trajectory and not self.trajectory_finished.wait(timeout=timeout):
            self._publish_status('hold:trajectory_cancel_unconfirmed')
            return
        deadline = time.monotonic() + timeout
        threshold = float(self.get_parameter('stopped_velocity_rad_s').value)
        joint_names = list(self.get_parameter('joint_names').value)
        while time.monotonic() <= deadline:
            if (time.monotonic() - self.joint_state_received_at <= timeout and
                    joints_stopped(self.joint_velocities, joint_names, threshold)):
                status = ('hold:trajectory_cancelled_and_stopped'
                          if had_trajectory and cancel_accepted else 'hold:servo_stopped')
                self._publish_status(status)
                return
            self.joint_state_event.wait(timeout=0.02)
            self.joint_state_event.clear()
        self._publish_status('hold:joint_stop_unconfirmed')

    def _publish_twist(self, linear, angular, frame=None):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame or self.get_parameter('servo_command_frame').value or getattr(self, 'detection_frame', '')
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self.twist_pub.publish(msg)

    def _stop_servo(self):
        self._publish_twist([0.0] * 3, [0.0] * 3)

    def _enter(self, state):
        self.state, self.phase_started = state, self._now_sec()
        self._set_servo_enabled(state == DockState.FINE_XY_ALIGNING)
        self._publish_status(state.name.lower())
        self.get_logger().info(f'Docking state -> {state.name}')

    def _terminal_failure(self, reason):
        if self.state == DockState.CANCELLED_HOLD:
            return
        self._set_servo_enabled(False)
        self._stop_servo()
        with self.motion_lock:
            had_trajectory = self.trajectory_active
        # State changes do not stop an active FollowJointTrajectory.  Cancel
        # the tracked MoveIt execution explicitly, then verify it settles.
        cancel_accepted = self.moveit2.cancel_execution() if had_trajectory else False
        self.get_logger().error(f'Docking aborted: {reason}')
        self._publish_status(f'failed:{reason}')
        self.state, self.target_tag_id, self.operation = DockState.CANCELLED_HOLD, None, None
        threading.Thread(
            target=self._confirm_motion_stopped_after_cancel,
            args=(had_trajectory, cancel_accepted), daemon=True).start()

    def _fail(self, reason):
        # There is deliberately no automatic recovery retreat after failure:
        # configured success retreat is the only path that can emit completion.
        self._terminal_failure(reason)

    def _operator_reset(self, _request, response):
        if self.state != DockState.CANCELLED_HOLD:
            response.success, response.message = False, 'executor is not awaiting operator reset'
            return response
        if self.active_tool_id != self.NONE:
            response.success, response.message = False, 'active tool must be confirmed empty before reset'
            return response
        self.target_tag_id = None
        self.operation = None
        self._enter(DockState.IDLE)
        self._publish_status('idle:operator_reset')
        response.success, response.message = True, 'operator reset accepted'
        return response

    def _publish_status(self, text):

        self.status_pub.publish(String(data=text))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _norm(v): return math.sqrt(sum(x * x for x in v))
    @classmethod
    def _limit(cls, v, maximum):
        magnitude = cls._norm(v)
        return [x * maximum / magnitude for x in v] if magnitude > maximum else v
    @staticmethod
    def _unit(v):
        magnitude = math.sqrt(sum(x * x for x in v))
        return [x / magnitude for x in v] if magnitude > 1e-9 else [0.0, 0.0, 0.0]


def main():
    rclpy.init()
    node = AprilTagToolDocking()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
