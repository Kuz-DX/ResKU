#!/usr/bin/env python3
"""Two-stage AprilTag tool docking controller for ROS 2 Humble.

The visual-servo frame must be the same frame used by the AprilTag detector's
poses.  In the supplied configuration this is the RealSense optical frame.
"""
import json
import math
import threading
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


class DockState(Enum):
    IDLE = auto()
    COARSE_MOVING = auto()
    FINE_XY_ALIGNING = auto()
    DESCENDING = auto()
    LOCK_ROTATING = auto()
    ATTACHING_SCENE = auto()
    ASCENDING = auto()
    RETREATING = auto()
    RETURNING_TO_WAIT = auto()
    MISSION_WAIT = auto()
    POST_LOCK_HOLD = auto()
    DOCKED = auto()


class AprilTagToolDocking(Node):
    """Plan once in joint space, then close the last few cm with MoveIt Servo."""

    def __init__(self):
        super().__init__('apriltag_tool_docking')
        self.cb_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self.goals = self._read_goals()
        self._validate_motion_params()
        self.state = DockState.IDLE
        self.target_tag_id = None
        self.phase_started = self._now_sec()
        self.last_detection_time = None
        self.error = None
        self.stable_frames = 0
        self.best_error = float('inf')
        self.diverging_frames = 0
        self.joint_positions = {}
        # Latched feedback from tool_attachment_manager.  No retreat with a
        # newly attached physical tool is allowed until its collision object is live.
        self.active_tool_id = 0

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=list(self.get_parameter('joint_names').value),
            base_link_name=self.get_parameter('base_link').value,
            end_effector_name=self.get_parameter('end_effector_link').value,
            group_name=self.get_parameter('planning_group').value,
            callback_group=self.cb_group,
        )
        self.twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter('servo_twist_topic').value, 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'docking_status', status_qos)
        self.complete_pub = self.create_publisher(Bool, 'docking_complete', status_qos)
        detection_topic = self.get_parameter('detection_topic').value
        if self.get_parameter('detection_message_type').value == 'vision_json':
            self.create_subscription(String, detection_topic, self._json_detections_cb, 10, callback_group=self.cb_group)
        else:
            self.create_subscription(AprilTagDetectionArray, detection_topic, self._detections_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            JointState, self.get_parameter('joint_state_topic').value, self._joint_state_cb, 20)
        self.create_subscription(
            Int32, self.get_parameter('active_tool_id_topic').value, self._active_tool_cb, 10)
        tool_topic = self.get_parameter('tool_id_topic').value
        # UI default is Int32. Set tool_id_message_type:=string when the UI emits String.
        if self.get_parameter('tool_id_message_type').value.lower() == 'string':
            self.create_subscription(String, tool_topic, self._tool_string_cb, 10)
        else:
            self.create_subscription(Int32, tool_topic, self._tool_int_cb, 10)
        rate = float(self.get_parameter('servo_rate_hz').value)
        self.create_timer(1.0 / rate, self._control_tick, callback_group=self.cb_group)
        self._publish_status('idle')

    def _declare_parameters(self):
        p = self.declare_parameter
        p('tool_id_topic', '/selected_tool_id')
        p('tool_id_message_type', 'int32')
        p('active_tool_id_topic', '/active_tool_id')
        p('detection_topic', '/arm/apriltag/centers')
        p('detection_message_type', 'vision_json')
        # Dotted fields accommodate apriltag_msgs variants/bridge message layouts.
        p('tag_id_field', 'id')
        p('tag_pose_field', 'pose.pose.pose')
        p('servo_twist_topic', '/servo_node/delta_twist_cmds')
        p('servo_command_frame', 'camera_color_optical_frame')
        p('planning_group', 'arm')
        p('base_link', 'base_actuator')
        p('end_effector_link', 'tcp_link')
        p('joint_state_topic', '/joint_states')
        p('joint_names', ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_yaw_joint'])
        # Preferred registry.  Each ID owns its taught coarse joint goal and
        # later also owns TCP, collision and actuator metadata.
        p('tools_config_file', '')
        # ROS parameters cannot reliably carry a heterogeneous YAML map, so use a
        # YAML string: '{10: [q1, q2, q3, q4], 11: [...]}' in docking.yaml.
        p('tool_joint_goals_yaml', '{}')
        # Named common posture executed before every tag-specific approach.
        p('docking_wait_joint_goal', [3.14159265, 0.0, 0.0, 0.0, 0.0])
        p('desired_tag_position', [0.0, 0.0, 0.20])
        p('kp_position', 0.8)
        p('max_linear_velocity', 0.03)
        p('servo_rate_hz', 50.0)
        p('xy_position_tolerance', 0.003)
        p('stable_frame_count', 8)
        p('coarse_timeout_sec', 30.0)
        p('xy_align_timeout_sec', 20.0)
        p('detection_timeout_sec', 0.5)
        p('divergence_ratio', 1.5)
        p('divergence_frames', 10)
        # Descent is deliberately open-loop after visual XY centring: it follows a
        # calibrated fixture axis, not a noisy tag Z measurement.
        p('descent_distance_m', 0.015)
        p('descent_speed_mps', 0.008)
        # Unlike visual XY commands, descent can be expressed in a fixed robot
        # frame so "down" stays vertical even if the wrist/camera is tilted.
        p('descent_command_frame', 'arm_world_frame')
        p('descent_direction', [0.0, 0.0, -1.0])
        p('descent_timeout_sec', 5.0)
        # A planned joint move gives an angle-confirmed lock rotation, unlike a
        # time-based angular Twist which can be scaled by Servo near a limit.
        p('lock_joint_name', 'wrist_yaw_joint')
        p('lock_rotation_rad', 1.5708)
        p('lock_timeout_sec', 15.0)
        # Negative ascent_distance_m deliberately means "not taught": after a
        # successful physical lock the arm holds instead of guessing a retreat.
        p('attachment_timeout_sec', 5.0)
        p('ascent_distance_m', -1.0)
        p('ascent_speed_mps', 0.008)
        p('ascent_timeout_sec', 5.0)
        p('post_lock_motion_timeout_sec', 45.0)
        # Final front-facing mission posture is intentionally empty until taught.
        p('mission_wait_joint_goal_yaml', '[]')

    def _validate_motion_params(self):
        if self.get_parameter('servo_rate_hz').value <= 0.0:
            raise RuntimeError('servo_rate_hz must be positive')
        if self.get_parameter('descent_distance_m').value < 0.0 or self.get_parameter('descent_speed_mps').value <= 0.0:
            raise RuntimeError('descent_distance_m must be >= 0 and descent_speed_mps must be positive')
        if self._norm(self.get_parameter('descent_direction').value) <= 1e-9:
            raise RuntimeError('descent_direction must not be a zero vector')
        if self.get_parameter('lock_joint_name').value not in self.get_parameter('joint_names').value:
            raise RuntimeError('lock_joint_name must be one of joint_names')
        joint_count = len(self.get_parameter('joint_names').value)
        if len(self.get_parameter('docking_wait_joint_goal').value) != joint_count:
            raise RuntimeError('docking_wait_joint_goal must have one value per joint_names entry')
        mission_goal = self._mission_wait_goal()
        if mission_goal and len(mission_goal) != joint_count:
            raise RuntimeError('mission_wait_joint_goal must be empty or have one value per joint_names entry')

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
        try:
            result = yaml.safe_load(self.get_parameter('tool_joint_goals_yaml').value) or {}
            goals = {int(tag): [float(q) for q in values] for tag, values in result.items()}
        except (yaml.YAMLError, TypeError, ValueError) as exc:
            raise RuntimeError('tool_joint_goals_yaml must be a YAML map of tag id to joint list') from exc
        expected = len(self.get_parameter('joint_names').value)
        if any(len(goal) != expected for goal in goals.values()):
            raise RuntimeError('every tool joint goal must have one value per joint_names entry')
        return goals

    def _tool_int_cb(self, msg):
        self._start_request(msg.data)

    def _tool_string_cb(self, msg):
        try:
            self._start_request(int(msg.data))
        except ValueError:
            self.get_logger().warning('Ignoring non-integer tool String: %r' % msg.data)

    def _start_request(self, tag_id):
        if self.state != DockState.IDLE:
            self.get_logger().warning('Docking request ignored: controller is busy or already docked')
            return
        if self.active_tool_id != 0:
            self._publish_status(f'failed:active_tool_{self.active_tool_id}_requires_release')
            return
        if tag_id not in self.goals:
            self.get_logger().error(f'No taught joint goal for requested tag {tag_id}')
            self._publish_status('failed:no_taught_goal')
            return
        self._stop_servo()
        self.target_tag_id = tag_id
        self._enter(DockState.COARSE_MOVING)
        # Planning/execution waits must not block perception or Servo publishing.
        threading.Thread(target=self._coarse_worker, args=(tag_id,), daemon=True).start()

    def _coarse_worker(self, tag_id):
        try:
            # docking_wait is the shared rack-facing joint posture. It is kept
            # separate from the tag goal so every UI request begins identically.
            self._publish_status('coarse_moving:docking_wait')
            self.moveit2.move_to_configuration(
                list(self.get_parameter('docking_wait_joint_goal').value))
            wait_result = self.moveit2.wait_until_executed()
            if wait_result is False:
                raise RuntimeError('docking_wait action returned failure')
            if self.state != DockState.COARSE_MOVING or self.target_tag_id != tag_id:
                return
            self._publish_status(f'coarse_moving:tool_{tag_id}')
            self.moveit2.move_to_configuration(self.goals[tag_id])
            # pymoveit2 waits for the FollowJointTrajectory action result here;
            # transition only after it reports completion (or raises on failure).
            result = self.moveit2.wait_until_executed()
            success = result is not False
        except Exception as exc:  # Planning/action failures are returned to a safe state.
            self.get_logger().error(f'Coarse move or docking_wait failed: {exc}')
            success = False
        if self.state != DockState.COARSE_MOVING or self.target_tag_id != tag_id:
            return
        if success:
            self.last_detection_time = None
            self.error = None
            self.stable_frames = self.diverging_frames = 0
            self.best_error = float('inf')
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
        # Tool geometry defines insertion Z and lock angle. The camera only closes
        # the two lateral degrees of freedom; tag depth/orientation are ignored.
        desired_p = self.get_parameter('desired_tag_position').value
        exy = [position[0] - desired_p[0], position[1] - desired_p[1]]
        self.error = exy
        self.last_detection_time = self._now_sec()
        xy_norm = self._norm(exy)
        if xy_norm <= self.get_parameter('xy_position_tolerance').value:
            self.stable_frames += 1
        else:
            self.stable_frames = 0
        if xy_norm < self.best_error:
            self.best_error = xy_norm
            self.diverging_frames = 0
        elif self.best_error > 0.0 and xy_norm > self.best_error * self.get_parameter('divergence_ratio').value:
            self.diverging_frames += 1
        else:
            self.diverging_frames = 0

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

    def _active_tool_cb(self, msg):
        self.active_tool_id = int(msg.data)

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
        if self.state == DockState.ATTACHING_SCENE:
            if self.active_tool_id == self.target_tag_id:
                if self.get_parameter('ascent_distance_m').value < 0.0:
                    self._enter(DockState.POST_LOCK_HOLD)
                    self._publish_status('hold:ascent_distance_unconfigured')
                else:
                    self._enter(DockState.ASCENDING)
            elif now - self.phase_started > self.get_parameter('attachment_timeout_sec').value:
                self._fail('planning_scene_attach_timeout')
            return
        if self.state == DockState.ASCENDING:
            self._ascent_tick(now)
            return
        if self.state in (DockState.RETREATING, DockState.RETURNING_TO_WAIT, DockState.MISSION_WAIT):
            if now - self.phase_started > self.get_parameter('post_lock_motion_timeout_sec').value:
                self._fail('post_lock_motion_timeout')
            return
        if self.state != DockState.FINE_XY_ALIGNING:
            return
        if now - self.phase_started > self.get_parameter('xy_align_timeout_sec').value:
            self._fail('xy_alignment_timeout')
            return
        if self.last_detection_time is None or now - self.last_detection_time > self.get_parameter('detection_timeout_sec').value:
            self._stop_servo()
            return
        if self.diverging_frames >= self.get_parameter('divergence_frames').value:
            self._fail('visual_servo_diverging')
            return
        exy = self.error
        self._publish_twist(self._limit([self.get_parameter('kp_position').value * exy[0],
                                         self.get_parameter('kp_position').value * exy[1], 0.0],
                                        self.get_parameter('max_linear_velocity').value),
                            [0.0, 0.0, 0.0])
        if self.stable_frames >= self.get_parameter('stable_frame_count').value:
            self._stop_servo()
            self._enter(DockState.DESCENDING)

    def _descent_tick(self, now):
        speed = self.get_parameter('descent_speed_mps').value
        duration = self.get_parameter('descent_distance_m').value / max(speed, 1e-6)
        if now - self.phase_started > self.get_parameter('descent_timeout_sec').value:
            self._fail('descent_timeout')
        elif now - self.phase_started >= duration:
            self._stop_servo()
            self._enter(DockState.LOCK_ROTATING)
            threading.Thread(target=self._lock_worker, daemon=True).start()
        else:
            direction = self._unit(self.get_parameter('descent_direction').value)
            self._publish_twist(
                [speed * value for value in direction], [0.0, 0.0, 0.0],
                frame=self.get_parameter('descent_command_frame').value)

    def _lock_worker(self):
        lock_joint = self.get_parameter('lock_joint_name').value
        joints = list(self.get_parameter('joint_names').value)
        if lock_joint not in joints or any(name not in self.joint_positions for name in joints):
            self.get_logger().error('Cannot lock: joint_states lacks the configured arm joints')
            self._fail('lock_joint_state_unavailable')
            return
        goal = [self.joint_positions[name] for name in joints]
        goal[joints.index(lock_joint)] += self.get_parameter('lock_rotation_rad').value
        try:
            # One collision-checked plan and FollowJointTrajectory result for the
            # mechanically meaningful final angle; it is not visual re-planning.
            self.moveit2.move_to_configuration(goal)
            result = self.moveit2.wait_until_executed()
            success = result is not False
        except Exception as exc:
            self.get_logger().error(f'Lock rotation failed: {exc}')
            success = False
        if self.state != DockState.LOCK_ROTATING:
            return
        if success:
            # Physical lock is complete.  Attachment manager now creates the
            # collision object; ascent cannot start until it confirms active_tool_id.
            self._enter(DockState.ATTACHING_SCENE)
            self._publish_status('lock_complete:awaiting_scene_attach')
            self.complete_pub.publish(Bool(data=True))
        else:
            self._fail('lock_rotation')

    def _ascent_tick(self, now):
        speed = self.get_parameter('ascent_speed_mps').value
        distance = self.get_parameter('ascent_distance_m').value
        duration = distance / max(speed, 1e-6)
        if now - self.phase_started > self.get_parameter('ascent_timeout_sec').value:
            self._fail('ascent_timeout')
        elif now - self.phase_started >= duration:
            self._stop_servo()
            self._enter(DockState.RETREATING)
            threading.Thread(target=self._post_lock_worker, args=(self.target_tag_id,), daemon=True).start()
        else:
            direction = self._unit(self.get_parameter('descent_direction').value)
            self._publish_twist([-speed * value for value in direction], [0.0, 0.0, 0.0],
                                frame=self.get_parameter('descent_command_frame').value)

    def _post_lock_worker(self, tag_id):
        try:
            # Step 8: use the same taught per-tag approach posture as the final
            # safe rack retreat.  It is executed after the collision object exists.
            self.moveit2.move_to_configuration(self.goals[tag_id])
            if self.moveit2.wait_until_executed() is False:
                raise RuntimeError('tag retreat execution failed')
            if self.state != DockState.RETREATING:
                return
            self._enter(DockState.RETURNING_TO_WAIT)
            self.moveit2.move_to_configuration(list(self.get_parameter('docking_wait_joint_goal').value))
            if self.moveit2.wait_until_executed() is False:
                raise RuntimeError('docking_wait return execution failed')
            if self.state != DockState.RETURNING_TO_WAIT:
                return
            mission_goal = self._mission_wait_goal()
            if not mission_goal:
                self._enter(DockState.DOCKED)
                self._publish_status('ready:mission_wait_pose_unconfigured')
                return
            self._enter(DockState.MISSION_WAIT)
            self.moveit2.move_to_configuration(mission_goal)
            if self.moveit2.wait_until_executed() is False:
                raise RuntimeError('mission_wait execution failed')
            if self.state == DockState.MISSION_WAIT:
                self._enter(DockState.DOCKED)
                self._publish_status('ready:mission_wait')
        except Exception as exc:
            self.get_logger().error(f'Post-lock retreat failed: {exc}')
            self._fail('post_lock_motion')

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
        self._publish_status(state.name.lower())
        self.complete_pub.publish(Bool(data=False))
        self.get_logger().info(f'Docking state -> {state.name}')

    def _fail(self, reason):
        self._stop_servo()
        self.get_logger().error(f'Docking aborted: {reason}')
        self._publish_status(f'failed:{reason}')
        self.state, self.target_tag_id = DockState.IDLE, None

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
