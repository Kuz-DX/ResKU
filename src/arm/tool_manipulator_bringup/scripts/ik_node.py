#!/usr/bin/env python3
"""One-shot grasp sequence for the six-axis tool manipulator.

This is the tool_manipulator adaptation of army_manipulator_bringup/ik_node.py.
Unlike the four-axis source robot, MoveIt's ``arm`` group already ends at
``tcp_link``.  The requested pose is therefore sent to MoveIt as the TCP pose;
there is no hard-coded wrist-to-TCP subtraction in this node.

The robot/tool-specific values below intentionally remain unset.  The node is
fail-safe: it may start and report the missing fields, but it will not send any
trajectory until every required value has been filled and is finite.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
)
END_EFFECTOR_JOINT_NAMES = ("ee_joint",)

# ---------------------------------------------------------------------------
# TODO(tool-manipulator commissioning)
#
# Fill these values in radians/metres before enabling this node on hardware.
# Keep ``None`` for every value that has not been measured and verified.  The
# validation in _configuration_errors() deliberately prevents motion while a
# placeholder remains.
# ---------------------------------------------------------------------------

STARTUP_ARM_POSITIONS = {
    "base_joint": None,
    "shoulder_joint": None,
    "elbow_joint": None,
    "wrist_pitch_joint": None,
    "wrist_roll_joint": None,
    "wrist_yaw_joint": None,
}

HOLD_ARM_POSITIONS = {
    "base_joint": None,
    "shoulder_joint": None,
    "elbow_joint": None,
    "wrist_pitch_joint": None,
    "wrist_roll_joint": None,
    "wrist_yaw_joint": None,
}

HOME_ARM_POSITIONS = {
    "base_joint": None,
    "shoulder_joint": None,
    "elbow_joint": None,
    "wrist_pitch_joint": None,
    "wrist_roll_joint": None,
    "wrist_yaw_joint": None,
}

BED_ARM_POSITIONS = {
    "base_joint": None,
    "shoulder_joint": None,
    "elbow_joint": None,
    "wrist_pitch_joint": None,
    "wrist_roll_joint": None,
    "wrist_yaw_joint": None,
}

# Tool-1 ee_joint positions [rad].
END_EFFECTOR_OPEN_POSITION = None
END_EFFECTOR_CLOSED_POSITION = None

# Desired tcp_link orientation in planning_frame, quaternion [x, y, z, w].
TARGET_TCP_ORIENTATION_XYZW = None

# Supply-box TCP height in planning_frame [m].  It is only required while
# use_fixed_target_z is true.
FIXED_TARGET_Z_M = None


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _pose_errors(name: str, values: Mapping[str, object]) -> list[str]:
    errors = []
    missing = [joint for joint in ARM_JOINT_NAMES if joint not in values]
    extra = sorted(set(values).difference(ARM_JOINT_NAMES))
    if missing or extra:
        errors.append(f"{name}: missing={missing}, extra={extra}")
        return errors
    unset = [joint for joint in ARM_JOINT_NAMES if not _finite_number(values[joint])]
    if unset:
        errors.append(f"{name}: set finite radians for {unset}")
    return errors


def _configuration_errors(use_fixed_target_z: bool) -> list[str]:
    errors = []
    for name, pose in (
        ("STARTUP_ARM_POSITIONS", STARTUP_ARM_POSITIONS),
        ("HOLD_ARM_POSITIONS", HOLD_ARM_POSITIONS),
        ("HOME_ARM_POSITIONS", HOME_ARM_POSITIONS),
        ("BED_ARM_POSITIONS", BED_ARM_POSITIONS),
    ):
        errors.extend(_pose_errors(name, pose))
    if not _finite_number(END_EFFECTOR_OPEN_POSITION):
        errors.append("END_EFFECTOR_OPEN_POSITION: set a finite ee_joint angle")
    if not _finite_number(END_EFFECTOR_CLOSED_POSITION):
        errors.append("END_EFFECTOR_CLOSED_POSITION: set a finite ee_joint angle")
    orientation = TARGET_TCP_ORIENTATION_XYZW
    if (
        not isinstance(orientation, Sequence)
        or isinstance(orientation, (str, bytes))
        or len(orientation) != 4
        or not all(_finite_number(value) for value in orientation)
    ):
        errors.append("TARGET_TCP_ORIENTATION_XYZW: set finite [x, y, z, w]")
    else:
        norm = math.sqrt(sum(float(value) ** 2 for value in orientation))
        if norm < 1e-9:
            errors.append("TARGET_TCP_ORIENTATION_XYZW: quaternion norm is zero")
    if use_fixed_target_z and not _finite_number(FIXED_TARGET_Z_M):
        errors.append("FIXED_TARGET_Z_M: set a finite height or disable use_fixed_target_z")
    return errors


def _ordered_pose(values: Mapping[str, float]) -> list[float]:
    return [float(values[name]) for name in ARM_JOINT_NAMES]


def _normalized_orientation() -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in TARGET_TCP_ORIENTATION_XYZW)
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(value / norm for value in values)


class IKNode(Node):
    """Accept one detected point and execute a configured grasp sequence."""

    def __init__(self) -> None:
        super().__init__("tool_manipulator_ik_node")
        callback_group = ReentrantCallbackGroup()

        self.declare_parameter("target_topic", "/arm/target_point")
        self.declare_parameter("planning_frame", "base_actuator")
        self.declare_parameter("move_group", "arm")
        self.declare_parameter("ik_link_name", "tcp_link")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("arm_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("end_effector_action", "/ee_controller/follow_joint_trajectory")
        self.declare_parameter("arm_duration_sec", 5.0)
        self.declare_parameter("end_effector_duration_sec", 3.0)
        self.declare_parameter("joint_position_tolerance_rad", 0.03)
        self.declare_parameter("ik_request_timeout_sec", 0.5)
        self.declare_parameter("ik_avoid_collisions", True)
        self.declare_parameter("settle_before_close_sec", 2.0)
        self.declare_parameter("use_fixed_target_z", True)
        self.declare_parameter("picking_topic", "/picking")
        self.declare_parameter("grasp_success_topic", "/arm/grasp_success")
        self.declare_parameter("calculation_failure_topic", "/arm/calculation_failed")
        self.declare_parameter("picking_command_topic", "/arm/picking_command")
        self.declare_parameter("target_point_base_topic", "/arm/target_point_base")
        self.declare_parameter("target_distance_topic", "/arm/target_distance_m")
        self.declare_parameter("manual_override_topic", "/control/arm_manual_override")

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.move_group = str(self.get_parameter("move_group").value)
        self.ik_link_name = str(self.get_parameter("ik_link_name").value)
        self.arm_duration = max(0.1, float(self.get_parameter("arm_duration_sec").value))
        self.end_effector_duration = max(
            0.1, float(self.get_parameter("end_effector_duration_sec").value))
        self.joint_tolerance = max(
            0.001, float(self.get_parameter("joint_position_tolerance_rad").value))
        self.ik_timeout = max(
            0.01, float(self.get_parameter("ik_request_timeout_sec").value))
        self.avoid_collisions = bool(self.get_parameter("ik_avoid_collisions").value)
        self.settle_before_close = max(
            0.0, float(self.get_parameter("settle_before_close_sec").value))
        self.use_fixed_target_z = bool(self.get_parameter("use_fixed_target_z").value)

        self._lock = threading.RLock()
        self._arm_positions: dict[str, float] = {}
        self._startup_complete = False
        self._busy = False
        self._grasp_done = False
        self._manual_override = False
        self._config_errors = _configuration_errors(self.use_fixed_target_z)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=callback_group)
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_action").value),
            callback_group=callback_group,
        )
        self.end_effector_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("end_effector_action").value),
            callback_group=callback_group,
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            20,
            callback_group=callback_group,
        )
        self.create_subscription(
            PointStamped,
            self.target_topic,
            self._on_target,
            10,
            callback_group=callback_group,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("manual_override_topic").value),
            self._on_manual_override,
            latched_qos,
            callback_group=callback_group,
        )
        self.picking_pub = self.create_publisher(
            Bool, str(self.get_parameter("picking_topic").value), latched_qos)
        self.grasp_success_pub = self.create_publisher(
            Bool, str(self.get_parameter("grasp_success_topic").value), 10)
        self.failure_pub = self.create_publisher(
            Empty, str(self.get_parameter("calculation_failure_topic").value), 10)
        self.picking_command_pub = self.create_publisher(
            Empty, str(self.get_parameter("picking_command_topic").value), 10)
        self.target_base_pub = self.create_publisher(
            PointStamped, str(self.get_parameter("target_point_base_topic").value), 10)
        self.target_distance_pub = self.create_publisher(
            Float64, str(self.get_parameter("target_distance_topic").value), 10)
        self.picking_pub.publish(Bool(data=False))

        if self._config_errors:
            self.get_logger().error(
                "IK motion is disabled until the TODO values are configured:\n- "
                + "\n- ".join(self._config_errors)
            )
        else:
            threading.Thread(target=self._startup_worker, daemon=True).start()

    def _on_joint_state(self, message: JointState) -> None:
        with self._lock:
            for name, position in zip(message.name, message.position):
                if name in ARM_JOINT_NAMES and math.isfinite(position):
                    self._arm_positions[name] = float(position)

    def _on_manual_override(self, message: Bool) -> None:
        with self._lock:
            self._manual_override = bool(message.data)

    def _on_target(self, message: PointStamped) -> None:
        if self._config_errors:
            self.get_logger().error("Target ignored: commissioning values are still unset.")
            return
        with self._lock:
            if self._manual_override or self._busy or self._grasp_done:
                return
            if not self._startup_complete:
                self.get_logger().warn("Target ignored until the startup pose is complete.")
                return
            self._busy = True
        try:
            target = self._transform_target(message)
            if self.use_fixed_target_z:
                target = (target[0], target[1], float(FIXED_TARGET_Z_M))
        except Exception as error:
            with self._lock:
                self._busy = False
            self.get_logger().error(f"Cannot transform target: {error}")
            self.failure_pub.publish(Empty())
            return

        transformed = PointStamped()
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.header.frame_id = self.planning_frame
        transformed.point.x, transformed.point.y, transformed.point.z = target
        self.target_base_pub.publish(transformed)
        self.target_distance_pub.publish(Float64(data=math.dist((0.0, 0.0, 0.0), target)))
        self.picking_pub.publish(Bool(data=True))
        threading.Thread(target=self._grasp_worker, args=(target,), daemon=True).start()

    def _transform_target(self, message: PointStamped) -> tuple[float, float, float]:
        if not message.header.frame_id or message.header.frame_id == self.planning_frame:
            point = message.point
        else:
            transform_time = (
                Time.from_msg(message.header.stamp)
                if message.header.stamp.sec or message.header.stamp.nanosec
                else Time()
            )
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                message.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.5),
            )
            point = do_transform_point(message, transform).point
        values = (float(point.x), float(point.y), float(point.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target contains a non-finite coordinate")
        return values

    def _startup_worker(self) -> None:
        while rclpy.ok():
            with self._lock:
                if self._manual_override:
                    time.sleep(0.1)
                    continue
                state_ready = all(name in self._arm_positions for name in ARM_JOINT_NAMES)
            if not state_ready:
                time.sleep(0.1)
                continue
            if not self._send_trajectory(
                self.end_effector_client,
                END_EFFECTOR_JOINT_NAMES,
                [float(END_EFFECTOR_OPEN_POSITION)],
                self.end_effector_duration,
                "startup end-effector open",
            ):
                time.sleep(1.0)
                continue
            if not self._send_arm(_ordered_pose(STARTUP_ARM_POSITIONS), "startup pose"):
                time.sleep(1.0)
                continue
            with self._lock:
                self._startup_complete = True
            self.get_logger().info("Startup pose reached; accepting one target.")
            return

    @staticmethod
    def _wait_future(future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _send_trajectory(self, client, names, positions, duration, label) -> bool:
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{label}: controller action unavailable")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        duration_ns = int(round(duration * 1_000_000_000))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        goal.trajectory.points = [point]
        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, 5.0):
            self.get_logger().error(f"{label}: goal response timeout")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, duration + 8.0):
            handle.cancel_goal_async()
            self.get_logger().error(f"{label}: execution timeout")
            return False
        wrapped = result_future.result()
        success = (
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not success:
            self.get_logger().error(f"{label}: controller execution failed")
        return success

    def _send_arm(self, target: Sequence[float], label: str) -> bool:
        if not self._send_trajectory(
            self.arm_client, ARM_JOINT_NAMES, target, self.arm_duration, label
        ):
            return False
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                errors = [
                    abs(self._arm_positions.get(name, math.inf) - expected)
                    for name, expected in zip(ARM_JOINT_NAMES, target)
                ]
            if max(errors) <= self.joint_tolerance:
                return True
            time.sleep(0.05)
        self.get_logger().error(f"{label}: measured joints did not reach target; errors={errors}")
        return False

    def _solve_ik(self, target_tcp: Sequence[float]) -> list[float] | None:
        if not self.ik_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error("/compute_ik service unavailable")
            return None
        with self._lock:
            current = [self._arm_positions.get(name) for name in ARM_JOINT_NAMES]
        if any(value is None for value in current):
            return None

        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.move_group
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = self.planning_frame
        pose = request.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = target_tcp
        orientation = _normalized_orientation()
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = orientation
        request.ik_request.avoid_collisions = self.avoid_collisions
        timeout_ns = int(round(self.ik_timeout * 1_000_000_000))
        request.ik_request.timeout.sec = timeout_ns // 1_000_000_000
        request.ik_request.timeout.nanosec = timeout_ns % 1_000_000_000
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = [float(value) for value in current]

        future = self.ik_client.call_async(request)
        if not self._wait_future(future, self.ik_timeout + 1.0):
            self.get_logger().error("IK request timed out")
            return None
        response = future.result()
        if response is None or response.error_code.val != 1:
            code = None if response is None else response.error_code.val
            self.get_logger().error(f"No IK solution for tcp_link target; error_code={code}")
            return None
        by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if not all(name in by_name for name in ARM_JOINT_NAMES):
            self.get_logger().error("IK response is missing one or more arm joints")
            return None
        solution = [float(by_name[name]) for name in ARM_JOINT_NAMES]
        return solution if all(math.isfinite(value) for value in solution) else None

    def _grasp_worker(self, target: Sequence[float]) -> None:
        completed = False
        try:
            solution = self._solve_ik(target)
            if solution is None or not self._send_arm(solution, "detected target"):
                self.failure_pub.publish(Empty())
                return
            if self.settle_before_close:
                time.sleep(self.settle_before_close)
            if not self._send_trajectory(
                self.end_effector_client,
                END_EFFECTOR_JOINT_NAMES,
                [float(END_EFFECTOR_CLOSED_POSITION)],
                self.end_effector_duration,
                "end-effector close",
            ):
                self.grasp_success_pub.publish(Bool(data=False))
                return
            self.grasp_success_pub.publish(Bool(data=True))
            for label, positions in (
                ("hold", HOLD_ARM_POSITIONS),
                ("home", HOME_ARM_POSITIONS),
            ):
                if not self._send_arm(_ordered_pose(positions), label):
                    self.failure_pub.publish(Empty())
                    return
            self.picking_command_pub.publish(Empty())
            if not self._send_arm(_ordered_pose(BED_ARM_POSITIONS), "bed"):
                self.get_logger().warn("Bed pose failed after picking_command was published")
            with self._lock:
                self._grasp_done = True
            completed = True
        except Exception as error:
            self.get_logger().error(f"Grasp sequence exception: {type(error).__name__}: {error}")
            self.failure_pub.publish(Empty())
        finally:
            self.picking_pub.publish(Bool(data=False))
            with self._lock:
                self._busy = False
            if not completed:
                self.get_logger().warn("Grasp attempt ended; a new target may be accepted")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
