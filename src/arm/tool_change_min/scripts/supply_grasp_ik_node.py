#!/usr/bin/env python3
"""One-shot supply-box grasp sequence for the current six-axis manipulator.

This is the current-platform counterpart of the former
``army_manipulator_bringup/ik_node.py`` flow.  It deliberately uses the
``arm`` MoveIt group (six joints), ``tcp_link`` and ``ee_joint`` from
``tool_manipulator.urdf.xacro``; none of the former four-axis seeds or joint
names are reused.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

from tool_change_min.grasp_sequence import (
    checked_joint_vector,
    fixed_grasp_z,
    normalized_quaternion,
    pitch_offset_quaternion,
    required_duration,
)
from tool_change_min.kinematics import ArmKinematics


ARM_JOINTS = (
    "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
    "wrist_roll_joint", "wrist_yaw_joint",
)
EE_JOINT = "ee_joint"


def _duration(seconds: float):
    from builtin_interfaces.msg import Duration as DurationMsg
    nanoseconds = int(round(seconds * 1_000_000_000))
    return DurationMsg(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


class SupplyGraspIkNode(Node):
    def __init__(self) -> None:
        super().__init__("supply_grasp_ik_node")
        callback_group = ReentrantCallbackGroup()
        bringup_share = Path(get_package_share_directory("tool_manipulator_bringup"))
        description_share = Path(get_package_share_directory("tool_manipulator_description"))

        defaults = (
            ("hardware_yaml", str(bringup_share / "config" / "hardware.yaml")),
            ("urdf_xacro", str(description_share / "urdf" / "tool_manipulator.urdf.xacro")),
            ("target_topic", "/arm/target_point"),
            ("planning_frame", "base_actuator"),
            ("ik_group", "arm"),
            ("ik_link", "tcp_link"),
            ("arm_action", "/arm_controller/follow_joint_trajectory"),
            ("ee_action", "/ee_controller/follow_joint_trajectory"),
            ("arm_duration_sec", 3.0),
            ("ee_duration_sec", 3.0),
            ("joint_state_max_age_sec", 0.5),
            ("joint_position_tolerance_rad", 0.04),
            ("use_fixed_target_z", True),
            ("base_height_m", 0.350),
            ("box_height_m", 0.095),
            ("box_grasp_height_ratio", 0.5),
            ("supplybox_tcp_offset_z", -0.0055),
            ("detected_z_warning_threshold_m", 0.05),
            ("ik_request_timeout_sec", 0.5),
            ("ik_avoid_collisions", True),
            ("orientation_pitch_offsets_rad", [0.0, 0.15, -0.15, 0.30, -0.30]),
            ("use_startup_tcp_orientation", True),
            ("target_orientation_xyzw", [0.0, 0.0, 0.0, 1.0]),
            # Two URDF-derived low-grasp seeds (near/middle radius), six joints
            # per profile. Their base value is rotated toward each target.
            ("ground_ik_seed_profiles", [
                0.0072, 1.6583, -1.3079, 0.4078, 0.0083, 1.8107,
                0.4392, 1.5932, -1.1028, 0.5387, -0.0673, 3.0264,
            ]),
            ("settle_before_close_sec", 2.0),
            ("ee_open_rad", -0.076699),
            ("ee_closed_rad", 1.251728),
            # Current-platform taught arm-only poses. EE captures from the old
            # encoder convention are intentionally not reused.
            ("startup_pose", [-2.662991, 0.283616, 0.943350, -1.647242, -0.081301, -0.225495]),
            ("hold_pose", [-2.662991, 0.355174, 0.730246, -1.655096, -0.079767, -0.477068]),
            ("home_pose", [-0.024544, 1.526814, 1.503776, -1.656143, -0.090505, -0.142660]),
            ("bed_pose", [-0.024544, 1.526814, 1.503776, -1.656143, -0.090505, -0.142660]),
            ("picking_topic", "/picking"),
            ("grasp_success_topic", "/arm/grasp_success"),
            ("calculation_failure_topic", "/arm/calculation_failed"),
            ("picking_command_topic", "/arm/picking_command"),
            ("target_point_base_topic", "/arm/target_point_base"),
            ("target_distance_topic", "/arm/target_distance_m"),
            ("manual_override_topic", "/control/arm_manual_override"),
        )
        for name, default in defaults:
            self.declare_parameter(name, default)

        hardware = self._load_hardware(Path(str(self.get_parameter("hardware_yaml").value)))
        self.limits = {
            name: tuple(map(float, hardware["joints"][name]["soft_limit_rad"]))
            for name in (*ARM_JOINTS, EE_JOINT)
        }
        self.velocity_limits = {
            name: float(hardware["joints"][name]["velocity_limit_rad_s"])
            for name in (*ARM_JOINTS, EE_JOINT)
        }
        self.model = ArmKinematics(
            str(self.get_parameter("urdf_xacro").value),
            str(self.get_parameter("hardware_yaml").value))
        if tuple(self.model.joint_names) != ARM_JOINTS:
            raise ValueError(
                f"URDF arm joint order {self.model.joint_names} differs from {ARM_JOINTS}")
        self.startup_pose = checked_joint_vector(
            ARM_JOINTS, self.get_parameter("startup_pose").value, self.limits, "startup_pose")
        self.hold_pose = checked_joint_vector(
            ARM_JOINTS, self.get_parameter("hold_pose").value, self.limits, "hold_pose")
        self.home_pose = checked_joint_vector(
            ARM_JOINTS, self.get_parameter("home_pose").value, self.limits, "home_pose")
        self.bed_pose = checked_joint_vector(
            ARM_JOINTS, self.get_parameter("bed_pose").value, self.limits, "bed_pose")
        self.ee_open = checked_joint_vector(
            (EE_JOINT,), [self.get_parameter("ee_open_rad").value], self.limits, "ee_open_rad")[0]
        self.ee_closed = checked_joint_vector(
            (EE_JOINT,), [self.get_parameter("ee_closed_rad").value], self.limits, "ee_closed_rad")[0]

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.ik_group = str(self.get_parameter("ik_group").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.arm_duration = self._positive("arm_duration_sec")
        self.ee_duration = self._positive("ee_duration_sec")
        self.max_state_age = self._positive("joint_state_max_age_sec")
        self.joint_tolerance = self._positive("joint_position_tolerance_rad")
        self.ik_timeout = self._positive("ik_request_timeout_sec")
        self.settle_time = max(0.0, float(self.get_parameter("settle_before_close_sec").value))
        self.use_fixed_z = bool(self.get_parameter("use_fixed_target_z").value)
        self.fixed_z = fixed_grasp_z(
            float(self.get_parameter("base_height_m").value),
            float(self.get_parameter("box_height_m").value),
            float(self.get_parameter("box_grasp_height_ratio").value),
            float(self.get_parameter("supplybox_tcp_offset_z").value),
        )
        self.z_warning = max(
            0.0, float(self.get_parameter("detected_z_warning_threshold_m").value))
        self.avoid_collisions = bool(self.get_parameter("ik_avoid_collisions").value)
        self.pitch_offsets = [
            float(value) for value in self.get_parameter("orientation_pitch_offsets_rad").value]
        if not self.pitch_offsets or not all(math.isfinite(value) for value in self.pitch_offsets):
            raise ValueError("orientation_pitch_offsets_rad must be a non-empty finite list")
        self.use_startup_orientation = bool(
            self.get_parameter("use_startup_tcp_orientation").value)
        self.target_orientation = normalized_quaternion(
            self.get_parameter("target_orientation_xyzw").value)
        raw_profiles = [
            float(value) for value in self.get_parameter("ground_ik_seed_profiles").value]
        if not raw_profiles or len(raw_profiles) % len(ARM_JOINTS):
            raise ValueError("ground_ik_seed_profiles must contain one or more six-joint vectors")
        self.ground_seed_profiles = [
            checked_joint_vector(
                ARM_JOINTS, raw_profiles[index:index + len(ARM_JOINTS)],
                self.limits, f"ground_ik_seed_profiles[{index // len(ARM_JOINTS)}]")
            for index in range(0, len(raw_profiles), len(ARM_JOINTS))
        ]

        self._lock = threading.RLock()
        self._snapshot_values: dict[str, float] | None = None
        self._snapshot_time: float | None = None
        self._manual_override = False
        self._startup_complete = False
        self._busy = False
        self._grasp_done = False
        self._motion_fault = ""

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=callback_group)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, str(self.get_parameter("arm_action").value),
            callback_group=callback_group)
        self.ee_client = ActionClient(
            self, FollowJointTrajectory, str(self.get_parameter("ee_action").value),
            callback_group=callback_group)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data,
            callback_group=callback_group)
        self.create_subscription(
            PointStamped, self.target_topic, self._on_target, 10,
            callback_group=callback_group)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, str(self.get_parameter("manual_override_topic").value),
            self._on_manual_override, latched, callback_group=callback_group)
        self.picking_pub = self.create_publisher(
            Bool, str(self.get_parameter("picking_topic").value), latched)
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
        threading.Thread(target=self._startup_worker, daemon=True).start()
        self.get_logger().info(
            "Current-platform grasp flow ready: six-axis arm/tcp_link + ee_joint; "
            f"target Z={'fixed %.4fm' % self.fixed_z if self.use_fixed_z else 'detected'}.")

    @staticmethod
    def _load_hardware(path: Path) -> dict:
        with path.expanduser().resolve(strict=True).open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or not isinstance(data.get("joints"), dict):
            raise ValueError(f"invalid hardware YAML: {path}")
        for name in (*ARM_JOINTS, EE_JOINT):
            spec = data["joints"].get(name, {})
            limit = spec.get("soft_limit_rad")
            velocity = spec.get("velocity_limit_rad_s")
            if not isinstance(limit, list) or len(limit) != 2 or velocity is None:
                raise ValueError(f"hardware YAML missing limits/velocity for {name}")
        return data

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return value

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        by_name = dict(zip(message.name, message.position))
        names = (*ARM_JOINTS, EE_JOINT)
        if not all(name in by_name and math.isfinite(float(by_name[name])) for name in names):
            return
        with self._lock:
            self._snapshot_values = {name: float(by_name[name]) for name in names}
            self._snapshot_time = time.monotonic()

    def _snapshot(self) -> dict[str, float]:
        with self._lock:
            values = None if self._snapshot_values is None else dict(self._snapshot_values)
            received = self._snapshot_time
        if values is None or received is None:
            raise RuntimeError("no complete seven-joint /joint_states snapshot")
        age = time.monotonic() - received
        if age > self.max_state_age:
            raise RuntimeError(f"stale /joint_states snapshot ({age:.3f}s)")
        return values

    def _on_manual_override(self, message: Bool) -> None:
        with self._lock:
            self._manual_override = bool(message.data)

    def _startup_worker(self) -> None:
        while rclpy.ok():
            try:
                with self._lock:
                    if self._manual_override:
                        raise RuntimeError("manual override active")
                self._snapshot()
                if not self._send_trajectory(
                        self.ee_client, (EE_JOINT,), [self.ee_open], self.ee_duration,
                        "startup EE open", require_feedback=True):
                    raise RuntimeError("startup EE open failed")
                if not self._send_arm(self.startup_pose, "startup grasp_wait"):
                    raise RuntimeError("startup grasp_wait failed")
                if self.use_startup_orientation:
                    self.target_orientation = self._lookup_tcp_orientation()
                with self._lock:
                    self._startup_complete = True
                self.get_logger().info(
                    "Startup grasp_wait reached with EE open; accepting one supply target.")
                return
            except Exception as exc:
                self.get_logger().warn(f"Startup not ready ({exc}); retrying in 1s.")
                time.sleep(1.0)

    def _lookup_tcp_orientation(self) -> tuple[float, float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, self.ik_link, Time(), timeout=Duration(seconds=1.0))
        rotation = transform.transform.rotation
        result = normalized_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))
        self.get_logger().info(f"Using startup TCP orientation XYZW={result} for grasp IK.")
        return result

    def _on_target(self, message: PointStamped) -> None:
        with self._lock:
            if self._grasp_done:
                self.get_logger().info(
                    "Grasp already completed; ignoring target.", throttle_duration_sec=10.0)
                return
            if not self._startup_complete or self._busy or self._manual_override:
                return
            self._busy = True
        try:
            detected = self._transform_target(message)
            target = (detected[0], detected[1], self.fixed_z if self.use_fixed_z else detected[2])
            if (self.use_fixed_z and self.z_warning > 0.0 and
                    abs(detected[2] - target[2]) > self.z_warning):
                self.get_logger().warn(
                    f"Detected Z={detected[2]:.3f} differs from fixed Z={target[2]:.3f}; "
                    "using fixed geometry Z.")
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = self.planning_frame
            point.point.x, point.point.y, point.point.z = target
            self.target_base_pub.publish(point)
            self.target_distance_pub.publish(Float64(data=math.sqrt(sum(v * v for v in target))))
            self.picking_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"Target accepted once: detected={detected}, TCP target={target}.")
            threading.Thread(target=self._grasp_worker, args=(target,), daemon=True).start()
        except Exception as exc:
            self.get_logger().error(f"Target transform failed: {exc}")
            self.failure_pub.publish(Empty())
            with self._lock:
                self._busy = False

    def _transform_target(self, message: PointStamped) -> tuple[float, float, float]:
        if message.header.frame_id and message.header.frame_id != self.planning_frame:
            stamp = (Time.from_msg(message.header.stamp)
                     if message.header.stamp.sec or message.header.stamp.nanosec else Time())
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, message.header.frame_id, stamp,
                timeout=Duration(seconds=0.5))
            point = do_transform_point(message, transform).point
        else:
            point = message.point
        xyz = (float(point.x), float(point.y), float(point.z))
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("target point contains a non-finite coordinate")
        return xyz

    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _send_trajectory(self, client, names, positions, requested_duration: float,
                         label: str, require_feedback: bool) -> bool:
        try:
            if self._motion_fault:
                raise RuntimeError(
                    "motion blocked until node restart and physical stop check: "
                    + self._motion_fault)
            snapshot = self._snapshot()
            current = [snapshot[name] for name in names]
            target = checked_joint_vector(names, positions, self.limits, label)
            duration = required_duration(
                names, current, target, self.velocity_limits, requested_duration)
            if not client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("controller action unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(names)
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=target, time_from_start=_duration(duration))]
            send_future = client.send_goal_async(goal)
            if not self._wait_future(send_future, 5.0):
                self._motion_fault = "goal acceptance unresolved; physical stop unconfirmed"
                send_future.add_done_callback(self._cancel_late_goal)
                raise TimeoutError(self._motion_fault)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("goal rejected")
            result_future = handle.get_result_async()
            if not self._wait_future(result_future, duration + 8.0):
                self._motion_fault = "controller result unresolved; physical stop unconfirmed"
                handle.cancel_goal_async()
                raise TimeoutError(self._motion_fault + "; cancellation requested")
            result = result_future.result()
            if (result is None or result.status != GoalStatus.STATUS_SUCCEEDED or
                    result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                raise RuntimeError("controller execution failed")
            if require_feedback:
                deadline = time.monotonic() + 2.0
                errors = [math.inf] * len(names)
                while time.monotonic() < deadline:
                    actual = self._snapshot()
                    errors = [abs(actual[name] - value) for name, value in zip(names, target)]
                    if max(errors) <= self.joint_tolerance:
                        break
                    time.sleep(0.05)
                else:
                    raise RuntimeError(f"post-result feedback outside tolerance: {errors}")
            return True
        except Exception as exc:
            self.get_logger().error(f"{label}: {exc}")
            return False

    def _cancel_late_goal(self, future) -> None:
        """Cancel a goal accepted after our response deadline; stop stays unconfirmed."""
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                self.get_logger().error(
                    "Late goal accepted and cancellation requested; physical stop unconfirmed.")
        except Exception as exc:
            self.get_logger().error(f"Late goal response could not be cancelled: {exc}")

    def _send_arm(self, target, label: str, require_feedback: bool = True) -> bool:
        return self._send_trajectory(
            self.arm_client, ARM_JOINTS, target, self.arm_duration, label, require_feedback)

    def _solve_ik(self, target) -> list[float] | None:
        if not self.ik_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error("/compute_ik service unavailable")
            return None
        snapshot = self._snapshot()
        current_seed = [snapshot[name] for name in ARM_JOINTS]
        attempts = [("startup", current_seed, self.target_orientation)]
        target_bearing = math.atan2(target[1], target[0])
        target_radius = math.hypot(target[0], target[1])
        ground_attempts = []
        for index, values in enumerate(self.ground_seed_profiles):
            seed = list(values)
            transform = self.model.fk(seed)
            source_bearing = math.atan2(transform[1, 3], transform[0, 3])
            delta = math.atan2(
                math.sin(target_bearing - source_bearing),
                math.cos(target_bearing - source_bearing))
            seed[0] += delta
            if not self.limits[ARM_JOINTS[0]][0] <= seed[0] <= self.limits[ARM_JOINTS[0]][1]:
                continue
            orientation = self._matrix_quaternion(transform[:3, :3])
            orientation = self._yaw_rotated_quaternion(orientation, delta)
            radius = math.hypot(transform[0, 3], transform[1, 3])
            ground_attempts.append((abs(radius - target_radius), f"ground-{index}", seed, orientation))
        attempts.extend(
            (label, seed, orientation)
            for _, label, seed, orientation in sorted(ground_attempts, key=lambda item: item[0]))
        for label, seed, base_orientation in attempts:
            for offset in self.pitch_offsets:
                orientation = pitch_offset_quaternion(base_orientation, offset)
                solution = self._request_ik(target, orientation, seed)
                if solution is not None:
                    self.get_logger().info(
                        f"IK solved with {label} seed, pitch offset {offset:+.3f}rad: "
                        f"{solution}")
                    return solution
        self.get_logger().error("No valid IK solution for the detected supply target")
        return None

    @staticmethod
    def _matrix_quaternion(rotation) -> tuple[float, float, float, float]:
        """Convert a 3x3 rotation matrix to normalized ROS XYZW form."""
        trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            values = (
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            )
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            values = (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                      (rotation[0, 2] + rotation[2, 0]) / scale,
                      (rotation[2, 1] - rotation[1, 2]) / scale)
        elif rotation[1, 1] > rotation[2, 2]:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            values = ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                      (rotation[1, 2] + rotation[2, 1]) / scale,
                      (rotation[0, 2] - rotation[2, 0]) / scale)
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            values = ((rotation[0, 2] + rotation[2, 0]) / scale,
                      (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                      (rotation[1, 0] - rotation[0, 1]) / scale)
        return normalized_quaternion(values)

    @staticmethod
    def _yaw_rotated_quaternion(orientation, yaw):
        x, y, z, w = orientation
        sine, cosine = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return normalized_quaternion((
            cosine * x - sine * y,
            sine * x + cosine * y,
            cosine * z + sine * w,
            cosine * w - sine * z,
        ))

    def _request_ik(self, target, orientation, seed) -> list[float] | None:
        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.ik_group
        ik.ik_link_name = self.ik_link
        ik.avoid_collisions = self.avoid_collisions
        ik.timeout = _duration(self.ik_timeout)
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = orientation
        ik.pose_stamped = pose
        state = RobotState()
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = seed
        state.is_diff = False
        ik.robot_state = state
        request.ik_request = ik
        future = self.ik_client.call_async(request)
        if not self._wait_future(future, self.ik_timeout + 1.0):
            return None
        response = future.result()
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        by_name = dict(zip(
            response.solution.joint_state.name, response.solution.joint_state.position))
        if not all(name in by_name for name in ARM_JOINTS):
            return None
        try:
            solution = checked_joint_vector(
                ARM_JOINTS, [by_name[name] for name in ARM_JOINTS],
                self.limits, "IK solution")
        except ValueError:
            return None
        return solution

    def _grasp_worker(self, target) -> None:
        try:
            solution = self._solve_ik(target)
            if solution is None:
                self.failure_pub.publish(Empty())
                return
            if not self._send_arm(solution, "detected target", require_feedback=False):
                self.failure_pub.publish(Empty())
                return
            if self.settle_time:
                time.sleep(self.settle_time)
            if not self._send_trajectory(
                    self.ee_client, (EE_JOINT,), [self.ee_closed], self.ee_duration,
                    "EE close", require_feedback=False):
                self.grasp_success_pub.publish(Bool(data=False))
                return
            self.grasp_success_pub.publish(Bool(data=True))
            if not self._send_arm(self.hold_pose, "hold", require_feedback=False):
                self.failure_pub.publish(Empty())
                return
            with self._lock:
                self._grasp_done = True
            if not self._send_arm(self.home_pose, "home", require_feedback=False):
                self.failure_pub.publish(Empty())
                return
            self.picking_command_pub.publish(Empty())
            self.get_logger().info("HOME complete; /arm/picking_command published.")
            if not self._send_arm(self.bed_pose, "bed", require_feedback=False):
                self.get_logger().warn("BED move failed after completion event; grasp remains latched.")
            else:
                self.get_logger().info("BED complete; further targets are ignored.")
        except Exception as exc:
            self.get_logger().error(f"Grasp sequence failed: {type(exc).__name__}: {exc}")
            self.failure_pub.publish(Empty())
        finally:
            self.picking_pub.publish(Bool(data=False))
            with self._lock:
                self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SupplyGraspIkNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger("supply_grasp_ik_node").fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
