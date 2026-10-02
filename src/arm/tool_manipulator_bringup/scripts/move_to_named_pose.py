#!/usr/bin/env python3
"""Send an SRDF arm named pose directly to arm_controller."""
from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_pose_safety import load_soft_limits, validate_joint_positions

ARM_JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move the arm to an arm group_state stored in tool_manipulator.srdf.")
    parser.add_argument("pose", nargs="?", help="Named arm pose, e.g. home or dock_pre_cw")
    parser.add_argument("--list", action="store_true", help="List available arm named poses and exit")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Trajectory duration in seconds (default: 5.0)")
    parser.add_argument("--wait-for-server", type=float, default=5.0,
                        help="Seconds to wait for arm_controller (default: 5.0)")
    parser.add_argument("--srdf", type=Path, help="Optional SRDF override")
    parser.add_argument("--hardware", type=Path, help="Optional hardware.yaml override")
    parser.add_argument("--limit-margin", type=float, default=0.0,
                        help="Required distance inside each soft limit in radians (default: 0.0)")
    parser.add_argument("--joint-state-topic", default="/joint_states",
                        help="JointState topic checked before motion (default: /joint_states)")
    parser.add_argument("--joint-state-timeout", type=float, default=2.0,
                        help="Seconds to wait for a complete current arm state (default: 2.0)")
    parser.add_argument("--max-start-velocity", type=float, default=0.02,
                        help="Maximum absolute start joint velocity in rad/s (default: 0.02)")
    parser.add_argument("--dry-run", action="store_true", help="Print the target without sending it")
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def load_arm_poses(srdf_path: Path) -> dict[str, list[float]]:
    root = ET.parse(srdf_path).getroot()
    poses: dict[str, list[float]] = {}
    for group_state in root.findall("group_state[@group='arm']"):
        values = {joint.attrib["name"]: float(joint.attrib["value"])
                  for joint in group_state.findall("joint")}
        missing = [name for name in ARM_JOINTS if name not in values]
        extra = sorted(set(values).difference(ARM_JOINTS))
        if missing or extra:
            raise ValueError(
                f"Invalid arm pose '{group_state.attrib['name']}' in {srdf_path}: "
                f"missing={missing}, extra={extra}")
        poses[group_state.attrib["name"]] = [values[name] for name in ARM_JOINTS]
    if not poses:
        raise ValueError(f"No arm group_state entries found in {srdf_path}")
    return poses


def seconds_to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1_000_000_000))


def load_motion_speeds(hardware: Path) -> dict[str, float]:
    with hardware.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    speeds = {name: float(config["joints"][name]["velocity_limit_rad_s"])
              for name in ARM_JOINTS}
    if any(not math.isfinite(value) or value <= 0.0 for value in speeds.values()):
        raise ValueError("All arm velocity_limit_rad_s values must be finite and positive")
    return speeds


def make_smooth_points(current, target, requested_duration, speeds):
    """Quintic rest-to-rest motion, limited to half the configured joint speeds."""
    start = [current[name] for name in ARM_JOINTS]
    delta = [end - begin for begin, end in zip(start, target)]
    # max(d(10*u^3 - 15*u^4 + 6*u^5)/du) = 1.875 at u=0.5.
    duration = max(requested_duration, max(
        1.875 * abs(distance) / (0.5 * speeds[name])
        for name, distance in zip(ARM_JOINTS, delta)))
    count = max(2, math.ceil(duration / 0.05))
    points = []
    for index in range(count + 1):
        u = index / count
        blend = 10*u**3 - 15*u**4 + 6*u**5
        velocity = 30*u**2 * (1-u)**2 / duration
        acceleration = 60*u * (1-u) * (1-2*u) / duration**2
        points.append(JointTrajectoryPoint(
            positions=[begin + distance * blend for begin, distance in zip(start, delta)],
            velocities=[distance * velocity for distance in delta],
            accelerations=[distance * acceleration for distance in delta],
            time_from_start=seconds_to_duration(duration * u),
        ))
    return points, duration


class NamedPoseRunner(Node):
    def __init__(self, joint_state_topic: str) -> None:
        super().__init__("move_to_named_pose")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.latest_positions: dict[str, float] | None = None
        self.latest_velocities: dict[str, float] | None = None
        self.tracking_peaks: dict[str, tuple[float, float, float, float]] = {}
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 20)

    def _on_action_feedback(self, message) -> None:
        feedback = message.feedback
        elapsed = (feedback.desired.time_from_start.sec +
                   feedback.desired.time_from_start.nanosec * 1e-9)
        for name, desired, actual, error in zip(
                feedback.joint_names, feedback.desired.positions,
                feedback.actual.positions, feedback.error.positions):
            if name not in ARM_JOINTS:
                continue
            if not all(math.isfinite(v) for v in (desired, actual, error, elapsed)):
                continue
            previous = self.tracking_peaks.get(name)
            if previous is None or abs(error) >= abs(previous[2]):
                self.tracking_peaks[name] = (desired, actual, error, elapsed)

    def _report_tracking(self) -> None:
        if not self.tracking_peaks:
            self.get_logger().error(
                "No usable action feedback received; inspect ros2_control_node logs "
                "and /arm_controller/controller_state for the failing joint.")
            return
        self.get_logger().error(
            "Peak observed position errors per joint (feedback samples may miss "
            "the abort instant; these are not final target errors):")
        for name, (desired, actual, error, elapsed) in sorted(
                self.tracking_peaks.items(), key=lambda item: abs(item[1][2]), reverse=True):
            self.get_logger().error(
                f"  {name}: desired={desired:.6f}, actual={actual:.6f}, "
                f"error={error:+.6f} rad ({math.degrees(error):+.3f} deg), "
                f"trajectory_time={elapsed:.3f}s")

    def _on_joint_state(self, message: JointState) -> None:
        observed = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }
        observed_velocity = {
            name: float(velocity)
            for name, velocity in zip(message.name, message.velocity)
        }
        if (all(name in observed for name in ARM_JOINTS) and
                all(name in observed_velocity for name in ARM_JOINTS)):
            self.latest_positions = {name: observed[name] for name in ARM_JOINTS}
            self.latest_velocities = {
                name: observed_velocity[name] for name in ARM_JOINTS}

    def _wait_for_joint_state(
        self, timeout_s: float
    ) -> tuple[dict[str, float], dict[str, float]] | None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and self.latest_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if self.latest_positions is None:
            self.get_logger().error(
                f"No complete current arm state received within {timeout_s:.2f}s")
            return None
        return dict(self.latest_positions), dict(self.latest_velocities or {})

    def run(
        self,
        positions: list[float],
        duration_s: float,
        wait_s: float,
        limits: dict[str, tuple[float, float]],
        joint_state_timeout_s: float,
        max_start_velocity: float,
        speeds: dict[str, float],
    ) -> int:
        if not self.client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().error(
                "arm_controller action server is unavailable. Start real_control first; "
                "do not run this with rmd_joint_state_bridge.")
            return 2
        current_state = self._wait_for_joint_state(joint_state_timeout_s)
        if current_state is None:
            return 2
        current, velocities = current_state
        try:
            validate_joint_positions(current, limits, 0.0, label="current arm state")
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return 2
        moving = {
            name: abs(value) for name, value in velocities.items()
            if not math.isfinite(value) or abs(value) > max_start_velocity
        }
        if moving:
            self.get_logger().error(
                "Arm must be stopped before a named-pose goal: " +
                ", ".join(f"{name}={value:.6f}rad/s" for name, value in moving.items())
            )
            return 2
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points, duration_s = make_smooth_points(
            current, positions, duration_s, speeds)
        self.tracking_peaks.clear()
        self.get_logger().info(
            f"Sending smooth named pose over {duration_s:.3f}s (50% hardware speed cap); start: " +
            ", ".join(f"{name}={current[name]:.6f}" for name in ARM_JOINTS))
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("arm_controller rejected the named-pose goal")
            return 3
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("No result returned by arm_controller")
            return 4
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = wrapped_result.result
            self.get_logger().error(
                f"Named-pose trajectory failed (status={wrapped_result.status}, "
                f"error_code={result.error_code}, message='{result.error_string}')")
            self._report_tracking()
            return 5
        self.get_logger().info("Named-pose trajectory completed")
        return 0


def main() -> int:
    args = parse_args()
    numeric_args = (
        args.duration,
        args.wait_for_server,
        args.limit_margin,
        args.joint_state_timeout,
        args.max_start_velocity,
    )
    if (not all(math.isfinite(value) for value in numeric_args) or
            args.duration <= 0.0 or args.wait_for_server < 0.0 or
            args.limit_margin < 0.0 or args.joint_state_timeout <= 0.0 or
            args.max_start_velocity < 0.0):
        print(
            "duration and joint-state timeout must be positive; wait and margin "
            "must be nonnegative finite values",
            file=sys.stderr,
        )
        return 2
    srdf = args.srdf or Path(get_package_share_directory(
        "tool_manipulator_moveit_config")) / "config" / "tool_manipulator.srdf"
    try:
        poses = load_arm_poses(srdf)
    except (ET.ParseError, OSError, ValueError) as error:
        print(f"Cannot load named poses: {error}", file=sys.stderr)
        return 2
    if args.list or args.pose is None:
        print("Available arm named poses:")
        for name in sorted(poses):
            print(f"  {name}")
        return 0 if args.list else 2
    if args.pose not in poses:
        print(f"Unknown arm pose '{args.pose}'. Use --list to see available poses.", file=sys.stderr)
        return 2
    positions = poses[args.pose]
    hardware = args.hardware or Path(get_package_share_directory(
        "tool_manipulator_bringup")) / "config" / "hardware.yaml"
    target = dict(zip(ARM_JOINTS, positions))
    try:
        limits = load_soft_limits(hardware, ARM_JOINTS)
        speeds = load_motion_speeds(hardware)
        validate_joint_positions(
            target, limits, args.limit_margin, label=f"target pose '{args.pose}'")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"Unsafe named pose: {error}", file=sys.stderr)
        return 2
    print(f"{args.pose}: " + ", ".join(
        f"{name}={value:.6f}" for name, value in zip(ARM_JOINTS, positions)))
    print(f"hardware: {hardware} (soft-limit margin={args.limit_margin:.6f} rad)")
    if args.dry_run:
        return 0
    print("WARNING: this sends a direct joint goal; it does not perform MoveIt collision planning.")
    rclpy.init()
    node = NamedPoseRunner(args.joint_state_topic)
    try:
        return node.run(
            positions,
            args.duration,
            args.wait_for_server,
            limits,
            args.joint_state_timeout,
            args.max_start_velocity,
            speeds,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
