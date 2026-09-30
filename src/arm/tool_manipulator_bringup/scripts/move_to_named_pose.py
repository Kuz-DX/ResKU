#!/usr/bin/env python3
"""Send an SRDF arm named pose directly to arm_controller."""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

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
    parser.add_argument("--dry-run", action="store_true", help="Print the target without sending it")
    return parser.parse_args()


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


class NamedPoseRunner(Node):
    def __init__(self) -> None:
        super().__init__("move_to_named_pose")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def run(self, positions: list[float], duration_s: float, wait_s: float) -> int:
        if not self.client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().error(
                "arm_controller action server is unavailable. Start real_control first; "
                "do not run this with rmd_joint_state_bridge.")
            return 2
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions, time_from_start=seconds_to_duration(duration_s))]
        send_future = self.client.send_goal_async(goal)
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
            return 5
        self.get_logger().info("Named-pose trajectory completed")
        return 0


def main() -> int:
    args = parse_args()
    if args.duration <= 0.0 or args.wait_for_server < 0.0:
        print("--duration must be > 0 and --wait-for-server must be >= 0", file=sys.stderr)
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
    print(f"{args.pose}: " + ", ".join(
        f"{name}={value:.6f}" for name, value in zip(ARM_JOINTS, positions)))
    if args.dry_run:
        return 0
    rclpy.init()
    node = NamedPoseRunner()
    try:
        return node.run(positions, args.duration, args.wait_for_server)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
