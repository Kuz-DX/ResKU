#!/usr/bin/env python3
"""Move ee_joint to a measured named gripper position from tools.yaml."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAME = "ee_joint"


def package_config(filename: str) -> Path:
    return Path(get_package_share_directory("tool_manipulator_bringup")) / "config" / filename


def load_target(tools_path: Path, hardware_path: Path, command: str) -> float:
    try:
        with tools_path.open(encoding="utf-8") as stream:
            tools = yaml.safe_load(stream)
        with hardware_path.open(encoding="utf-8") as stream:
            hardware = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML: {exc}") from exc

    tool = (tools.get("tools") or {}).get(0) if isinstance(tools, dict) else None
    actuator = tool.get("actuator") if isinstance(tool, dict) else None
    key = f"{command}_position_rad"
    if not isinstance(actuator, dict) or actuator.get("joint") != JOINT_NAME:
        raise ValueError("tools.yaml tool 0 actuator must control ee_joint")
    value = actuator.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"tools.yaml tools.0.actuator.{key} must be a finite number")

    joint = (hardware.get("joints") or {}).get(JOINT_NAME) if isinstance(hardware, dict) else None
    limits = joint.get("soft_limit_rad") if isinstance(joint, dict) else None
    if not isinstance(limits, list) or len(limits) != 2:
        raise ValueError("hardware.yaml ee_joint.soft_limit_rad must be [lower, upper]")
    lower, upper = map(float, limits)
    if not (math.isfinite(lower) and math.isfinite(upper) and lower <= value <= upper):
        raise ValueError(f"{command} target {value:.6f} outside [{lower:.6f}, {upper:.6f}]")
    return float(value)


def seconds_to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1_000_000_000)))


class GripperRunner(Node):
    def __init__(self) -> None:
        super().__init__("move_gripper")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/ee_controller/follow_joint_trajectory")
        self.position: float | None = None
        self.velocity: float | None = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

    def _on_joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index(JOINT_NAME)
        except ValueError:
            return
        if index < len(message.position) and index < len(message.velocity):
            self.position = float(message.position[index])
            self.velocity = float(message.velocity[index])

    def run(
        self,
        target: float,
        duration_s: float,
        wait_s: float,
        state_timeout_s: float,
        max_start_velocity: float,
    ) -> int:
        if not self.client.wait_for_server(timeout_sec=wait_s):
            self.get_logger().error("ee_controller action server is unavailable")
            return 2

        deadline = time.monotonic() + state_timeout_s
        while rclpy.ok() and self.position is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if self.position is None or self.velocity is None:
            self.get_logger().error("No complete ee_joint position/velocity received")
            return 2
        if not (math.isfinite(self.position) and math.isfinite(self.velocity)):
            self.get_logger().error("ee_joint state contains a non-finite value")
            return 2
        if abs(self.velocity) > max_start_velocity:
            self.get_logger().error(
                f"ee_joint must be stopped first: velocity={self.velocity:.6f}rad/s")
            return 2

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT_NAME]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[target], time_from_start=seconds_to_duration(duration_s))]
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("ee_controller rejected the gripper goal")
            return 3
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("gripper trajectory did not succeed")
            return 4
        self.get_logger().info(f"gripper completed: ee_joint={target:.6f}rad")
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move the gripper to measured open/close position")
    parser.add_argument("command", choices=("open", "close"))
    parser.add_argument("--tools", type=Path, default=None)
    parser.add_argument("--hardware", type=Path, default=None)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--wait-for-server", type=float, default=5.0)
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--max-start-velocity", type=float, default=0.02)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    numeric = (
        args.duration, args.wait_for_server, args.joint_state_timeout,
        args.max_start_velocity,
    )
    if (not all(math.isfinite(value) for value in numeric) or
            args.duration <= 0.0 or args.wait_for_server < 0.0 or
            args.joint_state_timeout <= 0.0 or args.max_start_velocity < 0.0):
        print("invalid duration, timeout, or velocity argument", file=sys.stderr)
        return 2

    tools_path = (args.tools or package_config("tools.yaml")).resolve()
    hardware_path = (args.hardware or package_config("hardware.yaml")).resolve()
    try:
        target = load_target(tools_path, hardware_path, args.command)
    except (OSError, TypeError, ValueError) as exc:
        print(f"gripper configuration invalid: {exc}", file=sys.stderr)
        return 2
    print(f"{args.command}: {JOINT_NAME}={target:.6f}rad")
    if args.dry_run:
        return 0

    print("WARNING: keep hands and objects clear of the gripper pinch zone.")
    rclpy.init()
    node = GripperRunner()
    try:
        return node.run(
            target, args.duration, args.wait_for_server,
            args.joint_state_timeout, args.max_start_velocity)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
