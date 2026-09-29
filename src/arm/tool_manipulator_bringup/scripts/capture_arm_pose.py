#!/usr/bin/env python3
"""Capture one read-only arm pose from ``/joint_states`` as JSON.

This tool never publishes a command. It is intended to run alongside
``rmd_joint_state_bridge`` while each joint is moved manually to a mechanical
limit. The expected names are the current tool_manipulator CAD arm joints,
plus every Dynamixel joint the bridge can expose. An absent/failed actuator is
written as JSON ``null`` instead of making the capture fail.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Keep this list synchronized with tool_manipulator.urdf.xacro. gripper_joint
# is an optional tool Dynamixel: the read-only bridge publishes it when its ID
# responds, and null otherwise.
EXPECTED_JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
    "gripper_joint",
)
DYNAMIXEL_JOINTS = ("base_joint", "wrist_yaw_joint", "gripper_joint")


class PoseCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("capture_arm_pose")
        self.positions: Optional[Dict[str, float]] = None
        self.stamp: Optional[dict] = None
        self.create_subscription(JointState, topic, self._on_joint_state, 10)

    def _on_joint_state(self, message: JointState) -> None:
        # A bridge can publish a partial state when an individual CAN/TTL
        # actuator is disconnected. Missing expected joints become null later.
        self.positions = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }
        self.stamp = {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save one /joint_states sample; absent expected joints become JSON null.")
    parser.add_argument("--topic", default="/joint_states", help="JointState topic (default: /joint_states)")
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="Seconds to wait for one JointState message (default: 3.0)")
    parser.add_argument("--output", type=Path,
                        help="Output JSON path (default: arm_pose_<UTC timestamp>.json in the current directory)")
    parser.add_argument("--label", default="",
                        help="Optional note, e.g. 'elbow_upper_limit'")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0:
        print("--timeout must be greater than zero", file=sys.stderr)
        return 2

    rclpy.init()
    node = PoseCapture(args.topic)
    try:
        deadline_ns = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while rclpy.ok() and node.positions is None and node.get_clock().now().nanoseconds < deadline_ns:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.positions is None:
            print(f"No JointState received on {args.topic} within {args.timeout:g}s.", file=sys.stderr)
            return 1

        captured_at = datetime.now(timezone.utc)
        output = args.output or Path.cwd() / (
            "arm_pose_" + captured_at.strftime("%Y%m%dT%H%M%SZ") + ".json")
        output.parent.mkdir(parents=True, exist_ok=True)
        joint_positions = {name: node.positions.get(name) for name in EXPECTED_JOINTS}
        missing = [name for name, value in joint_positions.items() if value is None]
        record = {
            "format": "tool_manipulator_arm_pose/v1",
            "captured_at_utc": captured_at.isoformat().replace("+00:00", "Z"),
            "label": args.label or None,
            "joint_state_topic": args.topic,
            "joint_state_stamp": node.stamp,
            "joint_positions_rad": joint_positions,
            "dynamixel_joint_names": list(DYNAMIXEL_JOINTS),
            "missing_joint_names": missing,
        }
        output.write_text(json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"Saved {output}")
        if missing:
            print("Missing (written as null): " + ", ".join(missing))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
