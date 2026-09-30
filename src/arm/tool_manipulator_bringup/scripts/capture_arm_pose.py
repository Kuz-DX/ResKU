#!/usr/bin/env python3
"""Interactively print read-only arm poses from ``/joint_states``.

This tool never publishes a command. It is intended to run alongside
``rmd_joint_state_bridge`` while each joint is moved manually. Type a label
and press Enter to print the latest values; type ``q`` to exit. It never
creates a JSON file.
"""

import argparse
import json
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Keep this list synchronized with tool_manipulator.urdf.xacro. ee_joint
# is the common optional tool Dynamixel. The bridge calls it gripper_joint;
# _on_joint_state normalizes that live name to ee_joint below.
EXPECTED_JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
    "ee_joint",
)
DYNAMIXEL_JOINTS = ("base_joint", "wrist_roll_joint", "wrist_yaw_joint", "ee_joint")

# Stable, short operator-facing labels. Keep this order aligned with the
# physical chain so a single Enter yields a readily comparable snapshot.
DISPLAY_NAMES = {
    "base_joint": "base",
    "shoulder_joint": "shoulder",
    "elbow_joint": "elbow",
    "wrist_pitch_joint": "wrist_pitch",
    "wrist_roll_joint": "wrist_roll",
    "wrist_yaw_joint": "wrist_yaw",
    "ee_joint": "ee",
}


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
            ("ee_joint" if name == "gripper_joint" else name): float(position)
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

        captured_at = datetime.now(timezone.utc)
        output = args.output or Path.cwd() / (
            "arm_pose_" + captured_at.strftime("%Y%m%dT%H%M%SZ") + ".json")
        output.parent.mkdir(parents=True, exist_ok=True)
        # If every reader is disconnected, the bridge has no valid JointState
        # entries to publish.  Still save a useful all-null measurement.
        received_positions = node.positions or {}
        joint_positions = {name: received_positions.get(name) for name in EXPECTED_JOINTS}
        missing = [name for name, value in joint_positions.items() if value is None]
        record = {
            "format": "tool_manipulator_arm_pose/v1",
            "captured_at_utc": captured_at.isoformat().replace("+00:00", "Z"),
            "label": args.label or None,
            "joint_state_topic": args.topic,
            "joint_state_stamp": node.stamp,
            "joint_state_received": node.positions is not None,
            "joint_positions_rad": joint_positions,
            # Preserve every source value as well. This makes a live
            # controller/URDF naming mismatch visible instead of discarding a
            # measurement; the bridge default matches wrist_pitch_joint.
            "observed_joint_positions_rad": received_positions,
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


def interactive_main() -> int:
    """Print the latest state whenever the operator confirms a label."""
    rclpy.init()
    node = PoseCapture("/joint_states")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print("Press Enter to print all joint angles; an optional label may be entered. q, quit, or exit ends the node.")
        while rclpy.ok():
            try:
                label = input("label> ").strip()
            except EOFError:
                print()
                break
            if label.lower() in {"q", "quit", "exit"}:
                break
            positions = node.positions or {}
            if not positions:
                print("No /joint_states received yet; try again.\n")
                continue
            print(f"\n[{label or 'capture'}] (rad)")
            for name in EXPECTED_JOINTS:
                value = positions.get(name)
                display_name = DISPLAY_NAMES[name]
                print(f"  {display_name}: {value:.6f}" if value is not None else f"  {display_name}: unavailable")
            print()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(interactive_main())
