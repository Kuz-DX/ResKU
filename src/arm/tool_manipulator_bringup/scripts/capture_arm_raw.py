#!/usr/bin/env python3
"""Interactively print calibrated arm positions and read-only raw encoders."""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
    "ee_joint",
)
DISPLAY_NAMES = {
    "base_joint": "base",
    "shoulder_joint": "shoulder",
    "elbow_joint": "elbow",
    "wrist_pitch_joint": "wrist_pitch",
    "wrist_roll_joint": "wrist_roll",
    "wrist_yaw_joint": "wrist_yaw",
    "ee_joint": "ee",
}
RMD_JOINTS = ("shoulder_joint", "elbow_joint", "wrist_pitch_joint")
DXL_JOINTS = ("base_joint", "wrist_roll_joint", "wrist_yaw_joint", "ee_joint")


class RawCapture(Node):
    def __init__(self) -> None:
        super().__init__("capture_arm_raw")
        self._lock = threading.Lock()
        self.positions: dict[str, float] = {}
        self.rmd_raw_deg: Optional[list[float]] = None
        self.rmd_encoder: Optional[list[int]] = None
        self.dxl_encoder: Optional[list[int]] = None

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/arm/rmd_raw_angle_deg", self._rmd_raw_deg_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/arm/rmd_encoder", self._rmd_encoder_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/arm/dxl_encoder", self._dxl_encoder_cb, 10)

    def _joint_state_cb(self, message: JointState) -> None:
        with self._lock:
            self.positions = {
                ("ee_joint" if name == "gripper_joint" else name): float(position)
                for name, position in zip(message.name, message.position)
            }

    def _rmd_raw_deg_cb(self, message: Float64MultiArray) -> None:
        with self._lock:
            self.rmd_raw_deg = (
                list(message.data) if len(message.data) == len(RMD_JOINTS) else None)

    def _rmd_encoder_cb(self, message: Float64MultiArray) -> None:
        with self._lock:
            self.rmd_encoder = (
                [round(value) for value in message.data]
                if len(message.data) == len(RMD_JOINTS) else None)

    def _dxl_encoder_cb(self, message: Float64MultiArray) -> None:
        with self._lock:
            self.dxl_encoder = (
                [round(value) for value in message.data]
                if len(message.data) == len(DXL_JOINTS) else None)

    def snapshot(self):
        with self._lock:
            positions = dict(self.positions)
            rmd_raw_deg = None if self.rmd_raw_deg is None else list(self.rmd_raw_deg)
            rmd_encoder = None if self.rmd_encoder is None else list(self.rmd_encoder)
            dxl_encoder = None if self.dxl_encoder is None else list(self.dxl_encoder)
        return positions, rmd_raw_deg, rmd_encoder, dxl_encoder


def _format_float(value: Optional[float], digits: int = 6) -> str:
    return "unavailable" if value is None else f"{value:.{digits}f}"


def _format_int(value: Optional[int]) -> str:
    return "unavailable" if value is None else str(value)


def main() -> int:
    rclpy.init()
    node = RawCapture()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print(
            "Press Enter to print calibrated rad and raw encoder values; "
            "an optional label may be entered. q, quit, or exit ends the node.")
        while rclpy.ok():
            try:
                label = input("label> ").strip()
            except EOFError:
                print()
                break
            if label.lower() in {"q", "quit", "exit"}:
                break

            positions, rmd_raw_deg, rmd_encoder, dxl_encoder = node.snapshot()
            if not positions and rmd_raw_deg is None and dxl_encoder is None:
                print("No bridge data received yet; try again.\n")
                continue

            rmd_deg_by_name = (
                dict(zip(RMD_JOINTS, rmd_raw_deg)) if rmd_raw_deg is not None else {})
            rmd_encoder_by_name = (
                dict(zip(RMD_JOINTS, rmd_encoder)) if rmd_encoder is not None else {})
            dxl_encoder_by_name = (
                dict(zip(DXL_JOINTS, dxl_encoder)) if dxl_encoder is not None else {})

            print(f"\n[{label or 'capture'}]")
            for name in JOINTS:
                rad = _format_float(positions.get(name))
                display_name = DISPLAY_NAMES[name]
                if name in DXL_JOINTS:
                    raw = _format_int(dxl_encoder_by_name.get(name))
                    print(f"  {display_name}: rad={rad}, raw_pulse={raw}")
                else:
                    raw_deg = _format_float(rmd_deg_by_name.get(name))
                    encoder = _format_int(rmd_encoder_by_name.get(name))
                    print(
                        f"  {display_name}: rad={rad}, raw_deg={raw_deg}, "
                        f"raw_encoder={encoder}")
            print()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
