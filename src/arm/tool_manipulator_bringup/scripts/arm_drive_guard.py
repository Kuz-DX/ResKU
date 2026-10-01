#!/usr/bin/env python3
"""Publish whether the arm is stably inside the configured drive-safe pose.

This node is a monitor, not a motion planner and not a vehicle command mux.
Consumers must treat a missing or false readiness topic as "driving forbidden".
"""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from arm_pose_safety import load_soft_limits, pose_errors, validate_joint_positions


ARM_JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
)


def load_named_pose(srdf_path: Path, pose_name: str) -> dict[str, float]:
    root = ET.parse(srdf_path).getroot()
    matches = [
        state for state in root.findall("group_state[@group='arm']")
        if state.get("name") == pose_name
    ]
    if len(matches) != 1:
        raise ValueError(
            f"{srdf_path}: expected exactly one arm pose named '{pose_name}', "
            f"found {len(matches)}"
        )
    values = {
        joint.attrib["name"]: float(joint.attrib["value"])
        for joint in matches[0].findall("joint")
    }
    missing = [name for name in ARM_JOINTS if name not in values]
    extra = sorted(set(values).difference(ARM_JOINTS))
    if missing or extra:
        raise ValueError(f"invalid '{pose_name}' joints: missing={missing}, extra={extra}")
    return {name: values[name] for name in ARM_JOINTS}


class ArmDriveGuard(Node):
    def __init__(self) -> None:
        super().__init__("arm_drive_guard")
        self.declare_parameter("pose_name", "drive_safe")
        self.declare_parameter("srdf_path", "")
        self.declare_parameter("hardware_path", "")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ready_topic", "/arm/drive_safe")
        self.declare_parameter("state_topic", "/arm/drive_guard_state")
        self.declare_parameter("position_tolerance_rad", 0.03)
        self.declare_parameter("stopped_velocity_rad_s", 0.02)
        self.declare_parameter("limit_margin_rad", 0.02)
        self.declare_parameter("max_state_age_sec", 0.5)
        self.declare_parameter("stable_sample_count", 5)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.pose_name = str(self.get_parameter("pose_name").value)
        tolerance = float(self.get_parameter("position_tolerance_rad").value)
        self.stopped_velocity = float(self.get_parameter("stopped_velocity_rad_s").value)
        margin = float(self.get_parameter("limit_margin_rad").value)
        self.max_state_age = float(self.get_parameter("max_state_age_sec").value)
        self.required_samples = int(self.get_parameter("stable_sample_count").value)
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if not self.pose_name:
            raise ValueError("pose_name must not be empty")
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("position_tolerance_rad must be finite and positive")
        if not math.isfinite(self.stopped_velocity) or self.stopped_velocity < 0.0:
            raise ValueError("stopped_velocity_rad_s must be finite and nonnegative")
        if not math.isfinite(self.max_state_age) or self.max_state_age <= 0.0:
            raise ValueError("max_state_age_sec must be finite and positive")
        if self.required_samples <= 0:
            raise ValueError("stable_sample_count must be positive")
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.tolerance = tolerance

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), qos)
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), qos)

        self.target: dict[str, float] | None = None
        self.limits: dict[str, tuple[float, float]] | None = None
        self.config_error: str | None = None
        try:
            srdf_raw = str(self.get_parameter("srdf_path").value)
            hardware_raw = str(self.get_parameter("hardware_path").value)
            srdf = Path(srdf_raw) if srdf_raw else Path(get_package_share_directory(
                "tool_manipulator_moveit_config")) / "config" / "tool_manipulator.srdf"
            hardware = Path(hardware_raw) if hardware_raw else Path(get_package_share_directory(
                "tool_manipulator_bringup")) / "config" / "hardware.yaml"
            self.target = load_named_pose(srdf, self.pose_name)
            self.limits = load_soft_limits(hardware, ARM_JOINTS)
            validate_joint_positions(
                self.target, self.limits, margin, label=f"target pose '{self.pose_name}'")
        except (ET.ParseError, OSError, TypeError, ValueError) as exc:
            self.config_error = str(exc)

        self.latest_positions: dict[str, float] | None = None
        self.latest_received_at: float | None = None
        self.latest_limit_error: str | None = None
        self.latest_motion_error: str | None = None
        self.latest_errors: dict[str, float] | None = None
        self.stable_samples = 0
        self.last_state = ""
        self.last_state_code = ""

        topic = str(self.get_parameter("joint_state_topic").value)
        self.create_subscription(JointState, topic, self._on_joint_state, 20)
        self.create_timer(1.0 / publish_rate, self._publish)
        if self.config_error:
            self.get_logger().error(f"drive guard configuration invalid: {self.config_error}")
        else:
            self.get_logger().info(
                f"monitoring pose '{self.pose_name}' on {topic}; "
                f"tolerance={self.tolerance:.3f} rad, stable_samples={self.required_samples}"
            )

    def _on_joint_state(self, message: JointState) -> None:
        observed = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }
        observed_velocity = {
            name: float(velocity)
            for name, velocity in zip(message.name, message.velocity)
        }
        if not all(name in observed for name in ARM_JOINTS):
            self.latest_positions = None
            self.latest_received_at = time.monotonic()
            self.latest_limit_error = "incomplete /joint_states"
            self.latest_motion_error = None
            self.latest_errors = None
            self.stable_samples = 0
            return

        current = {name: observed[name] for name in ARM_JOINTS}
        self.latest_positions = current
        self.latest_received_at = time.monotonic()
        if self.config_error or self.target is None or self.limits is None:
            self.stable_samples = 0
            return
        try:
            validate_joint_positions(current, self.limits, 0.0, label="current arm state")
            self.latest_limit_error = None
            self.latest_errors = pose_errors(current, self.target, ARM_JOINTS)
        except ValueError as exc:
            self.latest_limit_error = str(exc)
            self.latest_errors = None
            self.stable_samples = 0
            return

        if not all(name in observed_velocity for name in ARM_JOINTS):
            self.latest_motion_error = "joint velocities are missing"
        elif not all(math.isfinite(observed_velocity[name]) for name in ARM_JOINTS):
            self.latest_motion_error = "joint velocities contain a non-finite value"
        else:
            moving_joint, speed = max(
                ((name, abs(observed_velocity[name])) for name in ARM_JOINTS),
                key=lambda item: item[1],
            )
            self.latest_motion_error = (
                f"{moving_joint}_velocity={speed:.6f}rad/s"
                if speed > self.stopped_velocity else None
            )

        if (self.latest_motion_error is None and
                max(self.latest_errors.values()) <= self.tolerance):
            self.stable_samples = min(self.stable_samples + 1, self.required_samples)
        else:
            self.stable_samples = 0

    def _publish(self) -> None:
        ready = False
        if self.config_error:
            state_code = "CONFIG_ERROR"
            state = f"CONFIG_ERROR: {self.config_error}"
        elif self.latest_received_at is None:
            state_code = "WAITING_JOINT_STATE"
            state = "WAITING_JOINT_STATE"
        else:
            age = time.monotonic() - self.latest_received_at
            if age > self.max_state_age:
                self.stable_samples = 0
                state_code = "STALE_JOINT_STATE"
                state = f"STALE_JOINT_STATE: age={age:.3f}s"
            elif self.latest_limit_error:
                state_code = "INVALID_JOINT_STATE"
                state = f"INVALID_JOINT_STATE: {self.latest_limit_error}"
            elif self.latest_motion_error:
                state_code = "ARM_NOT_STOPPED"
                state = f"ARM_NOT_STOPPED: {self.latest_motion_error}"
            elif self.latest_errors is None:
                state_code = "WAITING_COMPLETE_JOINT_STATE"
                state = "WAITING_COMPLETE_JOINT_STATE"
            elif self.stable_samples < self.required_samples:
                joint, error = max(self.latest_errors.items(), key=lambda item: item[1])
                state_code = "NOT_DRIVE_SAFE"
                state = (
                    f"NOT_DRIVE_SAFE: {joint}_error={error:.6f}rad "
                    f"stable={self.stable_samples}/{self.required_samples}"
                )
            else:
                ready = True
                state_code = "DRIVE_SAFE"
                state = f"DRIVE_SAFE: pose={self.pose_name}"

        self.ready_pub.publish(Bool(data=ready))
        if state != self.last_state:
            self.state_pub.publish(String(data=state))
            self.last_state = state
        if state_code != self.last_state_code:
            if ready:
                self.get_logger().info(state)
            elif self.last_state_code:
                self.get_logger().warn(state)
            self.last_state_code = state_code


def main() -> None:
    rclpy.init()
    node = ArmDriveGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
