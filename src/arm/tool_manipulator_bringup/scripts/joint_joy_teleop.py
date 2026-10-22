#!/usr/bin/env python3
"""Jog arm joints with the three RMD pitch joints on one joystick axis.

Commands are sent to MoveIt Servo as ``JointJog`` messages so Servo remains
responsible for joint limits, collision checking, smoothing, and forwarding the
resulting trajectory to ``arm_controller``.
"""

from __future__ import annotations

import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from arm_pose_safety import load_soft_limits

import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String


DEFAULT_JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
]


def coordinated_velocities(names, velocities, positions, limits, margin,
                           transition=math.radians(10.0), guard=0.01):
    """Smoothly remove each joint's contribution as its directional range runs out.

    No compensation may exceed the operator's requested per-joint speed.
    Inward travel remains available when already near/outside a soft boundary.
    """
    result = []
    for name, velocity in zip(names, velocities):
        q = positions.get(name)
        if q is None or not math.isfinite(q):
            return [0.0] * len(names)
        lower, upper = limits[name]
        remaining = upper - margin - guard - q if velocity > 0 else q - lower - margin - guard
        u = max(0.0, min(1.0, remaining / transition))
        weight = u * u * (3.0 - 2.0 * u)
        result.append(velocity * weight)
    return result


class JointJoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__("joint_joy_teleop")
        p = self.declare_parameter
        p("joy_topic", "/joy")
        p("joint_jog_topic", "/servo_node/delta_joint_cmds")
        p("command_frame", "base_actuator")
        p("joint_names", DEFAULT_JOINTS)
        p("publish_rate_hz", 50.0)
        p("joint_speed_rad_s", 0.12)
        p("joy_timeout_sec", 0.25)
        p("deadzone", 0.12)
        config = Path(get_package_share_directory("tool_manipulator_bringup")) / "config"
        p("hardware_config", str(config / "hardware.yaml"))
        p("servo_config", str(config / "servo.yaml"))
        p("joint_state_topic", "/joint_states")
        p("joint_state_timeout_sec", 0.25)
        p("base_ccw_button", 4)
        p("base_cw_button", 5)
        p("rmd_pitch_axis", 4)  # Right stick up/down controls all three RMDs.
        p("wrist_roll_axis", 6)  # D-pad left/right: positive CCW.
        p("wrist_yaw_ccw_button", 3)  # Square
        p("wrist_yaw_cw_button", 1)  # Circle
        p("manage_focus", True)
        p("focus_topic", "/control/active_target")
        p("focus_button", 9)  # Options
        p("arm_focus_value", "arm")
        p("drive_focus_value", "drive")

        self._joints = list(self.get_parameter("joint_names").value)
        self._rate = float(self.get_parameter("publish_rate_hz").value)
        self._speed = float(self.get_parameter("joint_speed_rad_s").value)
        self._timeout = float(self.get_parameter("joy_timeout_sec").value)
        self._deadzone = float(self.get_parameter("deadzone").value)
        self._button_pairs = {
            "base_joint": (
                int(self.get_parameter("base_ccw_button").value),
                int(self.get_parameter("base_cw_button").value),
            ),
            "wrist_yaw_joint": (
                int(self.get_parameter("wrist_yaw_ccw_button").value),
                int(self.get_parameter("wrist_yaw_cw_button").value),
            ),
        }
        rmd_pitch_axis = int(self.get_parameter("rmd_pitch_axis").value)
        self._axes = {
            "shoulder_joint": rmd_pitch_axis,
            "wrist_roll_joint": int(self.get_parameter("wrist_roll_axis").value),
            "elbow_joint": rmd_pitch_axis,
            "wrist_pitch_joint": rmd_pitch_axis,
        }
        self._focus_button = int(self.get_parameter("focus_button").value)
        self._manage_focus = bool(self.get_parameter("manage_focus").value)
        self._arm_focus = str(self.get_parameter("arm_focus_value").value)
        self._drive_focus = str(self.get_parameter("drive_focus_value").value)
        self._validate_parameters()

        self._limits = load_soft_limits(Path(self.get_parameter("hardware_config").value), self._joints)
        self._limit_margin = float(yaml.safe_load(Path(
            self.get_parameter("servo_config").value).read_text())["joint_limit_margin"])
        if not math.isfinite(self._limit_margin) or self._limit_margin < 0:
            raise ValueError("Servo joint_limit_margin must be finite and nonnegative")
        if any(upper - lower <= 2 * (self._limit_margin + 0.01)
               for lower, upper in self._limits.values()):
            raise ValueError("Servo margin leaves no usable joint interval")
        self._state_timeout = float(self.get_parameter("joint_state_timeout_sec").value)
        if not math.isfinite(self._state_timeout) or self._state_timeout <= 0:
            raise ValueError("joint_state_timeout_sec must be finite and positive")
        self._positions = {}
        self._last_state_time = None
        self.create_subscription(JointState, self.get_parameter("joint_state_topic").value,
                                 self._state_cb, 20)
        self._joy: Joy | None = None
        self._last_joy_time = self.get_clock().now()
        self._previous_focus_button = False
        self._commanding = False
        # Fail safe: arm commands remain disabled until this node owns focus or
        # an external focus manager explicitly publishes the arm value.
        self._active_target = self._drive_focus if self._manage_focus else ""

        focus_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._focus_publisher = None
        if self._manage_focus:
            self._focus_publisher = self.create_publisher(
                String, self.get_parameter("focus_topic").value, focus_qos)
        self.create_subscription(
            String,
            self.get_parameter("focus_topic").value,
            self._focus_cb,
            focus_qos,
        )
        self.create_subscription(
            Joy,
            self.get_parameter("joy_topic").value,
            self._joy_cb,
            20,
        )
        self._publisher = self.create_publisher(
            JointJog, self.get_parameter("joint_jog_topic").value, 10)
        self.create_timer(1.0 / self._rate, self._publish_command)

        if self._manage_focus:
            self._publish_focus(self._drive_focus)
            activation_hint = f"press focus button {self._focus_button} to select arm"
        else:
            activation_hint = (
                f"waiting for {self.get_parameter('focus_topic').value}="
                f"{self._arm_focus}"
            )
        self.get_logger().info(
            "Joint joystick teleop ready but inactive: "
            f"{activation_hint}; grouped RMD pitch axis {rmd_pitch_axis} enabled"
        )

    def _validate_parameters(self) -> None:
        if not self._joints or any(not name for name in self._joints):
            raise ValueError("joint_names must contain at least one non-empty name")
        if len(set(self._joints)) != len(self._joints):
            raise ValueError("joint_names must not contain duplicates")
        for label, value in (
            ("publish_rate_hz", self._rate),
            ("joint_speed_rad_s", self._speed),
            ("joy_timeout_sec", self._timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be finite and positive")
        if not math.isfinite(self._deadzone) or not 0.0 <= self._deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        indexed_inputs = [("focus_button", self._focus_button)]
        indexed_inputs.extend(
            (f"{joint}_{direction}_button", index)
            for joint, pair in self._button_pairs.items()
            for direction, index in zip(("ccw", "cw"), pair)
        )
        indexed_inputs.extend(
            (f"{joint}_axis", index) for joint, index in self._axes.items()
        )
        for label, value in indexed_inputs:
            if value < 0:
                raise ValueError(f"{label} must be nonnegative")
        mapped_joints = set(self._button_pairs) | set(self._axes)
        if mapped_joints != set(self._joints):
            raise ValueError("joint_names must match the configured button/axis mappings")
        if not self._arm_focus or not self._drive_focus:
            raise ValueError("arm_focus_value and drive_focus_value must be non-empty")
        if self._arm_focus == self._drive_focus:
            raise ValueError("arm_focus_value and drive_focus_value must differ")

    @staticmethod
    def _button(message: Joy, index: int) -> bool:
        return index < len(message.buttons) and bool(message.buttons[index])

    @staticmethod
    def _axis(message: Joy, index: int) -> float:
        if index >= len(message.axes):
            return 0.0
        value = float(message.axes[index])
        return value if math.isfinite(value) else 0.0

    def _publish_focus(self, target: str) -> None:
        self._active_target = target
        if self._focus_publisher is not None:
            self._focus_publisher.publish(String(data=target))
        self.get_logger().info(f"Joystick focus: {target}")

    def _focus_cb(self, message: String) -> None:
        previous = self._active_target
        self._active_target = message.data
        if previous == self._arm_focus and message.data != self._arm_focus:
            self._publish_halt()

    def _joy_cb(self, message: Joy) -> None:
        self._joy = message
        self._last_joy_time = self.get_clock().now()

        focus_pressed = self._button(message, self._focus_button)
        if self._manage_focus and focus_pressed and not self._previous_focus_button:
            target = (
                self._drive_focus
                if self._active_target == self._arm_focus
                else self._arm_focus
            )
            self._publish_focus(target)
            if target != self._arm_focus:
                self._publish_halt()
        self._previous_focus_button = focus_pressed

    def _fresh_joy(self) -> bool:
        if self._joy is None:
            return False
        age = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
        return age <= self._timeout

    def _make_command(self, velocities: list[float]) -> JointJog:
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("command_frame").value)
        message.joint_names = self._joints
        message.velocities = velocities
        message.duration = 1.0 / self._rate
        return message

    def _button_velocity(self, message: Joy, ccw: int, cw: int) -> float:
        return self._speed * (float(self._button(message, ccw))
                              - float(self._button(message, cw)))

    def _axis_velocity(self, message: Joy, index: int) -> float:
        value = self._axis(message, index)
        if abs(value) < self._deadzone:
            return 0.0
        return self._speed * value

    def _velocities(self, message: Joy) -> list[float]:
        velocities = []
        for joint in self._joints:
            if joint in self._button_pairs:
                velocities.append(
                    self._button_velocity(message, *self._button_pairs[joint]))
            else:
                velocity = self._axis_velocity(message, self._axes[joint])
                # Preserve the original per-joint directions while sharing input.
                velocities.append(-velocity if joint == "elbow_joint" else velocity)
        return velocities

    def _publish_halt(self) -> None:
        if self._commanding:
            self._publisher.publish(self._make_command([0.0] * len(self._joints)))
            self._commanding = False

    def _state_cb(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in self._joints):
            self._positions = {}
            self._last_state_time = None
            return
        self._positions = positions
        self._last_state_time = self.get_clock().now()

    def _publish_command(self) -> None:
        if (self._active_target != self._arm_focus
                or not self._fresh_joy()
                or self._joy is None):
            self._publish_halt()
            return

        if (self._last_state_time is None or
                (self.get_clock().now() - self._last_state_time).nanoseconds * 1e-9 > self._state_timeout):
            self._publish_halt()
            return
        velocities = coordinated_velocities(
            self._joints, self._velocities(self._joy), self._positions,
            self._limits, self._limit_margin)
        self._publisher.publish(self._make_command(velocities))
        self._commanding = any(velocity != 0.0 for velocity in velocities)


def main() -> None:
    rclpy.init()
    node = JointJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_halt()  # Best-effort stop before Servo's own timeout.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
