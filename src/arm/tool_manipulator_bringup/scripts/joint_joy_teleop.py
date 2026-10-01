#!/usr/bin/env python3
"""Jog one arm joint at a time from a shared remote joystick.

Commands are sent to MoveIt Servo as ``JointJog`` messages so Servo remains
responsible for joint limits, collision checking, smoothing, and forwarding the
resulting trajectory to ``arm_controller``.
"""

from __future__ import annotations

import math

import rclpy
from control_msgs.msg import JointJog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


DEFAULT_JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
]


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
        p("deadman_button", 4)  # L1
        p("jog_axis", 4)  # Right stick vertical
        p("invert_jog_axis", True)
        p("select_axis", 6)  # D-pad horizontal: left/right
        p("select_threshold", 0.5)
        p("invert_select_axis", False)
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
        self._select_threshold = float(self.get_parameter("select_threshold").value)
        self._deadman_button = int(self.get_parameter("deadman_button").value)
        self._jog_axis = int(self.get_parameter("jog_axis").value)
        self._select_axis = int(self.get_parameter("select_axis").value)
        self._focus_button = int(self.get_parameter("focus_button").value)
        self._manage_focus = bool(self.get_parameter("manage_focus").value)
        self._arm_focus = str(self.get_parameter("arm_focus_value").value)
        self._drive_focus = str(self.get_parameter("drive_focus_value").value)
        self._validate_parameters()

        self._selected = 0
        self._joy: Joy | None = None
        self._last_joy_time = self.get_clock().now()
        self._previous_focus_button = False
        self._selection_latched = False
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
            f"{activation_hint}; selected joint={self._joint_name()}"
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
        if (not math.isfinite(self._select_threshold)
                or not 0.0 < self._select_threshold <= 1.0):
            raise ValueError("select_threshold must be in (0, 1]")
        for label, value in (
            ("deadman_button", self._deadman_button),
            ("jog_axis", self._jog_axis),
            ("select_axis", self._select_axis),
            ("focus_button", self._focus_button),
        ):
            if value < 0:
                raise ValueError(f"{label} must be nonnegative")
        if not self._arm_focus or not self._drive_focus:
            raise ValueError("arm_focus_value and drive_focus_value must be non-empty")
        if self._arm_focus == self._drive_focus:
            raise ValueError("arm_focus_value and drive_focus_value must differ")

    def _joint_name(self) -> str:
        return self._joints[self._selected]

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

        if self._active_target != self._arm_focus:
            self._selection_latched = False
            return

        select_value = self._axis(message, self._select_axis)
        if bool(self.get_parameter("invert_select_axis").value):
            select_value = -select_value
        selecting = abs(select_value) >= self._select_threshold
        deadman_held = self._button(message, self._deadman_button)
        if not deadman_held:
            # Stop the previous joint before selection can change the joint
            # name placed in the halt message.
            self._publish_halt()
        if selecting and not self._selection_latched and not deadman_held:
            # Linux joydev reports D-pad right as -1 for the established pad.
            step = 1 if select_value < 0.0 else -1
            self._selected = (self._selected + step) % len(self._joints)
            self.get_logger().info(
                f"Selected joint {self._selected + 1}/{len(self._joints)}: "
                f"{self._joint_name()}"
            )
        self._selection_latched = selecting

    def _fresh_joy(self) -> bool:
        if self._joy is None:
            return False
        age = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
        return age <= self._timeout

    def _make_command(self, velocity: float) -> JointJog:
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("command_frame").value)
        message.joint_names = [self._joint_name()]
        message.velocities = [velocity]
        message.duration = 1.0 / self._rate
        return message

    def _publish_halt(self) -> None:
        if self._commanding:
            self._publisher.publish(self._make_command(0.0))
            self._commanding = False

    def _publish_command(self) -> None:
        if (self._active_target != self._arm_focus
                or not self._fresh_joy()
                or self._joy is None
                or not self._button(self._joy, self._deadman_button)):
            self._publish_halt()
            return

        value = self._axis(self._joy, self._jog_axis)
        if abs(value) < self._deadzone:
            value = 0.0
        if bool(self.get_parameter("invert_jog_axis").value):
            value = -value
        self._publisher.publish(self._make_command(self._speed * value))
        self._commanding = True


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
