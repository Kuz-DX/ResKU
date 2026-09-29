#!/usr/bin/env python3
"""Convert joystick axes to deadman-gated TCP velocity commands for MoveIt Servo.

The node only publishes a Cartesian TwistStamped; MoveIt Servo performs the
inverse kinematics and sends the resulting JointTrajectory to arm_controller.
All commands are expressed in base_actuator, so joystick directions remain
fixed in the robot base frame.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TcpJoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__("tcp_joy_teleop")
        p = self.declare_parameter
        p("joy_topic", "/joy")
        p("twist_topic", "/servo_node/delta_twist_cmds")
        p("command_frame", "base_actuator")
        p("publish_rate_hz", 50.0)
        p("deadman_button", 4)  # L1 by the existing Logitech mapping.
        p("x_axis", 4)          # Right stick vertical: TCP forward/back.
        p("y_axis", 3)          # Right stick horizontal: TCP left/right.
        p("z_axis", 1)          # Left stick vertical: TCP up/down.
        p("linear_speed_mps", 0.03)
        p("deadzone", 0.12)
        p("joy_timeout_sec", 0.25)
        p("invert_x", True)
        p("invert_y", False)
        p("invert_z", False)

        self._joy = None
        self._last_joy_time = self.get_clock().now()
        self._publisher = self.create_publisher(
            TwistStamped, self.get_parameter("twist_topic").value, 10)
        self.create_subscription(Joy, self.get_parameter("joy_topic").value, self._joy_cb, 20)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.create_timer(1.0 / rate, self._publish_twist)
        self.get_logger().info(
            "TCP joystick teleop ready: hold deadman button "
            f"{self.get_parameter('deadman_button').value}; axes "
            f"x={self.get_parameter('x_axis').value} "
            f"y={self.get_parameter('y_axis').value} "
            f"z={self.get_parameter('z_axis').value}")

    def _joy_cb(self, message: Joy) -> None:
        self._joy = message
        self._last_joy_time = self.get_clock().now()

    def _axis(self, index: int, invert: bool) -> float:
        if self._joy is None or not 0 <= index < len(self._joy.axes):
            return 0.0
        value = float(self._joy.axes[index])
        if abs(value) < float(self.get_parameter("deadzone").value):
            return 0.0
        return -value if invert else value

    def _deadman_held(self) -> bool:
        if self._joy is None:
            return False
        index = int(self.get_parameter("deadman_button").value)
        return 0 <= index < len(self._joy.buttons) and bool(self._joy.buttons[index])

    def _publish_twist(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter("command_frame").value
        fresh = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9 <= float(
            self.get_parameter("joy_timeout_sec").value)
        if fresh and self._deadman_held():
            speed = float(self.get_parameter("linear_speed_mps").value)
            message.twist.linear.x = speed * self._axis(
                int(self.get_parameter("x_axis").value), self.get_parameter("invert_x").value)
            message.twist.linear.y = speed * self._axis(
                int(self.get_parameter("y_axis").value), self.get_parameter("invert_y").value)
            message.twist.linear.z = speed * self._axis(
                int(self.get_parameter("z_axis").value), self.get_parameter("invert_z").value)
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = TcpJoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
