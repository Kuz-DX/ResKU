#!/usr/bin/env python3
"""Provide gripper commands in both operator-controlled arm modes."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, String


class ManualGripperController(Node):
    ACTIVE_MODES = ('MANUAL_100', 'MANUAL_EE')
    JOINT = 'gripper_joint'

    def __init__(self):
        super().__init__('manual_gripper_controller')
        self.declare_parameter(
            'command_topic', '/manual_gripper_command')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('rate', 0.2)
        self.declare_parameter('lower_limit', 0.0)
        self.declare_parameter('upper_limit', 2.59396)
        self.declare_parameter('positive_button', 2)
        self.declare_parameter('negative_button', 0)
        self.declare_parameter('joy_timeout', 0.5)

        self.rate = float(self.get_parameter('rate').value)
        self.lower = float(self.get_parameter('lower_limit').value)
        self.upper = float(self.get_parameter('upper_limit').value)
        if not all(math.isfinite(v) for v in (self.rate, self.lower, self.upper)):
            raise ValueError('gripper rate and limits must be finite')
        if self.rate <= 0.0 or self.lower >= self.upper:
            raise ValueError('invalid gripper rate or limits')
        self.positive_button = int(
            self.get_parameter('positive_button').value)
        self.negative_button = int(
            self.get_parameter('negative_button').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)

        self.mode = 'OFF'
        self.measured_position: Optional[float] = None
        self.target: Optional[float] = None
        self.joy: Optional[Joy] = None
        self.last_joy_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            Float64, self.get_parameter('command_topic').value, 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '/control/mode', self.on_mode, state_qos)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        self.create_timer(1.0 / publish_rate, self.on_timer)

    def on_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        for name, value in zip(msg.name, msg.position):
            if name == self.JOINT and math.isfinite(value):
                self.measured_position = float(value)
                if self.target is None:
                    self.target = self.measured_position
                    self.get_logger().info(
                        'Gripper target synchronized from /joint_states.')
                return

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def on_mode(self, msg):
        requested = msg.data.strip().upper()
        was_active = self.mode in self.ACTIVE_MODES
        will_be_active = requested in self.ACTIVE_MODES
        self.mode = requested
        if will_be_active and not was_active:
            # Entering the arm from DRIVE always starts at the encoder position.
            self.target = self.measured_position
            self.get_logger().info(
                'Arm mode entered; gripper target resynchronized.')

    def button(self, index):
        return (
            self.joy is not None
            and 0 <= index < len(self.joy.buttons)
            and self.joy.buttons[index] != 0
        )

    def on_timer(self):
        now = self.get_clock().now()
        dt = min(
            max((now - self.last_time).nanoseconds * 1e-9, 0.0),
            0.1,
        )
        self.last_time = now
        if self.mode not in self.ACTIVE_MODES or self.target is None:
            return

        direction = 0.0
        joy_age = (now - self.last_joy_time).nanoseconds * 1e-9
        if self.joy is not None and joy_age <= self.joy_timeout:
            direction = (
                float(self.button(self.positive_button))
                - float(self.button(self.negative_button))
            )
        value = self.target + direction * self.rate * dt
        self.target = min(max(value, self.lower), self.upper)
        self.publisher.publish(Float64(data=self.target))


def main(args=None):
    rclpy.init(args=args)
    node = ManualGripperController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
