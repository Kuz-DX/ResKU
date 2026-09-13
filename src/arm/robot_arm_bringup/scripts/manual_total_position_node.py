#!/usr/bin/env python3
"""Generate manual targets for the four arm-pose joints.

The gripper is intentionally handled by manual_gripper_controller.py so the
same gripper controls remain available in both MANUAL_100 and MANUAL_EE modes.
"""

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool


class ManualTotalPositionNode(Node):
    JOINTS = [
        'shoulder_joint',
        'elbow_joint',
        'wrist_joint',
        'base_joint',
    ]

    def __init__(self):
        super().__init__('manual_total_position_node')
        self.declare_parameter('command_topic', '/manual_joint_commands')
        self.declare_parameter('publish_rate', 20.0)
        # Order: shoulder, elbow, wrist, base.  Elbow hardware is configured
        # for 5 deg/s (0.0873 rad/s), so command it at 0.07 rad/s to retain
        # tracking margin under load.
        self.declare_parameter('rates', [0.1, 0.07, 0.1, 0.1])
        self.declare_parameter(
            'lower_limits', [-1.50098, -1.63541, -1.78041, -1.46955])
        self.declare_parameter(
            'upper_limits', [1.71548, 1.65544, 1.78041, 1.72113])
        # Linux PlayStation mapping: right stick vertical=4, left stick
        # vertical=1, D-pad vertical=7, L1=4, R1=5. Positive stick/D-pad
        # direction increases the corresponding URDF joint coordinate.
        self.declare_parameter('shoulder_axis', 4)
        self.declare_parameter('elbow_axis', 1)
        self.declare_parameter('wrist_axis', 7)
        self.declare_parameter('base_yaw_positive_button', 4)
        self.declare_parameter('base_yaw_negative_button', 5)
        self.declare_parameter('axis_threshold', 0.5)
        self.declare_parameter('joy_timeout', 0.5)

        self.rates = self.float_array('rates')
        self.lower = self.float_array('lower_limits')
        self.upper = self.float_array('upper_limits')
        if not (
            len(self.rates) == len(self.lower) == len(self.upper)
            == len(self.JOINTS)
        ):
            raise ValueError(
                'rates/lower_limits/upper_limits must follow JOINTS order')
        for index, name in enumerate(self.JOINTS):
            values = (self.rates[index], self.lower[index], self.upper[index])
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'{name} manual parameters must be finite')
            if self.rates[index] <= 0.0:
                raise ValueError(f'{name} rate must be positive')
            if self.lower[index] >= self.upper[index]:
                raise ValueError(f'{name} lower limit must be below upper limit')

        self.shoulder_axis = int(self.get_parameter('shoulder_axis').value)
        self.elbow_axis = int(self.get_parameter('elbow_axis').value)
        self.wrist_axis = int(self.get_parameter('wrist_axis').value)
        self.base_positive_button = int(
            self.get_parameter('base_yaw_positive_button').value)
        self.base_negative_button = int(
            self.get_parameter('base_yaw_negative_button').value)
        self.axis_threshold = float(
            self.get_parameter('axis_threshold').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)

        self.positions: Dict[str, float] = {}
        self.target: Optional[list] = None
        self.joy: Optional[Joy] = None
        self.manual_100_enabled = False
        self.last_joy_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            JointState, self.get_parameter('command_topic').value, 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            '/control/manual_100_enabled',
            self.on_manual_100_enabled,
            state_qos,
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        self.create_timer(1.0 / publish_rate, self.on_timer)

    def float_array(self, name):
        return list(map(float, self.get_parameter(name).value))

    def on_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(value):
                self.positions[name] = float(value)
        if self.target is None and all(
            joint in self.positions for joint in self.JOINTS
        ):
            self.target = [self.positions[joint] for joint in self.JOINTS]
            self.get_logger().info(
                'Manual arm targets synchronized from /joint_states.')

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def on_manual_100_enabled(self, msg):
        enabled = bool(msg.data)
        if enabled and not self.manual_100_enabled:
            self.target = None
            self.get_logger().info(
                'MANUAL_100 enabled; waiting to resynchronize arm targets.')
        self.manual_100_enabled = enabled

    def axis(self, index):
        if self.joy is None or not 0 <= index < len(self.joy.axes):
            return 0.0
        value = self.joy.axes[index]
        if value > self.axis_threshold:
            return 1.0
        if value < -self.axis_threshold:
            return -1.0
        return 0.0

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
        if self.target is None or not self.manual_100_enabled:
            return

        directions = [0.0] * len(self.JOINTS)
        joy_age = (now - self.last_joy_time).nanoseconds * 1e-9
        if self.joy is not None and joy_age <= self.joy_timeout:
            directions = [
                self.axis(self.shoulder_axis),
                self.axis(self.elbow_axis),
                self.axis(self.wrist_axis),
                float(self.button(self.base_positive_button))
                - float(self.button(self.base_negative_button)),
            ]
        for index, direction in enumerate(directions):
            value = self.target[index] + direction * self.rates[index] * dt
            self.target[index] = min(
                max(value, self.lower[index]),
                self.upper[index],
            )

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = list(self.JOINTS)
        message.position = list(self.target)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ManualTotalPositionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
