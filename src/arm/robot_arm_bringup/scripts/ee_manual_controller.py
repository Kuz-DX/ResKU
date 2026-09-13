#!/usr/bin/env python3
"""Convert base-frame TCP joystick motion into arm joint targets with differential IK."""

import math
import xml.etree.ElementTree as ET
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    return rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def translation_joint_velocity(jacobian, linear_velocity, damping,
                               max_joint_speed):
    """Solve translation-only DLS while preserving the commanded direction."""
    linear_jacobian = np.asarray(jacobian, dtype=float)[:3, :]
    velocity = np.asarray(linear_velocity, dtype=float)
    regularizer = (damping ** 2) * np.eye(3)
    joint_velocity = linear_jacobian.T @ np.linalg.solve(
        linear_jacobian @ linear_jacobian.T + regularizer, velocity)

    # Scaling all joints by the same factor retains the Cartesian direction.
    # Per-joint clipping can turn a vertical command into diagonal TCP motion.
    peak_speed = float(np.max(np.abs(joint_velocity)))
    if peak_speed > max_joint_speed:
        joint_velocity *= max_joint_speed / peak_speed
    return joint_velocity


class UrdfChain:
    def __init__(self, robot_description, root_link, tip_link, controlled_joints):
        joints_by_child = {}
        for element in ET.fromstring(robot_description).findall('joint'):
            child = element.find('child').attrib['link']
            origin = element.find('origin')
            xyz = tuple(map(float, origin.attrib.get('xyz', '0 0 0').split())) if origin is not None else (0.0,) * 3
            rpy = tuple(map(float, origin.attrib.get('rpy', '0 0 0').split())) if origin is not None else (0.0,) * 3
            axis = element.find('axis')
            joints_by_child[child] = {
                'name': element.attrib['name'],
                'type': element.attrib['type'],
                'parent': element.find('parent').attrib['link'],
                'origin': transform(xyz, rpy),
                'axis': tuple(map(float, axis.attrib.get('xyz', '1 0 0').split())) if axis is not None else (1.0, 0.0, 0.0),
            }
        chain = []
        link = tip_link
        while link != root_link:
            if link not in joints_by_child:
                raise ValueError(f'no URDF chain from {root_link} to {tip_link}')
            joint = joints_by_child[link]
            chain.append(joint)
            link = joint['parent']
        self.chain = list(reversed(chain))
        found = [joint['name'] for joint in self.chain if joint['name'] in controlled_joints]
        if found != list(controlled_joints):
            raise ValueError(f'URDF chain joint order {found} does not match {controlled_joints}')
        self.controlled_joints = list(controlled_joints)

    def pose_and_jacobian(self, positions):
        current = np.eye(4)
        joint_frames = []
        for joint in self.chain:
            current = current @ joint['origin']
            if joint['name'] in self.controlled_joints:
                axis_world = current[:3, :3] @ np.asarray(joint['axis'])
                joint_frames.append((current[:3, 3].copy(), axis_world))
            if (joint['type'] in ('revolute', 'continuous') and
                    joint['name'] in positions):
                current[:3, :3] = current[:3, :3] @ rotation(
                    joint['axis'], positions[joint['name']])
        tip = current[:3, 3]
        jacobian = np.zeros((6, len(joint_frames)))
        for index, (origin, axis) in enumerate(joint_frames):
            jacobian[:3, index] = np.cross(axis, tip - origin)
            jacobian[3:, index] = axis
        return current, jacobian


class ManualEeController(Node):
    JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
    IK_JOINTS = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

    def __init__(self):
        super().__init__('manual_ee_controller')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('root_link', 'base_actuator')
        # Use the URDF's visible red TCP marker as the differential-IK tip.
        # This change is local to MANUAL_EE; the other control modes keep
        # their existing end-effector definitions.
        self.declare_parameter('end_effector_link', 'tcp_link')
        self.declare_parameter('command_topic', '/manual_ee_joint_commands')
        self.declare_parameter('ready_topic', '/control/manual_ee_ready')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('linear_speed', 0.03)
        self.declare_parameter('base_yaw_speed', 0.1)
        self.declare_parameter('damping', 0.08)
        # Match the mux's most conservative arm-joint rate (elbow) so the
        # differential-IK target cannot accumulate ahead of actual execution.
        self.declare_parameter('max_joint_speed', 0.07)
        self.declare_parameter('axis_threshold', 0.15)
        self.declare_parameter('joy_timeout', 0.5)
        # MANUAL_EE translation: right stick up/down -> forward/back along the
        # operator-selected base heading, left stick up/down -> up/down.
        # L1/R1 directly jog base yaw and are intentionally outside the IK.
        self.declare_parameter('forward_axis', 4)
        self.declare_parameter('vertical_axis', 1)
        self.declare_parameter('left_button', 4)   # L1
        self.declare_parameter('right_button', 5)  # R1
        self.declare_parameter('lower_limits', [-1.46955, -1.50098, -1.63541, -1.78041])
        self.declare_parameter('upper_limits', [1.72113, 1.71548, 1.65544, 1.78041])

        description = str(self.get_parameter('robot_description').value)
        if not description.strip():
            raise ValueError(
                'robot_description is required for MANUAL_EE control')
        self.chain = UrdfChain(
            description,
            str(self.get_parameter('root_link').value),
            str(self.get_parameter('end_effector_link').value),
            self.IK_JOINTS)
        self.lower = np.asarray(self.get_parameter('lower_limits').value, dtype=float)
        self.upper = np.asarray(self.get_parameter('upper_limits').value, dtype=float)
        if len(self.lower) != len(self.JOINTS) or len(self.upper) != len(self.JOINTS):
            raise ValueError('MANUAL_EE joint limits must follow JOINTS order')
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.base_yaw_speed = float(self.get_parameter('base_yaw_speed').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_speed = float(self.get_parameter('max_joint_speed').value)
        self.threshold = float(self.get_parameter('axis_threshold').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.forward_axis = int(self.get_parameter('forward_axis').value)
        self.vertical_axis = int(self.get_parameter('vertical_axis').value)
        self.base_yaw_buttons = [
            int(self.get_parameter('left_button').value),
            int(self.get_parameter('right_button').value),
        ]

        self.positions: Dict[str, float] = {}
        self.target: Optional[np.ndarray] = None
        self.joy: Optional[Joy] = None
        self.enabled = False
        self.ready = False
        self.last_joy_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.publisher = self.create_publisher(
            JointState, str(self.get_parameter('command_topic').value), 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/control/manual_ee_enabled', self.on_enabled, latched)
        self.create_subscription(
            Bool, str(self.get_parameter('ready_topic').value),
            self.on_ready, latched)
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().warn(
            'MANUAL_EE enabled: verify joystick/tool directions at low speed '
            'before operating real hardware.')

    def on_state(self, msg):
        for name, value in zip(msg.name, msg.position):
            if name in self.JOINTS and math.isfinite(value):
                self.positions[name] = float(value)
        if (self.enabled and self.ready and self.target is None and
                all(name in self.positions for name in self.JOINTS)):
            self.target = np.asarray([self.positions[name] for name in self.JOINTS])

    def on_joy(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def on_enabled(self, msg):
        self.enabled = bool(msg.data)
        self.target = None

    def on_ready(self, msg):
        # The mux owns the position controller until its clearance/home move
        # finishes.  Re-seed differential IK from measured joints at that
        # boundary so a pre-home target can never be replayed afterwards.
        self.ready = bool(msg.data)
        self.target = None
        if (self.enabled and self.ready and
                all(name in self.positions for name in self.JOINTS)):
            self.target = np.asarray(
                [self.positions[name] for name in self.JOINTS])

    def axis(self, index):
        if self.joy is None or not 0 <= index < len(self.joy.axes):
            return 0.0
        value = float(self.joy.axes[index])
        return value if abs(value) >= self.threshold else 0.0

    def button(self, index):
        return float(
            self.joy is not None and
            0 <= index < len(self.joy.buttons) and
            self.joy.buttons[index] != 0)

    def on_timer(self):
        now = self.get_clock().now()
        dt = min(max((now - self.last_time).nanoseconds * 1e-9, 0.0), 0.1)
        self.last_time = now
        if not self.enabled or not self.ready or self.target is None:
            return
        base_twist = np.zeros(6)
        if (self.joy is not None and
                (now - self.last_joy_time).nanoseconds * 1e-9 <= self.joy_timeout):
            # Linux joy reports this right-stick axis with the opposite sign:
            # stick up must command forward along the selected base heading.
            forward_speed = -self.axis(self.forward_axis) * self.linear_speed
            base_yaw = float(self.target[0])
            base_twist[0] = math.cos(base_yaw) * forward_speed
            base_twist[1] = math.sin(base_yaw) * forward_speed
            base_twist[2] = self.axis(self.vertical_axis) * self.linear_speed
            base_yaw_velocity = (
                self.button(self.base_yaw_buttons[0]) -
                self.button(self.base_yaw_buttons[1])) * self.base_yaw_speed
        else:
            base_yaw_velocity = 0.0
        values = dict(zip(self.JOINTS, self.target))
        _, jacobian = self.chain.pose_and_jacobian(values)
        # MANUAL_EE commands TCP translation, not tool orientation.  Solving
        # the full 6-D twist with only three pitch joints implicitly demanded
        # zero angular velocity as well.  That over-constrained the arm and
        # suppressed/distorted vertical motion, most visibly while lowering.
        joint_velocity = translation_joint_velocity(
            jacobian, base_twist[:3], self.damping, self.max_joint_speed)
        target_velocity = np.concatenate(([base_yaw_velocity], joint_velocity))
        self.target = np.clip(self.target + target_velocity * dt,
                              self.lower, self.upper)
        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = list(self.JOINTS)
        message.position = self.target.tolist()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ManualEeController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
