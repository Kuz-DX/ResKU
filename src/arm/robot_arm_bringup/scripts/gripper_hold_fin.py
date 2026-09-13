#!/usr/bin/env python3
"""Publish whether the manual-EE gripper has reached its holding condition."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def effort_exceeds_threshold(effort, threshold):
    """Return true for a finite gripper effort whose magnitude exceeds threshold."""
    return math.isfinite(effort) and abs(float(effort)) > threshold


def position_reaches_threshold(position, threshold):
    """Return true when a finite gripper position reaches the close threshold."""
    return math.isfinite(position) and float(position) >= threshold


def grasp_is_complete(position, position_threshold, effort, effort_threshold):
    """Use position as the primary completion signal and effort as a fallback."""
    return (
        position_reaches_threshold(position, position_threshold)
        or effort_exceeds_threshold(effort, effort_threshold)
    )


class GripperHoldFin(Node):
    def __init__(self):
        super().__init__('gripper_hold_fin')
        self.declare_parameter('joint_name', 'gripper_joint')
        self.declare_parameter('position_threshold', 1.8294)
        self.declare_parameter('effort_threshold', 130.0)
        self.declare_parameter('output_topic', '/gripper_hold_fin')

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.position_threshold = float(
            self.get_parameter('position_threshold').value)
        self.threshold = float(self.get_parameter('effort_threshold').value)
        if not math.isfinite(self.position_threshold):
            raise ValueError('position_threshold must be finite')
        if not math.isfinite(self.threshold) or self.threshold < 0.0:
            raise ValueError('effort_threshold must be finite and non-negative')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Bool, str(self.get_parameter('output_topic').value), latched_qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(String, '/control/mode', self.on_mode, latched_qos)

        self.manual_ee_enabled = False
        self.hold_finished = None
        self.publish_status(False)

    def publish_status(self, finished):
        finished = bool(finished)
        if self.hold_finished == finished:
            return
        self.hold_finished = finished
        self.publisher.publish(Bool(data=finished))

    def on_mode(self, msg):
        self.manual_ee_enabled = msg.data.strip().upper() == 'MANUAL_EE'
        if not self.manual_ee_enabled:
            self.publish_status(False)

    def on_state(self, msg):
        if not self.manual_ee_enabled:
            return
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return
        if index >= len(msg.position):
            return
        # Position is the primary MANUAL_EE completion signal.  Keep the
        # existing effort threshold as a fallback for a loaded gripper that
        # stops before reaching the configured close position.
        effort = msg.effort[index] if index < len(msg.effort) else math.nan
        self.publish_status(
            grasp_is_complete(
                msg.position[index], self.position_threshold,
                effort, self.threshold))


def main(args=None):
    rclpy.init(args=args)
    node = GripperHoldFin()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
