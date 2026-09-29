#!/usr/bin/env python3
"""Mock-only TCP reachability test for fixed tool-docking fixtures.

This node deliberately does not run AprilTag feedback, Servo descent, or lock
rotation.  It answers one commissioning question: after a UI tool ID arrives,
can MoveIt plan the no-tool TCP to the taught fixture entry point without
colliding with the robot's fixed chassis, electronics box, or dock fixture?
"""
import threading

import rclpy
import yaml
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String


class ToolDockTcpSimulation(Node):
    def __init__(self):
        super().__init__('tool_dock_tcp_sim')
        self.callback_group = ReentrantCallbackGroup()
        p = self.declare_parameter
        p('tool_id_topic', '/selected_tool_id')
        p('planning_group', 'arm')
        p('base_link', 'base_actuator')
        # tcp_link is the common yaw-output flange before a tool is attached.
        # Keeping it explicit makes the test's intent match future tool TCP use.
        p('tcp_link', 'tcp_link')
        p('joint_names', ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_yaw_joint'])
        p('target_position_tolerance_m', 0.002)
        # With six arm DOF, docking teaching first proves position.  A
        # pi orientation tolerance turns the pose request into a position goal.
        p('orientation_tolerance_rad', 3.14159265)
        p('planning_timeout_sec', 20.0)
        p('max_velocity', 0.15)
        p('max_acceleration', 0.15)
        p('target_poses_yaml',
          '{2: {frame_id: arm_world_frame, position: [-0.268, -0.10245, 0.187]}}')

        self.targets = self._read_targets()
        self.busy = False
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=list(self.get_parameter('joint_names').value),
            base_link_name=self.get_parameter('base_link').value,
            end_effector_name=self.get_parameter('tcp_link').value,
            group_name=self.get_parameter('planning_group').value,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = float(self.get_parameter('max_velocity').value)
        self.moveit2.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.status_pub = self.create_publisher(
            String, '/tool_dock_tcp_sim/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            Int32, self.get_parameter('tool_id_topic').value, self._tool_id_cb, 10,
            callback_group=self.callback_group)
        self._status('ready')

    def _read_targets(self):
        try:
            raw = yaml.safe_load(self.get_parameter('target_poses_yaml').value) or {}
            targets = {}
            for tool_id, target in raw.items():
                position = [float(v) for v in target['position']]
                if len(position) != 3:
                    raise ValueError('position must contain x, y, z')
                targets[int(tool_id)] = {
                    'frame_id': str(target.get('frame_id', 'arm_world_frame')),
                    'position': position,
                    'quat_xyzw': [float(v) for v in target.get('quat_xyzw', [0.0, 0.0, 0.0, 1.0])],
                }
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError('target_poses_yaml must map ID to frame_id and [x,y,z] position') from exc
        if any(len(target['quat_xyzw']) != 4 for target in targets.values()):
            raise RuntimeError('every quat_xyzw must contain four values')
        return targets

    def _tool_id_cb(self, msg):
        if self.busy:
            self.get_logger().warning('Ignoring tool ID while a TCP test is running')
            return
        target = self.targets.get(msg.data)
        if target is None:
            self.get_logger().info(f'No TCP simulation target for tool ID {msg.data}')
            return
        self.busy = True
        threading.Thread(target=self._plan_and_execute, args=(msg.data, target), daemon=True).start()

    def _plan_and_execute(self, tool_id, target):
        self._status(f'planning:tool_{tool_id}')
        self.get_logger().info(
            f"Tool {tool_id}: TCP -> {target['position']} in {target['frame_id']}")
        try:
            accepted = self.moveit2.move_to_pose(
                position=target['position'],
                quat_xyzw=target['quat_xyzw'],
                frame_id=target['frame_id'],
                target_link=self.get_parameter('tcp_link').value,
                tolerance_position=float(self.get_parameter('target_position_tolerance_m').value),
                tolerance_orientation=float(self.get_parameter('orientation_tolerance_rad').value),
                timeout_sec=float(self.get_parameter('planning_timeout_sec').value),
            )
            success = bool(accepted) and bool(self.moveit2.wait_until_executed(
                timeout_sec=float(self.get_parameter('planning_timeout_sec').value)))
        except Exception as exc:  # Planning failures are expected during teaching.
            self.get_logger().error(f'Tool {tool_id} TCP test failed: {exc}')
            success = False
        self._status(f'complete:tool_{tool_id}' if success else f'failed:tool_{tool_id}')
        self.busy = False

    def _status(self, text):
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = ToolDockTcpSimulation()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
