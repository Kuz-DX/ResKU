#!/usr/bin/env python3
"""XY-only visual servo.  It accepts only tag_pose_filter output."""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


class VisualServoNode(Node):
    def __init__(self) -> None:
        super().__init__('visual_servo_node')
        for name, default in (
            ('valid_pose_topic', '/tag_pose_valid'), ('twist_topic', '/servo_node/delta_twist_cmds'),
            ('command_frame', ''), ('desired_tag_position_m', [0.0, 0.0, 0.20]),
            ('kp_xy', 0.8), ('max_linear_velocity_mps', 0.03), ('rate_hz', 50.0),
            ('xy_tolerance_m', 0.003), ('stable_frame_count', 8), ('detection_timeout_sec', 0.5),
        ):
            self.declare_parameter(name, default)
        self.enabled = False
        self.last = None
        self.stable = 0
        self.frame_id = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.aligned_pub = self.create_publisher(Bool, '/servo_aligned', latched)
        self.status_pub = self.create_publisher(String, '/servo_status', latched)
        self.twist_pub = self.create_publisher(TwistStamped, self.get_parameter('twist_topic').value, 10)
        self.create_subscription(String, self.get_parameter('valid_pose_topic').value, self._pose_cb, 20)
        self.create_service(SetBool, '~/enable', self._enable_cb)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self._publish_aligned(False, 'disabled')

    def _enable_cb(self, request, response):
        self.enabled = bool(request.data)
        self.stable = 0
        self.last = None
        self._stop()
        self._publish_aligned(False, 'enabled_waiting_for_filtered_tag' if self.enabled else 'disabled')
        response.success, response.message = True, 'enabled' if self.enabled else 'disabled'
        return response

    def _pose_cb(self, msg: String) -> None:
        if not self.enabled:
            return
        try:
            payload = json.loads(msg.data)
            item = payload['detections'][0]
            pos = item['position_camera_cm']
            xy = [float(pos['x']) / 100.0, float(pos['y']) / 100.0]
            frame = str(payload['frame_id'])
        except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError):
            return
        if frame and all(math.isfinite(value) for value in xy):
            self.last = (self.get_clock().now().nanoseconds * 1e-9, xy)
            self.frame_id = frame

    def _tick(self) -> None:
        if not self.enabled:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is None or now - self.last[0] > float(self.get_parameter('detection_timeout_sec').value):
            self.stable = 0
            self._stop()
            self._publish_aligned(False, 'hold:filtered_tag_lost')
            return
        xy = self.last[1]
        desired = self.get_parameter('desired_tag_position_m').value
        error = [xy[0] - float(desired[0]), xy[1] - float(desired[1])]
        norm = math.hypot(*error)
        if norm <= float(self.get_parameter('xy_tolerance_m').value):
            self.stable += 1
        else:
            self.stable = 0
        if self.stable >= int(self.get_parameter('stable_frame_count').value):
            self._stop()
            self._publish_aligned(True, 'aligned')
            return
        speed = float(self.get_parameter('kp_xy').value)
        command = [speed * error[0], speed * error[1]]
        limit = float(self.get_parameter('max_linear_velocity_mps').value)
        magnitude = math.hypot(*command)
        if magnitude > limit:
            command = [value * limit / magnitude for value in command]
        self._twist(command[0], command[1])
        self._publish_aligned(False, f'aligning:error={norm:.4f}m')

    def _twist(self, x: float, y: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('command_frame').value or self.frame_id
        msg.twist.linear.x, msg.twist.linear.y = x, y
        self.twist_pub.publish(msg)

    def _stop(self) -> None:
        self._twist(0.0, 0.0)

    def _publish_aligned(self, aligned: bool, status: str) -> None:
        self.aligned_pub.publish(Bool(data=aligned))
        self.status_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = VisualServoNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
