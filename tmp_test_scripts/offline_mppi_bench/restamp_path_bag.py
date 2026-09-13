#!/usr/bin/env python3
"""restamp_path_bag.py -- [테스트용] rosbag 재생이 내는 /path의 header.stamp가
녹화 당시 시각 그대로라, 재생을 녹화 후 한참 지나서(또는 -l 루프로 반복) 틀면
tf2 버퍼(기본 10초 히스토리)가 이미 그 시각의 TF를 버린 뒤라 path_relay_node/
RViz 둘 다 TF lookup이 실패해서 경로가 안 뜨고 로봇도 안 움직인다.

경로 모양(좌표)은 그대로 두고 header.stamp(및 각 pose의 stamp)만 수신 시점의
지금 시각으로 다시 찍어서 중계 -- bag은 원래 토픽이 아닌 input_topic으로
remap해서 틀고, 이 스크립트가 그 자리를 대신해 /path(또는 output_topic)로
발행한다.

사용:
    ros2 bag play rosbag2_2026_08_25-00_14_42/ -l --remap /path:=/path_bag_raw
    python3 restamp_path_bag.py --ros-args -p input_topic:=/path_bag_raw -p output_topic:=/path
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path


class RestampPathBag(Node):
    def __init__(self):
        super().__init__('restamp_path_bag')

        self.declare_parameter('input_topic', '/path_bag_raw')
        self.declare_parameter('output_topic', '/path')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Path, output_topic, qos)
        self.sub = self.create_subscription(Path, input_topic, self._on_path, qos)

        self.get_logger().info(
            f"restamp_path_bag 시작: '{input_topic}' -> '{output_topic}' "
            f"(좌표는 그대로, stamp만 수신 시각으로 재발행)")

    def _on_path(self, msg: Path):
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        for pose in msg.poses:
            pose.header.stamp = now
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RestampPathBag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
