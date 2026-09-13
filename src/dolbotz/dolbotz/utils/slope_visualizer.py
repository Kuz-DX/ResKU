#!/usr/bin/env python3
"""
경사 시각화 디버그 노드

gradient_map_node가 발행하는 경사도 맵(/terrain/slope_deg)을 구독하여,
설정된 임계값(critical_slope_deg)을 초과하는 지점을 RViz에서 볼 수 있는
마커(visualization_msgs/MarkerArray)로 발행합니다.

이를 통해 "No path within max_slope_deg=..." 경고가 발생했을 때,
실제로 어느 영역의 경사가 가파른지 직관적으로 확인할 수 있습니다.

구독:
  /terrain/slope_deg (sensor_msgs/Image): 픽셀값이 경사각(도)인 이미지
  /terrain/elevation_map (sensor_msgs/Image): 마커의 Z좌표(높이)를 얻기 위함

발행:
  /debug/slope_markers (visualization_msgs/MarkerArray): RViz 시각화용 마커

실행:
  ros2 run dolbotz slope_visualizer --ros-args -p critical_slope_deg:=30.0

RViz 설정:
  1. Add -> By topic -> /debug/slope_markers (MarkerArray)
  2. Fixed Frame이 마커의 frame_id(기본값 'camera_link', gradient_map_node의
     path_frame_id와 일치)로 설정되었는지 확인
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


def _sensor_data_qos_depth1() -> QoSProfile:
    """/terrain/slope_deg, /terrain/elevation_map 구독용 — rclpy 기본
    qos_profile_sensor_data(BEST_EFFORT/VOLATILE/depth=5)와 동일하되 depth만
    1로 낮춘 프로필(path_visualizer.py의 _sensor_data_qos_depth1()와 동일
    근거/구현 — dolbotz.utils.qos.SENSOR_DATA_QOS_DEPTH1과 스펙 동일, 이
    파일은 dolbotz 패키지 무의존 설계라 인라인으로 중복 정의)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class SlopeVisualizerNode(Node):
    def __init__(self):
        super().__init__('slope_visualizer')

        self.declare_parameter('slope_topic', '/terrain/slope_deg')
        self.declare_parameter('elevation_topic', '/terrain/elevation_map')
        self.declare_parameter('critical_slope_deg', 30.0)
        self.declare_parameter('resolution_m', 0.15) # gradient_map과 동일해야 함
        self.declare_parameter('path_frame_id', 'camera_link') # gradient_map_node와 동일해야 함

        slope_topic = self.get_parameter('slope_topic').value
        elevation_topic = self.get_parameter('elevation_topic').value
        self.critical_slope = self.get_parameter('critical_slope_deg').value
        self.resolution = self.get_parameter('resolution_m').value
        self.path_frame_id = self.get_parameter('path_frame_id').value

        self.bridge = CvBridge()
        self.slope_map = None
        self.elevation_map = None

        self.marker_pub = self.create_publisher(MarkerArray, '/debug/slope_markers', 10)

        self.slope_sub = self.create_subscription(
            Image, slope_topic, self.on_slope, _sensor_data_qos_depth1())
        self.elevation_sub = self.create_subscription(
            Image, elevation_topic, self.on_elevation, _sensor_data_qos_depth1())

        self.get_logger().info(
            f'SlopeVisualizerNode ready. Critical slope: {self.critical_slope} deg. '
            f'Listening on {slope_topic} and {elevation_topic}. '
            f'Marker frame_id: {self.path_frame_id}.'
        )

    def on_slope(self, msg: Image):
        self.slope_map = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        self.process_maps()

    def on_elevation(self, msg: Image):
        self.elevation_map = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        self.process_maps()

    def process_maps(self):
        if self.slope_map is None or self.elevation_map is None:
            return

        if self.slope_map.shape != self.elevation_map.shape:
            self.get_logger().warn('Slope and elevation map dimensions do not match.', throttle_duration_sec=5.0)
            return

        marker_array = MarkerArray()
        h, w = self.slope_map.shape
        
        # 경사가 임계값을 넘는 모든 지점(y, x)을 찾음
        steep_points = np.argwhere(self.slope_map > self.critical_slope)

        # 이전 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, (r, c) in enumerate(steep_points):
            z = self.elevation_map[r, c]
            if np.isnan(z):
                continue

            marker = Marker()
            marker.header.frame_id = self.path_frame_id # gradient_map_node의 path_frame_id와 일치해야 함
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "steep_slopes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 맵의 (row, col)을 world 좌표 (x, y)로 변환
            marker.pose.position.x = (c - w / 2.0) * self.resolution
            marker.pose.position.y = (r - h / 2.0) * self.resolution
            marker.pose.position.z = float(z)
            
            marker.scale.x = self.resolution
            marker.scale.y = self.resolution
            marker.scale.z = self.resolution
            
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # 처리가 끝났으므로 맵을 초기화하여 중복 처리를 방지
        self.slope_map = None
        self.elevation_map = None

def main(args=None):
    rclpy.init(args=args)
    node = SlopeVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()