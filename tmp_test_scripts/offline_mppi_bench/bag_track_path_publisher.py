#!/usr/bin/env python3
"""bag_track_path_publisher.py -- [테스트용] extract_track_from_bag.py가
만든 JSON 트랙(manual 주행 bag에서 재구성한 실제 경로)을 로봇의 "현재"
위치에 재-앵커링해서, synthetic_track_loop_path.py와 동일한 방식(로봇 앞
window_length_m만 매 cycle 잘라 /path로 발행)으로 라이브 퍼블리시한다.

토픽/파라미터 이름을 synthetic_track_loop_path.py와 최대한 맞춰서(topic,
full_loop_topic, window_length_m, frame_id, base_frame, publish_rate_hz)
기존 RViz 설정(/synthetic_test_path_full 등)과 track_cross_error_logger.py를
수정 없이 그대로 재사용할 수 있게 했다.

재-앵커링을 하는 이유: bag을 녹화했던 원래 세션의 odom 원점(EKF가 그때 그
순간 임의로 잡은 원점)과 지금 이 세션의 odom 원점은 서로 다른 게 당연하다
-- 그래서 저장된 트랙은 "모양(shape)"으로만 쓰고, 실제 배치 위치/방향은
지금 로봇이 서있는 곳 기준으로 다시 잡는다(synthetic_track_loop_path.py의
start_from_current_pose와 동일 원리). 이 말은 **실행 전에 로봇을 manual로
출발했던 그 지점 근처에, 대략 같은 방향을 보고 세워둬야** 재구성된 트랙이
실제 코스와 맞아떨어진다는 뜻이다 -- 다른 곳/다른 방향에서 실행하면 트랙
모양은 같아도 엉뚱한 곳에 그려진다.

manual 코스는 폐루프가 아닐 수 있으므로(왕복/직선 등) window 추출은 wrap
없이 트랙 끝에서 그냥 멈춘다(synthetic_track_loop_path.py는 폐루프라 wrap
했던 것과 다른 부분).

사용:
    python3 bag_track_path_publisher.py --ros-args -p track_file:=manual_run_01_track.json
    python3 bag_track_path_publisher.py --ros-args -p track_file:=manual_run_01_track.json -p topic:=/path
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

try:
    from tf2_ros import Buffer, TransformListener
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False


def _rotate(vx, vy, angle):
    c, s = math.cos(angle), math.sin(angle)
    return vx * c - vy * s, vx * s + vy * c


def _yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def nearest_point_index(points, x, y):
    best_i, best_d2 = 0, float('inf')
    for i, (px, py, _yaw) in enumerate(points):
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
    return best_i


def extract_forward_window(points, start_idx, window_length_m):
    """synthetic_track_loop_path.py와 달리 폐루프가 아니므로 wrap 없이
    트랙 끝(n-1)에서 멈춘다."""
    n = len(points)
    window = [points[start_idx]]
    acc = 0.0
    i = start_idx
    while acc < window_length_m and i < n - 1:
        x0, y0, _ = points[i]
        x1, y1, _ = points[i + 1]
        acc += math.hypot(x1 - x0, y1 - y0)
        window.append(points[i + 1])
        i += 1
    return window


def rebase_track(points, anchor_x, anchor_y, anchor_yaw):
    """points의 첫 점/방향을 원점으로 보고, 그 상대 형태를 그대로
    (anchor_x, anchor_y, anchor_yaw)로 옮긴다."""
    if not points:
        return []
    x0, y0, yaw0 = points[0]
    delta = anchor_yaw - yaw0
    out = []
    for x, y, yaw in points:
        rx, ry = _rotate(x - x0, y - y0, delta)
        out.append((anchor_x + rx, anchor_y + ry, yaw + delta))
    return out


class BagTrackPathPublisher(Node):
    def __init__(self):
        super().__init__('bag_track_path_publisher')

        self.declare_parameter('track_file', '')
        self.declare_parameter('window_length_m', 3.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic', '/synthetic_test_path')
        self.declare_parameter('full_loop_topic', '/synthetic_test_path_full')
        self.declare_parameter('publish_rate_hz', 15.0)

        track_file = str(self.get_parameter('track_file').value)
        if not track_file:
            raise RuntimeError(
                "track_file 파라미터 필수 -- extract_track_from_bag.py 출력 json 경로를 "
                "-p track_file:=... 로 넘길 것")
        with open(track_file) as f:
            data = json.load(f)
        self._raw_track = [tuple(p) for p in data['points']]
        self.get_logger().info(f"{track_file}에서 {len(self._raw_track)}점 로드")

        self.window_length_m = float(self.get_parameter('window_length_m').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.topic = self.get_parameter('topic').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        if self.topic == '/path':
            self.get_logger().warn(
                "topic='/path'로 설정됨: 실제 로봇이 이 재구성된 트랙을 그대로 "
                "따라가려 시도합니다. 로봇을 manual 출발 지점 근처(같은 방향)에 "
                "세워뒀는지 확인하세요."
            )

        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, self.topic, qos)
        self.full_loop_topic = self.get_parameter('full_loop_topic').value
        self.full_loop_pub = self.create_publisher(Path, self.full_loop_topic, qos)

        self.tf_buffer = None
        self.tf_listener = None
        if _TF2_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # 재-앵커링된 트랙 -- 최초 TF 성공 시 딱 1번만 계산, 그 뒤로 안 바뀜
        # (synthetic_track_loop_path.py의 _loop_points와 동일 패턴).
        self._track = None

        self.timer = self.create_timer(self.publish_period, self._publish_path)
        self.get_logger().info(
            f"bag_track_path_publisher 시작: topic='{self.topic}' -- "
            f"{self.frame_id}->{self.base_frame} TF 필요(앵커링용)."
        )

    def _get_current_pose(self):
        if self.tf_buffer is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{self.frame_id}->{self.base_frame} TF lookup 실패 ({exc})",
                throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def _publish_path(self):
        pose = self._get_current_pose()
        if pose is None:
            return
        x, y, yaw = pose

        if self._track is None:
            self._track = rebase_track(self._raw_track, x, y, yaw)
            self.get_logger().info(
                f"트랙을 현재 위치(x={x:.3f} y={y:.3f})에 재-앵커링 완료 "
                f"({len(self._track)}점) -- 이후 다시 안 바뀜.")
            self.full_loop_pub.publish(self._points_to_path_msg(self._track))

        nearest_idx = nearest_point_index(self._track, x, y)
        window = extract_forward_window(self._track, nearest_idx, self.window_length_m)
        self.path_pub.publish(self._points_to_path_msg(window))

    def _points_to_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py, pyaw in points:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = px
            pose_msg.pose.position.y = py
            qz, qw = _yaw_to_quaternion_zw(pyaw)
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            msg.poses.append(pose_msg)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BagTrackPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
