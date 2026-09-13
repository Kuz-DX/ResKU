#!/usr/bin/env python3
"""synthetic_circle_path.py -- [테스트용] 로봇이 한 방향으로 계속 원을
그리며 도는 합성 폐루프 경로. 실차에서 지속적인 회전(steady-state 선회)
중 슬립이 반경을 얼마나 넓히는지 보는 용도 -- 자동차 실험의 "스키드패드"와
같은 개념. synthetic_track_loop_path.py와 동일 패턴(TF 앵커, 로봇 앞
window_length_m만 매 cycle 잘라 발행)이지만, 저건 직선+반원 조합인 데 비해
이건 순수 원 하나만 계속 돈다는 점이 다르다.

기본 발행 토픽은 '/path'가 아니라 '/synthetic_test_path'라서, path_relay_node가
떠 있어도 실제 로봇은 움직이지 않는다. 실제로 태워서 계속 돌게 하려면
topic 파라미터를 '/path'로 넘길 것.

사용 예:
    python3 synthetic_circle_path.py
    python3 synthetic_circle_path.py --ros-args -p topic:=/path -p radius_m:=1.2
    python3 synthetic_circle_path.py --ros-args -p radius_m:=1.0 -p direction:=-1.0   # 우회전
"""
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


def generate_circle(x0, y0, yaw0, radius_m, direction, num_points):
    """(x0,y0,yaw0)에서 시작해 radius_m 반경으로 360도 완전히 도는 원.
    direction: +1=좌회전, -1=우회전. 시작점을 poses에 포함해서 반환(폐루프
    윈도우 추출 시 인덱스 0이 곧 마지막 점 다음과 이어짐)."""
    sweep = 2.0 * math.pi
    perp_angle = yaw0 + direction * math.pi / 2.0
    cx = x0 + radius_m * math.cos(perp_angle)
    cy = y0 + radius_m * math.sin(perp_angle)
    vx0, vy0 = x0 - cx, y0 - cy

    points = [(x0, y0)]
    for i in range(1, num_points):
        t = direction * sweep * i / num_points
        vx, vy = _rotate(vx0, vy0, t)
        points.append((cx + vx, cy + vy))

    yaws = []
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        dx = points[j][0] - points[i][0]
        dy = points[j][1] - points[i][1]
        yaws.append(math.atan2(dy, dx))
    return [(p[0], p[1], yaw) for p, yaw in zip(points, yaws)]


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
    """폐루프(원)라 인덱스가 끝에 닿으면 wrap. synthetic_track_loop_path.py와
    동일 원리 -- perception이 매 cycle 로봇 앞 짧은 구간만 새로 보내는 것을
    흉내내서 PathHandler의 closest-point 탐색 범위 문제를 피한다."""
    n = len(points)
    window = [points[start_idx]]
    acc = 0.0
    i = start_idx
    while acc < window_length_m and len(window) <= n:
        j = (i + 1) % n
        x0, y0, _ = points[i]
        x1, y1, _ = points[j]
        acc += math.hypot(x1 - x0, y1 - y0)
        window.append(points[j])
        i = j
    return window


class SyntheticCirclePublisher(Node):
    def __init__(self):
        super().__init__('synthetic_circle_publisher')

        self.declare_parameter('radius_m', 1.2)
        self.declare_parameter('direction', 1.0)  # +1=좌회전, -1=우회전
        self.declare_parameter('points_per_circle', 120)
        self.declare_parameter('window_length_m', 3.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic', '/synthetic_test_path')
        self.declare_parameter('full_loop_topic', '/synthetic_test_path_full')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('start_from_current_pose', True)

        self.radius_m = float(self.get_parameter('radius_m').value)
        self.direction = 1.0 if float(self.get_parameter('direction').value) >= 0 else -1.0
        self.points_per_circle = int(self.get_parameter('points_per_circle').value)
        self.window_length_m = float(self.get_parameter('window_length_m').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.topic = self.get_parameter('topic').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.start_from_current_pose = bool(self.get_parameter('start_from_current_pose').value)

        if self.topic == '/path':
            self.get_logger().warn(
                "topic='/path'로 설정됨: 실제 로봇이 이 원을 계속(끝없이) 돌려고 "
                "시도합니다. 의도한 게 맞는지, 개활지인지 확인하세요."
            )

        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, self.topic, qos)
        self.full_loop_topic = self.get_parameter('full_loop_topic').value
        self.full_loop_pub = self.create_publisher(Path, self.full_loop_topic, qos)

        self.tf_buffer = None
        self.tf_listener = None
        if self.start_from_current_pose and _TF2_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self._loop_points = None  # 최초 TF 성공 시 1회만 계산, 이후 고정

        self.timer = self.create_timer(self.publish_period, self._publish_path)
        self.get_logger().info(
            f"synthetic_circle_publisher 시작: topic='{self.topic}' "
            f"radius={self.radius_m}m direction={'좌' if self.direction > 0 else '우'} "
            f"window={self.window_length_m}m -- {self.frame_id}->{self.base_frame} TF 필요."
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

        if self._loop_points is None:
            self._loop_points = generate_circle(
                x, y, yaw, self.radius_m, self.direction, self.points_per_circle)
            self.get_logger().info(
                f"기준 원을 {self.frame_id} 프레임에 고정: 중심 근처 x={x:.3f} y={y:.3f} "
                f"({len(self._loop_points)}점) -- 이후 로봇 앞 {self.window_length_m}m만 "
                f"매 cycle 새로 잘라 보냄.")
            self.full_loop_pub.publish(self._points_to_path_msg(self._loop_points))

        nearest_idx = nearest_point_index(self._loop_points, x, y)
        window = extract_forward_window(self._loop_points, nearest_idx, self.window_length_m)
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
    node = SyntheticCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
