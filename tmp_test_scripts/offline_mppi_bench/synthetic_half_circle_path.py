#!/usr/bin/env python3
"""synthetic_half_circle_path.py -- [테스트용] 직진하다가 반원(180도)을
돌아 반대 방향으로 나가는(U턴) 합성 경로. synthetic_sharp_turn_path.py가
"짧고 급한" 회전(작은 반경, 좁은 구간)에 초점을 둔다면, 이건 "더 크고
오래 지속되는" 선회(중간 반경, 반 바퀴 전체)에 초점을 둔다 -- 지속 시간이
길어서 EKF yaw 추적/누적 오차, local_costmap 범위(±3m) 안에 반원 전체가
들어오는지(PathAlignCritic 스킵 여부) 등을 sharp_turn과 다른 시간축에서
볼 수 있다.

폐루프가 아니다 -- 접근 구간(approach_m) -> 반원(radius_m) -> 이탈
구간(exit_m) 순서의 1회성 경로. 로봇이 끝에 다다르면 window가 짧아지며
멈춘다.

기본 발행 토픽은 '/path'가 아니라 '/synthetic_test_path'라서, path_relay_node가
떠 있어도 실제 로봇은 움직이지 않는다.

사용 예:
    python3 synthetic_half_circle_path.py
    python3 synthetic_half_circle_path.py --ros-args -p topic:=/path -p radius_m:=1.2
    python3 synthetic_half_circle_path.py --ros-args -p direction:=-1.0   # 우회전 U턴
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


def generate_straight(x0, y0, yaw0, length_m, num_points):
    c0, s0 = math.cos(yaw0), math.sin(yaw0)
    points = []
    for i in range(1, num_points + 1):
        s = length_m * i / num_points
        points.append((x0 + s * c0, y0 + s * s0))
    return points


def generate_half_loop(x0, y0, yaw0, radius_m, direction, num_points):
    """180도 반원. direction: +1=좌회전, -1=우회전."""
    sweep = math.pi
    perp_angle = yaw0 + direction * math.pi / 2.0
    cx = x0 + radius_m * math.cos(perp_angle)
    cy = y0 + radius_m * math.sin(perp_angle)
    vx0, vy0 = x0 - cx, y0 - cy
    points = []
    for i in range(1, num_points + 1):
        t = direction * sweep * i / num_points
        vx, vy = _rotate(vx0, vy0, t)
        points.append((cx + vx, cy + vy))
    return points


def tangent_yaws(points):
    n = len(points)
    if n <= 1:
        return [0.0] * n
    yaws = []
    for i in range(n - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        yaws.append(math.atan2(dy, dx))
    yaws.append(yaws[-1])
    return yaws


def generate_half_circle_path(x0, y0, yaw0, approach_m, radius_m, direction, exit_m, points_per_m):
    pts = [(x0, y0)]

    seg = generate_straight(x0, y0, yaw0, approach_m, max(2, int(approach_m * points_per_m)))
    pts.extend(seg)
    yaw_after_approach = yaw0 if len(seg) < 2 else math.atan2(
        pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
    x1, y1 = pts[-1]

    halfloop_points = max(20, int(math.pi * radius_m * points_per_m))
    seg = generate_half_loop(x1, y1, yaw_after_approach, radius_m, direction, halfloop_points)
    pts.extend(seg)
    yaw_after_turn = math.atan2(pts[-1][1] - pts[-2][1], pts[-1][0] - pts[-2][0])
    x2, y2 = pts[-1]

    seg = generate_straight(x2, y2, yaw_after_turn, exit_m, max(2, int(exit_m * points_per_m)))
    pts.extend(seg)

    yaws = tangent_yaws(pts)
    return [(p[0], p[1], yaw) for p, yaw in zip(pts, yaws)]


def _yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def nearest_point_index(points, x, y, min_idx=0):
    """min_idx 이후 구간에서만 최근접점을 찾는다 -- 폐루프가 아닌 경로는
    되돌아갈 일이 없으므로, 전체 경로에서 찾으면 U턴처럼 경로가 자기 자신과
    가까워지는 구간(접근/이탈 구간이 지름만큼만 떨어짐)에서 위치 오차/슬립
    때문에 반대쪽 구간이 유클리드 거리상 더 가깝게 잡혀 window가 이미 지나온
    지점으로 되돌아가고, 그 결과 같은 코너를 무한 반복하는 문제가 생긴다."""
    best_i, best_d2 = min_idx, float('inf')
    for i in range(min_idx, len(points)):
        px, py, _yaw = points[i]
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2, best_i = d2, i
    return best_i


def extract_forward_window(points, start_idx, window_length_m):
    """폐루프가 아니므로 wrap 없이 경로 끝에서 멈춘다."""
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


class SyntheticHalfCirclePublisher(Node):
    def __init__(self):
        super().__init__('synthetic_half_circle_publisher')

        self.declare_parameter('approach_m', 2.0)
        self.declare_parameter('radius_m', 1.2)
        self.declare_parameter('direction', 1.0)  # +1=좌회전, -1=우회전
        self.declare_parameter('exit_m', 2.0)
        self.declare_parameter('points_per_m', 20)
        self.declare_parameter('window_length_m', 3.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic', '/synthetic_test_path')
        self.declare_parameter('full_loop_topic', '/synthetic_test_path_full')
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('start_from_current_pose', True)

        self.approach_m = float(self.get_parameter('approach_m').value)
        self.radius_m = float(self.get_parameter('radius_m').value)
        self.direction = 1.0 if float(self.get_parameter('direction').value) >= 0 else -1.0
        self.exit_m = float(self.get_parameter('exit_m').value)
        self.points_per_m = int(self.get_parameter('points_per_m').value)
        self.window_length_m = float(self.get_parameter('window_length_m').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.topic = self.get_parameter('topic').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.start_from_current_pose = bool(self.get_parameter('start_from_current_pose').value)

        if self.topic == '/path':
            self.get_logger().warn(
                "topic='/path'로 설정됨: 실제 로봇이 이 U턴 경로를 그대로 "
                "따라가려 시도합니다. 안전 확보 후 진행하세요."
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

        self._path_points = None
        self._last_idx = 0  # nearest_point_index가 뒤로 안 가게 하한선으로 사용

        self.timer = self.create_timer(self.publish_period, self._publish_path)
        self.get_logger().info(
            f"synthetic_half_circle_publisher 시작: topic='{self.topic}' "
            f"approach={self.approach_m}m radius={self.radius_m}m "
            f"direction={'좌' if self.direction > 0 else '우'} exit={self.exit_m}m -- "
            f"{self.frame_id}->{self.base_frame} TF 필요."
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

        if self._path_points is None:
            self._path_points = generate_half_circle_path(
                x, y, yaw, self.approach_m, self.radius_m, self.direction,
                self.exit_m, self.points_per_m)
            self.get_logger().info(
                f"경로를 {self.frame_id} 프레임에 고정: x={x:.3f} y={y:.3f} "
                f"({len(self._path_points)}점).")
            self.full_loop_pub.publish(self._points_to_path_msg(self._path_points))

        nearest_idx = nearest_point_index(self._path_points, x, y, self._last_idx)
        self._last_idx = nearest_idx
        window = extract_forward_window(self._path_points, nearest_idx, self.window_length_m)
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
    node = SyntheticHalfCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
