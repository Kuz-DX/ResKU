#!/usr/bin/env python3
"""track_cross_error_logger.py -- [테스트용] synthetic_track_loop_path.py가
발행하는 전체 폐루프 기준 경로(/synthetic_test_path_full, latched)와 실제
로봇 위치(/odometry/filtered)를 매 odom 콜백마다 비교해서 횡방향(cross-track)
오차를 계산/발행/로그하는 노드.

목적: "실제 MPPI 주행이 RViz에 보이는 경로와 같은지, 슬립/실제 주행 조건이
critic에 잘 반영되는지"를 숫자로 보기 위함. RViz의 궤적(/trajectories,
/transformed_global_plan)이나 Odometry 화살표 트레일만 봐서는 "궤적이 좀
어긋난 것 같다" 정도의 정성적 판단만 가능한데, 이 노드는 그 어긋난 정도를
매 순간 부호 있는 거리(m)로 뽑아준다 -- 로봇이 기준 경로의 왼쪽/오른쪽 중
어느 쪽으로, 얼마나 밀려나는지.

중요: 이 노드는 오프라인 벤치(sim_diff_drive_odom.py)용이 아니다.
sim_diff_drive_odom.py는 슬립/관성을 전혀 모델링하지 않는 순수 기구학
적분기라서(스크립트 자체 docstring 참고) 거기서 재보면 cross-track error가
항상 0에 가깝게 나온다 -- 슬립이 애초에 없기 때문. 슬립/실제 주행 조건을
보려면 반드시 실제 로봇(autonomous.launch.py, 실제 EKF + 실제 모터)에
대고 재야 의미가 있다.

전제: synthetic_track_loop_path.py를 topic:=/path로 띄워서 실제 로봇이
그 폐루프를 추종하고 있어야 한다 (이 노드가 구독하는 /synthetic_test_path_full은
그 스크립트가 같이 발행하는, 전체 루프 모양의 latched 참조용 토픽).

사용 예:
    python3 track_cross_error_logger.py
    python3 track_cross_error_logger.py --ros-args -p log_period_s:=0.5

RViz에서 보려면: /debug/track_cross_error_marker (MarkerArray)를 추가하면
로봇 현재 위치 <-> 기준 경로 최근접점을 잇는 선 + 부호 있는 오차 텍스트가
매 cycle 갱신된다. 숫자로만 보려면 /debug/track_cross_error (std_msgs/Float32)를
`ros2 topic echo` 하거나 plotjuggler로 띄우면 된다.

Ctrl+C로 종료하면 이번 실행 전체의 평균/최대 |오차| 요약을 로그로 남긴다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def _point_to_segment(px, py, ax, ay, bx, by):
    """(px,py)에서 선분 (a->b)까지의 부호 있는 최단거리.
    부호: 선분 진행방향 기준 왼쪽이 양수, 오른쪽이 음수(2D cross product).
    Returns: (signed_dist, dist, closest_x, closest_y)."""
    abx, aby = bx - ax, by - ay
    seg_len2 = abx * abx + aby * aby
    if seg_len2 < 1e-9:
        t = 0.0
    else:
        t = ((px - ax) * abx + (py - ay) * aby) / seg_len2
        t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    dx, dy = px - cx, py - cy
    dist = math.hypot(dx, dy)
    cross = abx * (py - ay) - aby * (px - ax)
    sign = 1.0 if cross >= 0.0 else -1.0
    return sign * dist, dist, cx, cy


def nearest_signed_distance(loop_points, px, py):
    """닫힌 루프(loop_points, 마지막 점 다음이 다시 첫 점으로 이어짐)에서
    (px,py)까지의 부호 있는 최단거리. 모든 세그먼트를 훑어서 최솟값을 찾음
    (루프 점 개수가 수백 개 수준이라 매 odom 콜백마다 돌려도 부담 없음)."""
    n = len(loop_points)
    best_signed, best_dist, best_cx, best_cy = 0.0, float('inf'), px, py
    for i in range(n):
        ax, ay = loop_points[i]
        bx, by = loop_points[(i + 1) % n]
        signed, dist, cx, cy = _point_to_segment(px, py, ax, ay, bx, by)
        if dist < best_dist:
            best_signed, best_dist, best_cx, best_cy = signed, dist, cx, cy
    return best_signed, best_dist, best_cx, best_cy


class TrackCrossErrorLogger(Node):
    def __init__(self):
        super().__init__('track_cross_error_logger')

        self.declare_parameter('reference_topic', '/synthetic_test_path_full')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('marker_topic', '/debug/track_cross_error_marker')
        self.declare_parameter('cross_error_topic', '/debug/track_cross_error')
        self.declare_parameter('log_period_s', 1.0)

        self.log_period_s = float(self.get_parameter('log_period_s').value)

        ref_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._ref_points = None
        self._ref_frame = 'odom'
        self.create_subscription(
            Path, str(self.get_parameter('reference_topic').value), self._on_path, ref_qos)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10)

        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.error_pub = self.create_publisher(
            Float32, str(self.get_parameter('cross_error_topic').value), 10)

        self._current = 0.0
        self._sum_abs = 0.0
        self._max_abs = 0.0
        self._count = 0

        self.get_logger().info(
            f"track_cross_error_logger 시작: reference='{self.get_parameter('reference_topic').value}' "
            f"odom='{self.get_parameter('odom_topic').value}' -- 기준 경로 수신 대기 중."
        )

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warn("기준 경로가 비어있음 -- 무시")
            return
        self._ref_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._ref_frame = msg.header.frame_id
        self.get_logger().info(
            f"기준 루프 수신: {len(self._ref_points)}점, frame='{self._ref_frame}'")

    def _on_odom(self, msg: Odometry) -> None:
        if self._ref_points is None:
            self.get_logger().warn(
                "기준 경로 아직 없음 (synthetic_track_loop_path.py 떠 있는지 확인)",
                throttle_duration_sec=5.0)
            return

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        signed, _dist, cx, cy = nearest_signed_distance(self._ref_points, px, py)

        self._current = signed
        self._count += 1
        self._sum_abs += abs(signed)
        self._max_abs = max(self._max_abs, abs(signed))
        mean_abs = self._sum_abs / self._count

        self.error_pub.publish(Float32(data=float(signed)))
        self._publish_marker(px, py, cx, cy, signed)

        self.get_logger().info(
            f"cross-track error: current={signed:+.3f}m  |mean|={mean_abs:.3f}m  "
            f"max|.|={self._max_abs:.3f}m  (n={self._count})",
            throttle_duration_sec=self.log_period_s)

    def _publish_marker(self, px, py, cx, cy, signed) -> None:
        now = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=0.5).to_msg()

        line = Marker()
        line.header.frame_id = self._ref_frame
        line.header.stamp = now
        line.ns = 'track_cross_error'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.03
        line.color.a = 1.0
        line.color.r = 1.0 if abs(signed) > 0.15 else 0.2
        line.color.g = 0.2 if abs(signed) > 0.15 else 1.0
        line.color.b = 0.2
        line.lifetime = lifetime
        line.points = [Point(x=px, y=py, z=0.05), Point(x=cx, y=cy, z=0.05)]

        text = Marker()
        text.header.frame_id = self._ref_frame
        text.header.stamp = now
        text.ns = 'track_cross_error'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = px
        text.pose.position.y = py
        text.pose.position.z = 0.4
        text.scale.z = 0.18
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.lifetime = lifetime
        text.text = f"{signed:+.2f}m"

        self.marker_pub.publish(MarkerArray(markers=[line, text]))

    def print_summary(self) -> None:
        if self._count == 0:
            self.get_logger().info("수신된 odom 없음 -- 요약할 데이터 없음.")
            return
        mean_abs = self._sum_abs / self._count
        self.get_logger().info(
            f"=== 요약 (n={self._count}) === 마지막 오차={self._current:+.3f}m  "
            f"평균|오차|={mean_abs:.3f}m  최대|오차|={self._max_abs:.3f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackCrossErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
