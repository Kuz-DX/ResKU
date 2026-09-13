#!/usr/bin/env python3
"""synthetic_quarter_arc_path.py

우회전 사분원(90도) -> 좌회전 사분원(90도) -> 우회전 사분원(90도)으로 이어지는
합성 테스트 경로를 nav_msgs/Path로 발행. RViz에서 Path 디스플레이로 바로 확인.

기본 발행 토픽은 '/path'가 아니라 '/synthetic_test_path'라서, path_relay_node가
떠 있어도 실제 로봇은 움직이지 않는다 (순수 시각화용 기본값). MPPI에 실제로
태워서 주행까지 보고 싶으면 topic 파라미터를 '/path'로 넘길 것 -- 그 경우
path_relay_node/controller_server/ekf가 모두 떠 있어야 하고, 로봇이 실제로
이 경로를 따라 움직이니 실기 앞에서 주의.

사용 예:
    python3 synthetic_quarter_arc_path.py --ros-args -p radius_m:=1.0
    python3 synthetic_quarter_arc_path.py --ros-args -p topic:=/path

start_from_current_pose (기본 true): 최초 1회만 frame_id->base_frame TF를
조회해서 그 시점 위치/헤딩을 앵커로 고정하고, 이후로는 매 발행마다 그 고정된
좌표를 그대로 재사용한다 (2026-08-18: 예전엔 매 틱마다 TF를 다시 조회해서
로봇 현재 위치 기준으로 경로를 다시 그렸는데, 그러면 로봇이 움직일 때마다
경로가 같이 밀려서 "실제 트랙처럼" 고정된 경로를 따라가는 테스트가 안 됐음
-- 파라미터 이름의 "start"라는 의도에 맞게 최초 1회만 쓰도록 고침). TF가
아직 없으면(EKF 안 떠 있음 등) 원점(0,0,0)으로 계속 재시도하다가, 첫 성공
시점에 고정된다.
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


def _yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def generate_arc(x0, y0, yaw0, radius_m, direction, num_points, sweep_deg=90.0):
    """direction: +1=좌회전(CCW), -1=우회전(CW).

    (x0,y0,yaw0)에서 시작해 sweep_deg만큼 회전하는 원호를 따라 num_points개의
    (x, y, yaw) 점을 반환한다 (시작점 제외, 끝점까지 균등 분할).
    """
    sweep = math.radians(sweep_deg)
    # 회전 중심은 진행방향 기준 좌/우 90도 방향, radius_m 거리에 있음
    perp_angle = yaw0 + direction * math.pi / 2.0
    cx = x0 + radius_m * math.cos(perp_angle)
    cy = y0 + radius_m * math.sin(perp_angle)
    vx0, vy0 = x0 - cx, y0 - cy  # 중심 -> 시작점 벡터

    points = []
    for i in range(1, num_points + 1):
        t = direction * sweep * i / num_points
        vx, vy = _rotate(vx0, vy0, t)
        points.append((cx + vx, cy + vy, yaw0 + t))
    return points


def generate_right_left_right_path(x0, y0, yaw0, radius_m, points_per_arc):
    path_points = [(x0, y0, yaw0)]
    x, y, yaw = x0, y0, yaw0
    for direction in (-1, +1, -1):  # 우, 좌, 우
        arc = generate_arc(x, y, yaw, radius_m, direction, points_per_arc)
        path_points.extend(arc)
        x, y, yaw = arc[-1]
    return path_points


class SyntheticPathPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_path_publisher')

        self.declare_parameter('radius_m', 1.0)
        self.declare_parameter('points_per_arc', 30)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic', '/synthetic_test_path')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('start_from_current_pose', True)

        self.radius_m = float(self.get_parameter('radius_m').value)
        self.points_per_arc = int(self.get_parameter('points_per_arc').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.topic = self.get_parameter('topic').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.start_from_current_pose = bool(self.get_parameter('start_from_current_pose').value)

        if self.topic == '/path':
            self.get_logger().warn(
                "topic='/path'로 설정됨: path_relay_node가 떠 있으면 이 합성 "
                "경로가 그대로 controller_server에 FollowPath goal로 들어가서 "
                "실제 로봇이 움직입니다. 의도한 게 맞는지 확인하세요."
            )

        # transient_local(latched): RViz가 이 노드보다 늦게 켜져도 마지막 Path를 받음
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.path_pub = self.create_publisher(Path, self.topic, qos)

        self.tf_buffer = None
        self.tf_listener = None
        if self.start_from_current_pose and _TF2_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # 최초 1회 TF 조회에 성공하면 여기 고정된다 (아래 _get_anchor_pose 참고).
        self._cached_anchor_pose = None

        self.timer = self.create_timer(self.publish_period, self._publish_path)
        self.get_logger().info(
            f"synthetic_path_publisher 시작: topic='{self.topic}' "
            f"frame_id='{self.frame_id}' radius={self.radius_m}m 우->좌->우 사분원 3개"
        )

    def _get_anchor_pose(self):
        """앵커 좌표(x0, y0, yaw0)를 반환. 최초 성공한 TF 조회 결과에 이후
        영구히 고정된다 -- 로봇이 그 뒤에 움직여도 경로는 그 자리에 남는다."""
        if self._cached_anchor_pose is not None:
            return self._cached_anchor_pose
        if self.tf_buffer is None:
            return 0.0, 0.0, 0.0
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time())
        except Exception as exc:  # noqa: BLE001 -- TF 미가동 등 모든 실패를 원점 폴백으로 처리
            self.get_logger().warn(
                f"{self.frame_id}->{self.base_frame} TF lookup 실패 ({exc}); "
                "TF 확보될 때까지 원점(0,0,yaw=0)으로 재시도합니다 (아직 고정 안 됨).",
                throttle_duration_sec=5.0,
            )
            return 0.0, 0.0, 0.0
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (t.transform.translation.x, t.transform.translation.y, yaw)
        self._cached_anchor_pose = pose
        self.get_logger().info(
            f"경로 앵커를 {self.frame_id} 프레임에 고정: x={pose[0]:.3f} y={pose[1]:.3f} "
            f"yaw={math.degrees(pose[2]):.1f}deg -- 이후 로봇이 움직여도 경로는 이 자리에 고정됩니다."
        )
        return pose

    def _publish_path(self):
        x0, y0, yaw0 = self._get_anchor_pose()
        points = generate_right_left_right_path(x0, y0, yaw0, self.radius_m, self.points_per_arc)

        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qz, qw = _yaw_to_quaternion_zw(yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
