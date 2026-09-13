#!/usr/bin/env python3
"""synthetic_sine_path.py

사인파 형태(좌우로 굴곡지는)의 합성 테스트 경로를 nav_msgs/Path로 발행.
RViz에서 Path 디스플레이로 바로 확인. synthetic_quarter_arc_path.py와 같은
패턴(최초 1회 TF 조회 후 그 좌표에 영구 고정 -- "로봇에 비해 상대적으로
고정된" 경로, 로봇이 움직여도 경로는 안 밀림)을 그대로 따른다.

경로는 로봇 정면(+x, 앵커 시점 헤딩 기준) 방향으로 진행하면서 좌우(y)로
y = amplitude_m * sin(2*pi*x/wavelength_m) 만큼 흔들리는 곡선이다. yaw는
그 지점에서의 접선 방향(dy/dx의 atan)으로 채워서, 경로 자체의 헤딩과
곡률이 실제로 이어지는 곡선처럼 매끄럽다.

기본 발행 토픽은 '/path'가 아니라 '/synthetic_test_path'라서, path_relay_node가
떠 있어도 실제 로봇은 움직이지 않는다 (순수 시각화용 기본값). MPPI에 실제로
태워서 주행까지 보고 싶으면 topic 파라미터를 '/path'로 넘길 것 -- 그 경우
path_relay_node/controller_server/ekf가 모두 떠 있어야 하고, 로봇이 실제로
이 경로를 따라 움직이니 실기 앞에서 주의.

start_from_current_pose (기본 true): 최초 1회만 frame_id->base_frame TF를
조회해서 그 시점 위치/헤딩을 앵커로 고정하고, 이후로는 매 발행마다 그 고정된
좌표를 그대로 재사용한다. TF가 아직 없으면(EKF 안 떠 있음 등) 원점(0,0,0)으로
계속 재시도하다가, 첫 성공 시점에 고정된다.

wavelength_m 기본값 관련 (2026-08-19): 처음 기본값 1.0m는 amplitude_m=0.3m
기준 최대 접선각이 ~62도로 너무 급해서, 로봇이 파도 사이를 못 따라가고
그냥 직선으로 관통하는 증상이 실기에서 확인됨. 기본값을 2.5m로 완만하게
바꿈 (같은 amplitude 기준 최대 접선각 ~37도) -- 더 완만하게 하려면
wavelength_m을 더 키우거나 amplitude_m을 줄일 것.

사용 예:
    python3 synthetic_sine_path.py --ros-args -p amplitude_m:=0.3 -p wavelength_m:=1.0
    python3 synthetic_sine_path.py --ros-args -p topic:=/path
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


def _yaw_to_quaternion_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def generate_sine_path(x0, y0, yaw0, amplitude_m, wavelength_m, length_m, num_points):
    """(x0, y0, yaw0)를 원점/기준헤딩으로 삼는 국소좌표계에서 사인파를 그린 뒤,
    앵커 pose(x0, y0, yaw0)만큼 평행이동+회전해서 최종 좌표로 변환한다.

    국소좌표계: s(0~length_m)를 로봇 정면(+x) 진행거리로, y = amplitude_m *
    sin(2*pi*s/wavelength_m)로 좌우 굴곡을 준다. yaw는 그 지점의 접선각.
    """
    k = 2.0 * math.pi / wavelength_m
    c0, s0 = math.cos(yaw0), math.sin(yaw0)

    points = []
    for i in range(num_points):
        s = length_m * i / (num_points - 1)
        local_x = s
        local_y = amplitude_m * math.sin(k * s)
        local_yaw = math.atan2(amplitude_m * k * math.cos(k * s), 1.0)  # 접선 각도

        # 앵커 헤딩(yaw0)만큼 회전 후 앵커 위치(x0, y0)만큼 평행이동.
        x = x0 + local_x * c0 - local_y * s0
        y = y0 + local_x * s0 + local_y * c0
        yaw = yaw0 + local_yaw

        points.append((x, y, yaw))
    return points


class SyntheticSinePathPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_sine_path_publisher')

        self.declare_parameter('amplitude_m', 0.3)
        self.declare_parameter('wavelength_m', 2.5)
        self.declare_parameter('length_m', 2.0)
        self.declare_parameter('num_points', 60)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic', '/synthetic_test_path')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('start_from_current_pose', True)

        self.amplitude_m = float(self.get_parameter('amplitude_m').value)
        self.wavelength_m = float(self.get_parameter('wavelength_m').value)
        self.length_m = float(self.get_parameter('length_m').value)
        self.num_points = int(self.get_parameter('num_points').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.topic = self.get_parameter('topic').value
        self.publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.start_from_current_pose = bool(self.get_parameter('start_from_current_pose').value)

        if self.wavelength_m <= 0.0:
            raise ValueError('wavelength_m은 0보다 커야 합니다.')

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
            f"synthetic_sine_path_publisher 시작: topic='{self.topic}' "
            f"frame_id='{self.frame_id}' amplitude={self.amplitude_m}m "
            f"wavelength={self.wavelength_m}m length={self.length_m}m 사인파 경로"
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
        points = generate_sine_path(
            x0, y0, yaw0, self.amplitude_m, self.wavelength_m, self.length_m, self.num_points)

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
    node = SyntheticSinePathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
