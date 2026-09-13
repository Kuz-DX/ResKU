#!/usr/bin/env python3
"""
/path에 인위적인 좌/우회전(또는 직진) 경로를 15Hz로 계속 발행하는
테스트용 스크립트.

목적: gradient_map/flat_drive가 지금 depth/마스크 데이터 부족으로 짧은
경로(~0.5~0.8m)만 내는 상황에서, "경로가 충분히 길면 MPPI가 실제로
조향하는가?"를 검증하기 위함.

회전 구간(turn_length_m)과 그 이후 직진 연장 구간을 분리했다. 원호는
시작점 근처에서 작은각 근사 때문에 곡률 효과가 거의 0으로 수렴한다(진짜
원의 성질). MPPI 실측 도달거리가 0.1~0.3m 정도로 짧아서, 그 안에서는
"회전 경로 vs 직진 경로"의 위치 차이가 거의 없어 PathAlignCritic 신호가
약했다. 그래서 회전을 로봇 바로 앞 짧은 구간(turn_length_m, 작을수록 더
급하게/가깝게) 안에 몰아넣고, 그 뒤는 마지막 헤딩으로 쭉 직진 연장해서
총 길이(length_m)를 채운다.

점 밀도(--turn-points): --points(총 점 개수)를 전체 길이(length_m)에 균일
호길이 간격으로만 뿌리면, turn_length_m이 짧을 때 회전 구간에 점이 몇 개
안 들어가서(예: 60점/1.57m 중 0.15m 구간엔 5~6개뿐) RViz에서 원호가 아니라
꺾은선처럼 각지게 보인다. --turn-points로 회전 구간 전용 점 개수를 따로
떼어줘서(나머지는 그 뒤 직진 연장에 배분) 회전 구간만 촘촘하게 만든다 --
turn_deg=0(완전 직진)일 때는 애초에 곡률이 없으니 무시되고 전체 길이에
균일 배분된다.

프레임 앵커링 (--frame):
  - camera_link (기본값, 기존 동작): 매 틱 "지금 이 순간 로봇(카메라) 코앞"을
    원점으로 다시 그려서 발행. 로봇이 움직이면 경로 전체가 같이 밀림
    (RViz에서 "경로가 로봇을 따라다니는" 것처럼 보임) -- 이건 버그가 아니라
    상대경로를 계속 새로 주는 이 모드의 의도된 동작.
  - odom (신규): 노드 시작 후 odom -> camera_link TF를 딱 한 번만 읽어서
    그 순간의 카메라 위치/헤딩 기준으로 경로 좌표를 odom 프레임에 고정.
    이후 로봇이 움직여도 경로는 그 자리에 그대로 있어서, MPPI가 실제
    누적 crosstrack/헤딩 오차를 겪는 "진짜" 경로추종 테스트가 됨.
    odom -> camera_link TF가 있어야 하므로 ekf.launch.py(EKF)가 떠 있어야
    동작 -- 안 떠 있으면 대기하며 경고 로그만 찍음.

--drift-speed (odom 모드 전용, 기본 0=완전 고정):
  경로 전체를 앵커 시점 헤딩 방향으로 지정한 속도[m/s]만큼 시간에 비례해서
  강체 이동(rigid translate)시킴. 양수면 그 헤딩 방향으로 계속 멀어지고
  (가상의 목표가 도망가는 시나리오), 음수면 로봇 쪽으로 다가옴. 경로
  "모양"(회전 구간 등)은 그대로 유지된 채 전체가 같이 미끄러지기만 함 --
  camera_link 모드처럼 로봇 위치에 매 틱 재고정되는 게 아니라, 앵커 시점
  헤딩을 기준으로 한 독립적인 등속 이동이라는 점이 다름.

주의:
  - side_slope_trigger_node(slope_decision.py)도 /path에 발행 중이므로
    같이 띄우면 충돌합니다 -- 이 스크립트 테스트하는 동안은 그 노드를
    꺼두세요(sudo pkill -f slope_decision).
  - 실제 로봇 모터가 켜져 있으면 이 경로를 그대로 따라가려 시도합니다 --
    안전 확보(비상정지 대기 등) 후 실행하세요.

사용법:
  python3 publish_synthetic_left_turn.py                          # 기본: 0.3m 안에 90도 회전 후 나머지는 직진, 총 길이 1.57m, camera_link 기준
  python3 publish_synthetic_left_turn.py --turn-deg 90 --turn-length 0.15   # 로봇 코앞 0.15m 안에서 급좌회전
  python3 publish_synthetic_left_turn.py --turn-deg -90            # 우회전 90도
  python3 publish_synthetic_left_turn.py --turn-deg 0               # 완전 직진
  python3 publish_synthetic_left_turn.py --frame odom               # odom에 고정 -- 로봇이 움직여도 경로가 안 밀림
  python3 publish_synthetic_left_turn.py --frame odom --drift-speed 0.1   # 위 상태에서 앵커 헤딩 방향으로 0.1m/s로 계속 전진(멀어짐)
  python3 publish_synthetic_left_turn.py --turn-deg 45 --turn-length 0.15 --frame odom --turn-points 25   # 회전 구간을 25점으로 촘촘히 -- 더 매끄러운 원호
  Ctrl+C로 종료 -- 종료해도 마지막 goal은 controller_server에 남아있을 수
  있으니, 필요하면 side_slope_trigger_node를 다시 켜서 정상 경로로
  덮어쓰세요.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 -- PoseStamped용 do_transform 등록에 필요


class SyntheticLeftTurnPublisher(Node):
    def __init__(
        self, length_m: float, turn_length_m: float, turn_deg: float,
        n_points: int, rate_hz: float, anchor_frame: str, drift_speed: float,
        turn_points: int,
    ):
        super().__init__('synthetic_left_turn_publisher')
        self._length_m = max(length_m, turn_length_m)
        self._turn_length_m = turn_length_m
        self._turn_deg = turn_deg
        self._n_points = n_points
        self._anchor_frame = anchor_frame
        self._drift_speed = drift_speed
        # 회전 구간에 최소 2점은 있어야 원호가 그려짐. 총점보다 많이 달라고
        # 하면 총점으로 눌러서(나머지 직진 구간은 0점) 인덱스 오류를 막는다.
        self._turn_points = max(2, min(turn_points, n_points))
        self._pub = self.create_publisher(Path, '/path', 10)

        # --frame odom일 때만 씀: 최초 1회 계산한 뒤 고정되는 odom 기준 Pose 리스트,
        # 그리고 --drift-speed 적용의 기준이 되는 앵커 시점 헤딩/시각.
        self._cached_odom_poses: list[Pose] | None = None
        self._anchor_yaw = 0.0
        self._anchor_time = None

        if self._anchor_frame == 'odom':
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f'/path에 합성 경로 발행 시작: 회전 {turn_deg}deg를 처음 {turn_length_m}m 안에 '
            f'끝내고, 이후 총 {self._length_m}m까지 직진 연장 (n_points={n_points} '
            f'rate={rate_hz}Hz, frame={anchor_frame})'
        )
        if self._anchor_frame == 'odom':
            self.get_logger().info(
                'odom 고정 모드: odom -> camera_link TF 확보되는 즉시 그 시점 카메라 '
                '위치 기준으로 경로를 odom에 1회 고정합니다 (EKF가 떠 있어야 함).'
            )
            if self._drift_speed != 0.0:
                self.get_logger().info(
                    f'앵커 헤딩 방향으로 {self._drift_speed:+.3f} m/s 등속 이동 적용됩니다.'
                )

    @staticmethod
    def _quat_yaw(q) -> float:
        """평면(2D) 투영 yaw. 마운트 roll/pitch가 소량 섞여 있어도 근사적으로 유효."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _point_at(self, s, turn_sign, theta_max, turn_radius, x_end, y_end, straight_turn):
        """호길이 s(로봇으로부터의 누적 이동거리) 지점의 (x, y, yaw)."""
        if straight_turn:
            return s, 0.0, 0.0
        if s <= self._turn_length_m:
            theta = theta_max * (s / self._turn_length_m)
            x = turn_radius * math.sin(theta)
            y = turn_sign * turn_radius * (1.0 - math.cos(theta))
            yaw = turn_sign * theta
        else:
            # 회전 구간을 벗어난 뒤 -- 마지막 헤딩(theta_max) 방향으로 직진 연장.
            extra = s - self._turn_length_m
            x = x_end + extra * math.cos(theta_max)
            y = y_end + turn_sign * extra * math.sin(theta_max)
            yaw = turn_sign * theta_max
        return x, y, yaw

    def _local_points(self):
        """camera_link 기준 (x, y, yaw) 리스트. 로봇 정면이 +x, 좌회전이 +yaw."""
        turn_sign = 1.0 if self._turn_deg >= 0 else -1.0
        theta_max = math.radians(abs(self._turn_deg))
        straight_turn = theta_max < 1e-6
        turn_radius = (self._turn_length_m / theta_max) if not straight_turn else 0.0

        # 회전이 끝나는 지점(turn_length_m)의 좌표/헤딩 -- 그 뒤 직진 연장의 기준점.
        if straight_turn:
            x_end, y_end = self._turn_length_m, 0.0
        else:
            x_end = turn_radius * math.sin(theta_max)
            y_end = turn_sign * turn_radius * (1.0 - math.cos(theta_max))

        # 곡률이 없는 완전 직진이면 turn_points 개념이 무의미 -- 예전처럼
        # 총 길이에 균일 호길이 간격으로만 뿌린다.
        if straight_turn:
            s_values = [
                self._length_m * i / (self._n_points - 1) for i in range(self._n_points)
            ]
        else:
            # 회전 구간(0 ~ turn_length_m)에 turn_points개를 촘촘히 배분하고,
            # 남은 점은 그 뒤 직진 연장(turn_length_m ~ length_m)에 배분 --
            # 짧은 회전 구간이 전체 균일 간격에 묻혀 각지게 보이는 걸 막는다.
            turn_points = self._turn_points
            straight_points = max(self._n_points - turn_points, 0)
            s_values = [
                self._turn_length_m * i / (turn_points - 1) for i in range(turn_points)
            ]
            remaining_len = self._length_m - self._turn_length_m
            if straight_points > 0 and remaining_len > 0:
                s_values += [
                    self._turn_length_m + remaining_len * i / straight_points
                    for i in range(1, straight_points + 1)
                ]

        return [
            self._point_at(s, turn_sign, theta_max, turn_radius, x_end, y_end, straight_turn)
            for s in s_values
        ]

    @staticmethod
    def _make_pose_stamped(header: Header, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish(self):
        if self._anchor_frame == 'camera_link':
            self._publish_camera_relative()
        else:
            self._publish_odom_fixed()

    def _publish_camera_relative(self):
        msg = Path()
        msg.header.frame_id = 'camera_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self._local_points():
            msg.poses.append(self._make_pose_stamped(msg.header, x, y, yaw))
        self._pub.publish(msg)

    def _publish_odom_fixed(self):
        if self._cached_odom_poses is None:
            local_header = Header()
            local_header.frame_id = 'camera_link'
            try:
                fixed = []
                anchor_tf = self._tf_buffer.lookup_transform(
                    'odom', 'camera_link', Time(), timeout=Duration(seconds=0.2))
                for x, y, yaw in self._local_points():
                    local_ps = self._make_pose_stamped(local_header, x, y, yaw)
                    world_ps = self._tf_buffer.transform(
                        local_ps, 'odom', timeout=Duration(seconds=0.2))
                    fixed.append(world_ps.pose)
            except Exception as e:
                self.get_logger().warn(
                    f'odom -> camera_link TF 아직 없음, 대기 중 (EKF 떠 있나요?): {e}',
                    throttle_duration_sec=2.0)
                return
            self._cached_odom_poses = fixed
            self._anchor_yaw = self._quat_yaw(anchor_tf.transform.rotation)
            self._anchor_time = self.get_clock().now()
            p0 = fixed[0].position
            self.get_logger().info(
                f'경로를 odom 프레임에 1회 고정 완료 (시작점 x={p0.x:.3f}, y={p0.y:.3f}, '
                f'앵커 헤딩 {math.degrees(self._anchor_yaw):.1f}deg) '
                '-- 이후 로봇이 움직여도 경로는 이 자리에 고정됩니다.'
            )

        # --drift-speed가 0이 아니면 앵커 헤딩 방향으로 경과시간 * 속도만큼
        # 경로 전체를 강체 이동시킴 (모양은 그대로, 위치만 등속으로 미끄러짐).
        dx = dy = 0.0
        if self._drift_speed != 0.0:
            elapsed_s = (self.get_clock().now() - self._anchor_time).nanoseconds * 1e-9
            offset = self._drift_speed * elapsed_s
            dx = offset * math.cos(self._anchor_yaw)
            dy = offset * math.sin(self._anchor_yaw)

        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self._cached_odom_poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pose.position.x + dx
            ps.pose.position.y = pose.position.y + dy
            ps.pose.position.z = pose.position.z
            ps.pose.orientation = pose.orientation
            msg.poses.append(ps)
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length', type=float, default=1.57, help='총 경로 길이 [m] (회전+직진 연장 합)')
    parser.add_argument('--turn-length', type=float, default=0.3,
                         help='회전을 끝내는 데 쓸 전방 거리 [m] -- 작을수록 로봇 코앞에서 급하게 꺾음')
    parser.add_argument('--turn-deg', type=float, default=90.0,
                         help='회전각 [deg] (양수=좌회전, 음수=우회전, 0=직진)')
    parser.add_argument('--points', type=int, default=60, help='경로 점 개수 (총합)')
    parser.add_argument(
        '--turn-points', type=int, default=20,
        help=(
            '회전 구간(0~turn_length_m) 전용 점 개수. --points 중 이만큼을 회전 '
            '구간에 촘촘히 배분하고 나머지는 직진 연장에 배분 -- 클수록 원호가 '
            '더 매끄럽게 보임. turn-deg 0(직진)일 땐 무시됨.'
        ))
    parser.add_argument('--rate', type=float, default=15.0, help='발행 주기 [Hz]')
    parser.add_argument(
        '--frame', choices=['camera_link', 'odom'], default='camera_link',
        help=(
            'camera_link(기본): 매 틱 로봇 코앞 기준 상대경로 재발행(로봇 따라 밀림). '
            'odom: 시작 시점 카메라 위치 기준으로 경로를 odom에 1회 고정(안 밀림, EKF 필요).'
        ))
    parser.add_argument(
        '--drift-speed', type=float, default=0.0,
        help=(
            '--frame odom 전용. 앵커 헤딩 방향으로 경로 전체를 이 속도[m/s]로 '
            '등속 이동(양수=멀어짐, 음수=다가옴). 기본 0=완전 고정.'
        ))
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticLeftTurnPublisher(
        args.length, args.turn_length, args.turn_deg, args.points, args.rate,
        args.frame, args.drift_speed, args.turn_points)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
