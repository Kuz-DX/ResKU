#!/usr/bin/env python3
"""
autonomous.planz.py

[2026-09-02 복원] dolbotz 패키지 purepursuit.py의 2026-08-27 시점(커밋
3e3d81782f39b3a6ffb6887bf9ea74f5e4350664) 코드를 robot_bringup 패키지로
그대로 되살린 독립 사본. 그 이후 dolbotz/purepursuit.py에 추가된
escort_follow 전용 기능들(거리 비례 속도, bearing steering, 피벗 턴, 좌우
오프셋 데드밴드, 가감속 상한, LOST 유예시간 분리 등)은 전혀 포함하지 않은,
가장 단순했던 시점의 순수 pure pursuit 구현이다. 현재의
dolbotz/dolbotz/purepursuit.py와는 완전히 별개의 코드/노드이며 서로
수정을 공유하지 않는다.

Pure Pursuit 기반 경로추종 컨트롤러 -- Nav2 MPPI(controller_server)가 하던
"/path -> 모터 명령" 역할을 대체한다. planner_server/bt_navigator/
path_relay_node/controller_server(FollowPath 액션)를 전혀 거치지 않고,
/path(인지팀 최종 경로, slope_decision.py가 15Hz로 발행, camera_link 프레임의
body 좌표 x=전방/y=좌측)를 직접 구독해서 정적 TF(base_link<-camera_link,
reduced_odom_bringup.launch.py가 CAD 실측값으로 쏨)로 base_link 좌표로 옮긴 뒤, 매 프레임
그 자리에서 pure pursuit으로 조향을 계산한다. 로봇은 항상 base_link 원점이라
EKF(/odometry/filtered)나 전역(odom) 위치추정이 필요 없다 -- 경로 자체가 매
프레임 로봇 기준으로 새로 갱신되기 때문 (flat_drive.py/gradient_map.py도
동일한 상대좌표 규약을 씀).

산출된 v(선속도)/w(각속도)는 rmd_x8_driver_node._skid_steer_inverse() +
_send_speed_command()와 동일한 공식으로 좌우 바퀴 dps로 바꿔서, can_driver_node
(can_driver 패키지, manual.launch.py가 조종 모드에서 쓰는 그 노드)가 그대로
구독하는 /motor_speed_cmd(std_msgs/Float32MultiArray, [left_dps, right_dps])로
발행한다 -- rmd_x8_driver_node/controller_server/joy_mux_node/current_ramp_node
체인은 전혀 거치지 않는, can_driver_node만 재사용하는 독립된 새 경로다.

구독:
    /path (nav_msgs/Path) -- slope_decision.py가 발행하는 최종 경로.
                              frame_id는 보통 'camera_link', 좌표는 body
                              규약(x=전방, y=좌측). force_mode가 무엇이든
                              (flat_drive/gradient_map 어느 쪽이든) 최종
                              선택된 경로가 이 토픽 하나로 나온다.

발행:
    /motor_speed_cmd (std_msgs/Float32MultiArray) -- [left_dps, right_dps].
                              can_driver_node가 그대로 구독. can_driver_node
                              의 cmd_timeout_sec(기본 0.3s) 워치독에 안 걸리게
                              control_rate_hz(기본 20Hz)로 항상 재발행한다
                              (정지 상태에서도 [0, 0]을 계속 보냄).

TF:
    base_frame(기본 'base_link') <- 경로의 header.frame_id 를 경로의
    header.stamp 시점으로 조회해서 각 포즈를 변환한다 (path_relay_node.cpp의
    TF 변환 로직과 동일한 방식). 조회 실패/경로 수신 자체가 끊기면
    path_timeout_sec 안전망이 정지 명령을 낸다.

파라미터 (실측/현장 튜닝 전 placeholder 다수):
    path_topic                str    '/path'
    base_frame                str    'base_link'
    tf_lookup_timeout_sec     float  0.1    (path_relay_node와 동일 기본값)
    path_timeout_sec          float  1.0    (path_relay_node와 동일 기본값 --
                                             이보다 오래 /path가 안 오면 정지)
    lookahead_distance_m      float  0.8    실측 전 placeholder, 현장 튜닝 필요
    target_linear_speed_m_s   float  0.3    실측 전 placeholder, 현장 튜닝 필요
    goal_tolerance_m          float  0.3    경로 마지막 점까지 이 거리 이내면
                                            도착으로 보고 정지
    track_width_m              float 0.50   rmd_x8_driver_node의
                                            effective_track_width_m 기본값과
                                            동일 (실물 캘리브레이션 필요, 그쪽
                                            control_guide 3.1 참고)
    wheel_radius_m              float 0.1125 rmd_x8_driver_node와 동일
    left_motor_sign/right_motor_sign
                                 float 1.0/-1.0  manual_joy_control_node와
                                            동일 극성 (같은 can_driver_node를
                                            그대로 쓰므로 -- 배선이 바뀌면
                                            거기와 같이 맞춰서 뒤집을 것)
    max_wheel_speed_dps          float 800.0  manual_joy_control_node의
                                            max_speed_dps 기본값과 동일
    control_rate_hz               float 20.0
    cmd_topic                     str  '/motor_speed_cmd'

    use_outer_wheel_boost         bool  False  [2026-08-27 신규] 경사 주행용.
                                            기본(False)은 기존과 동일한 대칭
                                            스큐-스티어(v_left=v-w*halftrack,
                                            v_right=v+w*halftrack) -- 회전할수록
                                            안쪽 바퀴가 target_linear_speed_m_s
                                            밑으로 깎임. True면 _skid_steer_to_dps()
                                            에서 v 자체를 v+|w|*halftrack으로
                                            올려서 보내, 안쪽 바퀴는
                                            target_linear_speed_m_s에 그대로
                                            고정하고 바깥쪽만 부스트한다 --
                                            slope_traverse_node.cpp의
                                            computeBoostedVx()와 동일한 근거
                                            (매뉴얼 주행 실측: 한쪽 300dps/
                                            반대쪽 200dps 조합이 경사에서 잘
                                            됐다는 피드백). 경사 주행 시
                                            target_linear_speed_m_s를 그
                                            "안쪽" 목표(예: 0.4m/s, 약
                                            200dps)로 잡을 것.

실행:
    ros2 run robot_bringup autonomous.planz.py
    (robot_bringup은 ament_cmake 패키지이므로 setup.py entry_point가 아니라
    scripts/ 아래 install(PROGRAMS ...)로 등록된 실행 파일이다. CMakeLists.txt
    참고.)
"""
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

import tf2_geometry_msgs  # noqa: F401 -- PoseStamped 변환 등록 (do_transform_pose_stamped 사용)
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('purepursuit_node')

        # ---- 파라미터 ----
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_lookup_timeout_sec', 0.1)
        self.declare_parameter('path_timeout_sec', 1.0)

        self.declare_parameter('lookahead_distance_m', 0.5)
        self.declare_parameter('target_linear_speed_m_s', 0.3)
        self.declare_parameter('goal_tolerance_m', 0.3)

        self.declare_parameter('track_width_m', 0.50)
        self.declare_parameter('wheel_radius_m', 0.1125)
        self.declare_parameter('left_motor_sign', 1.0)
        self.declare_parameter('right_motor_sign', -1.0)
        self.declare_parameter('max_wheel_speed_dps', 800.0)

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_topic', '/motor_speed_cmd')

        # [2026-08-27 신규] 경사 주행용 -- 모듈 docstring 참고.
        self.declare_parameter('use_outer_wheel_boost', False)

        p = self.get_parameter
        self.base_frame = p('base_frame').value
        self.tf_lookup_timeout_sec = float(p('tf_lookup_timeout_sec').value)
        self.path_timeout_sec = float(p('path_timeout_sec').value)

        self.lookahead_distance_m = float(p('lookahead_distance_m').value)
        self.target_linear_speed_m_s = float(p('target_linear_speed_m_s').value)
        self.goal_tolerance_m = float(p('goal_tolerance_m').value)

        self.track_width_m = float(p('track_width_m').value)
        self.wheel_radius_m = float(p('wheel_radius_m').value)
        self.left_motor_sign = float(p('left_motor_sign').value)
        self.right_motor_sign = float(p('right_motor_sign').value)
        self.max_wheel_speed_dps = float(p('max_wheel_speed_dps').value)
        self.use_outer_wheel_boost = bool(p('use_outer_wheel_boost').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 상태 ----
        self._path_xy = []           # base_frame 기준 [(x, y), ...], 근->원 순서 유지
        self._last_path_time = None  # None = /path 아직 한 번도 못 받음 (조기 정지 스팸 방지)
        self._goal_reached = False

        # ---- ROS I/O ----
        path_topic = p('path_topic').value
        self.create_subscription(Path, path_topic, self._on_path, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)

        rate_hz = float(p('control_rate_hz').value)
        self.control_timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'purepursuit_node started: {path_topic} -> {self.base_frame}, '
            f'lookahead={self.lookahead_distance_m}m, v_target={self.target_linear_speed_m_s}m/s, '
            f'use_outer_wheel_boost={self.use_outer_wheel_boost}, '
            f'cmd_topic={cmd_topic} @ {rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    def _on_path(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn(
                'Received an empty /path -- treating as no path (stopping).',
                throttle_duration_sec=2.0)
            self._path_xy = []
            self._last_path_time = self.get_clock().now()
            self._goal_reached = False
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'TF lookup {self.base_frame} <- {msg.header.frame_id} failed: {exc}',
                throttle_duration_sec=2.0)
            return  # 이전에 받았던 경로/타임스탬프 유지 -- path_timeout_sec 안전망이 처리

        path_xy = []
        for pose_in in msg.poses:
            pose_out = tf2_geometry_msgs.do_transform_pose_stamped(pose_in, transform)
            path_xy.append((pose_out.pose.position.x, pose_out.pose.position.y))

        self._path_xy = path_xy
        self._last_path_time = self.get_clock().now()
        # 주의: 여기서 _goal_reached를 리셋하지 않는다 -- /path는 목표 근처에
        # 도달해 있는 동안에도 계속(15Hz) 재발행되므로, 매 수신마다 리셋하면
        # "한 번만 로그" 가드가 매번 풀려서 _compute_pure_pursuit()가 매 컨트롤
        # 틱마다 "Goal reached"를 다시 로그해버린다 (dryrun으로 확인된 스팸).
        # _goal_reached의 리셋은 _compute_pure_pursuit()가 실제 거리 기준으로
        # 판단해서 처리한다.

    # ------------------------------------------------------------------
    def _control_loop(self):
        left_dps, right_dps = 0.0, 0.0

        age_s = (
            (self.get_clock().now() - self._last_path_time).nanoseconds * 1e-9
            if self._last_path_time is not None else None
        )
        if age_s is None:
            pass  # /path 아직 한 번도 못 받음 -- 정지 유지
        elif age_s > self.path_timeout_sec:
            self.get_logger().warn(
                f'No /path for {age_s:.2f}s (timeout={self.path_timeout_sec}s). Stopping.',
                throttle_duration_sec=1.0)
        elif not self._path_xy:
            pass  # 빈 경로 -- 정지 유지
        else:
            v, w = self._compute_pure_pursuit(self._path_xy)
            left_dps, right_dps = self._skid_steer_to_dps(v, w)

        msg = Float32MultiArray()
        msg.data = [left_dps, right_dps]
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    def _compute_pure_pursuit(self, path_xy):
        """base_frame 기준 경로(근->원 순서)에서 lookahead 지점을 찾아 (v, w)를
        계산한다. 로봇은 항상 base_frame 원점, 전방 = +x, 좌측 = +y."""
        target = None
        for x, y in path_xy:
            if math.hypot(x, y) >= self.lookahead_distance_m:
                target = (x, y)
                break

        goal_x, goal_y = path_xy[-1]
        dist_to_goal = math.hypot(goal_x, goal_y)

        if target is None:
            # 경로 전체가 lookahead 반경 안 -- 마지막 점(경로 끝)을 목표로 삼는다.
            if dist_to_goal <= self.goal_tolerance_m:
                if not self._goal_reached:  # 도착 순간에만 한 번 로그 (엣지 트리거)
                    self.get_logger().info(f'Goal reached (dist={dist_to_goal:.2f}m). Stopping.')
                    self._goal_reached = True
                return 0.0, 0.0
            self._goal_reached = False
            target = (goal_x, goal_y)
        else:
            self._goal_reached = False

        x, y = target
        look_dist = max(math.hypot(x, y), 1e-3)  # 0 나눗셈 방지
        curvature = 2.0 * y / (look_dist * look_dist)  # 표준 pure pursuit 공식: kappa = 2*sin(alpha)/L = 2y/L^2

        v = self.target_linear_speed_m_s
        w = v * curvature
        return v, w

    # ------------------------------------------------------------------
    def _skid_steer_to_dps(self, v: float, w: float):
        """rmd_x8_driver_node._skid_steer_inverse() + _send_speed_command()와
        동일한 공식 (v[m/s], w[rad/s] -> 좌우 바퀴 각속도[dps]).

        use_outer_wheel_boost=True(경사 주행용, 모듈 docstring 참고)면 v를
        v+|w|*halftrack으로 올려서 안쪽 바퀴가 v(=target_linear_speed_m_s)
        밑으로 안 깎이고 바깥쪽만 부스트되게 한다 -- slope_traverse_node.cpp의
        computeBoostedVx()와 동일 공식(부호에 안 걸리게 |w| 사용, 대입해보면
        어느 방향으로 돌든 안쪽 바퀴가 정확히 v로 나옴을 검산할 수 있음)."""
        half_track = self.track_width_m / 2.0
        if self.use_outer_wheel_boost:
            v = v + abs(w) * half_track
        v_left = v - w * half_track
        v_right = v + w * half_track

        left_dps = math.degrees(v_left / self.wheel_radius_m) * self.left_motor_sign
        right_dps = math.degrees(v_right / self.wheel_radius_m) * self.right_motor_sign

        left_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, left_dps))
        right_dps = max(-self.max_wheel_speed_dps, min(self.max_wheel_speed_dps, right_dps))
        return left_dps, right_dps

    def destroy_node(self):
        # 종료 시 정지 명령을 한 번 명시적으로 발행 (can_driver_node의 워치독
        # 타임아웃만 믿지 않고, 노드가 정상 종료되는 경로에서는 즉시 멈추게)
        try:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.cmd_pub.publish(msg)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl+C(SIGINT) 시 rclpy의 기본 시그널 핸들러가 이미
        # rclpy.shutdown()을 먼저 호출해버리는 경우가 있어서(TF 리스너
        # 스레드가 있어서 종료가 살짝 늦어지는 이 노드에서 특히 재현됨),
        # 여기서 또 부르면 "rcl_shutdown already called" 예외가 나고
        # ros2 launch가 정상 종료를 "process has died"로 잘못 표시한다.
        # 이미 종료됐으면 건너뛴다 -- 정상 종료 경로를 크래시처럼 안 보이게.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
