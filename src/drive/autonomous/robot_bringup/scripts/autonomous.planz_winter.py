#!/usr/bin/env python3
"""
autonomous.planz_winter.py

[2026-09-05 신규] autonomous.planz.py(PlanZ, 2026-08-27 시점 purepursuit.py
복원본)를 그대로 복사한 뒤, autonomous.planz_spring.py(STARTUP 직진)와 반대
방향의 타이밍 동작 하나만 얹은 겨울 구간 전용 사본. autonomous.planz.py
원본은 손대지 않고, 이 파일만 따로 갈라져 나왔다 -- 원본/spring 사본과
서로 수정을 공유하지 않는다.

추가된 동작(STRAIGHT_LOCK): 노드 시작 후 straight_after_sec(기본 35초)
동안은 원본 autonomous.planz.py와 완전히 동일하게 /path 기반 pure
pursuit으로 주행한다. straight_after_sec가 지나면 그 순간부터
"영구적으로" /path를 더 이상 보지 않고 straight_drive_dps(기본 200dps,
좌우 바퀴 dps 크기 그대로) 직진으로 고정된다 -- 도로 pure pursuit으로
복귀하지 않는다(1회성 래치). 배경(사용자 확인): 겨울 트랙 경사 구간을
일정 시간 pure pursuit으로 오르고 나면, 그 이후 구간은 인지 경로를 믿지
않고 그냥 곧장 직진해도 되는(오히려 그게 더 안전한) 구간이라 시간 기준
전환으로 충분하다는 판단.

straight_after_sec/straight_drive_dps는 launch 인자로 노출한다
(mission_winter_drive_planz.launch.py 참고) -- 실기에서 재빌드 없이 값만
바꿔가며 튜닝하기 위함.

그 외 로직(구독/발행/TF/pure pursuit 계산/스키드조향 역기구학)은
autonomous.planz.py와 완전히 동일 -- 아래 원본 docstring 그대로 유지.

---- 아래는 autonomous.planz.py 원본 docstring ----

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

파라미터 (실측/현장 튜닝 전 placeholder 다수, autonomous.planz.py와 공통분은
그쪽 docstring 참고):

    [2026-09-05 신규, 이 사본 전용] straight_after_sec/straight_drive_dps --
    모듈 상단 설명 참고.

실행:
    ros2 run robot_bringup autonomous.planz_winter.py
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

        # [2026-08-27 신규] 경사 주행용 -- autonomous.planz.py docstring 참고.
        self.declare_parameter('use_outer_wheel_boost', False)

        # [2026-09-05 신규, 이 사본 전용] 모듈 상단 설명 참고 -- 이 시간이
        # 지나면 영구적으로(1회성 래치) /path를 무시하고 straight_drive_dps로
        # 직진 고정.
        self.declare_parameter('straight_after_sec', 35.0)
        self.declare_parameter('straight_drive_dps', 200.0)

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

        self.straight_after_sec = max(0.0, float(p('straight_after_sec').value))
        self.straight_drive_dps = float(p('straight_drive_dps').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 상태 ----
        self._path_xy = []           # base_frame 기준 [(x, y), ...], 근->원 순서 유지
        self._last_path_time = None  # None = /path 아직 한 번도 못 받음 (조기 정지 스팸 방지)
        self._goal_reached = False
        # [2026-09-05 신규] 노드 생성 시각 기준점 -- straight_after_sec가
        # 지나는 순간을 판정하는 데 쓴다.
        self._node_start_time = self.get_clock().now()
        # [2026-09-05 신규] 한 번 straight_after_sec를 넘으면 계속 True로
        # 래치 -- 그 뒤로는 시각 재계산 없이 곧장 직진 분기로 감(원본
        # planz_spring의 _startup_done_logged와 동일한 "한 번만 로그" 목적
        # 겸, 여기서는 실제 분기 자체도 이 플래그로 고정).
        self._straight_locked = False

        # ---- ROS I/O ----
        path_topic = p('path_topic').value
        self.create_subscription(Path, path_topic, self._on_path, 10)

        cmd_topic = p('cmd_topic').value
        self.cmd_pub = self.create_publisher(Float32MultiArray, cmd_topic, 10)

        rate_hz = float(p('control_rate_hz').value)
        self.control_timer = self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f'purepursuit_node (winter/STRAIGHT_LOCK) started: {path_topic} -> {self.base_frame}, '
            f'pure pursuit lookahead={self.lookahead_distance_m}m, v_target={self.target_linear_speed_m_s}m/s '
            f'for straight_after_sec={self.straight_after_sec}s, then permanently straight at '
            f'straight_drive_dps={self.straight_drive_dps}dps, '
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
    def _publish_straight_lock(self):
        # straight_drive_dps는 "바퀴 dps 크기" 그대로다 -- m/s로 변환하지
        # 않고, radians(dps)*wheel_radius_m으로 역산한 v를 w=0으로
        # _skid_steer_to_dps()에 넣으면 좌우 부호(left_motor_sign/
        # right_motor_sign)까지 기존 로직 그대로 재사용해서 정확히
        # 크기=straight_drive_dps인 직진 명령이 나온다(검산: w=0이면
        # v_left=v_right=v이므로 dps = degrees(radians(dps))*sign = dps*sign).
        v = math.radians(self.straight_drive_dps) * self.wheel_radius_m
        left_dps, right_dps = self._skid_steer_to_dps(v, 0.0)
        msg = Float32MultiArray()
        msg.data = [left_dps, right_dps]
        self.cmd_pub.publish(msg)

    def _control_loop(self):
        # [2026-09-05 신규] STRAIGHT_LOCK -- 한 번 straight_after_sec를
        # 넘으면 이후로는(elapsed_s를 다시 계산할 필요도 없이) 영구히 이
        # 분기만 탄다. /path 수신 여부와 무관하게 계속 직진.
        if self._straight_locked:
            self._publish_straight_lock()
            return

        elapsed_s = (self.get_clock().now() - self._node_start_time).nanoseconds * 1e-9
        if elapsed_s >= self.straight_after_sec:
            self._straight_locked = True
            self.get_logger().info(
                f'straight_after_sec={self.straight_after_sec}s elapsed -- '
                f'locking to permanent straight drive at {self.straight_drive_dps}dps '
                '(no longer following /path).')
            self._publish_straight_lock()
            return

        # straight_after_sec 전: 원본 autonomous.planz.py와 완전히 동일한
        # /path 기반 pure pursuit.
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

        use_outer_wheel_boost=True(경사 주행용, autonomous.planz.py docstring
        참고)면 v를 v+|w|*halftrack으로 올려서 안쪽 바퀴가 v 밑으로 안 깎이고
        바깥쪽만 부스트되게 한다."""
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
