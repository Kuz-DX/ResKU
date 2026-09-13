#!/usr/bin/env python3
"""ekf_divergence_monitor.py -- [테스트용] robot_localization ekf_node의
출력(/odometry/filtered)과 입력 센서(/imu, /wheel/odom)를 실시간으로 보면서
"발산"으로 볼 수 있는 징후들을 잡아내는 진단 노드.

배경(ekf_local.yaml 주석, 2026-08-23): myAHRS+가 raw gyro 없이 orientation만
~10.1Hz로 주는데, sensor_timeout이 그 주기에 너무 타이트하면(예전 0.1s)
지터로 IMU 보정이 스킵되는 사이클이 생기고, 그 사이 부정확한(stale) yaw로
vx를 적분해서 위치가 발산하는 게 실기에서 확인된 적 있음(그래서 0.3s로
상향). 이 노드는 그 특정 실패 모드(IMU 갭 -> 발산)를 포함해 일반적인 EKF
발산 징후를 잡는다:

  1. NaN/Inf -- pose/twist/covariance 어디든 하나라도 있으면 명백한 발산.
  2. covariance 급증 -- 최근 N개 대비 갑자기 몇 배로 뛰면(수렴 안 하고
     불확실성이 계속 커지는 중일 가능성) 경고.
  2-1. covariance 지속 증가(완만한 발산) -- 위 급증 탐지는 "최근 평균 대비
     갑자기 뛰는 것"만 잡아서, 매 사이클 조금씩 계속 커지기만 하고 안
     꺾이는 패턴(급하지 않지만 절대 수렴하지 않는 발산)은 놓친다(실측:
     이 노드로 실제 ekf_node를 IMU 갭 없이도 돌려보니 y/yaw covariance가
     처음부터 끝까지 한 번도 안 꺾이고 계속 커지는데 급증 탐지는 0건이었음
     -- 원인 추적 후 이 체크 추가). 그래서 "N회 연속으로 한 번도 감소
     안 함"을 별도로 센다.
  3. 비현실적 위치 점프 -- 연속된 두 pose 사이 내포된 속도가
     max_plausible_speed_mps를 넘으면(실제로 그렇게 빠를 수 없으므로) 경고.
  4. 센서 갭 -- /imu, /wheel/odom 각각 sensor_timeout_s보다 오래 안 오면
     경고 + 그 직후(gap_correlation_window_s 이내) 나오는 odom 이상 징후에
     "직전 센서 갭과 관련 있을 수 있음" 표시를 붙여서 인과관계를 눈으로
     보기 쉽게 함.
  5. (선택) 회전 추적 오차 -- true_yaw_topic 파라미터를 채우면
     synthetic_ekf_stress_feeder.py의 /debug/true_yaw(실제 시뮬레이션한
     참값 yaw)와 EKF가 추정한 yaw를 매 odom 콜백마다 비교해서 오차를
     로그한다. process_noise_covariance의 vyaw를 낮췄을 때 "회전 반응이
     느려지는" 트레이드오프가 실제로 있는지 정량적으로 보는 용도.

Ctrl+C 종료 시 전체 요약(발산 징후 총 횟수, 최대 covariance, 센서 갭
횟수/최대 길이)을 로그로 남긴다.

사용:
    python3 ekf_divergence_monitor.py
    python3 ekf_divergence_monitor.py --ros-args -p sensor_timeout_s:=0.3
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


def _has_nan_or_inf(values) -> bool:
    return any(math.isnan(v) or math.isinf(v) for v in values)


class EkfDivergenceMonitor(Node):
    def __init__(self):
        super().__init__('ekf_divergence_monitor')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        # ekf_local.yaml의 sensor_timeout과 맞출 것 -- 이 값보다 오래 센서가
        # 안 오면 EKF 입장에서도 "보정 못 받은 상태"로 dead-reckoning 중.
        self.declare_parameter('sensor_timeout_s', 0.3)
        # 이 배수 이상으로 covariance가(최근 window 대비) 급증하면 경고.
        self.declare_parameter('covariance_spike_ratio', 3.0)
        self.declare_parameter('covariance_window', 20)
        # 이만큼 연속으로 covariance가 한 번도 안 줄어들면(완만해도) "수렴 안
        # 하는 중"으로 보고 경고 -- 급증 탐지가 못 잡는 완만한 발산용.
        self.declare_parameter('monotonic_streak_warn', 15)
        # 이보다 빠른 내포 속도가 나오면 "물리적으로 불가능"으로 간주.
        # 현재 vx_max(0.45)보다 넉넉히 큰 값 -- 실제 상한이 아니라 "이 정도면
        # 명백히 이상하다"는 러프한 안전 마진.
        self.declare_parameter('max_plausible_speed_mps', 2.0)
        self.declare_parameter('gap_correlation_window_s', 1.0)
        self.declare_parameter('log_period_s', 1.0)
        # 비워두면 회전 추적 오차 체크 비활성화 (synthetic_ekf_stress_feeder.py의
        # /debug/true_yaw와 짝지어 쓸 것).
        self.declare_parameter('true_yaw_topic', '')
        self.declare_parameter('yaw_error_warn_rad', 0.1)

        self.sensor_timeout_s = float(self.get_parameter('sensor_timeout_s').value)
        self.covariance_spike_ratio = float(self.get_parameter('covariance_spike_ratio').value)
        self.max_plausible_speed_mps = float(self.get_parameter('max_plausible_speed_mps').value)
        self.gap_correlation_window_s = float(self.get_parameter('gap_correlation_window_s').value)
        self.log_period_s = float(self.get_parameter('log_period_s').value)

        cov_window = int(self.get_parameter('covariance_window').value)
        self._cov_x_hist = deque(maxlen=cov_window)
        self._cov_y_hist = deque(maxlen=cov_window)
        self._cov_yaw_hist = deque(maxlen=cov_window)
        self.monotonic_streak_warn = int(self.get_parameter('monotonic_streak_warn').value)
        # {label: [이전 값, 현재 연속 증가 횟수, 이번 streak을 이미 경고했는지]}
        self._monotonic_state = {
            'x': [None, 0, False], 'y': [None, 0, False], 'yaw': [None, 0, False]}
        self._n_monotonic_growth = 0

        self._prev_odom = None  # (t_sec, x, y)
        self._last_imu_time = None
        self._last_wheel_time = None
        self._last_gap_event_time = None  # 가장 최근 센서 갭이 감지된 시각(모니터 기준 시각)

        self._n_odom = 0
        self._n_nan_inf = 0
        self._n_cov_spike = 0
        self._n_jump = 0
        self._n_imu_gap = 0
        self._n_wheel_gap = 0
        self._max_gap_s = 0.0
        self._max_cov = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        self._true_yaw = None
        self.yaw_error_warn_rad = float(self.get_parameter('yaw_error_warn_rad').value)
        self._max_yaw_error = 0.0
        self._sum_abs_yaw_error = 0.0
        self._n_yaw_error_samples = 0

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 20)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._on_imu, 20)
        # [2026-08-24 수정] rmd_x8_driver_node의 /wheel/odom은 BEST_EFFORT로
        # 발행됨(실측: ros2 topic info -v) -- 기본 RELIABLE로 구독하면 DDS가
        # "그 약속을 못 지켜준다"고 판단해 아예 연결을 안 시켜줘서, 메시지를
        # 하나도 못 받으면서도 에러 없이 조용히 실패했었음(wheel_odom 갭
        # 감지가 처음부터 죽어있던 원인). 실제 발행 QoS에 맞춰 BEST_EFFORT로
        # 구독.
        wheel_odom_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            Odometry, str(self.get_parameter('wheel_odom_topic').value), self._on_wheel_odom,
            wheel_odom_qos)

        true_yaw_topic = str(self.get_parameter('true_yaw_topic').value)
        if true_yaw_topic:
            self.create_subscription(Float32, true_yaw_topic, self._on_true_yaw, 20)
            self.get_logger().info(f"회전 추적 오차 체크 활성화: true_yaw_topic='{true_yaw_topic}'")

        self.get_logger().info(
            f"ekf_divergence_monitor 시작: odom='{self.get_parameter('odom_topic').value}' "
            f"imu='{self.get_parameter('imu_topic').value}' "
            f"wheel_odom='{self.get_parameter('wheel_odom_topic').value}' "
            f"sensor_timeout_s={self.sensor_timeout_s}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _check_sensor_gap(self, topic_label, last_time_attr):
        now = self._now_s()
        last = getattr(self, last_time_attr)
        setattr(self, last_time_attr, now)
        if last is None:
            return
        gap = now - last
        if gap > self.sensor_timeout_s:
            self._max_gap_s = max(self._max_gap_s, gap)
            self._last_gap_event_time = now
            if topic_label == 'imu':
                self._n_imu_gap += 1
            else:
                self._n_wheel_gap += 1
            self.get_logger().warn(
                f"[센서 갭] {topic_label} 메시지 간격 {gap:.3f}s > "
                f"sensor_timeout_s({self.sensor_timeout_s}) -- EKF가 이 구간 동안 "
                f"이 센서 보정 없이 dead-reckoning 중이었을 가능성.")

    def _on_imu(self, _msg: Imu) -> None:
        self._check_sensor_gap('imu', '_last_imu_time')

    def _on_wheel_odom(self, _msg: Odometry) -> None:
        self._check_sensor_gap('wheel_odom', '_last_wheel_time')

    def _on_true_yaw(self, msg: Float32) -> None:
        self._true_yaw = msg.data

    def _correlation_note(self) -> str:
        if self._last_gap_event_time is None:
            return ''
        dt = self._now_s() - self._last_gap_event_time
        if dt <= self.gap_correlation_window_s:
            return f' (※ {dt:.2f}초 전 센서 갭과 관련 있을 수 있음)'
        return ''

    def _on_odom(self, msg: Odometry) -> None:
        self._n_odom += 1
        now = self._now_s()

        pose = msg.pose.pose
        twist = msg.twist.twist
        flat_vals = [
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ]
        if _has_nan_or_inf(flat_vals) or _has_nan_or_inf(msg.pose.covariance) or \
                _has_nan_or_inf(msg.twist.covariance):
            self._n_nan_inf += 1
            self.get_logger().error(
                f"[발산 의심] /odometry/filtered에 NaN/Inf 발견!{self._correlation_note()}")
            return  # NaN이 섞인 상태로 아래 covariance/jump 계산하면 의미 없음

        # nav_msgs/Odometry의 6x6 covariance는 row-major [x,y,z,roll,pitch,yaw]
        # 순서라 대각선 인덱스는 0,7,14,21,28,35 -- yaw는 인덱스 35.
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        cov_yaw = msg.pose.covariance[35]
        self._max_cov['x'] = max(self._max_cov['x'], cov_x)
        self._max_cov['y'] = max(self._max_cov['y'], cov_y)
        self._max_cov['yaw'] = max(self._max_cov['yaw'], cov_yaw)

        for label, hist, val in (
                ('x', self._cov_x_hist, cov_x),
                ('y', self._cov_y_hist, cov_y),
                ('yaw', self._cov_yaw_hist, cov_yaw)):
            if len(hist) >= 5:
                baseline = sum(hist) / len(hist)
                if baseline > 1e-9 and val > baseline * self.covariance_spike_ratio:
                    self._n_cov_spike += 1
                    self.get_logger().warn(
                        f"[발산 의심] covariance({label}) 급증: {val:.5f} "
                        f"(최근 평균 {baseline:.5f}의 {val / baseline:.1f}배)"
                        f"{self._correlation_note()}")
            hist.append(val)

            state = self._monotonic_state[label]
            prev, streak, warned = state
            if prev is not None and val > prev:
                streak += 1
            else:
                streak, warned = 0, False
            if streak >= self.monotonic_streak_warn and not warned:
                self._n_monotonic_growth += 1
                self.get_logger().warn(
                    f"[발산 의심] covariance({label})가 {streak}회 연속 한 번도 안 줄고 "
                    f"계속 증가 중 (지금 {val:.5f}) -- 수렴 안 하고 있을 가능성"
                    f"{self._correlation_note()}")
                warned = True
            self._monotonic_state[label] = [val, streak, warned]

        if self._true_yaw is not None:
            q = pose.orientation
            ekf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            err = math.atan2(math.sin(ekf_yaw - self._true_yaw), math.cos(ekf_yaw - self._true_yaw))
            self._n_yaw_error_samples += 1
            self._sum_abs_yaw_error += abs(err)
            self._max_yaw_error = max(self._max_yaw_error, abs(err))
            if abs(err) > self.yaw_error_warn_rad:
                self.get_logger().warn(
                    f"[회전 추적] yaw 오차 {math.degrees(err):+.1f}deg "
                    f"(EKF={math.degrees(ekf_yaw):.1f} true={math.degrees(self._true_yaw):.1f}) "
                    f"-- yaw_error_warn_rad({math.degrees(self.yaw_error_warn_rad):.1f}deg) 초과",
                    throttle_duration_sec=0.5)

        x, y = pose.position.x, pose.position.y
        if self._prev_odom is not None:
            t0, x0, y0 = self._prev_odom
            dt = now - t0
            if dt > 1e-6:
                implied_speed = math.hypot(x - x0, y - y0) / dt
                if implied_speed > self.max_plausible_speed_mps:
                    self._n_jump += 1
                    self.get_logger().warn(
                        f"[발산 의심] 비현실적 위치 점프: {implied_speed:.2f} m/s 내포 "
                        f"(dt={dt:.3f}s) -- max_plausible_speed_mps"
                        f"({self.max_plausible_speed_mps}) 초과{self._correlation_note()}")
        self._prev_odom = (now, x, y)

        self.get_logger().info(
            f"odom OK: pos=({x:.3f},{y:.3f}) cov(x,y,yaw)=({cov_x:.5f},{cov_y:.5f},{cov_yaw:.5f})",
            throttle_duration_sec=self.log_period_s)

    def print_summary(self) -> None:
        yaw_track_line = ''
        if self._n_yaw_error_samples > 0:
            mean_abs = self._sum_abs_yaw_error / self._n_yaw_error_samples
            yaw_track_line = (
                f"  회전 추적 오차: 평균|.|={math.degrees(mean_abs):.2f}deg "
                f"최대|.|={math.degrees(self._max_yaw_error):.2f}deg "
                f"(n={self._n_yaw_error_samples})\n")
        self.get_logger().info(
            "=== EKF 발산 모니터 요약 ===\n"
            f"{yaw_track_line}"
            f"  odom 메시지 수: {self._n_odom}\n"
            f"  NaN/Inf 발생: {self._n_nan_inf}\n"
            f"  covariance 급증 경고: {self._n_cov_spike}\n"
            f"  covariance 지속 증가(완만한 발산) 경고: {self._n_monotonic_growth}\n"
            f"  비현실적 위치 점프: {self._n_jump}\n"
            f"  IMU 갭(>{self.sensor_timeout_s}s): {self._n_imu_gap}회\n"
            f"  wheel_odom 갭(>{self.sensor_timeout_s}s): {self._n_wheel_gap}회\n"
            f"  관측된 최대 센서 갭: {self._max_gap_s:.3f}s\n"
            f"  관측된 최대 covariance(x,y,yaw): "
            f"({self._max_cov['x']:.5f}, {self._max_cov['y']:.5f}, {self._max_cov['yaw']:.5f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EkfDivergenceMonitor()
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
