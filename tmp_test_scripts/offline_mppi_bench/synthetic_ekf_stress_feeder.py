#!/usr/bin/env python3
"""synthetic_ekf_stress_feeder.py -- [테스트용] 실제 robot_localization
ekf_node를 하드웨어 없이 스트레스 테스트하기 위한 가짜 /imu, /wheel/odom
퍼블리셔. ekf_local.yaml이 문서화한 실제 발산 시나리오(myAHRS+ ~10.1Hz
orientation-only IMU가 잠깐 끊기는 동안에도 휠은 계속 vx를 보고하면,
EKF가 stale yaw로 vx를 적분해서 위치가 발산)를 그대로 재현한다:

    0 ~ gap_start_s      : IMU/휠 둘 다 정상 주기로 발행 (직진, vx 일정)
    gap_start_s ~ +gap_duration_s : IMU만 끊음(휠은 계속 정상 발행 --
                            "바퀴는 계속 도는데 IMU만 잠깐 죽음" 시나리오)
    그 이후            : IMU 다시 정상 발행 (회복 여부 관찰용)
gap_duration_s:=0으로 주면 이 갭 주입은 완전히 꺼진다.

[회전 추적 시나리오 추가, process_noise_covariance.vyaw 실험용] turn_start_s
~ +turn_duration_s 구간 동안 turn_wz_rad_s(rad/s)로 실제 회전하는 것처럼
IMU orientation의 yaw를 그 시점까지 적분한 값으로 발행한다(회전 후엔 그
최종 yaw로 고정 유지). "진짜 yaw"를 매 IMU 발행 시점마다 /debug/true_yaw
(std_msgs/Float32)로도 같이 발행하므로, ekf_divergence_monitor.py의
true_yaw_topic 파라미터로 구독시키면 EKF가 추정한 yaw가 이 실제 회전을
얼마나 빠르고 정확하게 따라가는지(반응성) 볼 수 있다 -- vyaw process
noise를 낮췄을 때 "회전 반응이 느려지는" 트레이드오프가 실제로 있는지
확인하는 용도.

ekf_divergence_monitor.py를 같이 띄워서, gap 구간 직후 covariance 급증/
위치 발산 경고가 뜨는지, 또는(turn 시나리오에서는) yaw 추적 오차가 커지는지
보는 용도.

사용:
    python3 synthetic_ekf_stress_feeder.py
    python3 synthetic_ekf_stress_feeder.py --ros-args -p gap_duration_s:=0.6
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class SyntheticEkfStressFeeder(Node):
    def __init__(self):
        super().__init__('synthetic_ekf_stress_feeder')

        # [2026-08-23 수정] 10.1 -> 50.0. ekf.launch.py의 myahrs_driver_node가
        # 오늘(commit 7290f9c) output_divider:10(~10Hz)에서 output_divider:2
        # (50Hz)로 이미 바뀌어 있었음 -- ekf_local.yaml 주석의 "10.1Hz 실측"은
        # 그 이전 상태를 설명한 것이라 그대로 베꼈다가 실제 설정과 어긋났음.
        # 실제 launch 파일(ekf.launch.py)의 driver 파라미터를 소스로 삼을 것.
        self.declare_parameter('imu_rate_hz', 50.0)
        self.declare_parameter('wheel_rate_hz', 50.0)
        self.declare_parameter('vx_mps', 0.3)
        self.declare_parameter('gap_start_s', 5.0)
        # sensor_timeout_s(ekf_local.yaml 0.3)보다 확실히 크게 -- 기본값은
        # 그 2배(0.6s)라 갭이 timeout을 확실히 넘기게 함.
        self.declare_parameter('gap_duration_s', 0.6)
        self.declare_parameter('turn_start_s', -1.0)  # -1.0 = 회전 시나리오 비활성화
        self.declare_parameter('turn_duration_s', 2.0)
        self.declare_parameter('turn_wz_rad_s', 0.5)

        self.turn_start_s = float(self.get_parameter('turn_start_s').value)
        self.turn_duration_s = float(self.get_parameter('turn_duration_s').value)
        self.turn_wz_rad_s = float(self.get_parameter('turn_wz_rad_s').value)

        self.imu_period = 1.0 / float(self.get_parameter('imu_rate_hz').value)
        self.wheel_period = 1.0 / float(self.get_parameter('wheel_rate_hz').value)
        self.vx_mps = float(self.get_parameter('vx_mps').value)
        self.gap_start_s = float(self.get_parameter('gap_start_s').value)
        self.gap_duration_s = float(self.get_parameter('gap_duration_s').value)

        self.imu_pub = self.create_publisher(Imu, '/imu', 20)
        self.wheel_pub = self.create_publisher(Odometry, '/wheel/odom', 20)
        self.true_yaw_pub = self.create_publisher(Float32, '/debug/true_yaw', 20)

        self._start_time = self.get_clock().now()
        self._gap_active_logged = False
        self._gap_done_logged = False

        self.create_timer(self.imu_period, self._publish_imu)
        self.create_timer(self.wheel_period, self._publish_wheel_odom)

        turn_desc = (
            f"t={self.turn_start_s}s부터 {self.turn_duration_s}s간 {self.turn_wz_rad_s}rad/s 회전"
            if self.turn_start_s >= 0.0 else "회전 시나리오 비활성화(직진만)")
        self.get_logger().info(
            f"synthetic_ekf_stress_feeder 시작: imu={1.0/self.imu_period:.1f}Hz "
            f"wheel={1.0/self.wheel_period:.1f}Hz vx={self.vx_mps}m/s -- "
            f"t={self.gap_start_s}s부터 {self.gap_duration_s}s간 IMU만 끊음, {turn_desc}."
        )

    def _elapsed_s(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _in_gap(self) -> bool:
        t = self._elapsed_s()
        active = self.gap_start_s <= t < (self.gap_start_s + self.gap_duration_s)
        if active and not self._gap_active_logged:
            self.get_logger().warn(f"[주입 시작] t={t:.2f}s -- IMU 발행 중단 (휠은 계속 발행)")
            self._gap_active_logged = True
        if not active and self._gap_active_logged and not self._gap_done_logged:
            self.get_logger().warn(f"[주입 종료] t={t:.2f}s -- IMU 발행 재개")
            self._gap_done_logged = True
        return active

    def _true_yaw(self) -> float:
        """turn_start_s < 0이면 회전 시나리오 비활성화(항상 직진, yaw=0).
        활성화 시 turn_start_s부터 turn_duration_s 동안 turn_wz_rad_s로
        일정하게 회전하다가, 그 뒤로는 도달한 최종 yaw를 그대로 유지."""
        if self.turn_start_s < 0.0:
            return 0.0
        t = self._elapsed_s()
        if t <= self.turn_start_s:
            return 0.0
        elapsed_in_turn = min(t - self.turn_start_s, self.turn_duration_s)
        return self.turn_wz_rad_s * elapsed_in_turn

    def _publish_imu(self) -> None:
        if self._in_gap():
            return  # 이 구간엔 아예 발행 안 함 (실제 드롭아웃 재현)

        yaw = self._true_yaw()
        self.true_yaw_pub.publish(Float32(data=yaw))

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        # roll/pitch는 평지 주행 가정으로 0 고정, yaw만 _true_yaw()를 따름.
        msg.orientation.w = math.cos(yaw / 2.0)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw / 2.0)
        # myAHRS+와 동일하게 raw gyro 없음 표시 (imu0_config가 각속도는
        # 어차피 안 쓰지만, driver 실제 동작과 맞춤).
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        # orientation은 실제로 신뢰해서 쓰므로 사실적인(작은) 분산.
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.001
        self.imu_pub.publish(msg)

    def _publish_wheel_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = self.vx_mps
        msg.twist.covariance[0] = 0.01  # vx만 신뢰(odom0_config) -- 사실적인 작은 분산
        self.wheel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticEkfStressFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
