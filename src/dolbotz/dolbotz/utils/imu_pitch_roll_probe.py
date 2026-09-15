#!/usr/bin/env python3
"""
imu_pitch_roll_probe.py — /drive/camera/imu를 몇 초간 구독해서 body 프레임
(x-전방, y-왼쪽, z-위) 기준 (roll, pitch)[rad/deg]를 계산해주는 1회성 실측
스크립트. dolbotz 패키지 의존성 없이 rclpy + sensor_msgs + numpy만
사용하므로 빌드/설치 없이
`python3 imu_pitch_roll_probe.py`로 바로 실행된다.

계산 로직은 이 파일에 독립적으로 정의되어 있으며 reduced_odom_bringup.
launch.py의 [2026-08-27 회전 실측]/[2026-08-30 회전 실측] 절에서 실제로 쓰인
수동 절차(ros2 topic echo + 직접 계산)를 자동화한 것뿐이다.

[중요] 측정 조건: 로봇 몸체가 완전히 정지 + 평평한 바닥 위에 있어야 한다.
로봇 자체가 기울어져 있으면 이 값도 같이 오염된다. 카메라 IMU는 카메라 자체
내장 센서라 섀시 기울기와 마운트 기울기가 결합된 총 기울기를 측정함).

[참고] 이전 실측(2026-08-27) 때 MIPI 스트림 에러로 12번 시도 중 4번만
성공한 전례가 있다 -- 샘플이 안 모이면(valid_count가 너무 적으면) 재시도할 것.
중앙값(median)을 쓰는 이유도 이런 드문 이상치에 덜 흔들리게 하기 위함이다.

사용법:
    ros2 launch dolbotz drive_cam.launch.py   # 카메라를 먼저 띄워야 함
    python3 imu_pitch_roll_probe.py                       # 기본 5초 구독
    python3 imu_pitch_roll_probe.py --duration-sec 10      # 더 길게 구독
    python3 imu_pitch_roll_probe.py --topic /drive/camera/imu
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

# Body 프레임(x=전방, y=왼쪽, z=위)에서 카메라 optical 프레임(x=오른쪽,
# y=아래, z=전방)으로 변환하는 상수.
R_BODY_TO_OPTICAL = np.array([
    [0., -1., 0.],
    [0., 0., -1.],
    [1., 0., 0.],
])
R_OPTICAL_TO_BODY = R_BODY_TO_OPTICAL.T


def roll_pitch_from_accel_body(accel_body: np.ndarray) -> tuple[float, float]:
    """표준 2축 기울기 공식.
    roll: 전방(x)축 중심 회전(양수=오른쪽이 아래로).
    pitch: 왼쪽(y)축 중심 회전(양수=기수가 아래로)."""
    ax, ay, az = accel_body
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.hypot(ay, az))
    return float(roll), float(pitch)


def _sensor_data_qos_depth1() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ImuProbe(Node):
    def __init__(self, topic: str):
        super().__init__('imu_pitch_roll_probe')
        self._samples_optical = []  # [(ax, ay, az), ...] optical frame, m/s^2
        self.create_subscription(Imu, topic, self._on_imu, _sensor_data_qos_depth1())

    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        self._samples_optical.append((a.x, a.y, a.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default='/drive/camera/imu')
    parser.add_argument('--duration-sec', type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImuProbe(args.topic)

    print(f"'{args.topic}' 구독 중 ({args.duration_sec:.1f}초)... "
          "로봇이 완전히 정지 + 평평한 바닥 위에 있는지 확인하세요.")
    start = time.monotonic()
    while time.monotonic() - start < args.duration_sec:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    n = len(node._samples_optical)
    if n == 0:
        print(f"샘플을 하나도 못 받았습니다 -- '{args.topic}' 토픽이 실제로 "
              "발행 중인지(drive_cam.launch.py가 떠 있는지) 확인하세요.")
        return

    accel_optical = np.array(node._samples_optical)  # (n, 3)
    rolls, pitches = [], []
    for row in accel_optical:
        accel_body = R_OPTICAL_TO_BODY @ row
        r, p = roll_pitch_from_accel_body(accel_body)
        rolls.append(r)
        pitches.append(p)
    rolls = np.array(rolls)
    pitches = np.array(pitches)

    roll_med, pitch_med = float(np.median(rolls)), float(np.median(pitches))
    roll_std, pitch_std = float(np.std(rolls)), float(np.std(pitches))

    print(f"\n샘플 수: {n} (std가 크면 로봇이 흔들렸거나 진짜 정지 상태가 "
          "아니었을 가능성 -- 재측정 권장)")
    print(f"roll  = {roll_med:+.4f} rad ({np.degrees(roll_med):+.2f} deg)  "
          f"std={roll_std:.4f} rad")
    print(f"pitch = {pitch_med:+.4f} rad ({np.degrees(pitch_med):+.2f} deg)  "
          f"std={pitch_std:.4f} rad")
    print(f"\n--roll {roll_med:.4f} --pitch {pitch_med:.4f}"
          " (static_transform_publisher 인자에 그대로 사용)")
    print(f"camera_roll_rad:={roll_med:.4f} camera_pitch_rad:={pitch_med:.4f}"
          " (mission_escort_drive.launch.py 오버라이드용)")


if __name__ == '__main__':
    main()
