#!/usr/bin/env python3
"""yaw 부호 검증(TEST A/B)용 실시간 모니터.

reduced_odom_node(use_imu_gyro:=true)의 /imu/* 디버그 토픽과 /imu의
angular_velocity.z, (있으면) /odometry/filtered yaw를 deg로 한 줄씩 출력하고,
종료 시(Ctrl+C 또는 --duration) 구간별 극값으로 TEST A/B를 판정한다.

  TEST A: 왼쪽(CCW) ~30deg  -> relative/unwrapped/gyro_integrated > 0, wz > 0
  TEST B: 원위치 후 오른쪽(CW) ~30deg -> 모두 < 0

사용:
  python3 yaw_sign_check.py [--duration 60]
"""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

KEYS = ['yaw_relative', 'yaw_unwrapped', 'yaw_gyro_integrated']
TURN_DEG = 15.0      # 이 이상 돌았을 때를 "회전"으로 판정
WZ_DEG_S = 5.0       # 이 이상일 때를 "회전 중"으로 판정


class YawSignCheck(Node):

    def __init__(self):
        super().__init__('yaw_sign_check')
        self.v = {k: None for k in KEYS + ['yaw_zero', 'yaw_raw', 'wz', 'ekf']}
        for k in KEYS + ['yaw_zero', 'yaw_raw']:
            self.create_subscription(Float64, f'/imu/{k}', self._mk(k), 10)
        self.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 10)
        self.extreme = {k: [0.0, 0.0] for k in KEYS + ['wz', 'ekf']}
        # 회전 중 부호 일치 여부: relative가 +/-TURN_DEG를 넘은 순간 각 값의 부호
        self.agree = {'CCW': [], 'CW': []}
        self.create_timer(0.2, self._print)

    def _mk(self, k):
        def cb(m):
            self.v[k] = math.degrees(m.data)
            if k in self.extreme:
                self._track(k, self.v[k])
        return cb

    def _imu(self, m):
        self.v['wz'] = math.degrees(m.angular_velocity.z)
        self._track('wz', self.v['wz'])

    def _odom(self, m):
        q = m.pose.pose.orientation
        self.v['ekf'] = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self._track('ekf', self.v['ekf'])

    def _track(self, k, x):
        e = self.extreme[k]
        e[0] = min(e[0], x)
        e[1] = max(e[1], x)

    def _print(self):
        f = lambda x: '   --  ' if x is None else f'{x:+7.2f}'
        v = self.v
        print(f"raw {f(v['yaw_raw'])} zero {f(v['yaw_zero'])} | rel {f(v['yaw_relative'])} "
              f"unwrap {f(v['yaw_unwrapped'])} gyro_int {f(v['yaw_gyro_integrated'])} "
              f"| wz {f(v['wz'])} deg/s | ekf {f(v['ekf'])}", flush=True)

    def report(self):
        print('\n==== 결과 (deg, 세션 중 최소/최대) ====')
        for k, (lo, hi) in self.extreme.items():
            print(f'{k:22s} min {lo:+8.2f}   max {hi:+8.2f}')
        ok = True
        for label, sign in (('TEST A (CCW)', +1), ('TEST B (CW)', -1)):
            res = []
            for k in KEYS:
                lo, hi = self.extreme[k]
                res.append((k, (hi if sign > 0 else -lo) >= TURN_DEG))
            lo, hi = self.extreme['wz']
            res.append(('angular_velocity.z', (hi if sign > 0 else -lo) >= WZ_DEG_S))
            passed = all(r for _, r in res)
            ok &= passed
            print(f'{label}: {"PASS" if passed else "FAIL"}  ' +
                  ', '.join(f'{k}={"O" if r else "X"}' for k, r in res))
        if not ok:
            print('-> 한 방향만 FAIL이면 그 방향으로 충분히(>15deg) 안 돌린 것일 수 있음.\n'
                  '-> CCW에서 모두 음수/CW에서 모두 양수로 나왔다면 부호가 반대: '
                  'imu_heading_sign(또는 드라이버 heading_sign)을 1.0으로.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0.0, help='초 단위, 0이면 Ctrl+C까지')
    args = ap.parse_args()
    rclpy.init()
    n = YawSignCheck()
    t_end = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (t_end is None or time.time() < t_end):
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.report()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
