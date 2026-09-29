"""
gyro_odom_monitor -- 실차 주행 중 gyro 기반 odom(reduced_odom, use_imu_gyro=true)을
감시하는 모니터. 로컬 PC에서 로봇 토픽을 받아 1초 요약을 화면/CSV에 찍고,
중요한 순간은 이벤트 줄(>>)로 바로 알린다.

쓰는 법 (로컬 PC, 로봇과 같은 ROS_DOMAIN_ID):
    python3 gyro_odom_monitor.py

1초 요약 열:
    ekf     : /odometry/filtered yaw (deg, 연속 누적) -- 실제 odom이 쓰는 값
    gyro    : /imu/yaw_gyro_integrated (deg) -- gyro만 적분한 yaw
    wheel   : /wheel/odom wz 를 모니터가 따로 적분한 yaw (deg, 시작 시 ekf에 맞춤)
    slip    : wheel - ekf (deg) -- 휠만 썼다면 생겼을 yaw 오차(스키드 슬립)
    ahrs    : /imu/yaw_relative (deg) -- 참고용, odom에는 안 들어감
    wz g/w  : gyro(bias 보정) / wheel 각속도 (deg/s)
    cmd v/w : /cmd_vel (m/s, rad/s)
    src     : EKF yaw rate 출처 GYRO / WHEEL(gyro 끊김 또는 보정 전)
    Hz i/w  : /imu, /wheel/odom 수신 주기

이벤트(>>):
    - 복귀 상태 변화, gyro 보정 완료(bias), gyro 끊김/복귀(WHEEL 전환)
    - 회전 정지 후 미끄러짐: 회전 명령(|cmd w|)이 0이 된 순간부터 gyro가 멈출
      때까지 더 돈 각도. 궤도 스키드 조향 오버슈트를 직접 잰다.

로그 파일: gyro_odom_monitor.csv (append)
"""
import csv
import math
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String

CSV_PATH = 'gyro_odom_monitor.csv'
IMU_GAP_S = 0.3           # reduced_odom sensor_timeout과 동일
CMD_W_ACTIVE = 0.05       # rad/s, 이 이상이면 "회전 명령 중"
STILL_WZ_DPS = 1.0        # deg/s, gyro가 이 이하로 STILL_HOLD_S 유지되면 "멈춤"
STILL_HOLD_S = 0.3


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Unwrap:
    def __init__(self):
        self.prev = None
        self.acc = 0.0

    def __call__(self, a):
        if self.prev is not None:
            self.acc += (a - self.prev + math.pi) % (2 * math.pi) - math.pi
        else:
            self.acc = a
        self.prev = a
        return self.acc


class GyroOdomMonitor(Node):
    def __init__(self):
        super().__init__('gyro_odom_monitor')
        self.t0 = time.time()
        self.state = '?'
        self.ekf_unwrap = Unwrap()
        self.ekf_yaw = None          # rad, unwrapped
        self.gyro_yaw = None         # rad
        self.ahrs_yaw = None         # rad
        self.wz_gyro = None          # rad/s (bias corrected)
        self.bias = None             # rad/s
        self.wz_wheel = None         # rad/s
        self.wheel_yaw = None        # rad, integrated here
        self.wheel_t = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.src = '?'
        self.calibrated = None
        self.last_imu = None
        self.imu_gap_reported = False
        self.imu_n = 0
        self.wheel_n = 0
        # 회전 정지 미끄러짐 측정
        self.rotating_cmd = False
        self.stop_yaw = None
        self.stop_t = None
        self.still_since = None

        self.create_subscription(String, '/mission/return/state', self.on_state, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Float64, '/imu/yaw_gyro_integrated', self.on_gyro_yaw, 10)
        self.create_subscription(Float64, '/imu/yaw_relative', self.on_ahrs, 10)
        self.create_subscription(Float64, '/imu/gyro/z_corrected', self.on_wz_gyro, 10)
        self.create_subscription(Float64, '/imu/gyro/bias_z', self.on_bias, 10)
        self.create_subscription(DiagnosticArray, '/odometry/diagnostics', self.on_diag, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        self.csv_f = open(CSV_PATH, 'a', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['wall_time', 'kind', 'state', 'ekf_deg', 'gyro_deg', 'wheel_deg',
                             'slip_deg', 'ahrs_deg', 'wz_gyro_dps', 'wz_wheel_dps', 'cmd_v',
                             'cmd_w', 'src', 'imu_hz', 'wheel_hz', 'note'])
        self.create_timer(1.0, self.tick)
        self.create_timer(0.1, self.check_imu_gap)
        print(f'gyro_odom_monitor 시작 -- 로그: {CSV_PATH} (열 설명은 파일 상단 docstring)')
        self.header()

    # ---------- callbacks ----------
    def on_state(self, m):
        if m.data != self.state:
            self.event(f'state {self.state} -> {m.data}')
            self.state = m.data

    def on_odom(self, m):
        self.ekf_yaw = self.ekf_unwrap(yaw_of(m.pose.pose.orientation))
        if self.wheel_yaw is None:
            self.wheel_yaw = self.ekf_yaw

    def on_wheel(self, m):
        self.wheel_n += 1
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.wz_wheel = m.twist.twist.angular.z
        if self.wheel_t is not None and self.wheel_yaw is not None and 0.0 < t - self.wheel_t < 0.5:
            self.wheel_yaw += self.wz_wheel * (t - self.wheel_t)
        self.wheel_t = t

    def on_imu(self, m):
        self.imu_n += 1
        self.last_imu = time.time()
        if self.imu_gap_reported:
            self.imu_gap_reported = False
            self.event('IMU 수신 재개')

    def on_gyro_yaw(self, m):
        self.gyro_yaw = m.data

    def on_ahrs(self, m):
        self.ahrs_yaw = m.data

    def on_wz_gyro(self, m):
        self.wz_gyro = m.data
        self.track_stop_slide()

    def on_bias(self, m):
        self.bias = m.data

    def on_cmd(self, m):
        self.cmd_v = m.linear.x
        self.cmd_w = m.angular.z

    def on_diag(self, m):
        for s in m.status:
            if s.name != 'reduced_odom/estimator':
                continue
            kv = {v.key: v.value for v in s.values}
            if 'gyro_calibrated' not in kv:
                if self.src != 'NO-GYRO':
                    self.event('reduced_odom가 gyro 모드가 아님 (use_imu_gyro=false?)')
                self.src = 'NO-GYRO'
                return
            cal = kv['gyro_calibrated'] == '1'
            if cal and self.calibrated is False:
                b = float(kv.get('gyro_bias_z', 'nan'))
                self.event(f'gyro 보정 완료: bias {math.degrees(b):+.3f} deg/s -> odom yaw 0에서 시작')
            if not cal and self.calibrated is not False:
                self.event('gyro 보정 대기 중 -- 로봇을 정지시켜 두세요')
            was_cal = self.calibrated
            self.calibrated = cal
            src = 'GYRO' if kv.get('wz_source_gyro') == '1' else 'WHEEL'
            # 보정 전의 WHEEL -> 보정 완료 시 GYRO 는 정상 시작이라 알리지 않는다.
            if src != self.src and self.src != '?' and was_cal:
                if src == 'WHEEL' and cal:
                    self.event('!! EKF yaw rate가 WHEEL로 전환 (gyro 끊김) -- 슬립 오차 누적 중')
                elif src == 'GYRO':
                    self.event('EKF yaw rate가 GYRO로 복귀')
            self.src = src

    # ---------- detectors ----------
    def check_imu_gap(self):
        if self.last_imu is None or self.imu_gap_reported:
            return
        gap = time.time() - self.last_imu
        if gap > IMU_GAP_S:
            self.imu_gap_reported = True
            self.event(f'!! IMU 끊김 {gap:.2f}s (포트/케이블/드라이버 확인)')

    def track_stop_slide(self):
        if self.gyro_yaw is None or self.wz_gyro is None:
            return
        now = time.time()
        if abs(self.cmd_w) > CMD_W_ACTIVE:
            self.rotating_cmd = True
            self.stop_yaw = None
            return
        if self.rotating_cmd:
            # 회전 명령이 방금 0이 됨 -> 이 순간 yaw 기록
            self.rotating_cmd = False
            self.stop_yaw = self.gyro_yaw
            self.stop_t = now
            self.still_since = None
            return
        if self.stop_yaw is None:
            return
        if abs(math.degrees(self.wz_gyro)) < STILL_WZ_DPS:
            self.still_since = self.still_since or now
            if now - self.still_since >= STILL_HOLD_S:
                slide = math.degrees(self.gyro_yaw - self.stop_yaw)
                dur = self.still_since - self.stop_t
                self.event(f'회전 정지 후 미끄러짐 {slide:+.1f} deg ({dur:.2f}s 동안 더 돎)')
                self.stop_yaw = None
        else:
            self.still_since = None

    # ---------- output ----------
    @staticmethod
    def d(x, fmt='{:+7.1f}'):
        return '   --  ' if x is None else fmt.format(math.degrees(x))

    def header(self):
        print(f"{'time':>8} {'state':<18} {'ekf':>7} {'gyro':>7} {'wheel':>7} {'slip':>7} "
              f"{'ahrs':>7} | {'wz g':>6} {'wz w':>6} | {'cmd v':>5} {'cmd w':>6} | "
              f"{'src':<5} {'Hz i/w':>7}")

    def tick(self):
        elapsed = int(time.time() - self.t0)
        if elapsed % 20 == 0:
            self.header()
        slip = (self.wheel_yaw - self.ekf_yaw
                if self.wheel_yaw is not None and self.ekf_yaw is not None else None)
        imu_hz, wheel_hz = self.imu_n, self.wheel_n
        self.imu_n = self.wheel_n = 0
        print(f"{time.strftime('%H:%M:%S')} {self.state[:18]:<18} {self.d(self.ekf_yaw)} "
              f"{self.d(self.gyro_yaw)} {self.d(self.wheel_yaw)} {self.d(slip)} {self.d(self.ahrs_yaw)} | "
              f"{self.d(self.wz_gyro, '{:+6.1f}')} {self.d(self.wz_wheel, '{:+6.1f}')} | "
              f"{self.cmd_v:5.2f} {self.cmd_w:+6.2f} | {self.src:<5} {imu_hz:3d}/{wheel_hz:<3d}",
              flush=True)
        self.row('tick', slip, imu_hz, wheel_hz, '')

    def event(self, text):
        print(f"{time.strftime('%H:%M:%S')} >> {text}", flush=True)
        self.row('event', None, '', '', text)

    def row(self, kind, slip, imu_hz, wheel_hz, note):
        g = lambda x: '' if x is None else f'{math.degrees(x):.2f}'
        self.csv_w.writerow([f'{time.time():.3f}', kind, self.state, g(self.ekf_yaw),
                             g(self.gyro_yaw), g(self.wheel_yaw), g(slip), g(self.ahrs_yaw),
                             g(self.wz_gyro), g(self.wz_wheel), f'{self.cmd_v:.3f}',
                             f'{self.cmd_w:.3f}', self.src, imu_hz, wheel_hz, note])
        self.csv_f.flush()


def main():
    rclpy.init()
    n = GyroOdomMonitor()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.csv_f.close()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
