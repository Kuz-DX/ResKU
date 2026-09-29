"""
calib_monitor -- 새 바닥에서 회전 보정치(effective_track_width_m)를 재는 동안
쓰는 모니터. 1초 상태 요약을 화면/로그 파일에 찍고, 각 회전 시험 구간마다
필터 yaw 변화량을 CSV에 남긴다.

쓰는 법:
    python3 calib_monitor.py

시험 절차 (사용자):
    1. 조이스틱으로 3~5초 제자리 회전 (a/rotate 버튼), 정확한 시작/끝 시각을
       화면에서 확인
    2. 바닥에 표시한 선으로 실제 회전각을 측정
    3. "회전 N: 시작 HH:MM:SS.s, 끝 HH:MM:SS.s, 실제 XX도 (좌/우)" 형식으로 알려주면
       compute_calibration()으로 바로 계산

로그 파일: calib_monitor.csv (append), 형식:
    wall_time, state, filt_yaw_deg_unwrapped, cmd_v, cmd_w
"""
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

CSV_PATH = 'calib_monitor.csv'
be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class Unwrap:
    def __init__(self):
        self.prev = None
        self.acc = 0.0

    def __call__(self, yaw_rad: float) -> float:
        if self.prev is None:
            self.prev = yaw_rad
            self.acc = yaw_rad
            return math.degrees(self.acc)
        d = (yaw_rad - self.prev + math.pi) % (2 * math.pi) - math.pi
        self.acc += d
        self.prev = yaw_rad
        return math.degrees(self.acc)


class CalibMonitor(Node):
    def __init__(self):
        super().__init__('claude_calib_monitor')
        self.unwrap = Unwrap()
        self.state = '?'
        self.last_state = None
        self.filt_yaw = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.t0 = time.time()
        self.csv_f = open(CSV_PATH, 'a', newline='')
        self.csv_w = csv.writer(self.csv_f)
        if self.csv_f.tell() == 0:
            self.csv_w.writerow(['wall_time', 'iso_time', 'state', 'filt_yaw_deg', 'cmd_v', 'cmd_w'])

        self.create_subscription(String, '/mission/return/state', self.on_state, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, be)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_return_path', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'logging to {CSV_PATH} -- waiting for /odometry/filtered ...')

    def on_state(self, msg: String):
        self.state = msg.data
        if msg.data != self.last_state:
            print(f'[{time.strftime("%H:%M:%S")}] STATE -> {msg.data}', flush=True)
            self.last_state = msg.data

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = 2 * math.atan2(q.z, q.w)
        self.filt_yaw = self.unwrap(yaw)

    def on_cmd(self, msg: Twist):
        self.cmd_v, self.cmd_w = msg.linear.x, msg.angular.z

    def tick(self):
        now = time.time()
        row = [f'{now:.2f}', time.strftime('%H:%M:%S', time.localtime(now)),
               self.state, f'{self.filt_yaw:.3f}' if self.filt_yaw is not None else '',
               f'{self.cmd_v:.3f}', f'{self.cmd_w:.3f}']
        self.csv_w.writerow(row)
        self.csv_f.flush()
        yaw_s = f'{self.filt_yaw:7.1f}' if self.filt_yaw is not None else '   n/a'
        print(f'[{row[1]}] state={self.state:<20} filt_yaw={yaw_s} deg  cmd_v={self.cmd_v:5.2f} cmd_w={self.cmd_w:+5.2f}',
              flush=True)


def main():
    rclpy.init()
    n = CalibMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.csv_f.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
