#!/usr/bin/env python3
"""sim_diff_drive_odom.py -- [테스트용] cmd_vel을 실제로 적분해서 로봇이
"굴러가는" 것처럼 odom<->base_link를 움직이는 초간단 유니사이클 시뮬레이터.
mock_localization.py(방법 A: 고정 pose, vx=0/wz=0)와 달리 이건 실제 폐루프
(로봇이 움직이면 다음 cycle의 MPPI가 그 새 pose에서부터 다시 계획)를 보기
위한 것 -- RViz에 합성(synthetic) 경로를 띄우고 MPPI가 그걸 실제로
추종하는지, 그리고 imu_slope_mode_node의 /cmd_vel_safety override가 실제로
우선 반영되는지 검증하는 용도.

/cmd_vel(=/offline/cmd_vel, MPPI 출력)과 /cmd_vel_safety(imu_slope_mode_node/
stability_monitor_node 등의 긴급/복구 override) 둘 다 구독해서, 어느 쪽을
따를지는 rmd_x8_driver_node.py의 _control_loop()를 그대로 포팅한 로직이다
-- /cmd_vel_safety가 cmd_vel_safety_timeout(기본 0.5s) 이내에 최근 수신됐으면
/cmd_vel을 완전히 무시하고 그 값을 그대로 쓰고, 아니면 /cmd_vel을 쓰되
cmd_vel_timeout(기본 0.3s)보다 오래됐으면 0으로 정지한다. 실제 모터/CAN/
RMD-X8은 전혀 관여하지 않는다 -- 순수 수학적으로만 pose를 적분한다.
슬립/관성/모터 응답 지연 등은 전혀 모델링하지 않는 이상적인 적분이라,
실제 로봇 거동과 다를 수 있음(critic/override가 실제로 의도한 명령을
내는지 정성적으로 보는 용도이지, 정량적 주행 시뮬레이션이 아님).

odom->base_link를 dynamic TF(/tf)로 발행한다 -- mock_localization.py가
static(/tf_static)을 쓰는 이유(rosbag replay의 과거 timestamp 문제)가
여기서는 해당 없음: 이건 실시간 합성 경로 테스트라 전부 지금(wall clock)
시각 기준이라서.

실행:
    source ~/dolbotZ/install/setup.bash
    python3 sim_diff_drive_odom.py --ros-args -p cmd_vel_topic:=/offline/cmd_vel
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimDiffDriveOdomNode(Node):
    def __init__(self):
        super().__init__('sim_diff_drive_odom_node')

        self.declare_parameter('cmd_vel_topic', '/offline/cmd_vel')
        self.declare_parameter('cmd_vel_safety_topic', '/cmd_vel_safety')
        # rmd_x8_driver_node.py(ekf.launch.py 기본값)와 동일 -- 이 시뮬레이터가
        # 실제 모터 드라이버와 같은 우선순위 판단을 하게 하려는 것.
        self.declare_parameter('cmd_vel_timeout_s', 0.3)
        self.declare_parameter('cmd_vel_safety_timeout_s', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)
        self._cmd_vel_safety_timeout_s = float(self.get_parameter('cmd_vel_safety_timeout_s').value)

        self._x = float(self.get_parameter('initial_x').value)
        self._y = float(self.get_parameter('initial_y').value)
        self._yaw = float(self.get_parameter('initial_yaw').value)
        # 마지막으로 받은 값을 다음 명령이 올 때까지 유지(zero-order hold) --
        # 실제 모터 컨트롤러가 새 명령 안 오면 마지막 값 유지하는 것과 같은 방식.
        self._desired_v = 0.0
        self._desired_w = 0.0
        self._safety_v = 0.0
        self._safety_w = 0.0
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_safety_time = None  # None = /cmd_vel_safety 아직 한 번도 안 옴

        self._cmd_vel_sub = self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value), self._on_cmd_vel, 10)
        self._cmd_vel_safety_sub = self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_safety_topic').value),
            self._on_cmd_vel_safety, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._dt = 1.0 / rate_hz
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'sim_diff_drive_odom_node ready -- {self.get_parameter("cmd_vel_topic").value} / '
            f'{self.get_parameter("cmd_vel_safety_topic").value}(우선) 적분 -> dynamic '
            f'{self._odom_frame}->{self._base_frame} TF + /odometry/filtered @ {rate_hz}Hz'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._desired_v = msg.linear.x
        self._desired_w = msg.angular.z
        self._last_cmd_vel_time = self.get_clock().now()

    def _on_cmd_vel_safety(self, msg: Twist) -> None:
        self._safety_v = msg.linear.x
        self._safety_w = msg.angular.z
        self._last_safety_time = self.get_clock().now()

    def _resolve_velocity(self) -> tuple[float, float]:
        """rmd_x8_driver_node.py::_control_loop()와 동일한 우선순위 판단."""
        now = self.get_clock().now()
        safety_age_s = (
            (now - self._last_safety_time).nanoseconds * 1e-9
            if self._last_safety_time is not None else None)

        if safety_age_s is not None and safety_age_s <= self._cmd_vel_safety_timeout_s:
            return self._safety_v, self._safety_w

        age_s = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
        if age_s > self._cmd_vel_timeout_s:
            return 0.0, 0.0
        return self._desired_v, self._desired_w

    def _step(self) -> None:
        self._vx, self._wz = self._resolve_velocity()

        # 표준 유니사이클 적분 (슬립/관성 미모델링, 순수 기구학).
        self._x += self._vx * math.cos(self._yaw) * self._dt
        self._y += self._vx * math.sin(self._yaw) * self._dt
        self._yaw += self._wz * self._dt

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self._yaw / 2.0), math.cos(self._yaw / 2.0)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self._vx
        odom_msg.twist.twist.angular.z = self._wz
        self._odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimDiffDriveOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
