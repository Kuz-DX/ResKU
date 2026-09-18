import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class ManualStabilityNode(Node):
    """[하림 수정] manual 조종 전용 긴급정지 (A안: IMU pitch/roll만, 전류 보호 없음).

    autonomous 쪽 stability_monitor_node(robot_bringup)와 판단 임계값은 동일하게
    맞췄지만, 완전히 독립된 별도 구현이다 (미션이 원격 1회/자율 1회로 나뉘어서
    공유할 필요가 없음 -- 코드가 겹치더라도 서로 안 건드리는 쪽을 택함).

    [manual+return 통합] can_driver_node(더 이상 launch되지 않음)의
    /motor_speed_cmd_safety 대신, rmd_x8_driver_node가 이미 최우선으로 처리하는
    /cmd_vel_safety(geometry_msgs/Twist, stability_monitor_node와 동일 토픽)로
    직접 발행한다. rmd_x8_driver_node 쪽은 변경 없음 -- 원래도 여러 publisher가
    같은 안전 토픽에 발행할 수 있는 구조였다.
    """

    def __init__(self):
        super().__init__('manual_stability_node')

        self.declare_parameter('critical_pitch_deg', 25.0)
        self.declare_parameter('critical_roll_deg', 20.0)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('cmd_safety_topic', '/cmd_vel_safety')
        self.declare_parameter('check_rate_hz', 100.0)

        p = self.get_parameter
        self.critical_pitch = p('critical_pitch_deg').value
        self.critical_roll = p('critical_roll_deg').value

        self.current_pitch_deg = 0.0
        self.current_roll_deg = 0.0

        imu_topic = p('imu_topic').value
        cmd_safety_topic = p('cmd_safety_topic').value

        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.safety_pub = self.create_publisher(Twist, cmd_safety_topic, 10)

        rate_hz = p('check_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.check_loop)

        self.get_logger().info(
            f'manual_stability_node started: critical_pitch={self.critical_pitch}deg '
            f'critical_roll={self.critical_roll}deg, imu_topic={imu_topic}, '
            f'cmd_safety_topic={cmd_safety_topic}'
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        # roll (x축 회전)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y축 회전)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)

        self.current_roll_deg = math.degrees(roll_rad)
        self.current_pitch_deg = math.degrees(pitch_rad)

    def check_loop(self):
        abs_p = abs(self.current_pitch_deg)
        abs_r = abs(self.current_roll_deg)

        if abs_p >= self.critical_pitch or abs_r >= self.critical_roll:
            # [하림 수정] 완전 정지(v=0, w=0)로 발행. autonomous 쪽은 사람이 옆에
            # 없을 수 있어 초저속 탈출(0.05m/s)을 택했지만, manual은 조종자가
            # 바로 옆에서 조이스틱을 잡고 있으므로 완전 정지가 더 안전하다고
            # 판단함 (재개는 자세가 임계값 아래로 내려오는 순간 자동으로 됨 --
            # rmd_x8_driver_node의 cmd_vel_safety_timeout_s가 지나면 조이스틱
            # 명령으로 자동 복귀).
            msg = Twist()
            self.safety_pub.publish(msg)
            self.get_logger().warn(
                f'Emergency stop: pitch={self.current_pitch_deg:.1f}deg '
                f'roll={self.current_roll_deg:.1f}deg (limits: pitch={self.critical_pitch} '
                f'roll={self.critical_roll})',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ManualStabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ros2 launch가 SIGINT를 보내면 rclpy의 자체 시그널 핸들러가 먼저
        # context.shutdown()을 호출해버려서, 여기서 또 한 번 부르면
        # "rcl_shutdown already called" RCLError로 종료 시 exit code 1 +
        # 트레이스백이 찍힘 (launch로 여러 노드 같이 띄웠을 때 흔한 증상).
        # 이미 내려간 context를 또 내리지 않도록 가드.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
