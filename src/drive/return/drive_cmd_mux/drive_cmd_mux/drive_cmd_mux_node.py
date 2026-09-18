"""
drive_cmd_mux_node

Single arbitration point between manual joystick driving and the autonomous
RETURN mission, keyed off /mission/return/state. Sits between the two
command sources and the single low-level CAN driver (rmd_x8_driver_node),
which must only ever see one /cmd_vel stream.

[manual+return 통합, 2026 사용자 결정] manual 쪽 원본은 dps
(Float32MultiArray, [left_dps, right_dps]) 그대로 유지한다 -- 디버깅 시
숫자가 더 직관적이라는 사용자 판단. dps -> Twist 변환은 이 노드가 전담한다
(rmd_x8_driver_node가 실제로 소비하는 /cmd_vel은 Twist라 어딘가에서는
변환이 필요함).

Subscribes:
    /motor_speed_cmd_manual   (std_msgs/Float32MultiArray, [left_dps, right_dps])
                               -- manual_joy_control_node
    /cmd_vel_return            (geometry_msgs/Twist)  -- return_state_machine_node
    /mission/return/state      (std_msgs/String)      -- return_state_machine_node

Publishes:
    /cmd_vel                   (geometry_msgs/Twist)  -- rmd_x8_driver_node

State -> source mapping:
    IDLE, MANUAL_RECORDING, WAIT_RETURN_COMMAND, FINISHED, "" (no state yet)
        -> pass manual (converted to Twist)
    STOP_BEFORE_TURN
        -> force zero (neither source allowed -- this is the "confirm the
           robot actually stopped" window before the 180 turn)
    TURN_180, FOLLOW_RETURN_PATH
        -> pass /cmd_vel_return

NOTE: /cmd_vel_safety (IMU tip-over / motor-fault emergency stop) is
deliberately NOT an input here -- it stays wired directly into
rmd_x8_driver_node exactly as before, so it remains top priority and
independent of this node (and of return_state_machine_node) being alive.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

MANUAL_STATES = {'IDLE', 'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND', 'FINISHED', ''}
RETURN_STATES = {'TURN_180', 'FOLLOW_RETURN_PATH'}
ZERO_STATES = {'STOP_BEFORE_TURN'}


class DriveCmdMuxNode(Node):
    def __init__(self):
        super().__init__('drive_cmd_mux_node')

        self.declare_parameter('manual_topic', '/motor_speed_cmd_manual')
        self.declare_parameter('return_topic', '/cmd_vel_return')
        self.declare_parameter('state_topic', '/mission/return/state')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 50.0)
        # 선택된 source가 이 시간보다 오래 갱신되지 않으면 마지막 값을 그대로
        # 내보내지 않고 0으로 강제한다 (조인트/arm 쪽 joint_command_mux.py의
        # freshness-timeout 패턴과 동일).
        self.declare_parameter('source_timeout_s', 0.5)

        # [manual+return 통합] manual_joy_control_node가 보내는 부호 없는 바퀴
        # dps(물리적으로 양수=전진)를 Twist(v, w)로 변환하는 데 쓰인다.
        # rmd_x8_driver_node의 실제 운영값(reduced_odom_bringup.launch.py
        # override 기준)과 반드시 같이 맞출 것 -- 값이 어긋나면 조이스틱
        # 입력과 실제 로봇 속도 사이 비례관계만 어긋날 뿐 안전에는 영향
        # 없지만, 조이스틱 감각이 왜곡된다.
        self.declare_parameter('wheel_radius_m', 0.1125)
        self.declare_parameter('effective_track_width_m', 0.4904)

        p = self.get_parameter
        manual_topic = p('manual_topic').value
        return_topic = p('return_topic').value
        state_topic = p('state_topic').value
        output_topic = p('output_topic').value
        rate_hz = p('publish_rate_hz').value
        self.source_timeout_s = float(p('source_timeout_s').value)
        self.wheel_radius_m = float(p('wheel_radius_m').value)
        self.effective_track_width_m = float(p('effective_track_width_m').value)

        self._manual_cmd = Twist()
        self._manual_time = None
        self._return_cmd = Twist()
        self._return_time = None
        # 아직 return_state_machine_node가 안 떠 있어도(Phase 1 단독 테스트)
        # manual 통과가 기본값이 되도록 빈 문자열로 시작.
        self._state = ''

        self.create_subscription(Float32MultiArray, manual_topic, self._on_manual, 10)
        self.create_subscription(Twist, return_topic, self._on_return, 10)
        self.create_subscription(String, state_topic, self._on_state, 10)
        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)

        self.create_timer(1.0 / rate_hz, self._publish_cmd)

        self.get_logger().info(
            f'drive_cmd_mux_node started: manual={manual_topic} (dps), return={return_topic}, '
            f'state={state_topic} -> output={output_topic}'
        )

    def _on_manual(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(f'{self.get_parameter("manual_topic").value} expects '
                                    f'[left_dps, right_dps], got fewer values')
            return
        left_dps, right_dps = msg.data[0], msg.data[1]
        # 부호 없는 바퀴 dps(물리적으로 양수=전진) -> 바퀴 선속도(m/s) ->
        # body Twist(v, w). rmd_x8_driver_node의 _skid_steer_inverse()/
        # _publish_feedback()과 동일한 관례(양쪽 바퀴 선속도 양수=전진,
        # v_right가 클수록 반시계 회전)를 따른다.
        v_left = math.radians(left_dps) * self.wheel_radius_m
        v_right = math.radians(right_dps) * self.wheel_radius_m
        cmd = Twist()
        cmd.linear.x = (v_left + v_right) / 2.0
        cmd.angular.z = (v_right - v_left) / self.effective_track_width_m
        self._manual_cmd = cmd
        self._manual_time = self.get_clock().now()

    def _on_return(self, msg: Twist):
        self._return_cmd = msg
        self._return_time = self.get_clock().now()

    def _on_state(self, msg: String):
        if msg.data not in MANUAL_STATES and msg.data not in RETURN_STATES and msg.data not in ZERO_STATES:
            self.get_logger().warn(f'Unknown /mission/return/state "{msg.data}" -- treating as MANUAL.',
                                    throttle_duration_sec=5.0)
        self._state = msg.data

    def _fresh(self, last_time) -> bool:
        if last_time is None:
            return False
        age_s = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        return age_s <= self.source_timeout_s

    def _publish_cmd(self):
        out = Twist()

        if self._state in RETURN_STATES:
            if self._fresh(self._return_time):
                out = self._return_cmd
            else:
                self.get_logger().warn(
                    f'/mission/return/state={self._state} but /cmd_vel_return is stale -- '
                    f'commanding zero.', throttle_duration_sec=1.0)
        elif self._state in ZERO_STATES:
            pass  # out stays zero
        else:
            # MANUAL_STATES 또는 알 수 없는 상태 -- manual 통과가 fail-open 기본값.
            if self._fresh(self._manual_time):
                out = self._manual_cmd
            # manual이 stale이면 out은 이미 zero (manual_joy_control_node의
            # joy_timeout이 그 전에 0을 발행하겠지만, 이 노드 스스로도 안전망을
            # 하나 더 둔다).

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DriveCmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
