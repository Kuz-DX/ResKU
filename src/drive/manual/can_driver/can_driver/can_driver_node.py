import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can


class CanDriverNode(Node):
    def __init__(self):
        super().__init__('can_driver_node')

        # ---- 파라미터 ----
        # [하림 수정] 기본값을 can_drive로 변경 (실물 어댑터가 can0로 잡힌 뒤
        # can_drive로 리네임되어 쓰이는 이 리포의 운영 관례, rmd_x8_driver와 동일)
        self.declare_parameter('can_channel', 'can_drive')
        self.declare_parameter('left_can_id', 1)   # -> 0x141
        self.declare_parameter('right_can_id', 2)  # -> 0x142
        self.declare_parameter('cmd_topic', '/motor_speed_cmd')
        self.declare_parameter('cmd_timeout_sec', 0.3)  # 이 시간 동안 명령 없으면 안전정지
        self.declare_parameter('control_rate_hz', 50.0)
        # [하림 수정] 안전 개입 경로 (rmd_x8_driver_node의 /cmd_vel_safety와 동일 패턴).
        # manual_stability_node(IMU pitch/roll 임계값)가 여기로 발행하면, 정상
        # 조이스틱 명령보다 우선해서 즉시 정지 명령을 낸다.
        self.declare_parameter('cmd_safety_topic', '/motor_speed_cmd_safety')
        self.declare_parameter('cmd_safety_timeout_sec', 0.5)
        # [하림 수정] 2026-08-20: USB-CAN 어댑터(gs_usb)가 물리적으로 잠깐 빠졌다
        # 붙는 사고가 실기에서 확인됨 -- 그동안은 인터페이스가 죽으면
        # can.interface.Bus가 물고 있던 소켓이 그대로 죽어서, 사람이 인터페이스를
        # 다시 올려줘도(`ip link set ... up`) 이 노드는 재시작 전까진 계속 전송
        # 실패만 반복했다. 연속 전송 실패가 이 횟수를 넘으면 소켓을 스스로 닫고
        # 다시 여는 걸 시도한다 (인터페이스 자체가 아직도 안 살아있으면 재오픈도
        # 실패하고 다시 대기 -- udev 규칙 등으로 인터페이스 쪽 자동복구를 같이
        # 해두면 사람 개입 없이 전체가 복구됨).
        self.declare_parameter('reconnect_after_failures', 10)
        self.declare_parameter('reconnect_cooldown_sec', 2.0)

        p = self.get_parameter
        self.left_id = 0x140 + p('left_can_id').value
        self.right_id = 0x140 + p('right_can_id').value
        self._can_channel = p('can_channel').value
        cmd_topic = p('cmd_topic').value
        self.cmd_timeout = p('cmd_timeout_sec').value
        self.cmd_safety_timeout = p('cmd_safety_timeout_sec').value
        rate_hz = p('control_rate_hz').value
        self.reconnect_after_failures = int(p('reconnect_after_failures').value)
        self.reconnect_cooldown_sec = float(p('reconnect_cooldown_sec').value)

        self._consecutive_send_failures = 0
        self._last_reconnect_attempt = None

        # ---- CAN 버스 연결 ----
        # [수정] 이전엔 여기서 실패하면 raise해서 노드 자체가 죽었다 -- 로그 한
        # 줄 찍고 바로 프로세스가 사라지니 ros2 launch로 여러 노드를 같이
        # 띄웠을 때 터미널 스크롤에 묻히기 쉬웠다. 이제 raise 안 하고 self.bus를
        # None으로 둔 채 노드는 계속 살아있는다 -- _send_frame()이 self.bus가
        # None이면 매 명령마다 재연결을 시도하고(_attempt_reconnect(), 쿨다운
        # 있음) 그때마다 눈에 띄게 로그를 남긴다.
        self._open_bus()

        self._latest_left = 0.0
        self._latest_right = 0.0
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = False

        self._safety_left = 0.0
        self._safety_right = 0.0
        self._last_safety_time = None  # None = 안전 개입 명령 아직 한 번도 안 옴

        cmd_safety_topic = p('cmd_safety_topic').value
        self.sub = self.create_subscription(
            Float32MultiArray, cmd_topic, self.cmd_callback, 10
        )
        self.sub_safety = self.create_subscription(
            Float32MultiArray, cmd_safety_topic, self.cmd_safety_callback, 10
        )

        # [하림 수정] 명령 수신 즉시 보내던 방식 -> 주기적 제어 루프로 변경
        # (rmd_x8_driver_node와 동일 패턴). 매 사이클 안전 명령 신선도를 먼저
        # 확인해서, 신선하면 그걸 최우선으로 내보내고 아니면 평소 명령을 낸다.
        self.control_timer = self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(
            f'CAN driver started. left_id=0x{self.left_id:X}, right_id=0x{self.right_id:X}, '
            f'listening on {cmd_topic}, safety override on {cmd_safety_topic}'
        )

    def cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('motor_speed_cmd expects [left_dps, right_dps], got fewer values')
            return

        self._latest_left, self._latest_right = msg.data[0], msg.data[1]
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = True

    def cmd_safety_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('motor_speed_cmd_safety expects [left_dps, right_dps], got fewer values')
            return

        self._safety_left, self._safety_right = msg.data[0], msg.data[1]
        self._last_safety_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()

        safety_age_s = (
            (now - self._last_safety_time).nanoseconds * 1e-9
            if self._last_safety_time is not None else None
        )
        if safety_age_s is not None and safety_age_s <= self.cmd_safety_timeout:
            # 안전 개입이 신선함 -> 조이스틱 명령 무시하고 이걸 최우선으로 전송
            self.send_speed_command(self.left_id, self._safety_left)
            self.send_speed_command(self.right_id, self._safety_right)
            return

        if not self._has_received_cmd:
            return  # 아직 정상 명령을 한 번도 못 받았으면 대기 (조기 정지 스팸 방지)

        cmd_age_s = (now - self._last_cmd_time).nanoseconds * 1e-9
        if cmd_age_s > self.cmd_timeout:
            self.get_logger().warn(
                f'No /motor_speed_cmd for {cmd_age_s:.2f}s (timeout={self.cmd_timeout}s). '
                f'Sending stop command.',
                throttle_duration_sec=1.0,
            )
            self.send_stop_command(self.left_id)
            self.send_stop_command(self.right_id)
            return

        self.send_speed_command(self.left_id, self._latest_left)
        self.send_speed_command(self.right_id, self._latest_right)

    def send_speed_command(self, arb_id: int, speed_dps: float):
        # 0.01dps/LSB 단위로 스케일링 (프로토콜 스펙)
        speed_control = int(round(speed_dps * 100.0))

        # int32_t 범위 클램프 (안전상 과도한 값 방지)
        speed_control = max(min(speed_control, 2_147_483_647), -2_147_483_648)

        data = bytearray(8)
        data[0] = 0xA2
        data[1] = 0x00
        data[2] = 0x00
        data[3] = 0x00
        data[4:8] = struct.pack('<i', speed_control)  # little-endian int32

        self._send_frame(arb_id, data)

    def send_stop_command(self, arb_id: int):
        # 0x81: 모터 정지 (closed-loop 유지, 속도만 0으로)
        data = bytearray(8)
        data[0] = 0x81
        self._send_frame(arb_id, data)

    def _open_bus(self) -> bool:
        """CAN 버스를 (재)연다. self.bus는 항상 "쓸 수 있는 Bus 객체" 아니면
        None 둘 중 하나만 되도록 유지한다 -- 실패해도 예외를 다시 던지지
        않고 self.bus를 명시적으로 None에 둔다(이전엔 재오픈 실패 시
        self.bus가 방금 shutdown()한 죽은 객체를 그대로 가리키고 있어서,
        _send_frame()이 그걸 계속 쓰려다 어떤 예외가 날지 보장이 안 되는
        구멍이 있었다). 성공하면 True."""
        try:
            self.bus = can.interface.Bus(channel=self._can_channel, bustype='socketcan')
            self.get_logger().info(f'CAN bus opened on {self._can_channel}')
            return True
        except Exception as e:
            self.bus = None
            self.get_logger().error(
                f'Failed to open CAN bus {self._can_channel}: {e} -- '
                f'노드는 계속 떠 있고, 명령이 들어올 때마다 재연결을 재시도합니다 '
                f'(`ip link show {self._can_channel}`로 인터페이스 상태 확인).')
            return False

    def _send_frame(self, arb_id: int, data: bytearray):
        if self.bus is None:
            # self.bus가 없으면(최초 연결 실패 또는 재오픈 실패) 매번
            # _attempt_reconnect()를 불러본다 -- 그 안의 쿨다운(reconnect_cooldown_sec)
            # 이 폭주를 막아주므로 그냥 매 tick 호출해도 안전하다.
            self._attempt_reconnect()
            if self.bus is None:
                self.get_logger().warn(
                    f'CAN bus not open on {self._can_channel} -- command (id=0x{arb_id:X}) '
                    f'not sent.', throttle_duration_sec=1.0)
            return

        msg = can.Message(arbitration_id=arb_id, data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg)
            self._consecutive_send_failures = 0
        except can.CanError as e:
            self._consecutive_send_failures += 1
            self.get_logger().error(
                f'CAN send failed (id=0x{arb_id:X}): {e}', throttle_duration_sec=1.0)
            if self._consecutive_send_failures >= self.reconnect_after_failures:
                self._attempt_reconnect()

    def _attempt_reconnect(self):
        """CAN 소켓을 (필요하면 닫고) 다시 연다 -- self.bus가 아예 없는
        경우(_send_frame()의 가드)와 연속 전송 실패가 임계치를 넘은 경우
        (죽었지만 아직 self.bus가 남아있는 경우) 둘 다 여기로 온다.

        USB-CAN 어댑터가 물리적으로 재연결되면(재열거) 커널이 완전히 새
        인터페이스로 취급하는데, 이 노드가 처음에 연 소켓은 옛날(사라진)
        인터페이스에 그대로 묶여있어서 재시작 전까진 계속 실패만 반복한다.
        여기서는 소켓만 다시 열 뿐, 인터페이스 자체를 `ip link ... up`으로
        올리는 건 하지 않는다 -- 그건 보통 CAP_NET_ADMIN/root가 필요해서
        일반 권한으로 뜨는 이 노드가 직접 하기 부적절하고, 대신 udev 규칙 등
        인터페이스 쪽 자동복구와 짝을 이루도록 설계했다(인터페이스가 아직도
        안 살아있으면 아래 재오픈 자체가 실패하고 다음 임계치까지 다시 대기).
        재시도 폭주를 막기 위해 재시도 사이 최소 간격(reconnect_cooldown_sec)을 둔다.
        """
        now = self.get_clock().now()
        if self._last_reconnect_attempt is not None:
            age_s = (now - self._last_reconnect_attempt).nanoseconds * 1e-9
            if age_s < self.reconnect_cooldown_sec:
                return
        self._last_reconnect_attempt = now

        self.get_logger().warn(
            f'CAN bus unusable ({self._consecutive_send_failures} consecutive send failures) '
            f'-- attempting to (re)open {self._can_channel} '
            f'(인터페이스가 아직 down이면 실패할 수 있음, `ip link show {self._can_channel}`로 확인)'
        )
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        if self._open_bus():
            self._consecutive_send_failures = 0

    def destroy_node(self):
        # 노드 종료 시 안전하게 정지 명령 전송 후 CAN 버스 닫기
        try:
            self.send_stop_command(self.left_id)
            self.send_stop_command(self.right_id)
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanDriverNode()
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
