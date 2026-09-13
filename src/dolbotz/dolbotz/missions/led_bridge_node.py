"""led_bridge_node — /led_control(std_msgs/String)을 시리얼로 아두이노에
전달하는 ROS2<->시리얼 브릿지.

[예외 케이스 안내] 이 파일의 spring_ifof.py 상단 docstring 참고 — 봄 미션
(피아식별)만 인식부터 LED 하드웨어 제어까지 한 사람이 전 구간을 담당하는
예외라, 이 노드가 그 체인의 마지막 단계(실제 시리얼 전송)를 맡는다. 다른
미션에는 이런 하드웨어 직결 노드가 없다. [2026-08-30] /led_control 발행
주체가 led_relay_node에서 spring_ifof_node로 바뀌었지만(spring_ifof.py
참고) 이 노드는 토픽 이름/값 형식만 보므로 코드 변경 없음.

구독: /led_control (std_msgs/String) — "roka" | "enemy" | "none"
전송: "<cmd>\\n" 형태로 시리얼에 그대로 씀 (소문자, 앞뒤 공백 제거된 문자열)

안정성 처리:
  * reopen_interval_ms 주기로 포트 자동 재연결 시도 (뽑았다 다시 꽂아도 복구됨)
  * DTR/RTS 비활성화 — 아두이노가 시리얼 연결될 때마다 자동 리셋되는 걸 최소화
  * exclusive=True — 리눅스에서 같은 포트를 다른 프로세스가 동시에 못 열게 함
  * debug_tx 파라미터 — 켜면 전송한 데이터를 로그로 남김
"""
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
from serial import SerialException


class LedBridgeNode(Node):
    """/led_control 토픽과 시리얼 포트 사이를 잇는 브릿지 노드."""

    def __init__(self) -> None:
        super().__init__('led_bridge_node')

        # ---------- 파라미터 ----------
        self.declare_parameter('port', '/dev/LED')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reopen_interval_ms', 1000)
        self.declare_parameter('debug_tx', False)
        self.declare_parameter('topic', '/led_control')

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.reopen_interval_ms = int(self.get_parameter('reopen_interval_ms').value)
        self.debug_tx = bool(self.get_parameter('debug_tx').value)
        self.topic = str(self.get_parameter('topic').value)

        # ---------- 상태 ----------
        self._ser = None
        self._ser_lock = threading.Lock()
        self._rx_running = False
        self._rx_thread = None
        self._shutting_down = False

        # ---------- ROS I/O ----------
        self.create_subscription(String, self.topic, self.cb_led, 10)

        # 주기적으로 포트 재오픈 시도
        self.create_timer(self.reopen_interval_ms / 1000.0, self.try_open_timer)

        # 최초 오픈 시도
        self.try_open_timer()

    def cb_led(self, msg: String) -> None:
        """/led_control에서 명령을 받아 시리얼로 전송한다."""
        cmd = (msg.data or '').strip().lower()
        if cmd not in ('roka', 'enemy', 'none'):
            self.get_logger().warn(f"알 수 없는 LED 명령: '{msg.data}' (허용값: roka|enemy|none)")
            return
        self._send_line(cmd + '\n')

    def try_open_timer(self) -> None:
        """주기적으로 포트 오픈을 시도한다 — 장치를 뽑았다 다시 꽂아도
        자동으로 재연결되게 하기 위함."""
        if self._shutting_down:
            return
        with self._ser_lock:
            if self._ser is not None and getattr(self._ser, 'is_open', False):
                return
            try:
                self._ser = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=0.05,
                    write_timeout=0.2,
                    exclusive=True,  # 리눅스에서 포트 단독 접근 보장
                )
                # 아두이노 자동 리셋 최소화
                try:
                    self._ser.dtr = False
                    self._ser.rts = False
                except Exception:
                    pass
                # 버퍼 비우기
                try:
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                except Exception:
                    pass

                self.get_logger().info(f'시리얼 포트 열림: {self.port} @ {self.baud}')

                # 선택: 들어오는 시리얼 데이터를 읽어서 로그로 남기는 스레드 시작
                if not self._rx_running:
                    self._rx_running = True
                    self._rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                    self._rx_thread.start()

            except Exception as e:
                self._ser = None
                self.get_logger().warn(f'포트 열기 실패: {e}')

    def close_serial(self) -> None:
        """시리얼 포트를 안전하게 닫는다."""
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None

    def _send_line(self, line: str) -> None:
        """문자열 한 줄을 시리얼로 전송한다(개행 포함해서 넘길 것)."""
        with self._ser_lock:
            ser = self._ser
        if ser is None or not getattr(ser, 'is_open', False):
            self.get_logger().warn('시리얼 미연결 — 전송 스킵')
            return
        try:
            ser.write(line.encode('ascii'))
            if self.debug_tx:
                self.get_logger().info(f"TX: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f'전송 실패: {e}')
            self.close_serial()

    def rx_loop(self) -> None:
        """시리얼 포트에서 들어오는 데이터를 계속 읽어서 로그로 남긴다 —
        아두이노 쪽 디버그 출력(Serial.println 등)을 보는 용도."""
        buf = bytearray()
        while self._rx_running and not self._shutting_down:
            with self._ser_lock:
                ser = self._ser
            if ser is None or not getattr(ser, 'is_open', False):
                time.sleep(0.05)
                continue
            try:
                chunk = ser.read(64)
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    idx = buf.find(b'\n')
                    if idx < 0:
                        break
                    line = buf[:idx].decode('ascii', errors='ignore').strip()
                    del buf[:idx + 1]
                    if line:
                        self.get_logger().info(f'RX: {line}')
            except (SerialException, OSError, ValueError) as e:
                if not self._shutting_down:
                    self.get_logger().warn(f'RX 에러: {e}')
                self.close_serial()
                time.sleep(0.1)

    def shutdown(self) -> None:
        """노드 종료 시 자원 정리."""
        self._shutting_down = True
        self._rx_running = False
        self.close_serial()
        time.sleep(0.05)


def main() -> None:
    rclpy.init()
    node = LedBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
