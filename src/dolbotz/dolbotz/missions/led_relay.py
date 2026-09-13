"""봄 미션 전용 — MissionResult -> /led_control 매핑 릴레이 노드.

[2026-08-30, 사용자 결정] 이 노드(LedRelayNode)는 mission_spring.launch.py에서
더 이상 기본 실행되지 않음 — spring_ifof_node가 이미 판정(디바운싱+락)을
하고 있어서, 그 결과를 다시 이 노드가 받아 매핑하는 중간 단계 없이
spring_ifof_node가 /led_control을 직접 발행하도록 단순화했다(spring_ifof.py
상단 docstring 참고). 이 파일이 하던 매핑 로직(result_to_led_command())은
spring_ifof_node가 그대로 import해서 재사용 — 기준이 두 군데로 갈라지지
않게 하기 위해 이 함수는 여기 남겨둔다. 레이어를 분리해서 예전 방식대로
테스트하고 싶으면 이 노드를 수동으로 띄워도 되는데(led_bridge_node는
/led_control만 구독하므로 발행 주체가 spring_ifof_node든 이 노드든 무관),
그러면 두 발행자가 같은 토픽에 값을 쏘게 되니 동시에 띄우지 말 것.

[예외 케이스 안내] spring_ifof.py 상단 docstring 참고 — 봄 미션(피아식별)만
인식부터 LED 하드웨어 제어까지 한 사람이 전 구간을 담당하는 예외다. 이
노드는 그 체인의 중간 단계(MissionResult -> 표준 /led_control 문자열)이고,
다른 미션에는 이런 하드웨어 직결 릴레이가 없다 — 이 구조를 다른 미션의
템플릿으로 오해하지 말 것.

/mission/spring_ifof/result(MissionResult)만 구독한다 — mission_name이
'spring_ifof'가 아닌 메시지는 무시한다(다른 미션의 MissionResult가 같은
토픽 이름 규칙을 쓸 일은 없지만, 방어적으로 확인).

매핑:
  state='friend' (그리고 valid=True) -> "roka"
  state='enemy'  (그리고 valid=True) -> "enemy"
  그 외(valid=False 포함, 'unknown' 등)              -> "none"

led_bridge_node가 /led_control을 구독해서 실제 시리얼로 아두이노에
전송한다 — 이 노드는 시리얼을 전혀 모르고, 표준 String 토픽만 다룬다
(레이어 분리: MissionResult 해석 vs 하드웨어 전송).

값이 바뀔 때만 발행한다 — 매 프레임(수십Hz) 그대로 다시 쏘면 시리얼
브릿지가 아두이노로 같은 명령을 계속 재전송하게 되는데, 그 자체가 문제는
아니지만(아두이노 쪽도 같은 명령 반복 적용은 멱등) 불필요한 시리얼 트래픽/
로그 스팸을 줄이기 위함.

구독: /mission/spring_ifof/result (mission_manager_interfaces/MissionResult)
발행: /led_control (std_msgs/String) — "roka" | "enemy" | "none"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mission_manager_interfaces.msg import MissionResult

_STATE_TO_LED = {'friend': 'roka', 'enemy': 'enemy'}


def result_to_led_command(state: str, valid: bool) -> str:
    """MissionResult의 state/valid -> LED 명령 문자열 순수 매핑 함수.

    valid=False면 state 값과 무관하게 "none" — 미확정 상태를 임의의 색으로
    표시하면 안 되기 때문(오탐 시 잘못된 피아식별 표시로 이어질 수 있음).
    """
    if not valid:
        return 'none'
    return _STATE_TO_LED.get(state, 'none')


class LedRelayNode(Node):
    def __init__(self):
        super().__init__('led_relay_node')

        self.declare_parameter('mission_name', 'spring_ifof')
        self._mission_name = str(self.get_parameter('mission_name').value)

        self._last_cmd: str | None = None
        self._led_pub = self.create_publisher(String, '/led_control', 10)
        self.create_subscription(
            MissionResult, '/mission/spring_ifof/result', self._on_result, 10)

        self.get_logger().info(
            f"LedRelayNode ready — mission_name='{self._mission_name}' 필터, "
            f"/mission/spring_ifof/result -> /led_control")

    def _on_result(self, msg: MissionResult) -> None:
        if msg.mission_name != self._mission_name:
            return

        cmd = result_to_led_command(msg.state, msg.valid)
        if cmd == self._last_cmd:
            return  # 값 안 바뀌었으면 재발행 안 함
        self._last_cmd = cmd

        self._led_pub.publish(String(data=cmd))
        self.get_logger().info(f"LED 명령 변경: '{cmd}' (state='{msg.state}', valid={msg.valid})")


def main(args=None):
    rclpy.init(args=args)
    node = LedRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
