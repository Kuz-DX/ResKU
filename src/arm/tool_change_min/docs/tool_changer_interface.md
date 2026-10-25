# Arduino Tool Changer 인터페이스 설계 — 6단계

## 역할과 현재 구현 범위

팔은 도킹 위치로 이동하고 실제 도착·정지를 확인한다. 이후 Arduino에
명령을 한 번 발행하고, 같은 명령의 완료 응답을 받은 뒤에만 후퇴한다.
모터 구동·센서 판정·체결 완료 판단은 전적으로 Arduino가 담당한다.

기존 wrist yaw 회전 체결 서비스, 실행 함수, FSM 호출, 회전량·시간 설정,
체결용 named pose와 raw capture, 변환기의 해당 출력은 삭제했다.
정상적인 wrist_yaw 관절 제어와 Manual 매핑은 유지한다.
`ee_joint`는 Manual EE 출력과 gripper mimic 등에서도 사용하므로 삭제하지
않는다. `EE_ALIGN`은 별도 도킹 정렬 단계로 현재 유지하지만, 새 기구에서
필요한지 확인 전에는 자동 제거하거나 새 체결 동작으로 사용하지 않는다.

이번 단계는 통신 계약과 FSM 설계까지다. ROS 토픽 송수신과 Arduino 펌웨어는
아직 구현하지 않았다. 따라서 `lock` 이후 설정은 사전 검증에서 거부하며,
FSM의 해당 구간도 예외로 종료해 후퇴하지 않는다. 기본 home 개발 정지점과
기존 안전 제한은 유지한다. 성공 응답을 가정하거나 팔 회전으로 대체하지 않는다.

## 토픽 계약 (제안 확정안)

양쪽 메시지는 기존 `std_msgs/msg/String` 안의 UTF-8 JSON을 사용한다.
새 메시지 패키지는 필요 없다. 명령과 응답은 별도 토픽이며 `/tool_change/status`
(FSM 진행 상태)와 `/tool_changer/status` (Arduino 상태)를 구분한다.

| 방향 | 토픽 | JSON 예시 |
|---|---|---|
| ROS → Arduino/브리지 | `/tool_changer/command` | `{"id":"고유 UUID","command":"LOCK"}` |
| Arduino/브리지 → ROS | `/tool_changer/status` | `{"id":"동일 UUID","state":"LOCKED","detail":""}` |

명령은 `LOCK`, `UNLOCK`, `STOP`. 상태는 `IDLE`, `MOVING`, `LOCKED`,
`UNLOCKED`, `ERROR`. `detail`은 선택적 오류 설명이다. id는 문자열이며
ROS가 요청마다 새 UUID를 생성한다. 상태 보고만 하는 IDLE에는 빈 id를
사용할 수 있지만 완료 응답으로 수락하지 않는다.

- LOCK의 완료는 같은 id의 LOCKED만 인정한다.
- UNLOCK의 완료는 같은 id의 UNLOCKED만 인정한다.
- MOVING은 진행 알림이며 선택 사항이다. MOVING 없이 최종 응답만 와도 된다.
- 반대 완료 상태, IDLE, 다른 id, 이전 작업 응답은 다음 팔 이동을 허용하지 않는다.
- 같은 id의 ERROR는 실패로 처리한다. 잘못된 JSON은 무시·기록하며 완료로 간주하지 않는다.
- 한 번에 하나의 명령만 진행한다. 중복 완료로 후퇴를 두 번 실행하지 않는다.
- 완료 대기 한계는 `timeouts_s.lock`에 지정할 예정이며 현재 null이다.
  Arduino 동작 시간을 확인해 정한다. 완료만 보내는 방식이므로 MOVING heartbeat를 필수로 요구하지 않는다.

ROS QoS 제안: RELIABLE, VOLATILE, KEEP_LAST(10). 명령은 transient-local로
보존하지 않는다. Arduino가 재연결됐을 때 과거 체결 명령이 자동 실행되지
않아야 한다. 물리 링크가 시리얼이면 브리지가 같은 id/command/state를
전달하며 모터 제어 로직은 갖지 않는다. 직접 micro-ROS 연결도 같은 계약을 쓴다.

## FSM 흐름

```text
도킹 경로 실행
  → 실제 도착·정지 검사
  → 새 id로 LOCK 또는 UNLOCK 한 번 발행
  → 팔 정지 목표 유지, Arduino 응답 대기
      ├ 일치하는 완료 응답 → 후퇴 → 다음 명령
      └ ERROR / timeout / 취소 / 팔 fault → STOP 요청 → HOLD, 후퇴 금지
```

도착 판정은 단순 액션 성공이나 발행 완료로 대체하지 않는다. 실제 구현 시
목표 관절 순서와 일치하는 유한한 최신 position/velocity를 확인하고, 목표 오차와
속도가 승인된 허용값 안에서 연속 표본 동안 유지되는지 검사한다. 초기 제안은
관절 오차 0.005 rad 이하, 속도 0.02 rad/s 이하, 서로 다른 피드백 5회다.
이는 실기 도킹 정밀도에 대한 확정값이 아니며 실제 TCP 정렬과 체결 센서 조건도
검토해야 한다. 대기 중 feedback 유실·도킹 위치 이탈·HOLD/FAULT 전환 시
늦게 온 완료 응답으로 다음 동작을 시작하면 안 된다.

STOP은 진행 중인 id를 사용한다. Arduino는 그 id의 구동을 중지하고 상태를
회신한다. STOP 이후 ROS는 해당 거래를 취소된 것으로 고정하며 늦은 LOCKED/
UNLOCKED를 받아도 후퇴하지 않는다. STOP 완료/실패도 자동 팔 이동을 허용하지
않는다. 새 시도는 작업자 확인 후 새 id로 시작한다. STOP은 물리 E-stop 대체가
아니며, 통신 유실 시 Arduino 자체 timeout/정지 정책이 필요하다.

같은 (id, command)가 중복 수신되면 Arduino는 모터를 다시 구동하지 않고
현재/최종 상태를 재보고한다. 같은 id의 STOP은 기존 LOCK/UNLOCK을 중단하는
특별 명령이다. STOP 후 같은 id의 LOCK/UNLOCK은 재실행하지 않는다.
ROS는 timeout 때 LOCK/UNLOCK을 자동 재발행하지 않는다. Arduino 재시작 후
모터 재동작 및 저장된 상태 해석은 펌웨어 구현 전에 정해야 한다.

## 기존 제어 경로와의 연결 작업

기존 tool_change_min MotionExecutor는 JTC 액션을 사용하며 managed 제어는
ForwardCommandController와 Mode Manager를 사용한다. 이 둘은 현재 연결되어
있지 않다. 실제 배포 전에 팔 도킹/후퇴도 manager의 단일 소유권 아래 연결해야
하며, 기존 JTC를 managed와 동시에 켜는 방식으로 우회하지 않는다.
현재 HOLD 문자열 발행만으로 물리적인 팔 정지를 보장한다고 간주하지 않는다.

후속 구현 대상:

1. 도킹 도착 검사와 manager 권한 유지/취소 연동.
2. ROS 측 command publisher, status subscriber, id·deadline·완료 latch.
3. 콜백이 응답 처리를 막지 않는 비동기 대기 상태.
4. LOCK/UNLOCK/STOP의 모의 응답기로 순서·실패 검증.
5. Arduino 연결 방식과 시간 한계 확정 후 별도 승인된 실기 시험.

## 검증 시나리오

- 도착 전 command 없음, 실제 도착 후 정확히 한 번 발행.
- MOVING만 수신하거나 다른 id/반대 완료를 받으면 후퇴 없음.
- 일치하는 완료는 후퇴를 한 번만 허용; 중복·이전 응답 무시.
- ERROR, 무응답, 취소, 도킹 이탈, feedback/controller 유실은 후퇴 금지.
- STOP 뒤 늦은 완료는 무시; 재연결·재시작으로 자동 재시도 없음.
- 새 프로세스의 id는 이전 작업과 달라야 함.
- 모든 경우 체결을 위해 wrist_yaw나 ee_joint 명령을 만들지 않음.

ROS 패키지는 빌드하지 않았다. 기존 install에 남은 generated 서비스 파일은
이번 source 삭제만으로 없어지지 않을 수 있다. 배포 시 source/install 일치 확인이
필요하며, 여기서는 install 산출물을 임의 수정하거나 실행하지 않았다.
