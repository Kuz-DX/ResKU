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

ROS 측 publisher/subscriber와 완료 대기, 명시적 취소가 구현되었다. Arduino
펌웨어는 작성하지 않았다. 기본 `tool_changer.enabled: false`, 개발 정지점 home,
미확정 docking_timeout_s 및 timeouts_s.lock은 유지한다. 활성화 설정과 실제 시간,
도킹 자세를 검증한 뒤에만 전체 시퀀스를 사용할 수 있다.

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

ROS QoS: RELIABLE, VOLATILE, KEEP_LAST(10). 명령은 transient-local로
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

도착 판정은 단순 액션 성공이나 발행 완료로 대체하지 않는다. 현재 구현은
목표 관절 순서와 일치하는 유한한 최신 position/velocity를 확인하고, 목표 오차와
속도가 승인된 허용값 안에서 연속 표본 동안 유지되는지 검사한다. 기본 설정은
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

현재 구현:

- 도킹 경로 끝점과 동일 순서의 최신 position/velocity 검사. 액션 완료 후
  새로 관찰한 서로 다른 표본 5회가 정지 조건을 만족해야 명령 발행.
- `/tool_changer/command` publisher, `/tool_changer/status` subscriber,
  UUID·deadline·완료 latch, LOCK/UNLOCK 지원. 실제 명령은 설정으로 선택.
- 완료 대기는 별도 callback group으로 상태/feedback/취소 콜백을 허용하는
  MultiThreadedExecutor 내에서 10 ms 간격으로 확인한다. 기본 요청 그룹은
  한 시퀀스만 처리한다. timeout 동안 다른 도구 요청을 병렬 실행하지 않는다.
- `/tool_change/cancel` (std_srvs/Trigger)은 LOCK/UNLOCK의 도킹 검사 또는
  응답 대기만 취소한다. 팔이 이미 접근/후퇴 중인 단계의 범용 cancel이 아니다.
- 대기 중 도킹 이탈·잘못된 feedback·hardware fault·controller 상태 유실은
  STOP을 보내고 후퇴를 차단한다. arm_controller 활성 및
  manual_position_controller 비활성을 조회하며 0.75 s watchdog을 사용한다.
- STOP에는 원 명령 ID를 사용하고 중복 발행을 막는다. 취소는 이미 수신된
  완료보다 우선한다. 오류 후 자동 재시도하지 않는다.

남은 배포 작업: 기존 EE_ALIGN/실행기와 현재 하드웨어의 호환성 해결,
managed 단일 소유권으로의 도킹/후퇴 연결, Arduino 통신 연결 방식·시간 확정,
실제 ROS Mock 및 별도 승인된 실기 검증. 이 변경은 기존 JTC 기반 FSM에
토픽 단계를 추가한 것이며 managed와 동시에 실행할 수 있게 만든 변경이 아니다.

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

## 이번 검증 결과와 기존 불일치

Tool Changer 오프라인 테스트: 30개 통과, 6개 실패. 새 삭제/차단 회귀 7개는
통과했다. 실패 5개는 `EeAlignment`가 continuous ee_joint에 없는 URDF lower/upper를
읽는 문제이고, 1개는 raw 변환기가 현재 ee_joint에 없는 soft_limit_raw를 요구하는
문제다. 해당 모델/하드웨어 설정을 수정하지 않았다. EE_ALIGN과 예전 JTC 실행기
구조는 현재 하드웨어에 그대로 배포 가능한 상태로 간주하지 않는다.

## ROS 측 구현 검증

실제 production 함수와 순수 프로토콜을 호출하는 오프라인 테스트로 명령 ID,
중복/이전/반대 응답, ERROR, timeout, 취소, 도킹 이탈, feedback/controller 유실,
subscriber 부재, 완료 후에만 후퇴하는 FSM 순서를 검증했다. ROS 토픽·서비스 통신은 아래 Mock 시험으로 검증했다.
Arduino 하드웨어 통신과 실제 팔 동작은 아직 시험하지 않았다. 빌드와 실기 명령은 실행하지 않았다.

## ROS Mock 검증 결과 (2026-10-25)

`test/changer_ros_mock.py`를 격리된 ROS domain 187, localhost-only 환경에서
실행해 10개 시나리오가 모두 통과했다. 결과는 `changer_ros_mock_results.json`.
하드웨어·controller·MoveIt 실행 프로세스 없이 FSM과 모의 peer 두 노드만 사용했다.

| 시나리오 | 실제 command 토픽 | 결과 |
|---|---|---|
| LOCK 정상 완료 | LOCK 1회 | 완료 응답 후 Mock 후퇴, DONE |
| UNLOCK 정상 완료 | UNLOCK 1회 | 완료 응답 후 Mock 후퇴, DONE |
| 잘못된 JSON/이전 ID/반대 상태 후 정상 완료 | LOCK 1회 | 올바른 완료 전 후퇴 없음 |
| 무응답 | LOCK → STOP | HOLD, 후퇴 없음 |
| ERROR | LOCK → STOP | HOLD, 후퇴 없음 |
| 실제 cancel 서비스 호출 | LOCK → STOP | 취소 성공, HOLD, 후퇴 없음 |
| 도킹 위치 이탈 | LOCK → STOP | HOLD, 후퇴 없음 |
| joint_states 중단 | LOCK → STOP | HOLD, 후퇴 없음 |
| controller inactive 응답 | LOCK → STOP | HOLD, 후퇴 없음 |
| hardware_fault 토픽 | LOCK → STOP | HOLD, 후퇴 없음 |

각 STOP에는 원 명령 ID가 유지됐고, STOP 후 늦은 LOCKED 응답을 주입해도
후퇴하지 않았다. 성공 시 중복 완료를 보내도 후퇴는 한 번만 발생했다.
테스트 프로세스는 정상 종료됐다. 운영 config는 변경하지 않았다.

범위: 실제 production FSM 생성자·요청/순서·Arduino callback·대기·안전 검사와
실제 ROS 토픽/서비스를 사용했다. 설치본에 프로젝트의 생성된 srv 타입이 없어
빌드 없이 시험하기 위해 custom 서비스 import 타입만 시험용으로 대체했다.
팔 named 이동·IK/도킹 실행·EE 정렬의 호출 경계는 Mock 처리했고, 후퇴는
메서드 호출 기록으로 확인했다. 실제 팔 액션, 기구학, physical stopping,
설치된 패키지의 실행 가능성 및 Arduino 펌웨어 검증을 의미하지 않는다.

초기 시험 환경의 모의 Request 타입 및 hardware_fault QoS를 보정한 뒤 최종
실행이 통과했다. 이 검증 때문에 production 제어 코드를 수정하지 않았다.

재현 (실기와 분리된 시험 domain이 비어 있는지 먼저 확인):

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/changer_mock_ros_logs \
  PYTHONDONTWRITEBYTECODE=1 python3 src/arm/tool_change_min/test/changer_ros_mock.py
```
