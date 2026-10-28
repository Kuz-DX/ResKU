# Managed arm HOLD 대응표

적용 범위: `ik_manual.launch.py`의 mode manager, TCP 조이스틱, tool0/1/2 경로와 선택적 도킹·픽앤플레이스. `manual.launch.py`는 별도 제어기이므로 이 절차의 모드·서비스가 적용되지 않는다. 센서, 충돌 형상, 접촉 상태는 소프트웨어만으로 검증할 수 없으므로 이 표가 모든 물리적 실패를 보증하지는 않는다.

## 1. 움직임이 멈추면 먼저 상태를 고정해서 확인

조이스틱을 중립으로 두고 구동축이 실제로 정지했는지 확인한다. 충돌, 케이블 걸림, 비정상 전류 또는 계속 움직이는 축이 보이면 경로를 다시 누르지 말고 현장의 정지 절차와 하드웨어 점검을 우선한다. 정상적으로 정지한 상태에서 다음을 수집한다.

```bash
ros2 topic echo /arm/control_status --once --full-length
ros2 topic echo /arm/tool_trajectory/status --once --full-length
ros2 topic echo /arm/tcp_joy/status --once --full-length
ros2 topic echo /control/hardware_fault --once
ros2 topic echo /arm/docking_status --once
ros2 topic echo /arm/docked --once
```

`/arm/control_status`는 `mode`, `requested`, `transitioning`, `ready`, `startup_complete`, `focus`, `reason`, `last_execution_id`, `last_execution_result`, `last_execution_reason`, `execution_id`, `remaining_waypoints`, `last_error` 순으로 읽는다. `last_execution_*`는 **마지막** 실행의 기록이므로 현재 요청의 ID와 일치하는지 확인한다. `reason`은 이후 모드 변경으로 바뀔 수 있고 `last_error`는 늦은 패킷 하나로 덮일 수 있다. 실행 실패 당시의 `/arm/tool_trajectory/status`와 로그를 함께 기록한다. `path_*_error_rad`는 실행 종료 때 0으로 초기화될 수 있으므로, 종료 후 0이라는 이유만으로 정상 완료라고 판단하지 않는다.

## 2. 상태별 조치

| 상태와 근거 | 의미 | 운용자 조치 |
| --- | --- | --- |
| `mode=hold`, `last_execution_result=succeeded`, `execution_id=null` | 마지막 자동 경로가 완료되어 HOLD. HOME 복귀를 뜻하지 않음 | 위치와 작업 결과를 직접 확인한 뒤 다음 작업을 명시적으로 요청. 그리퍼/도킹의 물리적 체결은 성공 메시지만으로 확정하지 않음 |
| `mode=hold`, `transitioning=true` | 이전 목표를 멈추거나 모드를 교체하는 중 | 속도와 전환 완료를 기다림. 이 동안 새 명령을 연속 입력하지 않음. 계속 지연되면 `reason`과 controller 상태 확인 |
| `mode=hold`, `ready=false` 또는 feedback/controller age가 큼 | controller나 관절 피드백이 준비되지 않음 | 통신·컨트롤러·`/joint_states` 확인. 준비되기 전 재요청하지 않음 |
| `mode=hold`, `focus=drive` | 팔 조작권이 없음 | Options로 팔 focus 선택. 첫 진입 DXL 정렬 완료를 기다린 뒤 요청 |
| `mode=hold`, `startup_complete=false` 또는 `entry_alignment_complete=false` | RMD 시작 복구 또는 DXL 정렬 중 | 움직임이 정상이고 장애물이 없으면 완료를 기다림. 멈춤/시간 초과/fault라면 아래 FAULT 분기로 이동 |
| `mode=hold`, `last_execution_result=aborted` | 자동 경로가 중단됐지만 manager가 재사용 가능한 HOLD로 복귀 | `last_execution_reason`과 경로 진단의 최초 `failed` 이유를 먼저 확인. 원인 분기를 해결한 뒤 새 요청. 같은 오류가 재발하면 반복 시도 중지 |
| `mode=fault` | latch된 결함. 자동 경로 재요청은 허용되지 않음 | `reason`, `/control/hardware_fault`, 관절 위치/속도 확인. 하드웨어·컨트롤러 문제를 해결한 뒤에만 reset 검토 |

`last_error: rejected stale or inactive source: tool_auto`는 이미 HOLD 또는 새 epoch가 된 후 도착한 명령일 수 있다. 이것만으로 최초 중단 원인을 판정하지 않는다.

## 3. `aborted` 또는 경로 진단 `failed`의 원인별 대응

| 최초 원인 문구 | 발생 단계·추론 | 다음 조치 |
| --- | --- | --- |
| `docking deadman L2 released` | 검증·실행·태그 확인·도킹 중 L2가 해제된 것으로 판정 | L2를 시퀀스 종료까지 유지. 누르고 있었는데도 재발하면 `/joy`의 `axes[2]`가 `-0.5` 미만인지 확인. UI/카메라 요청과 조이스틱 입력 혼동도 확인 |
| `docking joystick stream stale` | Joy heartbeat가 설정 시간(기본 0.3초)을 넘김 | 조이스틱 연결과 `/joy` 발행 주기를 복구하고 새 요청 |
| `waypoint stalled: <joint>` | 관절 명령 lead가 남았고 2초 동안 해당 목표 쪽 진행이 부족 | 해당 축의 접촉·하중·전류·엔코더·명령 대 실측을 점검. 특히 목표를 지나친 뒤 역방향 보정이 막힌 사례를 구분. 원인 없이 같은 점을 반복 통과시키지 않음 |
| `following error: <joint>` | 명령과 실측 차이가 0.15 rad 초과 | 기구부 간섭, 부하, motor protection, feedback를 확인하고 필요하면 속도·경로를 재검토. 자동 재시도 금지 |
| `RMD current protection active` | RMD 보호가 목표를 완화했고 경로를 중단 | 해당 관절 전류·접촉·기구 상태를 점검. 점검 전 같은 경로를 재시도하지 않음 |
| `waypoint timeout`, `tool path completion timed out`, `ee motion stalled/timeout` | 명령 진행/수렴이 제한 시간 내 확인되지 않음 | 관절/EE 피드백, 구동 여부, 전류, 일시적 통신 지연을 확인. 장비가 아직 움직이는지 먼저 확인 |
| `collision/state invalid`, `MoveIt entry planning failed`, `HOME plan rejected` | 시작→경로 접속 또는 후보 경로가 검증되지 않음 | 같은 요청을 강행하지 않음. 현재 자세, 충돌 형상, 관절 범위, 태그/물체 상태를 확인하고 안전한 진입 경로 재계획 |
| `robot moved`, `feedback became stale`, `ownership changed`, `controller inactive` | 검증 당시의 정지 자세·피드백 또는 모드 소유권이 바뀜 | 조이스틱 중립, controller 및 `/joint_states`를 안정화한 뒤 새 검증부터 시작. 다른 팔 launch를 동시에 켜지 않음 |
| `target tag missing/stale`, `CameraInfo`, `TF`, `IK`, `tag stand-off`, `Arduino ... timeout` | 도킹 후속 단계의 관측·기하·구동 확인 실패 | 태그 품질, 광학 프레임, 카메라 정보, 거리, docking 상태를 점검. 자동 삽입·체결 성공으로 간주하지 않음 |
| `person observation lost/unsafe`, `pick/place ownership changed`, `descent IK` | tool2 픽앤플레이스 후속 단계 중단 | 사람 위치 추정, 물체/그리퍼 상태, 팔 정지 상태를 확인. 잡은 물체의 상태를 모르는데 새 pick/place를 곧바로 반복하지 않음 |
| `operator cancelled`, Options focus 변경 | 명시적 취소·소유권 변경 | 새로운 작업 의도를 확인하고 해당 모드에서 다시 검증. HOME은 자동으로 실행되지 않음 |

조이스틱 L2 경로 재요청은 현재 자세와 가장 가까운 **기록 경로 구간**에서 순방향으로 다시 시작한다. 접속 구간도 MoveIt으로 검증하며, 멀거나 충돌이면 planning/checking 단계에서 거부될 수 있다. 도킹·체결·그리퍼 동작까지 이미 진행된 경우에는 관절 위치만으로 작업 상태를 복원할 수 없으므로 상태를 먼저 확인한다. tool2 픽앤플레이스는 전체 접근 경로가 왕복의 전제이므로 일반 경로와 동일한 중간 재개를 적용하지 않는다.

## 4. `fault` 분기

| `reason` 또는 서비스 응답 | 조치 |
| --- | --- |
| `invalid joint position`, `joint cannot stop before soft limit` | 측정 위치·속도와 설정된 실제 소프트리밋을 확인. 관절 영점·엔코더·물리 끝점 여부를 확인한 후 복구. 소프트리밋 숫자를 눈대중으로 넓히거나 무시하지 않음 |
| `hardware fault: <joint>`, RMD 통신/전류 오류, DXL 패킷 오류 | 해당 모터의 전원·버스·온도·전류·관절 구동 상태 확인. `/control/hardware_fault`가 `ok`이고 피드백이 신선한지 확인 |
| `joint feedback timeout`, `invalid joint velocity`, `controller ...`, activation 오류 | `/joint_states`와 controller 상태 복구. 다른 launch/controller 중복 여부 확인 |
| `startup_complete=false`인 fault 또는 `incomplete startup recovery requires inspection and restart` | `/arm/reset_fault`로 복구 불가. 시작 복구의 기구·통신 원인을 점검하고 노드를 재시작해 복구를 처음부터 수행 |
| `/arm/reset_fault`: `healthy controllers and stopped fresh feedback required` | controller 활성, 새 관절 피드백, 정지 속도를 충족한 뒤 다시 요청. 움직이는 관절을 강제로 reset하지 않음 |

시작 복구가 완료된 fault에서만, 원인을 해결하고 실측이 정지한 뒤 다음을 시도한다. 응답 `success=true`는 HOLD 복귀일 뿐 경로 재실행 허가가 아니다.

```bash
ros2 service call /arm/reset_fault std_srvs/srv/Trigger '{}'
ros2 topic echo /arm/control_status --once --full-length
```

## 5. 재시작 전 최소 판정

`mode=hold`, `requested=hold`, `transitioning=false`, `ready=true`, `startup_complete=true`, `entry_alignment_complete=true`, `focus=arm`을 확인한다. 관절 피드백은 신선하고 정지해 있어야 한다. 경로 재요청에는 새 epoch와 전체 충돌 검사가 필요하다. `checking` 중에는 모터가 아직 그 경로를 실행한 것이 아니다. `executing`으로 들어간 뒤의 실패는 중간 자세에서 멈출 수 있으므로 실제 위치와 작업물을 확인한다. TCP 조작은 L1을 놓았다 다시 눌러 재무장하고, 도킹 경로는 L2를 전체 시퀀스 동안 유지한다.

## 6. 툴별 실제 실행 알고리즘과 HOLD 후 분기

공통 시작 순서: 요청 → 신선한 arm focus/HOLD·정지 피드백 확인 → 현재 자세에서 기록 경로의 가장 가까운 구간 선택 → 필요 시 MoveIt 접속 계획 → 모든 경로 샘플의 충돌·관절 범위 검사 → `tool_auto` 소유권 획득 → 실행 → 완료 시 HOLD. `checking` 중 L2 해제나 상태 변경은 움직임 전에 실패한다. `executing` 중 HOLD는 현재 관절에서 멈추며, 기록 시작점이나 HOME으로 자동 복귀하지 않는다. 접속 경로의 검사가 통과해도 실제 랙·공구·케이블 간섭은 별도 실측 확인이 필요하다.

| 툴 | 설정된 순방향과 후속 단계 | 정상 종료가 증명하는 것 | 중간 HOLD 후 판단 |
| --- | --- | --- | --- |
| tool0 | 기록 17→1, base 약 0→3.04 rad. 이어 tag 0 기준 coarse→fine→dock의 TCP 위치를 순차 확인·보정. 최종 정지 자세를 확인한 뒤 `/arm/docking=true` 발행 | 새 Arduino `DOCKED` 응답까지 확인. 실제 체결은 운용자가 카메라로 판정 | 기록 경로 이동 중이라면 현재 자세에서 순방향 suffix를 새로 검증. 태그 보정 중이라면 같은 요청은 기록 경로의 접속/후속 단계를 다시 수행할 수 있으므로 공구·랙 간격과 체결 상태를 먼저 확인 |
| tool1 | 기록 접근 4점, base 약 0→3.29 rad. 정지 후 tag 1의 법선을 5개 샘플로 확인하고, 30 mm 상대 직선 이동을 전체 계획·검증. 끝 자세를 확인한 뒤 `/arm/docking=true` 발행 | 새 Arduino `DOCKED` 응답까지 확인. 실제 체결은 운용자가 카메라로 판정 | 기록 경로 중단인지 30 mm advance 중단인지 구분. advance 후반에서 같은 요청을 반복하면 새 태그 기준으로 **다시 30 mm**를 계획할 수 있으므로 거리·접촉 상태 확인 전 재요청 금지 |
| tool2 | 현재 저장 경로는 복귀 방향 4점, base 약 -0.14→-1.07 rad. 태그 도킹 설정은 `enabled=false` | 4점 기록 경로 완료만 증명 | 단순 경로 HOLD면 가장 가까운 순방향 suffix를 재검증. 파지·배치는 기본 설정 `enabled=false` 및 open/close 보정값 `null`이므로 사용 가능한 단계로 간주하지 않음 |

tool2 픽앤플레이스를 **실측 보정 후 별도로 활성화한 경우에만** 접근→EE 파지→기록 경로 역방향→TCP ready→사람 관측→base 정렬→사전 배치→하강→EE 해제 순서가 된다. 이때 중단 후에는 관절 위치만으로 물체를 잡고 있는지, 놓았는지, 사람이 안전한지 알 수 없다. 이 시퀀스는 중간 suffix 재개를 사용하지 않으며, 새 요청은 처음부터 접근한다. 물체 상태와 사람·배치 공간을 확인하기 전 자동 재요청하지 않는다.

| 중단 지점 | 운용자 판정 | 허용할 다음 행동 |
| --- | --- | --- |
| 요청 거부·접속 계획·`checking` | `execution_id`가 없고 실제 경로 이동이 없었는지 확인 | 입력·피드백·충돌 원인 해결 후 새 검증 요청 |
| 기록 경로 `executing` | 마지막 실행 ID/사유, 정지 자세, 랙·케이블·공구 위치 확인 | 장애가 없고 작업 상태가 알려졌으면 가장 가까운 구간에서 새 요청. stall·following error면 축 점검부터 수행 |
| tag 관측·보정 계획 | 태그 ID/크기·CameraInfo·TF·거리·검출 품질과 실물 간격 확인 | 관측이 회복되고 접촉하지 않았음이 확인된 뒤 새 요청 |
| tool0 최종 위치 또는 tool1 advance 도중·직후 | 실제 접촉, 체결 여부, 잔여 여유 거리 확인 | 재삽입 가능 거리와 상태가 확인된 경우에만 새 접근 계획. 단순 재요청으로 같은 도킹 이동을 반복하지 않음 |
| Arduino/EE 구동 명령 후 | 상태 토픽은 구동 완료 응답이지 실제 체결력·파지력 센서가 아님 | 현장 체결·물체 상태를 확인하고, 불명확하면 자동 재시도 보류 |
| `fault` 또는 관절이 계속 움직임 | 하드웨어 상태·실측 피드백 우선 | 원인 점검 및 정지 후 fault 절차. 경로 재요청 금지 |

`enable_docking_motion:=false`이면 태그 목표 오차를 `preview`로 계속 출력하지만 보정 이동과 Arduino 명령은 실행하지 않는다. 이는 물리적 도킹 완료가 아니다. 관측/시퀀스 제한시간이 지나면 실패로 종료할 수 있다.

현재 설정에서 tool0·tool1은 각각 마지막 보정 이동의 실측 도착을 확인한 뒤 `/arm/docking=true`를 한 번 발행한다. **새로 도착한** `/arm/docking_status` 또는 `/arm/docked`가 도킹 상태를 보고할 때까지 기다린다. `DOCKED`는 Arduino가 보고한 구동 상태이며 실제 체결력 센서가 아니므로 운용자가 카메라로 도킹 상태를 확인한다. `UNKNOWN`·`MOVING`·`ERROR` 상태나 통신 끊김에서 성공으로 판정하지 않는다. 명령 발행 후 L2가 풀리거나 timeout이 나도 Arduino 모터가 이미 움직였을 수 있으므로, 카메라와 새 상태 피드백으로 기구 상태를 확인하기 전 같은 경로를 재실행하지 않는다.
