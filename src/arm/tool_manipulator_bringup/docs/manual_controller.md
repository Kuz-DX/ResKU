# 메인 수동 제어: manual

`manual.launch.py`는 현재 임시로 `manual.py`를 실행한다. 충돌 검사 장면과
`manual_collision.py`는 시작하지 않는다. 기존 충돌 검사 구현은 복구를 위해 보존했다.
기존 manual_controller 협조·독립 수동 제어는 `~/Resku_arm_backup`으로 옮겼다.
메인은 `manual_control_common.py`의 Servo 없는 FK·IK·입력 공통 코드를 사용한다.
`manual_collision.py`는 기존 `Manual` 제어에 충돌 검사 게이트를 추가하지만,
현재 수동 launch에서는 비활성화돼 있다. 기본 하드웨어 제한, 피드백 확인,
시작 복귀와 입력 중단 시 위치 유지는 `manual.py`에서 계속 수행한다.

### 실행과 키 매핑

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
source util/resku_arm_aliases.bash

vr_arm       # 가상 하드웨어
Resku_arm    # 실기; 별도 CAN/모터 준비 및 감독 필요
```

`~/.bashrc`에 `source ~/ResKU/util/resku_arm_aliases.bash`를 등록하면 새 터미널에도 적용된다.
alias 파일은 자신의 위치에서 workspace 경로를 계산하므로 사용자 홈 이름을 고정하지 않는다.
alias는 각각 `manual.launch.py`의 `real_hardware:=false`/`true`를 실행한다.
초기 적재 자세에서 RMD 3축을 가장 가까운 `[하한+2°, 상한-2°]`로 저속 복귀시킨다.
이는 2026-10-24 사용자 실기 로그 확인 후 추가한 처리다. 자동 home·자동 집기는 수행하지 않는다.
복귀 중 DXL 3축의 목표는 초기 측정 위치를 유지하며, EE는 0속도를 보낸다.

필요한 초기 복귀는 조이스틱 연결이나 `/joy` 수신을 기다리지 않고 자동 진행된다.
복귀 중 조이스틱 이동 입력은 무시하고 EE는 정지시킨다. 시작 상태는 `drive`이며,
`Startup RMD recovery verified from joint feedback` 로그와 조이스틱 연결을 확인한 뒤
스틱·D-pad·L1/R1/O를 중립으로 놓고 Options를 눌러 `arm`으로 전환한다.
최초 `arm` 진입 시 DXL base/roll/yaw 3축을 0 rad로 최대 0.03 rad/s로 정렬한다.
시작 각도 제한은 없으며, 각 축의 소프트 리밋 안에서 현재 자세부터 이동한다.
제한 시간은 가장 오래 걸리는 축의 예상 이동 시간에 15초를 더해 정한다.
정렬 중에는 RMD 목표를 유지하고 EE 속도는 0으로 보내며 조이스틱 이동 입력은 무시한다.
정렬 완료 후 수동 조작은 신선한 입력·arm 포커스·중립 확인 후에만 허용한다. 키 매핑은 이전과 같다.

조이스틱은 런치 실행 후 연결해도 된다. 연결 여부와 joy_node의 생존 상태는 구동 프로세스의
필수 검사에서 제외한다. 드라이버가 종료되면 드라이버만 재시작하고 하드웨어·컨트롤러·
초기 복귀는 유지한다. 연결 때마다 터미널에 `조이스틱 연결 완료`를 한 줄만 표시하며,
미연결·연결 해제·대기 상태 메시지는 반복 출력하지 않는다. 원본 드라이버 출력은 로그 파일에
보존한다. 수동 입력이 끊겼을 때의 위치 유지와 EE 0속도, 중립 재개 조건은 유지한다.

| 조이스틱 입력 | `Resku_arm` 동작 | ROS 입력/관절 |
|---|---|---|
| 네모 | Stop_pose 정렬 (버튼 3, 한 번 누름) | DXL 3축 0 rad; RMD 어깨 +1.4835298641951802, 엘보 +1.5184364492350666, 손목 피치 -1.5533430342749532 rad |
| Options | drive ↔ arm 입력 허용 전환 | 버튼 9 |
| 오른쪽 스틱 위/아래 | RMD 어깨·팔꿈치·손목 피치 3축 협조 숙이기/젖히기 | 축 4 부호 반전; 위=숙이기, 아래=젖히기 |
| L1 / R1 | 베이스 + / − 방향 직접 회전 | 버튼 4 / 5; base_joint, DXL ID 1 |
| D-pad 왼쪽/오른쪽 | 손목 롤 + / − 방향 직접 조작 | 축 6; wrist_roll_joint, DXL ID 2 |
| 왼쪽 스틱 왼쪽/오른쪽 | 손목 요 + / − 방향 직접 조작 | 축 0; wrist_yaw_joint, DXL ID 3 |
| O 유지 + D-pad 왼쪽/오른쪽 | EE + / − 속도 명령 | 버튼 1 + 축 6, 별도 EE velocity controller |
| O 유지 + D-pad 위/아래 | 손목 Pitch + / − 단독 조작 | 버튼 1 + 축 7 |
| O 유지 + 왼쪽 스틱 위/아래 | 말단 Pitch 유지 상하 이동 | 버튼 1 + 축 1, 위=+Z, 아래=−Z |
| 입력 해제 | 해당 관절의 마지막 유효한 목표 유지 | 새로운 이동 목표 누적 중단 |

축의 좌/우·위/아래 부호는 기존 Linux PlayStation 매핑 기준이다. 실제 조이스틱의
부호와 모터의 육안상 방향은 저속 실기 확인 전 확정하지 않는다. 특히 오른쪽 스틱은
관절 하나의 +/− 입력이 아니라 TCP 전후 병진 요청이므로 자세에 따라 관절 증분이 달라진다.
오른쪽 스틱 입력과 DXL 세 입력은 동시에 사용할 수 있다. L1/R1 동시 입력은 상쇄된다.
O를 누르면 EE·손목 Pitch·수직 이동 매핑만 동작하며 기존 롤·요·베이스·전후 입력은 억제한다.
손목 Pitch와 수직 이동을 동시에 입력하면 손목 Pitch 조정을 우선한다.
손목을 조정한 뒤 D-pad를 놓고 O + 왼쪽 스틱 상하로 이동하면 현재 목표 말단 기울기를 유지한다.
수직 이동은 base_actuator의 Z축 기준이며 손목을 포함한 Pitch 3축이 자동 보정된다.
특이 자세에서는 감속하거나 정지한다. 기존 독립 모드는 백업으로 이관했다.
이전 문서의 "EE는 조이스틱 매핑에 포함하지 않는다"는 설명과 달리 현재 실제 소스는
O + D-pad 좌우로 별도 EE 속도 명령을 내며 신규 경로는 이 구현을 유지한다.

### 구현과 진단

**아래 충돌 검사 설명은 현재 비활성화된 구현에 대한 기록이다.**

수동 충돌 검사는 `/manual/check_state_validity`를 사용한다. 같은 URDF/SRDF로 실행하는
`/manual/collision_scene`의 궤적 실행은 비활성화돼 있다. 수동 전용 SRDF 그룹은 팔 6축과
`ee_joint`가 움직이는 링크를 검사하므로 회로박스·차체·고정 공구함 및 다른 팔 링크와의
충돌을 검사한다. 고정물끼리의 장착 접촉은 검사 그룹 밖이고, 기존 SRDF의 조립부 접촉
예외는 적용된다. 자동 제어의 SRDF·launch·실행 경로는 변경하지 않는다.

위치 명령은 기존 50 Hz이며, 충돌 **검사 묶음은 최대 10 Hz**다. 입력 방향을 유지한다고
가정한 최대 0.35초의 짧은 관절 구간을 미리 검사하고, 승인된 직선 구간 안에서 속도 제한을
지키며 작은 위치 명령으로 이동한다. 병진/자세 입력은 다음 검사에서 다시 계산한다.
검사 대기 중에는 마지막 승인 위치 목표를 유지하고 EE 속도는 0이다. 한 번에 검사 묶음
하나만 진행하며, 팔 관절 간격 0.005 rad, EE 간격 0.02 rad로 최대 96개 상태를 검사한다.
이는 경로 계획 호출이 아니지만 CPU·메시지 지연에 따라 조작 속도는 낮아질 수 있다.

충돌·검사 오류·150 ms 응답 초과·서비스 단절 시 미승인 목표와 남은 구간을 폐기하고,
**신선한 실측 관절 위치를 HOLD 목표로 발행하며 EE는 0속도**를 보낸다. 입력 방향 변경이나
해제 시에도 이전 구간을 폐기한다. 모터 관성과 통신 지연으로 실제 정지는 즉시 완료되지
않는다. 일반 수동 입력, 네모 Stop_pose, 초기 RMD 복귀가 모두 같은 검사를 거친다.
현재 자세 자체가 충돌/무효로 판정되면 자동 탈출 동작은 수행하지 않는다.

기본 파라미터는 `collision_check_rate_hz:=10.0`, `collision_lookahead_sec:=0.35`,
`collision_timeout_sec:=0.15`다. 앞보기 시간은 검사 주기+응답 제한+0.04초 이상이어야 한다.
`/manual/diagnostics`의 `collision`에 검사 묶음 수, 승인·거부·시간초과 수, 최근 응답 시간,
충돌 링크 쌍을 기록한다. 회로박스 좌표와 실제 조립 오차·제동 거리는 실기 확인 대상이며,
이 기능은 샘플 상태 검사이므로 연속 공간 전체의 충돌 없음을 보증하지 않는다.

2026-10-27: 구현 중 단위 테스트를 수행했으나, 사용자 요청으로 추가 검사 실행을 중단했다.
최종 수정본의 실제 MoveIt 연결·지연 및 하드웨어 동작은 미검증이다.

`manual_control_common.py`는 실행 진입점이 아닌 공통 모듈이다. 기존 FK·RMD 3축
병진 DLS·속도 제한·조이스틱 매핑을 유지하며 Servo 발행 코드는 포함하지 않는다.
메인 노드가 6축 위치 목표를 `/manual_position_controller/commands`로 전달하고,
EE는 별도 `/ee_velocity_controller/commands`로 속도를 전달한다.
기존 전후 조작의 RMD 경계·목표 lead 제한은 관절별로 적용한다.
새 수직 이동에서는 기울기 유지를 위해 경계에서 3축 증분을 함께 축소하고,
한 축의 목표 lead가 초과하면 3축 이동을 함께 보류한다.
조작하지 않는 관절은 마지막 유효 목표를 유지하며 입력 재개 시 중립 확인을 요구한다.

`/manual/diagnostics`는 원본 Joy, 목표·측정값·오차, focus/fault/phase,
컨트롤러 상태, RMD 요청·적용 속도 및 제한 관절을 50Hz JSON String으로 발행한다.
시각은 monotonic 초이며 rosbag 수신 시각으로 기록과 대응시킨다.

```bash
ros2 bag record -o /tmp/resku_manual_bag /joy /joint_states \
  /manual_position_controller/commands /manual/diagnostics \
  /ee_velocity_controller/commands /control/active_target /control/hardware_fault
```

### 검증

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
python3 -m pytest src/arm/tool_manipulator_bringup/test/test_manual_control_common.py \
  src/arm/tool_manipulator_bringup/test/test_manual.py \
  src/arm/tool_manipulator_bringup/test/test_manual_joy_log.py -q
```

기존 분석·실기 로그 이력과 정리 전 원본은 `~/Resku_arm_backup/before_changes`에 보존한다.

Stop_pose는 수동 조종 실행 파일을 종료하기 직전에 정렬하는 안정적인 종료용 자세다.
모터 전원이 꺼져도 팔이 큰 충격 없이 내려앉도록 하기 위한 자세로,
arm 모드에서 조작 입력을 중립으로 놓고 네모 버튼(버튼 3)을 눌러 정렬한다.
정렬이 완료된 뒤 조종 실행 파일을 종료하고 모터 전원을 끈다.
종료 시 자동으로 정렬하지 않으므로 종료 전에 직접 실행한다.
목표 각도는 DXL 3축 모두 0 rad, RMD 어깨 1.4835298641951802 rad(+85°),
엘보 1.5184364492350666 rad(+87°), 손목 피치 -1.5533430342749532 rad(-89°)다.

Stop_pose는 arm 포커스·중립 확인 후 버튼을 누르면 충돌 검사, 관절별 속도와 목표 lead 제한으로
이동하며 EE 속도는 0을 유지한다. 이동 중 다른 이동 입력은 무시하며, 완료 후 중립을
확인해야 수동 조작을 재개한다. 입력/피드백 유실·포커스 변경·fault 시 정렬은 취소된다.
목표는 제한 밖으로 잘라서 실행하지 않고 전체 요청을 거부한다. 변경한 어깨 +85°와
손목 피치 -89°를 포함한 목표 전체가 현재 hardware.yaml의 수동 조작 제한 범위 안에 있다.
