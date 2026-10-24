# 메인 수동 제어: manual

`Resku_arm` → `manual.launch.py` → `manual.py`.
기존 manual_controller 협조·독립 수동 제어는 `~/Resku_arm_backup`으로 옮겼다.
메인은 `manual_control_common.py`의 Servo 없는 FK·IK·입력 공통 코드를 사용한다.
충돌 검사는 없으며 기존 초기 RMD 복귀와 입력·피드백 보호를 유지한다.

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
수동 조작은 신선한 입력·arm 포커스·중립 확인 후에만 허용한다. 키 매핑은 이전과 같다.

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

Stop_pose는 arm 포커스·중립 확인 후 버튼을 누르면 관절별 속도와 목표 lead 제한으로
이동하며 EE 속도는 0을 유지한다. 이동 중 다른 이동 입력은 무시하며, 완료 후 중립을
확인해야 수동 조작을 재개한다. 입력/피드백 유실·포커스 변경·fault 시 정렬은 취소된다.
목표는 제한 밖으로 잘라서 실행하지 않고 전체 요청을 거부한다. 변경한 어깨 +85°와
손목 피치 -89°를 포함한 목표 전체가 현재 hardware.yaml의 수동 조작 제한 범위 안에 있다.
