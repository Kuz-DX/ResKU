# 메인 수동 제어: manual

`Resku_arm` → `manual.launch.py` → `manual.py`.
기존 manual_controller 협조·독립 수동 제어는 `~/Resku_arm_backup`으로 옮겼다.
메인은 `manual_control_common.py`의 Servo 없는 FK·IK·입력 공통 코드를 사용한다.
충돌 검사는 없으며 기존 초기 RMD 복귀와 입력·피드백 보호를 유지한다.

### 실행과 키 매핑

```bash
cd /home/sw/ResKU
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tool_manipulator_bringup
source install/setup.bash
source util/resku_arm_aliases.bash

vr_arm       # 가상 하드웨어
Resku_arm    # 실기; 별도 CAN/모터 준비 및 감독 필요
```

`/home/sw/.bashrc`에 `util/resku_arm_aliases.bash`를 source하도록 등록해 새 터미널에도 적용된다.
이미 열려 있던 터미널은 `source /home/sw/ResKU/util/resku_arm_aliases.bash`로 적용한다.
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
| Options | drive ↔ arm 입력 허용 전환 | 버튼 9 |
| 오른쪽 스틱 위/아래 | RMD 어깨·팔꿈치·손목 피치 3축 협조 숙이기/젖히기 | 축 4 부호 반전; 위=숙이기, 아래=젖히기 |
| L1 / R1 | 베이스 + / − 방향 직접 회전 | 버튼 4 / 5; base_joint, DXL ID 1 |
| D-pad 왼쪽/오른쪽 | 손목 롤 + / − 방향 직접 조작 | 축 6; wrist_roll_joint, DXL ID 2 |
| 왼쪽 스틱 왼쪽/오른쪽 | 손목 요 + / − 방향 직접 조작 | 축 0; wrist_yaw_joint, DXL ID 3 |
| O 유지 + D-pad 위/아래 | EE 시계/반시계 속도 명령 | 버튼 1 + 축 7, 별도 EE velocity controller |
| 입력 해제 | 해당 관절의 마지막 유효한 목표 유지 | 새로운 이동 목표 누적 중단 |

축의 좌/우·위/아래 부호는 기존 Linux PlayStation 매핑 기준이다. 실제 조이스틱의
부호와 모터의 육안상 방향은 저속 실기 확인 전 확정하지 않는다. 특히 오른쪽 스틱은
관절 하나의 +/− 입력이 아니라 TCP 전후 병진 요청이므로 자세에 따라 관절 증분이 달라진다.
오른쪽 스틱 입력과 DXL 세 입력은 동시에 사용할 수 있다. L1/R1 동시 입력은 상쇄된다.
왼쪽 스틱 상하와 개별 RMD 관절 조작은 이 모드에 할당하지 않는다. 기존 독립 모드는 백업으로 이관했다. O는 EE에만 관여한다.
이전 문서의 "EE는 조이스틱 매핑에 포함하지 않는다"는 설명과 달리 현재 실제 소스는
O + D-pad로 별도 EE 속도 명령을 내며 신규 경로는 이 구현을 유지한다.

### 구현과 진단

`manual_control_common.py`는 실행 진입점이 아닌 공통 모듈이다. 기존 FK·RMD 3축
병진 DLS·속도 제한·조이스틱 매핑을 유지하며 Servo 발행 코드는 포함하지 않는다.
메인 노드가 6축 위치 목표를 `/manual_position_controller/commands`로 전달하고,
EE는 별도 `/ee_velocity_controller/commands`로 속도를 전달한다.
RMD 경계·목표 lead 제한은 관절별로 적용해 한 축의 제한이 다른 두 축을 막지 않는다.
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
source /home/sw/ResKU/install/setup.bash
python3 -m pytest src/arm/tool_manipulator_bringup/test/test_manual_control_common.py \
  src/arm/tool_manipulator_bringup/test/test_manual.py \
  src/arm/tool_manipulator_bringup/test/test_manual_joy_log.py -q
```

기존 분석·실기 로그 이력과 정리 전 원본은 `~/Resku_arm_backup/before_changes`에 보존한다.
