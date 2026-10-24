# RMD 협조 수동 조작

Servo 비사용 비교 시험 경로와 `Resku_arm_servox` 실행법·키 매핑은 아래의
[신규 Servo 비사용 비교 시험](#신규-servo-비사용-비교-시험-manual_servo_outpy)을 참고한다.

`manual_controller.py`는 dolbotZ의 MANUAL_EE 미분 IK를 참고한다. 현재 ResKU
URDF와 관절 피드백으로 shoulder/elbow/wrist_pitch 3축의 이동량을 매번 계산한다.
고정 회전 비율, 6축 TCP 자세 제어, 장애물 우회 경로계획은 아니다. 그리퍼 방향은
유지하지 않는다. 베이스와 손목 롤/요 입력에 따른 끝점 이동을 RMD가 자동 보상하지 않는다.

| 입력 | 기능 |
|---|---|
| Options (버튼 9) | drive/arm 전환; 시작 시 drive로 비활성 |
| 오른쪽 스틱 상하 (축 4) | RMD 3축 협조 숙이기/젖히기; 기존 대비 방향 반전 |
| 왼쪽 스틱 좌우 (축 0, 부호 반전) | 손목 요 조작; 다른 팔 관절과 동일한 제어 경로 |
| L1/R1 (버튼 4/5) | 베이스 직접 회전 |
| D-pad 좌우 (축 6) | 손목 롤 좌우 조작 |
| O 유지 + D-pad 위 (버튼 1 + 축 7) | EE 시계 방향 속도 명령 |
| O 유지 + D-pad 아래 (버튼 1 + 축 7) | EE 반시계 방향 속도 명령 |

끝점은 기본 `tcp_link`지만 계산에 참여하는 관절은 RMD 3개뿐이다. 나머지 관절의
실제 각도도 FK에 반영한다. `controlled_link`로 바꾸려면 RMD 3축 하류 링크여야 한다.
EE ID 4는 기존 ee_controller가 홀드하며 새 조이스틱 매핑에는 포함하지 않는다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tool_manipulator_bringup
source install/setup.bash

# 가상 하드웨어 (기본값); RViz 포함
ros2 launch tool_manipulator_bringup manual_control.launch.py launch_rviz:=true

# 실기: hardware.yaml 검사 → 2도 초기 복귀 → controller 전환 → 조작 허용
ros2 launch tool_manipulator_bringup manual_control.launch.py real_hardware:=true
```

협조 런치는 arm_controller를 유지하고 MoveIt Servo를 실행한다. 실기에서는 기존
recover-only를 먼저 수행한다. Servo 충돌 검사를 켜고 관절 속도를 JointJog로 전달한다.
독립 런치는 복귀 후 arm_controller를 비활성화하고 manual_position_controller를
활성화하며 Servo를 실행하지 않는다. 각 단계가 실패하면 다음 단계로 진행하지 않는다.
모드 간 전환은 런치를 종료하고 다시 실행한다. 두 런치를 동시에 실행하지 않는다.

독립 모드의 위치 목표는 모드 진입 때 측정값으로 시작하며 자동 home을 요구하지 않는다. 새 모드는
독립적인 0.02rad 경계 여유를 사용한다. 경계에서는 RMD 전체 증분을 같은 비율로 줄이고
안쪽으로 돌아오는 입력을 허용한다. 실제 하드웨어 한계 밖이면 조작을 차단한다.
50Hz, 선속도 0.03m/s, 관절속도 최대 0.12rad/s와 hardware.yaml 제한 중 작은 값을 쓴다.
손목 요는 왼쪽 스틱 좌우 입력을 사용하며 다른 5개 팔 관절과 동일한 명령 경로를 따른다.
협조 모드는 Servo에 6축 JointJog를 보내고 `arm_controller`가 위치 궤적으로 구동하며,
입력이 끊기면 0 속도를 보낸다.
독립 모드는 속도를 위치 목표에 누적하고, 스틱을 놓으면 마지막 목표 위치를 유지한다.
Dynamixel은 기존 Position Mode를 유지한다.
손목 롤은 D-pad 좌우로 조작한다.
EE는 별도 `/ee_velocity_controller/commands`에 1축 속도 명령을 보낸다. O 버튼을 계속
누른 상태에서만 D-pad 상하 입력이 허용되며, O 해제·D-pad 중립·포커스 해제·입력 timeout
시 0속도를 발행한다. 실제 EE velocity controller와 하드웨어 Operating Mode 구성은 별도다.
DLS 감쇠로 특이점 부근 속도를 완화하지만 모든 자세에서 입력 방향 실현을 보장하지는 않는다.
가상 기본 자세는 팔이 수직으로 펴져 있어 위쪽 이동이 불가능할 수 있다. 전후 입력으로
팔을 굽힌 뒤 상하 입력을 시험한다. 계산된 이동이 요청의 10% 미만이면 노드가 경고한다.

입력 0.25초 또는 관절별 피드백 0.5초 단절 시 협조 모드는 0 속도 조그를 반복 발행하고,
독립 모드는 마지막 위치 목표를 유지한다. 다음 진입에서는 측정값으로 다시 시작한다.
독립 모드의 목표와 측정값 차이는 0.10rad 이내로 제한하며, 관측된 차이가
이를 넘으면 홀드를 래치한다. 원인을 확인하고 focus를 drive→arm으로 전환해야 해제된다.
프로세스 사망은 토픽 기반 컨트롤러 자체의 watchdog이 아니며, 장치 고장 시 즉시 정지를
보장하는 기능도 아니다. 하드웨어 전류·속도 보호와 실제 통신 지연은 여전히 별도로 검증해야 한다.

`vr_arm`/`Resku_arm`은 이 런치의 mock/real 경로를 각각 사용한다. 협조 모드에서는
Servo가 충돌·관절 한계를 검사한다. 노드 자체에는 충돌 검사가 없다.

## 독립 관절 모드: manual_controller.sep.py

기존 `joint_joy_teleop_sep.py`를 `manual_controller.sep.py`로 교체했다. 이 모드도
Servo/IK 없이 같은 위치 컨트롤러에 직접 목표를 전달한다. 모델 구독은 필요하지 않다.
관절별 한계는 독립 적용하므로 한 관절이 한계에 도달해도 다른 관절은 조작할 수 있다.
피드백·입력 timeout, 속도 제한, following-error 홀드와 Options 포커스는 공통 구현이다.

| 입력 | 독립 모드 동작 |
|---|---|
| 오른쪽 스틱 상하 (축 4) | 어깨 +/− |
| 왼쪽 스틱 상하 (축 1) | 팔꿈치 −/+ |
| D-pad 상하 (축 7) | 손목 피치 +/− |
| D-pad 좌우 (축 6, 부호 반전) | 손목 요 속도형 조작; 입력을 제한된 위치 목표로 누적 |
| L1/R1 (버튼 4/5) | 베이스 +/− |

독립 모드에서도 손목 롤은 수동 입력에 할당하지 않으며 손목 요만 D-pad 좌우를 사용한다.

```bash
# 가상 / 실기 중 하나만 실행
ros2 launch tool_manipulator_bringup manual_controller.sep.launch.py launch_rviz:=true
ros2 launch tool_manipulator_bringup manual_controller.sep.launch.py real_hardware:=true
# 또는 공통 런치에 control_mode:=separate 전달
```

`vr_arm_sep`는 가상, `Resku_arm_sep`는 실기 독립 모드다. `vr_arm`/`Resku_arm`은
계속 협조 모드를 쓴다. 구형 sep 실행 파일과 호환 런치는 제거했다.
새 sep 런치는 전체 스택을 시작하므로 이미 실행 중인 arm stack과 함께 실행하지 않는다.

## 신규 Servo 비사용 비교 시험: manual_servo_out.py

2026-10-24 구현·검증. 이 경로는 충돌 검사가 없는 제한된 감독 시험용이다.
기존 `manual_controller.py`, `manual_controller.sep.py`, 두 기존 런치,
hardware.yaml과 실제 Hardware Interface는 수정하지 않았다. 시작 시 Git 작업 트리는
깨끗했다. 기존 협조·독립 모드와의 비교를 위해 새 실행 경로를 추가했다.

### 실행과 키 매핑

```bash
cd /home/sw/ResKU
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tool_manipulator_bringup
source install/setup.bash
source util/resku_arm_aliases.bash

vr_arm_servox       # 가상 하드웨어
Resku_arm_servox    # 실기; 별도 CAN/모터 준비 및 감독 필요
```

`/home/sw/.bashrc`에 `util/resku_arm_aliases.bash`를 source하도록 등록해 새 터미널에도 적용된다.
이미 열려 있던 터미널은 `source /home/sw/ResKU/util/resku_arm_aliases.bash`로 적용한다.
alias는 각각 `manual_servo_out.launch.py`의 `real_hardware:=false`/`true`를 실행한다.
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

| 조이스틱 입력 | `Resku_arm_servox` 동작 | ROS 입력/관절 |
|---|---|---|
| Options | drive ↔ arm 입력 허용 전환 | 버튼 9 |
| 오른쪽 스틱 위/아래 | RMD 어깨·팔꿈치·손목 피치 3축 협조 숙이기/젖히기 | 축 4; 기존 협조 모드와 같은 부호 |
| L1 / R1 | 베이스 + / − 방향 직접 회전 | 버튼 4 / 5; base_joint, DXL ID 1 |
| D-pad 왼쪽/오른쪽 | 손목 롤 + / − 방향 직접 조작 | 축 6; wrist_roll_joint, DXL ID 2 |
| 왼쪽 스틱 왼쪽/오른쪽 | 손목 요 − / + 방향 직접 조작 | 축 0 부호 반전; wrist_yaw_joint, DXL ID 3 |
| O 유지 + D-pad 위/아래 | EE 시계/반시계 속도 명령 | 버튼 1 + 축 7, 별도 EE velocity controller |
| 입력 해제 | 해당 관절의 마지막 유효한 목표 유지 | 새로운 이동 목표 누적 중단 |

축의 좌/우·위/아래 부호는 기존 Linux PlayStation 매핑 기준이다. 실제 조이스틱의
부호와 모터의 육안상 방향은 저속 실기 확인 전 확정하지 않는다. 특히 오른쪽 스틱은
관절 하나의 +/− 입력이 아니라 TCP 전후 병진 요청이므로 자세에 따라 관절 증분이 달라진다.
오른쪽 스틱 입력과 DXL 세 입력은 동시에 사용할 수 있다. L1/R1 동시 입력은 상쇄된다.
왼쪽 스틱 상하와 개별 RMD 관절 조작은 이 모드에 할당하지 않는다. 기존 독립 모드에서
RMD 개별 조작을 계속 사용하며 두 모드를 동시에 실행하지 않는다. O는 EE에만 관여한다.
이전 문서의 "EE는 조이스틱 매핑에 포함하지 않는다"는 설명과 달리 현재 실제 소스는
O + D-pad로 별도 EE 속도 명령을 내며 신규 경로는 이 구현을 유지한다.

### 원본 분석과 재사용 범위

`/home/sw/dolbotZ/src/arm/robot_arm_bringup`의 소스를 확인했다.

| 원본 | 실제 동작 |
|---|---|
| `gamepad_position_controller.py` → `manual_total_position_node.py` (MANUAL_100) | 20Hz. 어깨 축4, 팔꿈치 축1, wrist 축7, base L1/R1. 임계값 .5, timeout .5초. 목표를 측정값으로 초기화한 뒤 속도×dt 누적·관절별 clip. 관절 속도 기본 .1/.07/.1/.1rad/s, dt 최대 .1초. 입력 중단 시 최종 목표 유지. JointState를 `/manual_joint_commands`에 발행. |
| `ee_manual_controller.py` (MANUAL_EE) | 20Hz. 오른쪽 축4는 부호 반전한 전후, 왼쪽 축1은 수직 TCP 병진. base L1/R1은 IK 밖. URDF FK, 6행 Jacobian 중 병진 3행만 DLS에 사용. IK 대상 shoulder/elbow/wrist 3축, 감쇠 .08, 최대 .07rad/s 공통 축소, 한계 clip. 목표 자세로 FK 계산하며 ready 신호와 모드 진입 시 측정값으로 초기화. |
| `joint_command_mux.py` | 최종 position_controller 출력, 속도 램프, 추종 오차 감시, 측정값 hold. MANUAL_EE의 Circle home/Square 운반 자세는 단계별 동작이며 일반 충돌 회피 기능은 아님. |

원본은 ResKU와 관절 수·명칭 및 입력 방향이 다르다. ResKU에는 wrist roll/yaw가
추가되어 있으며 기존 협조 코드에는 수직 TCP 입력이 없다. 이를 새로 추가하지 않았다.
MANUAL_100/EE가 직접 ros2_control을 소유하는 구조도 아니며 mux가 최종 위치 명령을
전달한다. 별도 manual_gripper_controller 경로도 확인했고 ResKU의 EE 경로와 혼동하지 않았다.

ResKU의 `ManualController`에서 ArmChain, 병진 DLS, 속도/위치 한계, 입력 매핑,
포커스와 EE 출력 함수를 재사용한다. `manual_servo_out.py`는 서브클래스로 수명주기·
안전 게이트·진단만 추가하며 기존 파일을 덮어쓰지 않는다.

TCP 기준은 `base_actuator` → `tcp_link`. 요청 병진 속도는
`0.03 × axis4 × [cos(base), sin(base), 0]` m/s이며 방향 반전 설정은 기존과 같다.
실제 URDF FK에는 고정 취급하는 DXL의 **측정 각도**가 들어가지만 Jacobian 열과 IK
출력은 RMD 3축만 포함한다. 자세·각속도 제약이나 수직 이동 입력은 추가하지 않았다.
axis4가 0이면 FK/Jacobian/IK 호출 자체를 생략한다. DXL 입력 목표는 FK 입력으로
미리 적용하지 않으므로 같은 측정 자세에서 DXL 동시 입력이 RMD 계산을 덮어쓰지 않는다.
이후 실제 DXL 각도가 바뀌면 TCP 기하가 달라지는 것은 정상적인 FK 반영이다.

위치는 하나의 6축 벡터로 관리한다. RMD는 속도·경계에서 공통 비율로 축소하고,
DXL은 각 관절별 제한을 적용한다. 한 DXL의 목표 lead 한계가 다른 DXL의 조작을
막지 않는다. 비조작 관절에는 기존의 유효한 유지 목표를 전달한다.

### 컨트롤러·안전 정책

`manual_controllers.yaml`의 실제 유형은
`forward_command_controller/ForwardCommandController`, 인터페이스는 `position`, 메시지는
`Float64MultiArray`이다. 관절 순서는 base/shoulder/elbow/wrist_pitch/wrist_roll/wrist_yaw다.
신규 controller를 만들지 않았다. 기존 arm_controller는 위치 JTC이며 새 런치에서는
로드·활성화하지 않는다. MoveIt Servo와 JointJog 출력도 실행하지 않는다.

런치는 전용 도메인별 파일 잠금과 실행 중 ROS 노드 사전 검사를 수행한 뒤 상태 발행기,
ros2_control, joint_state_broadcaster, 별도 ee_velocity_controller를 시작한다.
manual_position_controller는 inactive 상태로 로드한다. 노드는 컨트롤러 유형·관절 순서·
인터페이스, 전체 피드백, 명령 구독자 연결을 확인하고 **측정값을 먼저 발행한 뒤** STRICT
활성화를 요청한다. 성공 응답뿐 아니라 그 이후의 active 상태 응답도 확인한다.
컨트롤러 활성화 중에는 측정 위치를 발행하며 이동 입력은 차단한다. 초기 복귀가 필요한
경우 캡처한 유지 목표를 고정하여 측정 잡음으로 바깥쪽 목표를 생성하지 않는다.

2026-10-24 무반응 진단에서 `phase=waiting`, `manual_position_controller=inactive`,
`target_rad=null`을 확인했다. 어깨 측정 1.550201rad는 상한 1.518070rad보다 1.841°,
손목 피치 -1.599769rad는 하한 -1.592421rad보다 0.421° 밖이었다. 기존 실기 런치의
초기 복귀와 달리 신규 경로에는 이 처리가 없어 초기 gate에서 대기한 것이 직접 원인이었다.
토크 OFF나 Options 입력 유실로 판정하지 않았다. 실제 장치에는 읽기 전용 조회만 수행했다.

추가한 초기 복귀는 IK·Servo·JTC 없이 manual_position_controller의 6축 벡터에서
RMD 3축만 최대 .03rad/s(각 하드웨어 속도보다 크지 않음)로 목표에 접근시킨다.
측정 위치를 먼저 유지한 뒤 작은 위치 증분을 누적하며 경계로 순간 clip하지 않는다.
한계 안쪽이지만 2° 여유가 없는 RMD도 가장 가까운 2° 안쪽으로 이동한다.
목표 도달 및 신선한 측정값의 원래 한계 내 복귀, 목표 오차 .005rad 이내를 확인한 후
일반 수동 입력을 허용한다. 이 처리는 시작 시 한 번뿐이며 조작 중 한계 이탈에는 자동 재복귀하지 않는다.
초기 복귀는 기존 recover-only와 같은 시작 시 전용 동작으로, 조이스틱 미연결·입력 단절과
독립적으로 진행된다. 복귀 중 조이스틱 이동 요청은 적용하지 않는다. 피드백 단절 시에는
새 복귀 증분을 중단하고 마지막 목표를 유지한다. 복귀 후 일반 수동 조작에는 기존 입력
timeout·중립 재개·포커스 정책을 그대로 적용한다.
복귀 시작 후 15초 timeout, .10rad 추종 오차 초과 또는 복귀 허용 범위 이탈 시 차단한다.
단순한 영점 오류까지 자동 보정하지 않도록 RMD 시작 이탈 허용값은 최대 5°로 제한한다.
DXL 한계 이탈은 복귀 대상으로 허용하지 않는다. 이탈 관절·값·초과 각도 및 시작 gate의
대기 이유는 경고 로그와 diagnostics의 activation_blockers/position_limit_violations로 표시한다.
소프트 리밋·영점·회전 방향·HI는 수정하지 않았다.

추가 사용자 로그에서는 조이스틱 연결 완료 없이 약 12초 동안
`startup RMD recovery requires a connected, neutral joystick`에 대기하다 종료됐다.
초기 복귀에 추가했던 이 의존성을 제거하여 기존처럼 조이스틱보다 먼저 복귀할 수 있게 했다.
따라서 새 버전에서 이 조이스틱 대기 경고는 발생하지 않는다. 하드웨어 구동 검증은 수행하지 않았다.

기본 50Hz, 관절속도 min(.12rad/s, hardware.yaml 속도), 위치 경계 여유 .02rad,
목표 lead 최대 .10rad, joystick deadzone .12, 입력 timeout .25초, 관절별 피드백
timeout .5초다. 컨트롤러 상태는 .2초 간격으로 확인하고 .75초 단절 시 래치 차단한다.
STRICT 활성화 실패·5초 초과·arm_controller 자원 점유·활성 상태 유실·설정 불일치는
명령을 차단한다. EE 컨트롤러 비활성화도 차단 조건이다. watchdog은 monotonic 시간을 쓴다.

입력 timeout·포커스 해제·추종 오차 때 유효한 측정 위치를 hold 목표로 한 번 캡처하고
기존 목표를 관절 속도 이내에서 그 위치로 줄인다. 따라서 목표가 순간적으로 .10rad
점프하지 않는다. 피드백이 없으면 마지막 유효 목표를 유지하며 추측 위치를 만들지 않는다.
복구 후 중립 입력을 확인해야 재개한다. 일반 중립 입력은 마지막 목표를 유지한다.
추종 오차 래치는 확인 후 Options로 drive→arm을 거쳐 해제한다. 하드웨어 fault와
컨트롤러 fault는 포커스로 해제하지 않으며 점검 후 전체 재시작이 필요하다.

`/control/hardware_fault`를 transient-local로 구독한다. RMD는 통신 fault 이후에도
고정 상태를 정상 주기의 joint_states로 전달할 수 있으므로 메시지 수신 시각만으로
건전성을 판정할 수 없다. DXL의 기존 오류 반환·전송 진단을 유지했으며 일부 global fault
래치가 기존 코드에서 비활성화되어 있다는 사실도 변경하지 않았다. 센서 갱신 시각이
joint_states에 직접 표시되지 않는 한 모든 종류의 고정 피드백을 검출할 수 있는 것은 아니다.

관절 영점·방향 변환은 기존 HI에서 수행한다. Mode 4의 nearest-turn 구현도 수정하지
않았고 신규 노드에 ±2π 선택을 추가하지 않았다. **현재 xacro는 nearest-turn 플래그를
넘기지 않으며 HI의 기본값은 false**다. 따라서 기능 보존과 현재 기능 활성화는 구분해야 한다.
이를 임의로 켜거나 raw zero/방향/프로파일 값을 바꾸지 않았다.

모의 URDF의 EE command interface만 이 런치 안에서 velocity로 맞추어 실제와 같은 별도
EE 출력 토픽을 사용한다. 기존 mock 모델 파일과 기존 런치의 EE 동작에는 영향이 없다.
EE는 IK 대상에 포함하지 않으며 자동 집기나 승인 우회 동작은 추가하지 않았다.

종료 시 노드는 EE 0속도·hold 및 bounded controller 비활성화를 시도하고,
controller_manager는 전체 컨트롤러와 하드웨어를 정리한다. 핵심 프로세스 종료는 전체
런치 종료를 유발한다. 동시 종료·kill -9·통신 단절에서 최종 패킷 도달은 보장할 수 없다.
위치 hold는 Torque Disable이 아니며, 종료 후 중력 하중 지지는 별도 하드웨어 정책이다.
두 모드를 전환할 때는 중립 → Options로 drive → Ctrl+C → 기존 런치 프로세스 종료 확인 →
`Resku_arm` 또는 `Resku_arm_sep`를 실행한다. 기존 실기 런치의 초기 복귀 동작에 주의한다.
사전 검사는 같은 ROS 도메인의 발견 가능한 노드와 신규 런치 잠금에 한정되며, 다른 도메인에서
동일 장치를 열거나 사전 검사 후 다른 구형 런치를 시작하는 경합까지 막는 전역 장치 잠금은 아니다.

| 보호 항목 | 기존 Servo 경로 | 신규 경로 |
|---|---|---|
| 모델 기반 충돌 검사 | Servo 설정 check_collisions=true | 제거됨; 별도 충돌 검사 없음 |
| 특이점 | Servo singularity 임계값 + 기존 DLS | RMD 병진 DLS 감쇠만 유지; Servo와 동등하지 않음 |
| 한계·입력 timeout | 기존 노드/Servo/HI | 노드 속도·위치·lead 제한, 입력/피드백 watchdog, 기존 HI |
| 추가 차단 | 기존 경로 정책 | 엄격한 controller handshake/상태 watchdog, 잘못된 Joy/JointState 차단, 중립 재개, hardware fault 래치 |
| 실기 미해결 위험 | 실제 하중·통신·튜닝에 의존 | 충돌 무보호, 고정 피드백 검출 한계, 종료 패킷 전달, 실제 DXL 추종·방향·전류/프로파일 미검증 |

### 검증 기록

아래 ROS mock 통합 기록은 **초기 복귀 추가 전 버전**의 결과다. 초기 복귀 수정 후에는
사용자의 요청에 따라 실기·가상 하드웨어 구동 검증을 수행하지 않았다. 코드 단위 검사와
빌드만 실시하며 실제 초기 복귀 동작은 사용자가 직접 확인한다.

| 검사 | 결과 | 범위 |
|---|---|---|
| colcon build, 패키지 설치 | 성공 | tool_manipulator_bringup |
| 기존 협조·독립 회귀 | 성공 | 기존 unit test 22개, 기존 제어 소스와 런치 변경 없음 |
| 신규 격리·제한·고장/활성화 검사 | 성공 | 최초 버전 전체 43개; 초기 복귀 추가 후 결과는 아래 참고 |
| DXL ID1/2/3 대응 관절 양방향·독립 누적·유지 | 성공 | 실제 ROS ForwardCommandController + GenericSystem mock |
| 실제 ResKU URDF RMD 숙이기/반대 입력·DXL 불변·동시 입력 | 성공 | 실제 ROS mock 통합 |
| 특이점·RMD 공통 경계·속도 제한 | 성공 | 기존 수치 시험 + 신규 벡터 제한 시험; 실기 충돌 안전성 검증 아님 |
| 입력 단절·중립 재개·controller 비활성화 차단 | 성공 | 실제 ROS mock 통합 |
| EE와 팔 목표 분리 | 성공 | 실제 ROS mock에서 EE 입력 후 6축 목표 불변 |
| 중복 스택 사전 검사 | 성공 | 실행 중 controller_manager/manual_servo_out 탐지, exit 1 |
| 50Hz 갱신 | 성공 | 최초 통합 300개 진단 sample, dt 중앙값 .019998초 |
| 초기 복귀 추가 전 코드 모의 재검사 | 성공 | 도메인 174에서 동일 통합 항목 재통과, 282개 sample, 중앙값 .019997초; 시험 프로세스 정리 확인 |
| 초기 적재 자세 복귀 및 시작 차단 진단 | 성공 | 실측 위치를 이용한 코드 단위 재현; RMD 2° 안쪽 목표·속도 제한·DXL 불변·조이스틱과 독립적인 초기 복귀·수동 중립 gate·timeout·재복귀 금지; 전체 58개 통과 |
| 초기 복귀 변경 후 ROS mock/실기 구동 | 미검증 | 사용자 요청으로 제외; 실제 장치 명령 발행/컨트롤러 전환 없음 |
| 조이스틱 선택 구동·연결 메시지 | 성공 | 연결·재연결 한 줄 출력, 미연결/해제 로그 필터, 출력 분할 처리 코드 검사; 전체 63개 통과. 장치 구동 검증 제외 |
| 기존 모드 전체 런치/실기 회귀 | 미검증 | unit 회귀와 별도; 기존 모드를 실제로 구동하지 않음 |
| 재시작 초기 목표 연속성·설정 순서 불일치·활성화 실패·피드백 유실·추종 오차 | 성공 | 서비스/피드백 모의 주입 unit 검사; 실제 단선/전원 주기 미시험 |
| DXL Goal/Present raw, 실제 모터 양방향 동작/증상 원인 | 미검증 | 실물 미구동 |

최초 sandbox 내 ROS 실행은 DDS socket 권한 오류와 joy_node 종료로 완료하지 못했다.
로컬 DDS를 허용한 후 별도 ROS_DOMAIN_ID=173, ROS_LOCALHOST_ONLY=1에서 mock 통합을
수행했다. 실패를 정상 동작으로 집계하지 않았다.

```bash
source /opt/ros/humble/setup.bash
source /home/sw/ResKU/install/setup.bash
python3 -m pytest src/arm/tool_manipulator_bringup/test/test_manual_controller.py \
  src/arm/tool_manipulator_bringup/test/test_manual_controller_sep.py \
  src/arm/tool_manipulator_bringup/test/test_manual_servo_out.py -q

# 터미널 1: 전용 mock 도메인
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ros2 launch tool_manipulator_bringup manual_servo_out.launch.py
# 터미널 2: URDF가 GenericSystem인지 먼저 확인한 뒤 모의 Joy 발행
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 \
  src/arm/tool_manipulator_bringup/test/manual_servo_out_mock_smoke.py
```

통합 스크립트는 마지막에 컨트롤러 비활성화를 주입하므로 완료 후 런치를 종료한다.

### 비교 진단

`/manual_servo_out/diagnostics`는 50Hz JSON String으로 원본 Joy axes/buttons,
관절명·목표·피드백·오차, 실제 timer dt, focus/fault/phase, controller 상태를 발행한다.
상태 시각은 monotonic 초이며 rosbag 수신 시각으로 ROS 기록과 대응시킨다.
`/manual_position_controller/commands`는 6축 ForwardCommandController 입력이며,
실제 HI 진입 값은 기존 DXL `DIAG_WRITE_ENTRY`를 함께 비교한다. 토픽 명령을 raw 모터
Goal Position으로 간주하지 않는다.

```bash
# 현재 실행 중인 경로에 맞춰 기록; 두 런치를 동시에 실행하지 않는다.
ros2 bag record -o /tmp/resku_servox_bag /joy /joint_states \
  /manual_position_controller/commands /manual_servo_out/diagnostics \
  /ee_velocity_controller/commands /control/active_target /control/hardware_fault

# Servo 비교 기록
ros2 bag record -o /tmp/resku_servo_bag /joy /joint_states \
  /servo_node/delta_joint_cmds /arm_controller/joint_trajectory \
  /arm_controller/state /servo_node/status /control/active_target /control/hardware_fault
```

실기에서는 기존 hardware.yaml의 position_diagnostics=true를 통해 launch 로그의
`DIAG_POSITION`(ID, ros_cmd_rad, trans_cmd_rad, sent_raw, goal_raw, present_raw,
hardware_state_rad, torque, hw_error, current/pwm)과 `DIAG_WRITE_ENTRY`를 보존한다.
장치에 추가 raw 읽기를 직접 실행하여 버스 소유권을 중복시키지 않는다. 기존 진단은 추가
레지스터 읽기 부하가 있으며 servo/비servo 비교 시 동일 설정과 짧은 감독 입력을 사용한다.

ID1 L1/R1, ID2 D-pad 좌우, ID3 왼쪽 스틱 좌우를 각각 같은 시간·양방향으로 비교한다.
원본 입력 → 목표 누적량/주기 → JTC/Forward controller 출력 → HI 진입 → sent_raw →
실제 Goal register → Present register → 보정된 관절 피드백 순서로 단계를 구분한다.
ID1/2 무반응, ID3 한쪽 저속/반대쪽 무반응 증상이 어느 단계에서 나타나는지 기록한다.
비Servo 경로가 개선되어도 Servo 자체가 원인이라고 단정할 수 없으며 JTC 보간,
위치 목표 누적, 주기, 오프셋/방향, raw 전송과 실제 추종의 차이를 따로 분석한다.
