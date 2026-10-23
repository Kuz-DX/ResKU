# RMD 협조 수동 조작

`manual_controller.py`는 dolbotZ의 MANUAL_EE 미분 IK를 참고한다. 현재 ResKU
URDF와 관절 피드백으로 shoulder/elbow/wrist_pitch 3축의 이동량을 매번 계산한다.
고정 회전 비율, 6축 TCP 자세 제어, 장애물 우회 경로계획은 아니다. 그리퍼 방향은
유지하지 않는다. 베이스와 손목 롤/요 입력에 따른 끝점 이동을 RMD가 자동 보상하지 않는다.

| 입력 | 기능 |
|---|---|
| Options (버튼 9) | drive/arm 전환; 시작 시 drive로 비활성 |
| 오른쪽 스틱 상하 (축 4) | RMD 3축 협조 숙이기/젖히기; 기존 대비 방향 반전 |
| 왼쪽 스틱 좌우 (축 0, 부호 반전) | 손목 요 속도형 조작; 입력을 제한된 위치 목표로 누적 |
| L1/R1 (버튼 4/5) | 베이스 직접 회전 |
| D-pad 좌우 (축 6) | 손목 롤 좌우 조작 |

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

# 실기: hardware.yaml 검사 → 기존 3도 초기 복귀 → controller 전환 → 조작 허용
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
손목 요는 왼쪽 스틱 좌우 입력을 각속도로 해석하고 축 부호를 반전한다.
협조 모드는 Servo에 6축 속도 조그를 보내며 입력이 끊기면 0 속도를 보낸다.
독립 모드는 속도를 위치 목표에 누적하고, 스틱을 놓으면 마지막 목표 위치를 유지한다.
Dynamixel은 기존 Position Mode를 유지한다.
손목 롤은 D-pad 좌우로 조작한다.
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
