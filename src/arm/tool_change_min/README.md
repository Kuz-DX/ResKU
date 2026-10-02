# 도구 0·1 장착

명령은 모두 **작업공간 루트 `ResKU/`에서 실행**한다. 파일 경로는 이 위치를
기준으로 한 상대경로다. ROS 토픽·액션 이름의 `/`는 파일 경로가 아니다.

## 조이스틱 매니퓰레이터 수동 조작

측정 자세 `d`와 `llllllllllllllllllllllllllllll`을 허용하도록 현재 raw/영점
변환 기준으로 shoulder 상한을 `1.64 rad`, elbow 상한을 `1.65 rad`로
조정했다. `hardware.yaml`과 URDF에 동일하게 적용했으며 두 자세는
`0.01 rad` 여유를 둔 위치 검사도 통과한다. 전체 리밋 해제가 아니며
전류·속도 보호는 유지한다. bridge의 출력 rad는 영점이 다를 수 있으므로
위치 검증에는 raw 값을 현재 설정으로 변환해 사용한다.

실행 중인 노드에는 자동 반영되지 않는다. 팔을 지지하고 기존 제어기를
종료한 뒤 아래 컨트롤러와 수동 노드를 다시 실행한다. 이 작업공간의 설치
URDF는 소스에 연결되어 있지만, 별도 구동 PC에서는 설치 URDF도 같은
리밋인지 확인해야 한다. 위치 검사 통과가 충돌·부하 검증을 의미하지는 않는다.

`joystick_manipulator.py`는 이 패키지의 `ArmKinematics`로 URDF/하드웨어
리밋과 보정 설정을 검증하고 `/arm_controller/follow_joint_trajectory`로
6축 중 한 관절씩 조작한다. MoveIt Servo, IK 서버, `poses.yaml`의 HOME
티칭값은 필요 없다. EE 출력축은 제어하지 않는다.

자동 FSM·motion_executor·다른 teleop·Servo를 종료한 상태에서 사용한다.
이 노드는 컨트롤러의 명령 소유권을 강제로 잠그지 않으므로 다른 명령원과
동시 실행하면 안 된다. 충돌 검사는 제공하지 않는다.

### 공통 환경 설정

각 터미널에서 먼저 환경을 설정한다. 아래 ROS 경로는 현재 작업공간
배치(`/home/kuzdx/ResKU`) 기준이다:

```bash
source ../../../opt/ros/humble/setup.bash
source install/setup.bash
```

### 터미널 1 — 실기 컨트롤러

로봇 PC에서 실행한다. 이미 실행 중이면 생략한다.

```bash
ros2 launch src/arm/tool_manipulator_bringup/launch/real_control.launch.py \
  hardware_config:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

### 터미널 2 — 조이스틱 발행기

조이스틱이 연결된 PC에서 실행한다. 이미 `/joy`를 발행하면 생략한다.

```bash
ros2 launch src/arm/tool_manipulator_bringup/launch/remote_joy.launch.py
```

### 터미널 3 — 매니퓰레이터 수동 조작 노드

ROS 환경과 기존 description/bringup 의존성이 준비된 로봇 PC에서
소스를 직접 실행한다. 이 방식은 새 노드 설치나 생성 서비스를
요구하지 않으며 ROS 패키지 빌드를 수행하지 않는다:

```bash
PYTHONPATH="src/arm/tool_change_min${PYTHONPATH:+:$PYTHONPATH}" \
python3 src/arm/tool_change_min/scripts/joystick_manipulator.py --ros-args \
  --params-file src/arm/tool_change_min/config/joystick.yaml \
  -p hardware_yaml:=src/arm/tool_manipulator_bringup/config/hardware.yaml \
  -p urdf_xacro:=src/arm/tool_manipulator_description/urdf/tool_manipulator.urdf.xacro
```

### 설치된 노드의 launch 실행

새 노드가 설치되어 있는 환경에서는 터미널 3 명령 대신 다음을 실행한다:

```bash
ros2 launch src/arm/tool_change_min/launch/joystick.launch.py \
  params_file:=src/arm/tool_change_min/config/joystick.yaml \
  hardware_yaml:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

이 launch도 실행 파일은 설치 트리에서 찾는다. 터미널 3의 직접 실행과
동시에 사용하지 않는다.

### 조이스틱 조작 방법

| 조작 | 기본 동작 |
| --- | --- |
| Options (`buttons[9]`) | `/control/active_target`의 drive ↔ arm 전환, 시작은 drive |
| L1 해제 + D-pad 좌우 (`axes[6]`) | base → shoulder → elbow → wrist_pitch → wrist_roll → wrist_yaw 순서 선택 |
| L1 (`buttons[4]`) + 오른쪽 스틱 상하 (`axes[4]`) | 선택 관절의 상대 이동, 기본 축 반전 적용 |
| L1 해제 또는 스틱 중립 | 진행 중인 이동 취소 요청 |

팔 포커스 진입 및 연결 복구 후에는 L1을 한 번 놓아야 이동할 수 있다.
선택 관절은 로그로 표시한다. 패드별 축/버튼 번호, 방향, 속도는
`config/joystick.yaml`에서 변경한다. 외부 포커스 관리자를 사용할 때는
`manage_focus: false`로 설정하고 해당 관리자가 `arm`을 발행하도록 한다.

기본 최대 명령 속도는 0.08 rad/s(하드웨어 속도 제한으로 추가 제한),
이동 구간은 0.2초다. 측정값을 기준으로 한 짧은 위치 궤적이므로 연속 Servo보다
움직임이 끊길 수 있다. 새 목표는 이전 결과 이후 수신한 완전한 6축 피드백에서만
생성한다. Joy 0.25초/피드백 0.5초 초과, 잘못된 피드백, 포커스 해제 시 취소한다.
거부·액션 실패·응답 시간 초과는 재시작 전까지 명령을 차단한다.
취소 요청이나 프로세스 종료는 물리적 정지를 보장하지 않는다. 통신 단절 시
이미 보낸 짧은 구간이 끝까지 실행될 수 있다. 먼저 mock 제어기에서 축 방향과
버튼 매핑을 확인한다.

## stand 이동의 path tolerance 오류 확인

`bash utils/move_to_pose.sh stand --duration 20.0`은 별도 스크립트
`move_to_named_pose.py`를 실행한다. 이 명령의 `stand` 목표는 SRDF의 6축
0 rad이며, FSM의 `poses.yaml` 이동 시간과는 별개다. 영점 설정 명령은
현재 자세를 URDF의 0 rad로 간주해 보정을 저장하므로, 이동 실패를 없애기
위해 임의 자세에서 `set_current_zero.py --apply`를 실행하지 않는다.

`status=6, error_code=-4`는 이동 중 경로 추종 오차로 중단됐다는 뜻이다.
현재 소스의 컨트롤러 설정은 각 관절의 이동 중 허용 오차가 0.05 rad이다.
위치 리밋 확대와는 다른 설정이며, 전류 보호·부하·피드백 이상 등 원인을
확인하기 전 허용 오차를 해제하지 않는다.

이동 명령 없이 실행 중인 설정과 피드백을 조회한다:

```bash
ros2 param dump /arm_controller
ros2 topic echo /arm_controller/controller_state --once
ros2 topic echo /joint_states --once
```

`move_to_named_pose.py`는 시작 관절값과 요청 시간을 출력하며, 실패 시
액션 피드백에서 관측한 관절별 최대 위치 오차와 당시 목표·실제 위치를
큰 오차 순서로 출력한다. 각 관절의 최대값은 서로 다른 시각일 수 있고,
피드백 주기 때문에 실제 중단 순간은 놓칠 수 있다. 해당 출력과 같은
시각의 `ros2_control_node` 전류 보호/통신 오류 로그를 함께 확인한다.
이미 중단된 실행의 원인은 이후 정지 상태 피드백만으로 확정할 수 없다.
이 진단 변경은 소스 직접 실행 wrapper에 적용되며 빌드는 필요 없다.

## 기록한 raw 자세 검증

7개 자세의 raw·출력 rad·RMD encoder 기록을
`config/measured_raw_poses.yaml`에 보존했다. `too1_pre`는 `tool1_pre`로 정규화했다.
bridge 출력의 rad는 bridge 자체 영점을 사용하므로 현재 `hardware.yaml`의
관절값으로 복사하면 안 된다. 기존 `poses.yaml`의 named_poses는 이 출력값을
담고 있어 현재 보정 기준으로 재검증이 필요하다.

아래 경로 예시는 모두 작업공간 루트에서 실행한다. 다음 명령은 장치를 열거나
이동 명령을 보내지 않고 현재 보정값으로 변환한 7축 값과 raw/rad 한계 초과 항목을 출력한다:

```bash
python3 src/arm/tool_change_min/scripts/convert_raw_poses.py
```

현재 기록은 모든 자세의 wrist_yaw raw가 허용 범위 `[-1887, 2201]` 밖이다.
HOME의 shoulder/elbow, 일부 자세의 elbow/wrist_roll도 변환 후 한계를 초과한다.
EE raw `305`, `281`도 허용 범위 `[459, 3843]` 밖이다. 오류 시 종료코드는 2이며
적용용 파일을 생성하지 않는다. 이 값들을 맞추기 위해 자동으로 각도를 wrap하거나
영점·리밋을 변경하지 않는다. 실제 영점 기준과 허용 범위를 확인한 뒤 재티칭한다.

확인된 raw 기록으로 갱신한 후 적용 후보 파일 생성:

```bash
source ../../../opt/ros/humble/setup.bash
python3 src/arm/tool_change_min/scripts/convert_raw_poses.py \
  --output ./tool_change_poses_candidate.yaml
```

검사 통과 시 6축 named_poses와 도킹 raw 기반 FK 목표를 생성한다. 기존 파일은
덮어쓰지 않는다. EE 측정값은 검증·보존하되 6축 팔 자세에 넣지 않는다.
`tool_lock`은 측정 차이의 참고값이며 FSM의 +90° 명령은 그대로다.
EE_ALIGN 시간·허용오차·기준과 개발 정지 단계도 자동 변경하지 않는다.
공급 자세는 저장되지만 현재 tool1 FSM에는 공급 이동 단계가 없다.
한계/FK 검사 통과만으로 이동 경로·충돌·실기 구동이 검증되지는 않는다.


## UI tag0·tag1 이동 요청

`tool_change_fsm`은 기존 UI 토픽 `/selected_tool_id`를 구독한다.
0과 1 모두 HOME → DOCKING_WAIT → EE_ALIGN → 선택 도구 PRE → 선택 도구 TARGET
→ LOCK → RETURN_DOCKING_WAIT → RETURN_HOME 순서를 사용한다.
공통 EE 정렬·잠금 설정을 사용하며 선택한 목표는 `poses.yaml`에서 읽는다.
AprilTag 검출이 목표를 자동 갱신하지는 않는다.

- tag0: `named_poses.tool0_pre`, `cartesian_targets.tool0_target`,
  `timeouts_s.tool0_pre`, `timeouts_s.tool0_target`.
- tag1: 기존 `tool1_pre`, `tool1_target` 설정.

현재 tag0의 자세·Cartesian 목표는 미측정이라 null이다. 요청을 받으면 선택한
개발 정지 단계까지 필요한 설정을 먼저 검사하고, 누락 시 이동 없이 HOLD한다.
`stop_after_state: home`이면 두 요청 모두 HOME까지만 실행한다.
기존 단계 이름 `tool1_pre`, `tool1_target`은 공통 정지 지점으로 유지하며,
tag0 요청에서는 각각 TOOL0_PRE, TOOL0_TARGET 뒤에 멈춘다.
실행 중이거나 PAUSED 상태에서는 새 요청을 무시한다.

상태 감시를 별도 터미널에서 켜고, 실기 준비가 끝난 후 tag0 이동을 요청한다:

```bash
ros2 topic pub --once /selected_tool_id std_msgs/msg/Int32 '{data: 0}'
```

## 도킹 전 EE_ALIGN

현재 요청 ID 1 흐름은 HOME → DOCKING_WAIT → EE_ALIGN → TOOL1_PRE →
TOOL1_TARGET → LOCK → RETURN_DOCKING_WAIT → RETURN_HOME → DONE이다.
tag0(그리퍼)도 같은 순서를 사용하며 TOOL0_PRE → TOOL0_TARGET으로 분기한다.
그리퍼 열기/닫기 명령은 이 흐름에 추가하지 않는다.

EE_ALIGN은 motion_executor의 align_ee 서비스를 호출한다. /joint_states의
ee_joint만 읽고 q=sign*(pulse-zero_raw)*2*pi/4096으로 기준 위상을 변환한다.
q_ref+n*pi 후보 중 리밋 안의 가장 가까운 점을 선택한다. 현재 EE도 리밋
안에 있어야 하며 범위 밖이면 복구 이동 없이 HOLD다. 이 검사는 관절 범위
검사이며 충돌 검사가 아니므로 docking_wait 자세의 EE 회전 여유를 확인한다.

poses.yaml에서 다음 네 키를 측정·승인 후 입력한다:
motion.ee_align.reference_raw, motion.ee_align.duration_s,
motion.ee_align.tolerance_rad, timeouts_s.ee_align.
현재는 null이다. development.stop_after_state가 ee_align 이후이면 미입력
키를 로그로 출력하고 종료한다. 기준 raw=0이면 기존 허용 범위 459..3843
안의 후보는 raw=2048 하나이다. raw=0으로 자동 복귀하지 않는다.

기존 URDF 및 raw 리밋에 맞춰 hardware.yaml의 EE rad 리밋만
[-2.351592547829, 2.839398438376]로 정정했다. zero_raw=1992와 raw 범위는
유지한다. 모델은 시작 시 EE raw/rad/URDF 리밋의 일치를 검사한다.

외부 ee_controller가 활성화되어 있어야 한다. 액션 주소는 launch 인자
ee_action (기본 /ee_controller/follow_joint_trajectory)로 변경 가능하다.
EE 궤적에는 ee_joint만 들어가며 팔 컨트롤러에는 새 명령을 보내지 않는다.
액션 성공 후 새 피드백에서 목표 오차가 tolerance_rad 이내인지 확인한다.
결과 timeout에는 취소를 요청하고 모션을 잠근다. 취소 요청은 물리적 정지를
보장하지 않으며 실패 후 재시작 전 실제 정지 여부를 확인해야 한다.

Mock 확인: 외부 mock 스택의 arm_controller와 ee_controller를 활성화하고
EE 초기값을 허용 범위 안에 둔다. 승인된 설정 복사본에서 정렬 기준과 시간,
허용오차를 입력하고 development.stop_after_state=ee_align로 실행한다.
요청 ID 1 후 EE_ALIGN → PAUSED_EE_ALIGN 및 실제 ee_joint 값을 확인한다.
전체 흐름은 stop_after_state=done으로 확인한다. 실패 시험은 mock EE를
범위 밖으로 놓거나 ee_controller를 비활성화하여 수행한다.
EE_ALIGN → HOLD 뒤 TOOL1_PRE나 추가 목표가 발생하지 않아야 한다.
IK·실행기·FSM 3개 노드 launch에 새 노드를 추가하지 않는다.

## 실기 준비 상태와 보정

`tool_manipulator_bringup/config/hardware.yaml`에는 ID, 버스, 영점,
방향, 전류·속도·소프트 리밋이 이미 입력되어 있다. 미입력 상태가 아니다.
Wizard로 설정한 Dynamixel 값은 유지한다. 다만 Wizard의 표시각과 ROS 관절각은
서로 다른 기준이므로 기존 `zero_raw`와 실제 장착 방향의 일치는 확인해야 한다.
파일 검증 통과는 실제 모터 레지스터까지 읽어 확인했다는 뜻은 아니다.

남은 실측값은 이 패키지의 `config/poses.yaml`에 있는 HOME/대기/접근 자세,
`tool1_target` XYZ·방향과 카메라 장착 TF다. 카메라 외부 보정 파일
`tool_manipulator_bringup/config/arm_camera_extrinsics.yaml`은 현재
`enabled: false`이며 0 값은 임시값이다. 실제 보정을 입력한 뒤 활성화한다.

### RMD 영점 맞추기 (운영 실행 전에 별도로 수행)

팔을 지지하고 실기 제어·조이스틱·자동 도킹 노드를 종료한 상태에서 RMD 관절을
URDF의 0 rad 자세에 맞춘다. 임의의 도킹 자세를 영점으로 저장하지 않는다.
아래 보정기는 ROS의 소프트웨어 영점을 저장하며 모터 EEPROM 영점은 쓰지 않는다.
세 축을 동시에 맞추기 어려우면 `--joints`에 한 축만 지정해 차례로 수행한다.

각 터미널 공통 환경 (작업공간 루트에서 실행):

```bash
source ../../../opt/ros/humble/setup.bash
source install/setup.bash
```

`ModuleNotFoundError: No module named 'rmd_sdk'`가 발생하면 위 환경 설정을
**bridge를 실행할 터미널에서** 적용하고, 하드웨어에 연결하지 않는 import 확인을 먼저 한다:

```bash
python3 -c 'from rmd_sdk import rmd_sdk_py; print(rmd_sdk_py.__file__)'
```

계속 실패하면 구동 PC에서 설치 파일과 기존 CMake 설정을 확인한다:

```bash
find install -name '*rmd_sdk*' -print
grep '^PYTHON_BINDINGS:' build/rmd_sdk/CMakeCache.txt
```

`rmd_sdk`는 C++ SDK와 Python 확장 모듈을 제공한다. C++ 라이브러리만
설치되어 있으면 이 Python bridge는 실행할 수 없다. `PYTHON_BINDINGS:BOOL=OFF`이면
기존 설정에서 Python 바인딩을 비활성화한 것이다. `ON`이어도 설치 완료를
보장하지 않으므로 `rmd_sdk_py*.so`의 존재와 위 import 결과를 함께 확인한다.
소스 경로만 `PYTHONPATH`에 추가해도 누락된 확장 모듈은 해결되지 않는다.
여기서는 ROS 패키지 빌드를 실행하지 않는다.

보정 터미널 A — 읽기 전용 raw bridge (운영 컨트롤러와 동시 실행 금지):

```bash
python3 src/arm/rmd_joint_state_bridge/scripts/joint_state_bridge_node.py
```

보정 터미널 B — RMD 3축만 미리보기:

```bash
python3 src/arm/tool_manipulator_bringup/scripts/set_current_zero.py \
  --config src/arm/tool_manipulator_bringup/config/hardware.yaml \
  --joints shoulder_joint elbow_joint wrist_pitch_joint
```

값을 확인한 뒤 같은 자세에서 저장:

```bash
python3 src/arm/tool_manipulator_bringup/scripts/set_current_zero.py \
  --config src/arm/tool_manipulator_bringup/config/hardware.yaml \
  --joints shoulder_joint elbow_joint wrist_pitch_joint --apply
```

`q = sign × (raw_deg × π/180 − q_offset_rad)`이며 현재 raw 각도가 새
`q_offset_rad`가 된다. Dynamixel 설정은 이 명령의 대상에 포함되지 않는다.
원본 백업은 보정기가 생성한다. 기존 bridge 기본 보정값은 `hardware.yaml`과
다르므로 이 단계의 calibrated rad를 티칭값으로 복사하지 말고 **raw 값**을 사용한다.
저장 후 bridge를 종료하고 새 하드웨어 설정으로 컨트롤러를 재시작한다.
영점 변경 시 기존 티칭값과 물리 한계에 대응하는 소프트 리밋/URDF 한계를 재검토한다.

### 힌지 정렬, 태그 크기와 도킹 목표

힌지가 12시/6시인 체결 정렬은 사용자가 지정한 EE 쪽 기준이다.
이를 `wrist_yaw_joint=nπ` 제약으로 임의 변환하지 않는다. 현재 URDF에서는
`wrist_link_yaw → ee_actuator`가 고정 변환이고 `ee_actuator → tcp_link`는
위치·방향이 같은 고정 변환이다. 회전 출력축 `ee_joint`(ID 4)는 TCP 뒤에 있다.
6축 도킹 실행기는 ID 4를 명령하지 않으므로 출력축 정렬이 필요한 경우 접근 전에
맞추고 유지한다. wrist_yaw의 잠금 회전량은 이 정렬과 별도로 다룬다.

AprilTag 검은 외곽 정사각형은 **2 cm × 2 cm**로 확정:
검출기는 `tag_size_cm:=2.0`, 계측기는 `tag_size_m:=0.02`를 사용한다.

EE 도킹점↔태그의 배치가 TCP↔cam_link 배치와 같다는 장착 조건을 사용하되,
숫자 XYZ를 프레임 변환 없이 복사하지 않는다. `T_A_B`가 B 좌표를 A 좌표로
옮기는 변환일 때, 정확히 `T_target_tag = T_tcp_cam_link`인 장착이면
`T_tag_target = inverse(T_tcp_cam_link)`이다. 방향까지 같은지 확인해야 하며,
카메라 검출 좌표는 `cam_link`가 아니라 optical frame이다. 카메라 장착 및
optical TF를 포함해 다음 식으로 확인한다:

```text
T_base_tag    = T_base_optical × T_optical_tag
T_tag_target  = inverse(T_base_tag) × T_base_tcp_at_dock
T_base_target = T_base_tag × T_tag_target
```

도킹 위치의 **capture raw 값으로 FK를 계산해 tool1_target의 XYZ와 방향을
모두 만들 수 있다.** 같은 정지 자세에서 다음 값을 기록한다:

- DXL raw pulse: base(ID 1), wrist_roll(ID 2), wrist_yaw(ID 3).
- RMD raw_deg: shoulder(ID 4), elbow(ID 5), wrist_pitch(ID 6).
- 그때 사용한 `hardware.yaml` 영점·방향. EE 출력축 정렬 확인용 ID 4 DXL raw도 기록한다.

운영 제어기를 종료하고 보정 터미널 A의 읽기 전용 bridge를 실행한 뒤,
다른 터미널에서 아래 명령을 실행하고 정지 자세에서 Enter를 눌러 기록한다:

```bash
python3 src/arm/tool_manipulator_bringup/scripts/capture_arm_raw.py
```

출력 중 `raw_pulse`와 `raw_deg`를 보관한다. bridge의 이전 보정 기준 `rad`는
현재 하드웨어 설정 기준 FK에 그대로 넣지 않는다. 캡처가 끝나면 bridge를 종료한다.

DXL은 `q = direction × (raw − zero_raw) × 2π/4096`으로 변환한다
(`raw_increases_ccw=true`이면 direction=+1, 아니면 −1). RMD는 위 보정식을 쓴다.
6축 순서대로 `tool_change_min.kinematics.ArmKinematics.fk(q)`를 계산하면
`base_actuator → tcp_link`의 4×4 변환을 얻는다. 병진은 `position_m`, 회전은
쿼터니언 `[x,y,z,w]`로 바꿔 `orientation_xyzw`에 넣는다. 현재 고정 목표 FSM은
태그 계측 결과를 자동으로 목표에 반영하지 않는다. 기구학 치수·영점 오차도 FK에 남는다.

시험 기본값은 이름 자세 이동 60 s, 잠금 30 s, Cartesian 속도 0.005 m/s,
일반 단계 제한 120 s, 목표 접근 180 s, 잠금 제한 90 s다.
IK 간격은 2 mm/0.02 rad, 관절 한계 여유는 0.01 rad,
피드백 최대 나이는 0.5 s다. 큰 타임아웃이 오래된 피드백을 허용하지는 않는다.
거리와 관절 변화량에 따라 속도·시간을 재검토한다. 후퇴 속도 필드는 예약값이며
현재 FSM의 복귀는 이름 자세 이동 시간을 사용한다. 미측정 자세와 거리는 `null`을 유지한다.

## 실제 구동: 터미널별 실행

먼저 동일한 실기 PC에서 ROS 환경을 source한 뒤, 모터 명령 없는 준비 검사를 실행한다:

```bash
python3 src/arm/tool_change_min/scripts/check_real_readiness.py
```

이 검사는 설치된 노드·생성 서비스, 선택 단계의 티칭값, 장치 경로, 카메라 TF
활성화 설정만 확인한다. 실제 컨트롤러·UI·모터 응답은 아래 실시간 조회로 확인한다.
현재 이 작업 환경에서는 `tool_change_min` 패키지가 설치 목록에 없고,
`/dev/ttyUSB0`, `can_arm`도 없으며 HOME 티칭값이 비어 있다.
`ros2 topic pub`는 이 누락을 해결하지 않는다. ROS 빌드 금지 조건에 따라
여기서는 빌드하지 않았으며, 구동 PC에 설치된 패키지와 생성 서비스가 필요하다.

아래는 노드를 개별 실행하는 방식이다. 각 터미널에서 위 공통 환경을 먼저 설정한다.
보정 bridge는 종료한다. 설치된 ROS 패키지·생성 서비스가 필요하며, 여기서는
빌드하지 않는다. `ros2 run`에 실행 파일이 없으면 설치가 준비되지 않은 상태다.
`changer_manipulator_bringup`에는 현재 실행 가능한 실기 launch가 없으므로
기존 `tool_manipulator_bringup`의 실기 컨트롤러를 사용한다.

### 터미널 1 — 팔 컨트롤러와 로봇 TF

```bash
ros2 launch tool_manipulator_bringup real_control.launch.py \
  hardware_config:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

이 단계는 실기 하드웨어를 활성화한다. 보정·리밋 확인을 끝낸 뒤 실행한다.

### 터미널 2 — 카메라 드라이버

```bash
ros2 launch vision cameras.launch.py
```

이 launch는 팔 D435i뿐 아니라 주행 D435i와 좌우 C920도 시작한다.
장치 시리얼·포트는 `vision/camera/launch/cameras.launch.py`의 인자를 따른다.

### 터미널 3 — 카메라 장착 TF (보정 파일 활성화 후)

```bash
python3 src/arm/tool_manipulator_bringup/scripts/arm_camera_extrinsics_broadcaster.py \
  --ros-args --params-file src/arm/tool_manipulator_bringup/config/arm_camera_extrinsics.yaml
```

기존 노드가 이미 `cam_link → arm_camera_link`를 발행한다면 중복 실행하지 않는다.

### 터미널 4 — AprilTag 검출 토픽 발행

검출 송신과 계측 수신의 기본 토픽은 `/arm/apriltag/centers`
(`std_msgs/msg/String`, JSON의 `detections[].id`에 태그 ID)로 동일하다.
현재 tool_change_fsm은 이 토픽을 구독하지 않으며, `/selected_tool_id`를
받아 `poses.yaml`의 목표로 IK 경로를 요청한다. 태그 기반 목표 갱신은 별도 연결이 필요하다.

```bash
ros2 run vision apriltag --ros-args \
  -p tag_size_cm:=2.0 \
  -p centers_topic:=/arm/apriltag/centers
```

위 노드가 실제 검출 결과를 발행한다. 별도 터미널에서 송수신 연결과
JSON의 `detections[].id`를 확인한다:

```bash
ros2 topic info /arm/apriltag/centers --verbose
ros2 topic echo /arm/apriltag/centers std_msgs/msg/String --once
```

태그 ID만 `Int32`로 이 토픽에 발행하지 않는다. 수신기는 검출 시각·프레임·
검출 목록이 포함된 JSON 문자열을 읽는다. 아래 `/selected_tool_id` 발행은
태그 검출 데이터가 아니라 도구 변경 실행 요청이다.

### 터미널 5 — supply

```bash
ros2 run vision supply
```

supply는 공급상자 인식 노드이며 모델 파일과 카메라·TF가 필요하다.
태그 체결점 측정을 대신하지 않는다.

### 터미널 6 — IK

```bash
ros2 run tool_change_min ik_node.py --ros-args \
  -p poses_yaml:=src/arm/tool_change_min/config/poses.yaml \
  -p hardware_yaml:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

### 터미널 7 — 궤적 실행기

```bash
ros2 run tool_change_min motion_executor.py --ros-args \
  -p poses_yaml:=src/arm/tool_change_min/config/poses.yaml \
  -p hardware_yaml:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

### 터미널 8 — 장착 FSM

```bash
ros2 run tool_change_min tool_change_fsm.py --ros-args \
  -p poses_yaml:=src/arm/tool_change_min/config/poses.yaml
```

처음에는 `development.stop_after_state: home`을 유지한다.
현재 HOME 값이 미입력이므로 티칭 전에는 모션 노드 시작이 차단된다.

### 터미널 9 — 상태 확인과 실행 요청

```bash
ros2 control list_controllers
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_actuator arm_camera_color_optical_frame
```

TF 확인 후 Ctrl+C로 조회만 종료한다. 먼저 요청 구독자가 최소 FSM인지 확인한다:

```bash
ros2 topic info /selected_tool_id -v
ros2 topic echo /selected_tool_id
```

UI는 `/selected_tool_id`에 `std_msgs/msg/Int32`를 발행한다.
`data: 0`은 tag0(그리퍼), `data: 1`은 tag1 이동 요청이다. 0은 정지 명령이 아니다.
이 저장소에서는 실제 UI 발행 코드가 확인되지 않았으므로 구동 PC에서 버튼을 눌러
검증한다. 기존 `ui_tag1_docking`(tag1_recorded_path_node)도 `/selected_tool_id`를
구독해 이동하므로 최소 FSM과 함께 실행하지 않는다. 기존 도킹 제어 노드도
동시에 이동 요청을 처리하지 않도록 종료한다. UI 요청이 확인되면 별도 수동 발행은 필요 없다.
요청이 도착했는데 움직이지 않으면 재발행하지 말고 상태와 모션 노드 로그를 확인한다.

별도 터미널에서 상태 감시를 먼저 실행해 둔다:

```bash
ros2 topic echo /tool_change/status
```

UI를 대신해 **실제 동작을 요청할 때만** 다른 터미널에서 다음 명령을 한 번 실행한다.
`data: 1`은 도구 1 실행 요청이며 AprilTag 검출을 발행하는 명령이 아니다:

```bash
ros2 topic pub --once /selected_tool_id std_msgs/msg/Int32 '{data: 1}'
```

`PAUSED_HOME` 확인 후 다음 단계 설정으로 모션 노드를 재시작하며 순차 시험한다.
상태 `HOLD`만으로 물리 정지가 검증됐다고 간주하지 않는다.
실행기는 컨트롤러 결과 대기가 실패하면 해당 목표에 취소를 요청하고 새 동작을
차단한다. 목표 수락이 늦게 도착해도 취소를 요청한다. 취소 요청 자체는 정지
확인이 아니므로 실제 정지와 컨트롤러 상태를 확인한 뒤 재시작한다.
실행 서비스와 액션/관절 피드백은 별도 콜백 그룹을 사용한다.

### 별도 계측 터미널 — 태그와 체결점 관계 검증

자동 동작 요청 없이 팔을 정지시키고 카메라·AprilTag·TF가 준비된 상태에서 실행한다:

```bash
ros2 run tool_change_min measure_apriltag_docking.py --ros-args \
  -p tag_id:=1 -p tag_size_m:=0.02 -p sample_count:=100 \
  -p output_csv:=./tag1_target_offset.csv \
  -p base_frame:=base_actuator -p target_frame:=tcp_link
```

### 통합 실행을 선택할 때

위 터미널 **6·7·8을 종료한 뒤** 다음 launch로 IK·실행기·FSM을 함께 실행한다.
터미널 1의 팔 컨트롤러와 로봇 TF는 별도로 실행한다.
터미널 2·3·4·5의 카메라·장착 TF·AprilTag·supply는 이 launch에 포함되지 않으며,
필요한 경우 별도로 실행한 상태를 유지한다:

```bash
ros2 launch src/arm/tool_change_min/launch/tool1_attach.launch.py \
  poses_yaml:=src/arm/tool_change_min/config/poses.yaml \
  hardware_yaml:=src/arm/tool_manipulator_bringup/config/hardware.yaml
```

소스 launch를 지정해도 노드 실행 파일은 설치 트리에서 찾는다.
IK·실행기·FSM의 개별 실행과 통합 실행을 동시에 사용하지 않는다.

## 구성

`tool1_attach.launch.py`는 다음 세 노드만 시작합니다.
카메라 launch, AprilTag, supply는 실행하지 않습니다.

- `ik_node`: Cartesian 목표의 역기구학 경로를 생성합니다.
- `motion_executor`: 궤적을 검증하고 팔 컨트롤러에 전달합니다.
- `tool_change_fsm`: 도구 장착 단계를 순서대로 진행합니다.

이 launch는 TF, 로봇 상태, MoveIt 또는 컨트롤러 노드를 시작하지 않습니다.
특히 `cam_link -> arm_camera_link` 변환도 발행하지 않습니다.

외부 팔 컨트롤러의 확인된 액션 주소는
`/arm_controller/follow_joint_trajectory`이며, 관절 순서는 다음과 같습니다.

`base_joint, shoulder_joint, elbow_joint, wrist_pitch_joint, wrist_roll_joint, wrist_yaw_joint`

컨트롤러 설정은 `changer_manipulator_moveit_config`에 기록되어 있고,
외부 컨트롤러의 역할은 `changer_manipulator_bringup`에 문서화되어 있습니다.

## 안전한 설정

`config/poses.yaml`에는 측정하지 않은 값을 임의로 넣지 않습니다. FSM은 세
개의 관절 티칭 자세(`home`, `docking_wait`, `tool1_pre`)와
`base_actuator` 기준 XYZ·쿼터니언으로 표현한 고정 Cartesian 목표
`cartesian_targets.tool1_target`을 사용합니다. 이 목표는 이름 기반 관절
자세가 아닙니다.

IK 노드는 현재 `/joint_states` 전체를 읽어 여러 점으로 된 Cartesian 경로를
계산합니다. 위치는 직선으로 이동하고 방향은 SO(3)에서 가장 짧은 회전을
따릅니다. 모든 경유점은 IK와 관절 제한 검사를 통과해야 합니다. 경유점 수는
다음 두 값을 계산해 더 큰 쪽으로 정합니다.

- 실제 위치 거리 / `ik_step_m`
- 방향 회전각 / `ik_orientation_step_rad`

기본 설정 `development.stop_after_state: home`은 HOME 자세, 이름 자세 이동
시간, 피드백 유효 시간, IK 제한 여유값, HOME 제한 시간만 요구합니다. HOME
동작 후에는 `PAUSED_HOME`을 발행하고 멈춥니다. 동작을 검토한 다음 정지
단계를 `docking_wait`, `tool1_pre`, `tool1_target`, `lock`,
`return_docking_wait`, `return_home` 순서로 진행합니다. `done`에서만 FSM에
필요한 모든 설정값을 요구하고 `DONE`을 발행합니다.

선택된 단계에 필요한 설정값이 비어 있으면 해당 점 표기 키를 로그에 남기고,
새로 시작하는 각 모션 노드는 종료됩니다. 또한 노드는 6축 URDF 제한과
`tool_manipulator_bringup/config/hardware.yaml`의 제한을 비교합니다. 두 값이
다르면 임의 변환을 시도하지 않고 시작 단계에서 실패합니다.

## 모의 시험

1. 별도 터미널에서 기존 모의 팔·컨트롤러 스택을 실행합니다. 이 스택은 다섯
   노드를 실행하는 launch에 포함되어 있지 않습니다.

   ```bash
   ros2 launch tool_manipulator_bringup mock.launch.py
   ```

2. 측정·승인된 모의용 `poses.yaml` 복사본을 준비합니다. 6개 관절 순서를
   유지하고, 도달 가능한 `tool1_pre` 자세를 사용합니다. 승인된
   `base_actuator` 기준 `tool1_target` XYZ·쿼터니언을 입력하고 피드백 유효
   시간, 이동 시간, 제한 시간을 유한한 값으로 지정합니다. 해당 파일로 다섯
   노드를 실행합니다.

   ```bash
    ros2 launch tool_change_min tool1_attach.launch.py poses_yaml:=./poses_mock.yaml
   ```

3. 도구 1 장착을 요청하고 `/tool_change/status`가 아래 순서대로 바뀌는지
   확인합니다.

   `HOME`, `DOCKING_WAIT`, `TOOL1_PRE`, `TOOL1_TARGET`, `LOCK`,
   `RETURN_DOCKING_WAIT`, `RETURN_HOME`, `DONE`

   ```bash
   ros2 topic echo /tool_change/status
   ```

   위 상태 감시를 켜둔 뒤 다른 터미널에서 도구 1 실행 요청을 발행합니다:

   ```bash
   ros2 topic pub --once /selected_tool_id std_msgs/msg/Int32 '{data: 1}'
   ```

4. 실패 시 안전 정지 동작을 확인하려면 별도의 모의 설정 복사본에서 목표
   쿼터니언을 `[0, 0, 0, 0]`처럼 유효하지 않은 값으로 바꿔 다시 요청합니다.
   최종 상태는 `HOLD`여야 하며, 후퇴 동작이나 재시도 명령이 발생하면 안 됩니다.
   다음 시험 전에는 유효한 설정 파일로 되돌립니다.

이 절차는 모의 환경 전용입니다. 실물 로봇에 확인되지 않은 값을 사용하지
마십시오.

## AprilTag 도킹 계측

이 읽기 전용 계측 유틸리티는 3개 운영 노드 launch에 포함하지 않습니다.
기존 카메라와 AprilTag 노드를 먼저 실행하고 팔을 정지시킨 뒤 아래처럼
실행합니다.

    ros2 run tool_change_min measure_apriltag_docking.py --ros-args \
      -p tag_id:=1 \
      -p tag_size_m:=<실측한-검은-정사각형-한변-m> \
      -p sample_count:=<승인한-프레임수> \
      -p output_csv:=./tag1_stationary.csv

프레임별 CSV와 `tag1_stationary.summary.json`이 생성됩니다. 요약 파일에는
기존 검출기의 XYZ와 보정 코너·CameraInfo 왜곡계수로 다시 푼 PnP XYZ의 비교,
검출률, XYZ 흔들림, 재투영 RMSE, 촬영 timestamp부터 결과 수신까지 지연이
들어갑니다. `tag_size_m`은 출력한 검은 정사각형 외곽 한 변을 직접 재서
입력하고, 기존 AprilTag 노드의 `tag_size_cm`에도 같은 길이를 사용해야 합니다.

태그에서 체결 목표까지의 고정 관계를 재려면 팔을 검토된 실제 체결 목표에
수동으로 정지시키고 실제 목표 TF 프레임을 추가합니다.

    ros2 run tool_change_min measure_apriltag_docking.py --ros-args \
      -p tag_id:=1 \
      -p tag_size_m:=<실측한-검은-정사각형-한변-m> \
      -p sample_count:=<승인한-프레임수> \
      -p output_csv:=./tag1_target_offset.csv \
      -p base_frame:=base_actuator \
      -p target_frame:=tcp_link

이 모드는 외부에서 관리되는 TF를 읽기만 합니다. 카메라 TF를 발행하거나 팔을
움직이지 않습니다. 하나의 정지 데이터는 반복 정밀도만 나타내므로, 절대
정확도 평가는 실측 기준 좌표와 여러 거리·시야각의 데이터로 별도 수행합니다.

## 손목 yaw DYNAMIXEL Wizard 계측

계측 결과를 검토하기 전까지 `0.75 rad/s` 제한값을 수정하지 마십시오.
DYNAMIXEL Wizard 2.0에서 ID 3 손목 yaw MX-106T를 선택하고 Protocol 2.0인지
확인합니다. 계측 전후 스크린샷 또는 내보낸 표에 다음 항목을 기록합니다.

- 모델 번호, 펌웨어 버전, ID, 통신 속도(baud rate), 프로토콜 종류
- 구동 모드, 동작 모드, 홈 오프셋, 토크 활성화 상태
- 속도 제한, 프로파일 가속도, 프로파일 속도
- 최소·최대 위치 제한, 전류 제한
- 입력 전압, 온도, 하드웨어 오류 상태

팔의 부하를 제거하고 기계적으로 간섭이 없는 상태에서 다음 항목을 그래프로
기록합니다.

`Realtime Tick`, `Goal Position`, `Present Position`, `Velocity Trajectory`,
`Present Velocity`, `Present Current`, `Present PWM`, `Input Voltage`,
`Temperature`, `Hardware Error Status`

작은 양·음 방향 손목 yaw 이동을 각각 최소 세 번 수행하고, 그래프를 매번
CSV로 내보냅니다. 명령한 위치 변화량과 실제 시작·종료 위치도 기록합니다.
무부하 시험을 승인한 뒤에만 실제 도구 부하를 걸어 반복합니다.

`hardware.yaml`에 필요한 값은 검증된 원시 속도 제한, 원시 프로파일 가속도와
속도, 측정된 최대·정상 `Present Velocity`, 이동 시간, 회전 방향, 전류, 전압,
온도 및 오류 비트입니다. 이 원시 값과 관측된 SI 단위 속도가 일치하는지
확인한 뒤에만 소프트웨어의 rad/s 제한을 변경합니다.
