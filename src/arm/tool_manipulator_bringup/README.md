# Tool Manipulator Bringup

팔의 실기·모의 구동, 조이스틱 수동 구동, 설정 점검과 캘리브레이션 도구만 모아 둔
패키지다. 미완성 tool-change, AprilTag 도킹, 기록 경로 자동 재생, 설계 실험용 코드는
제거했다.

## 남긴 실행 경로

| 목적 | 실행 파일 | 모터 명령 | 시각화 |
| --- | --- | --- | --- |
| 조이스틱 수동 구동 | `manual.launch.py` | O | 선택 |
| IK/Manual 통합 모드 | `ik_manual.launch.py` | O | X |
| 실기 controller + MoveIt | `real_control.launch.py` | O | X |
| mock MoveIt/RViz | `mock.launch.py` | mock만 | O |
| 원격 PC의 `/joy` 발행 | `remote_joy.launch.py` | X | X |
| 주행 가능 자세 감시 | `arm_drive_guard.launch.py` | X | X |

`manual`은 오늘 pull 받은 Servo 없는 수동 제어 구현이다. 세부 버튼 매핑, 시작 복귀,
안전 조건과 rosbag 진단 방법은 [manual_controller.md](docs/manual_controller.md)를 따른다.

## 제어 모드 기반 구조

`arm_bringup_common.py`는 독립 실행 launch가 아니라 모든 팔 제어 모드가 공유하는
내부 구성 모듈이다. 다음 항목을 한 경로에서 생성한다.

- 동일한 URDF/xacro mapping과 `robot_description`
- 실기 `hardware.yaml` preflight
- 단일 `ros2_control_node`와 `robot_state_publisher`
- 공통 controller 설정과 controller spawner

현재 선택 가능한 모드는 다음처럼 실행한다.

| 모드 | 실행 명령 | 활성 팔 controller |
| --- | --- | --- |
| Manual | `manual.launch.py` | `manual_position_controller` |
| IK/Manual managed | `ik_manual.launch.py` | `manual_position_controller` |
| trajectory/MoveIt | `real_control.launch.py` | `arm_controller` |

세 launch는 같은 공통 base를 사용한다. `ik_manual.launch.py` 안에서는 Mode Manager가
MANUAL/TCP_IK/TOOL_AUTO를 중재하지만, 다른 launch와의 동시 실행은 지원하지 않는다.
반드시 기존 팔 launch를 완전히 종료한 뒤 다른 모드를 실행한다.

```bash
ros2 launch tool_manipulator_bringup ik_manual.launch.py
```

## 수동 구동

### mock, 시각화 없음

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup manual.launch.py
```

### mock, RViz 포함

```bash
ros2 launch tool_manipulator_bringup manual.launch.py launch_rviz:=true
```

### 실기, 시각화 없음

```bash
ros2 launch tool_manipulator_bringup manual.launch.py \
  real_hardware:=true \
  hardware_config:=$HOME/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml
```

실기에서는 `hardware.yaml` 사전 검사를 통과해야 하며, 시작 시 RMD 3축을 각 소프트
리밋 안쪽 2도로 저속 복귀한다. 수동 경로는 충돌 검사를 하지 않는다. 다른 hardware
launch나 읽기 전용 bridge와 동시에 실행하지 않는다.

로봇이 아닌 조이스틱 연결 PC에서 `/joy`만 발행하려면 두 PC의 ROS domain을 맞춘 뒤
다음을 실행한다. 이때 로봇 PC의 `manual.launch.py`에는 `launch_joy:=false`를 준다.

```bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0
ros2 launch tool_manipulator_bringup manual.launch.py \
  real_hardware:=true launch_joy:=false
```

## MoveIt 구동

### mock + RViz

```bash
# tool_id: 0=gripper, 1=drill, 2=Phillips bit, 99=no tool
ros2 launch tool_manipulator_bringup mock.launch.py tool_id:=0 use_mesh:=true
```

### 실기, RViz 없음

```bash
ros2 launch tool_manipulator_bringup real_control.launch.py \
  hardware_config:=$HOME/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml \
  tool_id:=0 use_mesh:=true
```

기본값은 controller 활성화 뒤 SRDF `home`으로 이동한다. 자동 이동을 막으려면
`move_home_on_start:=false`, 최소 이동 시간을 바꾸려면 `home_duration:=10.0`을 추가한다.
이 시작 이동은 joint trajectory이며 MoveIt 충돌 경로 계획은 수행하지 않는다.

실기 MoveIt에 RViz가 필요하면 위 launch를 실행한 상태에서 별도 터미널에 다음을 실행한다.

```bash
rviz2 -d $(ros2 pkg prefix tool_manipulator_moveit_config)/share/\
tool_manipulator_moveit_config/config/moveit.rviz
```

### named pose

팔 내부 MANUAL/TCP_IK/TOOL_AUTO 모드 중재는 별도
[Control Mode Manager](docs/control_mode_manager.md)를 참고한다.
기존 아래 named-pose 직접 실행 명령은 managed stack과 동시에 사용하지 않는다.

```bash
# pose 목록과 dry-run
bash ~/ResKU/utils/move_to_pose.sh --list
bash ~/ResKU/utils/move_to_pose.sh home --dry-run

# controller가 실행 중일 때 실제 이동
bash ~/ResKU/utils/move_to_pose.sh home --duration 8.0
```

현재 `ee_joint`는 velocity mode이므로 `real_control`과 `manual` 모두
`ee_velocity_controller`를 사용한다. position trajectory 방식의 구형
`move_gripper.py`는 현재 하드웨어 모드와 맞지 않아 제거했다.

## 출력·확인 도구

이 절의 도구는 별도 언급이 없는 한 모터 명령을 보내지 않는다.

### 설정 검사

```bash
ros2 run tool_manipulator_bringup check_hardware_config.py
ros2 run tool_manipulator_bringup check_hardware_config.py --show-values
ros2 run tool_manipulator_bringup read_hardware_safety_params.py --dry-run
```

### 현재 관절 위치와 raw 값

실기 controller를 끄고 읽기 전용 bridge만 실행한다.

```bash
# 터미널 A
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: Enter를 누를 때마다 현재 값 출력
ros2 run tool_manipulator_bringup capture_arm_pose.py

# raw encoder/pulse 출력
ros2 run tool_manipulator_bringup capture_arm_raw.py
ros2 run tool_manipulator_bringup capture_rmd_calibration.py
```

### world 좌표와 TF에서 역산한 관절값

이 파일은 의도적으로 패키지 실행 파일로 설치하지 않은 독립 점검 스크립트다.
`robot_state_publisher`와 TF가 실행 중일 때 시작하고, 출력하고 싶은 순간 Enter를 누른다.
각 joint/link와 `cam_link`의 world 좌표를 출력한다.

```bash
python3 ~/ResKU/src/arm/tool_manipulator_bringup/scripts/joint_world_coordinates.py
```

### 현재 위치를 영점으로 저장

이 명령만 `--apply`를 붙이면 `hardware.yaml`을 수정한다. 먼저 미리보기를 확인한다.

```bash
ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml

ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml --apply
```

## 유지 파일 기준

`trajectory_diagnostics.py`와 관련 문서·테스트는 DXL 수동 구동 진단
기록이므로 실행 진입점은 아니어도 분석 자료로 유지한다.

- `launch/`: `manual`, `ik_manual`, `real_control`, `mock`, `remote_joy`, `arm_drive_guard`
- `scripts/`: 실제 구동에 필요한 모듈과 읽기/검사/캘리브레이션 도구
- `config/`: `hardware`, `manual_controllers`, `servo` 안전 margin, RViz 설정
- `test/`: manual, named pose, 안전·영점·trajectory와 URDF 정적 검사

과거 영점 기준의 `d/l` raw 상수를 현재 `hardware.yaml`로 변환하던 테스트는 제거했다.
현재 기준 검증값이 필요하면 실제 자세를 다시 캡처하여 raw와 `/joint_states`를 함께
기록하고, 당시 사용한 `hardware.yaml`을 명시한다. 기존 hardware 백업은 보정 이력
확인용이므로 실행 설정은 아니지만 유지한다.

## 현재 기준으로 남은 작업

1. `manual.launch.py`가 사용하는 현재 `hardware.yaml` 상태에서 대표 수동 자세를
   다시 캡처한다. 같은 시각의 raw 값과 `/joint_states` rad 값을 함께 저장한다.
2. 재캡처 값으로 관절 제한·영점 변환 검증을 추가한다. 과거 raw 숫자를 현재 영점에
   그대로 대입하거나 각도를 임의로 wrap하지 않는다.
3. `ik_manual.launch.py real_hardware:=false`에서 시작 복귀, MANUAL/TCP_IK 전환,
   Deadman과 충돌 거부를 Mock 검증한다.
4. 별도 승인 후 실기에서 저속 TCP 이동을 검증한다. `manual.launch.py`와
   `ik_manual.launch.py`는 동시에 실행하지 않는다.
5. Tool Change에 사용할 PRE와 최종 TCP 자세는 위 보정이 확정된 뒤 YAML에 티칭한다.

ROS 패키지를 다시 빌드하기 전까지 설치 공간에는 삭제한 예전 실행 파일이 남아 있을 수
있다. 소스 기준 진입점은 이 README의 목록만 사용한다.
