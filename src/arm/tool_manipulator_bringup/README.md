# Tool Manipulator Bringup

팔의 실기·모의 구동, 조이스틱 수동 구동, 설정 점검과 캘리브레이션 도구만 모아 둔
패키지다. 미완성 tool-change, AprilTag 도킹, 기록 경로 자동 재생, 설계 실험용 코드는
제거했다.

## 남긴 실행 경로

| 목적 | 실행 파일 | 모터 명령 | 시각화 |
| --- | --- | --- | --- |
| 조이스틱 수동 구동 | `manual.launch.py` | O | 선택 |
| 실기 controller + MoveIt | `real_control.launch.py` | O | X |
| mock MoveIt/RViz | `mock.launch.py` | mock만 | O |
| 원격 PC의 `/joy` 발행 | `remote_joy.launch.py` | X | X |
| 주행 가능 자세 감시 | `arm_drive_guard.launch.py` | X | X |

`manual`은 오늘 pull 받은 Servo 없는 수동 제어 구현이다. 세부 버튼 매핑, 시작 복귀,
안전 조건과 rosbag 진단 방법은 [manual_controller.md](docs/manual_controller.md)를 따른다.

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

### 수동 경로·trajectory 진단

`docking_motion_recorder.py`는 이름만 기존 호환성을 유지하며, 실제로는 `/joint_states`를
구독해 JSON으로 저장하는 읽기 전용 관절 경로 기록기다.

```bash
ros2 run tool_manipulator_bringup docking_motion_recorder.py \
  --output /tmp/arm_motion.json
```

`trajectory_diagnostics.py`는 과거 Servo/JTC 로그를 해석할 때 재사용하는 수동 진단
모듈이며 독립 실행 노드는 아니다. 기존 로그 필드의 의미는
[dxl_trajectory_diagnostics.md](docs/dxl_trajectory_diagnostics.md)에 정리돼 있다.

## 유지 파일 기준

- `launch/`: `manual`, `real_control`, `mock`, `remote_joy`, `arm_drive_guard`
- `scripts/`: 실제 구동에 필요한 모듈과 읽기/검사/캘리브레이션 도구
- `config/`: `hardware`, `manual_controllers`, `servo` 안전 margin, RViz 설정
- `test/`: manual, named pose, 안전·영점·trajectory와 URDF 정적 검사

ROS 패키지를 다시 빌드하기 전까지 설치 공간에는 삭제한 예전 실행 파일이 남아 있을 수
있다. 소스 기준 진입점은 이 README의 목록만 사용한다.
