# Tool Manipulator Bringup

**Tag 1 측정 경로 자동실행:** [연결·사전 검사·실행 절차](docs/tag1_recorded_replay.md).
`ui_tag1_docking.launch.py`와 기존 UI 노드 진입점은 검토된 6축 기록 재생기로 연결된다.
현재 측정 파일은 단계·영점·경로 검토가 미완료라 실행을 차단한다.
이 모드를 아래의 시각 서보/조이스틱 절차와 동시에 실행하지 않는다.

실기 팔 제어와 **attach-only** AprilTag tool docking 패키지다. 실제 실행 순서와 Jetson/로컬 토픽 표는 저장소 루트의 [`howtorun.md`](../../../../howtorun.md) `Tool Manipulator — 실기 제어·툴 교체 실행 절차`를 따른다.

- URDF `world`는 차량 IMU 중심이며, `arm_world`는 IMU/world 기준 `(0.1921, 0, 0.235)` m offset이며 지면 기준 높이는 `0.350` m이다. 차체, 회로박스, 대칭 tool case는 fixed visual/collision links로 표시된다.
- `config/hardware.yaml`: U2D2 TTL/RS-485와 RMD CAN, 모든 관절 영점·리밋·register 설정의 단일 원본
- `config/tools.yaml`: tool/fixture collision, TCP, tag, 접근·yaw·직선 motion 설정의 단일 원본
- `config/docking.yaml`: existing vision AprilTag 입력, tag/depth gate, XY servo, 공통 pose의 단일 원본
- `config_validator.py`: 세 원본의 모든 필수 필드를 검사한다. 실패하면 `tool_change.launch.py`는 hardware를 시작하지 않는다.
- `real_control.launch.py`: 검사 완료한 `hardware.yaml`으로 real `ros2_control`, controller, MoveIt을 기동한다.
- `tool_change.launch.py`: real control + AprilTag vision + filter + XY servo + dock action + planning scene stack을 기동한다.
- `arm_tag_docking_vision.launch.py`: 수정하지 않은 `vision.apriltag`를 `docking.yaml`의 단일 입력 설정으로 실행한다.

`real_control.launch.py`는 기본적으로 `arm_controller` 활성화 성공 후 SRDF의 `home`으로
한 번 자동 이동한다. 현재 관절 상태 수신 및 정지·소프트 리밋 검사를 통과한 뒤 8초 동안
이동한다. 기존 `move_to_named_pose.py`의 직접 trajectory 명령을 사용하므로 MoveIt 충돌
경로 계획은 수행하지 않는다. 컨트롤러 활성화나 상태 검사가 실패하면 이동하지 않는다.

```bash
ros2 launch tool_manipulator_bringup real_control.launch.py \
  hardware_config:=src/arm/tool_manipulator_bringup/config/hardware.yaml

# 자동 home 이동 해제: 위 명령에 move_home_on_start:=false 추가
# 이동 시간 변경: 위 명령에 home_duration:=10.0 추가
```

도킹은 `yaw trajectory 성공 → /wrist_yaw_rotation_complete → configured retreat 성공 → /docking_complete → scene attach` 순서다. yaw 성공은 물리 체결이나 scene 부착 검증이 아니다. 별도 체결 센서는 현재 사용하지 않는다.

물리 detach/unlock 절차는 아직 구현되지 않았다. `Dock.mode=1`과 scene detach service는 명시적으로 거부되며 성공으로 보고하지 않는다. 재시작·도킹 실패 뒤 tool 상태가 불명확하면 scene manager는 `UNKNOWN`으로 시작/전환하며, 실물이 비어 있음을 확인한 운영자만 `~/operator_confirm_empty`로 복구할 수 있다.

## 수동 관절값 캡처 (RMD 리밋 측정)

팔을 손으로 움직여 RMD 영점·소프트 리밋을 측정할 때는, 제어 노드 대신 읽기 전용
`rmd_joint_state_bridge`와 `capture_arm_pose.py`를 사용한다. bridge는 RMD CAN과
Dynamixel U2D2 포트에서 현재 엔코더 위치를 읽기만 하며 위치·속도·토크 명령을 보내지 않는다.

```bash
# 터미널 A: 읽기 전용 엔코더 bridge
source ~/ResKU/install/setup.bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: 엔터마다 현재 전체 관절값 표시
source ~/ResKU/install/setup.bash
ros2 run tool_manipulator_bringup capture_arm_pose.py

raw 값 출력 

```bash
source ~/ResKU/install/setup.bash
ros2 run tool_manipulator_bringup capture_arm_raw.py
```

RMD 축은 `raw_deg`와 `raw_encoder`, Dynamixel 축은 `raw_pulse`를 출력한다.
영점 자세와 각 축의 안전한 최소·최대 자세에서 라벨을 입력해 캡처한다.


## 현재 자세를 새 영점으로 설정

6축 팔을 정확한 기계 영점 자세에 수동으로 놓은 뒤 `set_current_zero.py`로 현재 raw
엔코더 위치를 `hardware.yaml`의 새 영점으로 저장할 수 있다. 이 노드는 위의 읽기 전용
bridge 토픽만 구독하며 모터 명령을 보내지 않는다. 먼저 `real_control`을 완전히 종료하고
터미널 A에서 bridge만 실행한다.

```bash
# 터미널 A
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: 10개 샘플의 정지 상태를 검사하고 변경값만 미리보기
ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml

# 출력값을 확인한 뒤 실제 저장
ros2 run tool_manipulator_bringup set_current_zero.py \
  --config ~/ResKU/src/arm/tool_manipulator_bringup/config/hardware.yaml \
  --apply
```

기본 대상은 `ee_joint`를 제외한 팔 6축이다. 일부 축만 다시 맞추려면 예를 들어
`--joints shoulder_joint elbow_joint`를 지정한다. Dynamixel은 `zero_raw`의 변화량만큼
`soft_limit_raw`도 함께 이동하여 기존 영점 기준 좌우 가동 범위를 보존한다. RMD의
`soft_limit_rad`와 모든 축의 `soft_limit_rad`는 영점 기준 범위이므로 바꾸지 않는다.
저장 전 원본은 같은 폴더의 시간표시 `.bak-*` 파일로 백업되며, 새 값은 bridge를 종료하고
`real_control`을 다시 기동할 때부터 적용된다.

## Arm named pose 이동

`move_to_named_pose.py`는 MoveIt SRDF의 `arm` group state를 읽고
`/arm_controller/follow_joint_trajectory` action으로 한 개의 목표 trajectory를 보낸다.
현재 install 트리에 이 스크립트가 없을 수 있으므로, 소스와 현재 설정을 명시하는
워크스페이스 런처를 사용한다. 명령 형식은 다음과 같다.

```bash
bash ~/ResKU/utils/move_to_pose.sh <pose> [options]
```

현재 기본 SRDF pose는 `stand`, `home`, `back`, `tagid0_cw`, `tagid0_ccw`, `tagid0_unlock`,
`tagid0_lock_step1`, `tagid0_lock_step2`, `tagid0_lock`, `tagid1`, `dock_pre_cw`,
`dock_pre_ccw`, `dock_wait1`, `dock_wait2`, `dock_wait3`이다.

터미널 A에서 실기 controller를 먼저 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup real_control.launch.py
```

터미널 B에서 pose를 확인한 뒤 이동 명령을 보낸다.

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash

# 사용 가능한 pose 이름 확인
bash ~/ResKU/utils/move_to_pose.sh --list

# 목표값만 출력하고 모터 명령은 보내지 않음
bash ~/ResKU/utils/move_to_pose.sh home --dry-run

# 8초 동안 home pose로 이동하고 action server를 최대 10초 기다림
bash ~/ResKU/utils/move_to_pose.sh home \
  --duration 8.0 \
  --wait-for-server 10.0
```

지원 인자는 다음과 같다.

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `pose` | 없음 | 이동할 SRDF pose 이름. `--list`를 사용할 때는 생략 가능 |
| `--list` | 꺼짐 | 사용 가능한 `arm` pose 목록을 출력하고 종료 |
| `--duration <초>` | `5.0` | 현재 자세에서 목표 자세까지 이동할 trajectory 시간. 0보다 커야 함 |
| `--wait-for-server <초>` | `5.0` | `arm_controller` action server를 기다리는 시간. 0 이상이어야 함 |
| `--dry-run` | 꺼짐 | 관절 목표값만 출력하고 action goal을 보내지 않음 |
| `--srdf <경로>` | 설치된 기본 SRDF | 다른 SRDF 파일에서 named pose를 읽을 때 사용 |

다른 SRDF를 시험할 때도 먼저 `--dry-run`으로 목표값을 확인한다.

```bash
bash ~/ResKU/utils/move_to_pose.sh home \
  --srdf /absolute/path/to/tool_manipulator.srdf \
  --dry-run
```

이 실행기는 `base_joint`부터 `wrist_yaw_joint`까지 arm 6축만 제어하며
`ee_joint`(그리퍼)는 포함하지 않는다. 실행 전에 출력되는 6개 목표값과 이동 시간을
확인한다. 실기 제어가 기동되어 있고 리밋·전류·통신 preflight를 통과한 경우에만
실제 이동 명령을 사용한다. `rmd_joint_state_bridge` 또는 수동 캡처 노드와 동시에
실행하지 않는다.


## 조이스틱 TCP 수동 구동

`tcp_joy_teleop.py`는 조이스틱 입력을 `base_actuator` 기준 TCP 직선 속도로
변환한다. 각 관절을 직접 조작하지 않으며 TCP 회전과 `ee_joint` 제어도 하지 않는다.
기본 속도는 `0.03 m/s`이고, **L1 버튼을 누르고 있는 동안만** 움직인다.

실기에서는 `rmd_joint_state_bridge`를 먼저 종료하고 CAN/U2D2를 다른 프로세스가
사용하지 않는지 확인한다. `tcp_joy_teleop.launch.py`를 두 번 실행하거나
`real_control.launch.py`를 별도로 동시에 실행하면 controller와 하드웨어 포트가
충돌하므로 금지한다. 실기 launch는 `hardware.yaml`의 필수 안전값이 비어 있으면
모터를 시작하기 전에 의도적으로 종료된다.

### 원격 조이스틱으로 실기 구동

두 PC에서 `ROS_DOMAIN_ID`를 동일하게 설정하고 외부 통신을 허용한다. 숫자 `0`은
예시이므로 현장 네트워크에서 사용하는 값으로 맞춘다.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
```

조이스틱이 연결된 원격 PC에서 `/joy`를 발행한다.

```bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0

# 다른 터미널에서 버튼과 축 번호 확인
ros2 topic echo /joy
```

로봇 PC에서 CAN과 U2D2 장치를 먼저 확인한다. 이미 `can_arm`이 정상적으로 올라와
있다면 재설정하지 않아도 된다.

```bash
ip -details link show can_arm
ls -l /dev/ttyUSB0

# can_arm이 아직 설정되지 않았을 때만 실행
sudo ip link set can_arm down
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set can_arm up
```

> **실기 시작 전 필수:** 아래 검사 결과가 `READY`인지 확인한다. 이 검사는 필수값의
> 존재 여부만 확인하므로, 영점·리밋·속도·전류 값이 해당 실기와 일치하는지도 별도로
> 확인해야 한다. 실패하면 검사를 우회하거나 임의의 값을 넣지 않는다.

누락된 필드만 확인하려면 다음 읽기 전용 검사기를 실행한다. 모든 필수 필드와 현재값까지
보려면 `--show-values`를 붙인다. 이 명령은 모터와 통신하지 않는다.

```bash
ros2 run tool_manipulator_bringup check_hardware_config.py
ros2 run tool_manipulator_bringup check_hardware_config.py --show-values
```

로봇 PC에서는 실기 controller, MoveIt, Servo와 TCP teleop을 한 번에 실행한다.

```bash
ros2 launch tool_manipulator_bringup tcp_joy_teleop.launch.py \
  real_hardware:=true \
  launch_joy:=false
```

다른 로봇 PC 터미널에서 controller 상태를 확인한 후 Servo를 시작한다.

```bash
ros2 control list_controllers
# arm_controller와 joint_state_broadcaster가 active인지 확인

ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```

원격 PC의 `/joy`가 로봇 PC에서 보이지 않으면 팔을 움직이지 말고 두 PC의
`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, 같은 네트워크 연결과 방화벽을 먼저 확인한다.

### 조작 방법

축 번호는 현재 Logitech 계열 매핑 기준이다. 패드 종류에 따라 달라질 수 있으므로
반드시 `/joy` 출력과 아래 매핑을 비교한다.

| 조작 | 동작 |
| --- | --- |
| L1 계속 누름 (`button 4`) | 데드맨 활성화 |
| 오른쪽 스틱 상하 (`axis 4`, 기본 반전) | TCP X 이동 |
| 오른쪽 스틱 좌우 (`axis 3`) | TCP Y 이동 |
| 왼쪽 스틱 상하 (`axis 1`) | TCP Z 이동 |

L1을 놓거나 `/joy`가 `0.25초` 이상 끊기면 zero Twist를 발행한다. 수동 구동은
TCP 직선 이동만 지원하므로 wrist 회전이나 그리퍼 개폐는 별도 명령이 필요하다.
처음에는 장애물과 특이점에서 충분히 떨어진 자세에서 짧게 시험한다.

종료할 때는 L1을 놓고 Servo를 먼저 정지한 다음 각 launch를 `Ctrl-C`로 종료한다.

```bash
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger "{}"
```

### 관절별 조이스틱 조그

`joint_joy_teleop.py`는 선택한 arm 관절 하나만 `JointJog`로 움직인다. 명령은
`/servo_node/delta_joint_cmds`로 보내므로 MoveIt Servo의 관절 제한, 충돌 검사와
정지 timeout을 거친다. 이 노드와 `tcp_joy_teleop.py`를 동시에 실행하지 않는다.
또한 기존 `safety_manager`가 따로 실행 중이면 포커스 관리자가 중복되므로 이 노드에
`manage_focus:=false`를 주고 기존 `/control/active_target`을 사용한다.

로봇 PC에서 controller와 Servo를 시작한 다음 관절 조그 노드를 실행한다. 원격 PC의
`remote_joy.launch.py`는 위와 동일하게 계속 사용한다.

```bash
# 로봇 PC
ros2 launch tool_manipulator_bringup real_control.launch.py launch_servo:=true
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
ros2 launch tool_manipulator_bringup joint_joy_teleop.launch.py
```

| 조작 | 동작 |
| --- | --- |
| Options (`button 9`) | 조이스틱 포커스를 `drive`/`arm`으로 전환 |
| D-pad 좌우 (`axis 6`) | 조작할 관절 선택(데드맨을 놓은 상태에서만) |
| L1 (`button 4`) + 오른쪽 스틱 상하 (`axis 4`) | 선택 관절 조그 |

대상은 `base`, `shoulder`, `elbow`, `wrist_pitch`, `wrist_roll`, `wrist_yaw`
순서이며 기본 최대 속도는 `0.12 rad/s`이다. L1을 놓거나 `/joy`가 `0.25초` 이상
끊기거나 포커스가 `drive`로 바뀌면 정지 명령을 보낸다. `ee_joint`(그리퍼)는 Servo의
`arm` planning group에 포함되지 않으므로 이 노드에서 조작하지 않는다.

### 한 PC에서 mock 구동

실기 없이 확인할 때만 기본 mock 모드를 사용한다. 이 모드는 실제 모터를 구동하지 않는다.

```bash
ros2 launch tool_manipulator_bringup tcp_joy_teleop.launch.py \
  real_hardware:=false \
  launch_joy:=true \
  joy_dev:=/dev/input/js0

# 다른 터미널에서 실행
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
```



터미널 1에서 먼저 can_arm과 실제 제어를 시작합니다.
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm

source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup real_control.launch.py
터미널 2는 조이스틱 PC에서 실행합니다.
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0
터미널 4에서 controller가 모두 active인지 확인합니다.
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 control list_controllers
그 뒤 터미널 3의 Servo와 tcp_joy_teleop.py를 켜고, 터미널 4에서 Servo를 시작합니다.
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"
현재 소스에는 실기용 Servo + TCP teleop 통합 launch가 없습니다. tcp_joy_teleop.launch.py는 mock 하드웨어용이므로 실기에서 실행하면 안 됩니다. 실기용으로는 real_control과 연결된 servo_node_main 및 tcp_joy_teleop.py launch를 추가해야 합니다.
확인용 터미널에서는 아래만 보면 됩니다.
ros2 topic echo /joint_states
ros2 topic echo /joy
ros2 topic echo /servo_node/delta_twist_cmds

터미널 1:
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm

cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_bringup real_control.launch.py
터미널 3:
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision cameras.launch.py
터미널 4:
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_bringup arm_tag_docking_vision.launch.py
터미널 5:
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/arm/tool_manipulator_bringup/scripts/tag_pose_filter.py \
  --ros-args \
  --params-file src/arm/tool_manipulator_bringup/config/docking.yaml \
  -p tools_config_file:=src/arm/tool_manipulator_bringup/config/tools.yaml
터미널 6:
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/arm/tool_manipulator_bringup/scripts/visual_servo_node.py \
  --ros-args \
  --params-file src/arm/tool_manipulator_bringup/config/docking.yaml
터미널 7:
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/arm/tool_manipulator_bringup/scripts/ui_tag1_docking_node.py \
  --ros-args \
  --params-file src/arm/tool_manipulator_bringup/config/ui_tag1_docking.yaml
