# Tool Manipulator Bringup

실기 팔 제어와 **attach-only** AprilTag tool docking 패키지다. 실제 실행 순서와 Jetson/로컬 토픽 표는 저장소 루트의 [`howtorun.md`](../../../../howtorun.md) `Tool Manipulator — 실기 제어·툴 교체 실행 절차`를 따른다.

- URDF `world`는 차량 IMU 중심이며, `arm_world`는 IMU/world 기준 `(0.1921, 0, 0.235)` m offset이며 지면 기준 높이는 `0.350` m이다. 차체, 회로박스, 대칭 tool case는 fixed visual/collision links로 표시된다.
- `config/hardware.yaml`: U2D2 TTL/RS-485와 RMD CAN, 모든 관절 영점·리밋·register 설정의 단일 원본
- `config/tools.yaml`: tool/fixture collision, TCP, tag, 접근·yaw·직선 motion 설정의 단일 원본
- `config/docking.yaml`: existing vision AprilTag 입력, tag/depth gate, XY servo, 공통 pose의 단일 원본
- `config_validator.py`: 세 원본의 모든 필수 필드를 검사한다. 실패하면 `real_tool_change.launch.py`는 hardware를 시작하지 않는다.
- `real_control.launch.py`: 검사 완료한 `hardware.yaml`으로 real `ros2_control`, controller, MoveIt을 기동한다.
- `real_tool_change.launch.py`: real control + filter + XY servo + dock action + planning scene stack을 기동한다.
- `arm_tag_docking_vision.launch.py`: 수정하지 않은 `vision.apriltag`를 `docking.yaml`의 단일 입력 설정으로 실행한다.

도킹은 `yaw trajectory 성공 → /wrist_yaw_rotation_complete → configured retreat 성공 → /docking_complete → scene attach` 순서다. yaw 성공은 물리 체결이나 scene 부착 검증이 아니다. 별도 체결 센서는 현재 사용하지 않는다.

물리 detach/unlock 절차는 아직 구현되지 않았다. `Dock.mode=1`과 scene detach service는 명시적으로 거부되며 성공으로 보고하지 않는다. 재시작·도킹 실패 뒤 tool 상태가 불명확하면 scene manager는 `UNKNOWN`으로 시작/전환하며, 실물이 비어 있음을 확인한 운영자만 `~/operator_confirm_empty`로 복구할 수 있다.

## 수동 관절값 캡처 (RMD 리밋 측정)

팔을 손으로 움직여 RMD 영점·소프트 리밋을 측정할 때는, 제어 노드 대신 읽기 전용
`rmd_joint_state_bridge`와 `capture_arm_pose.py`를 사용한다. bridge는 RMD CAN과
Dynamixel U2D2 포트에서 현재 엔코더 위치를 읽기만 하며 위치·속도·토크 명령을 보내지 않는다.

```bash
# 터미널 A: 읽기 전용 엔코더 bridge
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: 엔터마다 현재 전체 관절값 표시
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 run tool_manipulator_bringup capture_arm_pose.py
```

영점과 소프트 리밋을 함께 기록할 때는 보정된 rad와 모터 raw 값을 동시에
출력하는 다음 노드를 사용한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 run tool_manipulator_bringup capture_arm_raw.py
```

RMD 축은 `raw_deg`와 `raw_encoder`, Dynamixel 축은 `raw_pulse`를 출력한다.
영점 자세와 각 축의 안전한 최소·최대 자세에서 라벨을 입력해 캡처한다.

`label>` 프롬프트에서 아무것도 입력하지 않고 Enter를 누르면 다음 순서로 최신
`/joint_states` 값이 rad 단위로 출력된다. 필요하면 `shoulder_min`처럼 라벨을
입력하고 Enter를 눌러 측정 지점을 구분한다.

```text
[capture] (rad)
  base: ...
  shoulder: ...
  elbow: ...
  wrist_pitch: ...
  wrist_roll: ...
  wrist_yaw: ...
  ee: ...
```

각 Dynamixel 또는 RMD가 통신하지 않으면 해당 축은 `unavailable`로 표시된다.
`q`, `quit`, `exit` 중 하나를 입력하면 캡처를 끝낸다.

`rmd_joint_state_bridge`는 CAN과 `/dev/ttyUSB0`를 직접 소유한다. 따라서 실제
`ros2_control`/`real_control.launch.py`, MoveIt 또는 named-pose 제어와 동시에 실행하지 않는다.
리밋 측정 중에는 모터 명령을 보내지 말고, 안전한 기계 범위 안에서만 수동으로 이동한다.

## Arm named pose 이동

`move_to_named_pose.py`는 MoveIt SRDF의 `arm` group state를 읽고
`/arm_controller/follow_joint_trajectory` action으로 한 개의 목표 trajectory를 보낸다.
명령 형식은 다음과 같다.

```bash
ros2 run tool_manipulator_bringup move_to_named_pose.py <pose> [options]
```

현재 기본 SRDF pose는 `stand`, `home`, `tagid0_cw`, `tagid0_ccw`, `tagid1`, `dock_pre_cw`, `dock_pre_ccw`,
`dock_wait1`, `dock_wait2`, `dock_wait3`이다. 빌드 후 다음 순서로 실행한다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-select tool_manipulator_moveit_config tool_manipulator_bringup
source ~/ResKU/install/setup.bash
```

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
ros2 run tool_manipulator_bringup move_to_named_pose.py --list

# 목표값만 출력하고 모터 명령은 보내지 않음
ros2 run tool_manipulator_bringup move_to_named_pose.py home --dry-run

# 8초 동안 home pose로 이동하고 action server를 최대 10초 기다림
ros2 run tool_manipulator_bringup move_to_named_pose.py home \
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
ros2 run tool_manipulator_bringup move_to_named_pose.py home \
  --srdf /absolute/path/to/tool_manipulator.srdf \
  --dry-run
```

이 실행기는 `base_joint`부터 `wrist_yaw_joint`까지 arm 6축만 제어하며
`ee_joint`(그리퍼)는 포함하지 않는다. 실행 전에 출력되는 6개 목표값과 이동 시간을
확인한다. 실기 제어가 기동되어 있고 리밋·전류·통신 preflight를 통과한 경우에만
실제 이동 명령을 사용한다. `rmd_joint_state_bridge` 또는 수동 캡처 노드와 동시에
실행하지 않는다.
