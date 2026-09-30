# Tool Manipulator: 실기 도킹 운영 절차

이 문서는 **실제 로봇팔에서 tool docking을 운용할 때**의 절차다.
로컬 PC는 UI·RViz·MoveIt, Jetson은 모터·카메라·센서 연결을 담당한다.

> 안전 원칙
>
> - 물리 E-stop과 전원 차단 방법을 확인한 상태에서만 실행한다.
> - Dynamixel Wizard와 ROS bridge는 같은 `/dev/ttyUSB0`를 동시에 열지 않는다.
> - `/active_tool_id != 0`이면 기존 툴 해제가 끝나기 전 새 장착 요청을 보내지 않는다.
> - `use_mock_hardware:=true`인 demo/현재 docking launch는 실기 구동용이 아니다.

## 1. 자동 도킹 시작 조건

아래 항목이 모두 충족되어야 자동 도킹을 시작할 수 있다.

- 실제 RMD + Dynamixel `ros2_control` hardware controller가 실행 중이다.
- `FollowJointTrajectory` action과 arm controller가 정상 동작한다.
- 모든 관절 raw 영점·소프트 리밋·방향이 코드에 반영돼 있다.
- 요청 툴의 `enabled: true`, `coarse_joint_goal`, collision geometry가 `tools.yaml`에 있다.
- tool별 tag ID, 체결 yaw 각도, 하강/상승 거리, retreat pose가 티칭돼 있다.
- `mission_wait_joint_goal_yaml`이 전면 미션 대기 자세로 채워져 있다.
- arm camera optical frame, AprilTag frame ID, Servo TF가 일치한다.

현재 저장소는 joint-state bridge와 도킹 상태기까지 구현돼 있다. **실제 통합 hardware controller는 아직 별도 구현 대상**이므로, 이 조건이 충족되기 전에는 자동 도킹 명령을 보내면 안 된다.

## 2. 공통 환경

로컬과 Jetson에서 같은 ROS domain을 사용한다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
```

변경 후 양쪽에서 필요한 패키지를 빌드한다.

```bash
colcon build --symlink-install \
  --packages-select \
  tool_manipulator_description \
  tool_manipulator_moveit_config \
  tool_manipulator_bringup \
  rmd_joint_state_bridge \
  vision
source install/setup.bash
```

## 3. Jetson: 하드웨어·카메라

### Terminal J1 — 실제 joint state bridge

이 bridge는 읽기 전용이다. 실제 모터 명령을 보내지 않는다.

```bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py \
  can_ifname:=can_arm \
  dxl_port_name:=/dev/ttyUSB0 \
  dxl_baud_rate:=1000000 \
  rmd_wrist_joint_name:=wrist_pitch_joint
```

확인:

```bash
ros2 topic echo /joint_states
```

### Terminal J2 — arm D435i

```bash
ros2 launch vision cameras.launch.py
```

```bash
ros2 topic list | rg '/arm/camera'
```

### Terminal J3 — arm AprilTag

아래는 태그가 **20 cm × 20 cm**일 때의 값이다. 단위가 mm라면 `tag_size_cm:=2.0`으로 바꾼다.

```bash
ros2 run vision apriltag --ros-args \
  -p input_topic:=/arm/camera/color/image_raw/compressed \
  -p camera_info_topic:=/arm/camera/color/camera_info \
  -p centers_topic:=/arm/apriltag/centers \
  -p tag_size_cm:=20.0
```

검출 확인:

```bash
ros2 topic echo /arm/apriltag/centers --once
```

`id`, `frame_id`, `position_camera_cm`가 모두 나와야 한다.

## 4. 로컬: 실제 상태 시각화·감시

### Terminal L1 — RViz

실제 `/joint_states`는 Jetson bridge가 제공하므로 GUI joint publisher를 끈다.

```bash
ros2 launch tool_manipulator_description display.launch.py \
  use_mesh:=true \
  tool_id:=0 \
  publish_joint_states:=false
```

현재 장착된 툴에 맞춰 `tool_id:=1`, `2`, `3`으로 바꾼다.

### Terminal L2 — controller/TF 사전 점검

```bash
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo world tcp_link
ros2 control list_controllers
ros2 action list | rg 'follow_joint_trajectory'
```

자동 도킹 시작 전 다음을 확인한다.

- 모든 관절이 `/joint_states`에 존재한다.
- `world -> tcp_link` TF가 끊기지 않는다.
- arm controller가 `active`다.
- FollowJointTrajectory action이 존재한다.

### Terminal L3 — 도킹 상태 감시

```bash
ros2 topic echo /docking_status
```

```bash
ros2 topic echo /tool_attachment_status
```

```bash
ros2 topic echo /active_tool_id
```

## 5. 실제 자동 도킹 실행

이 절은 1장의 시작 조건을 모두 만족하고, 실기 hardware controller가 연결된 뒤에만 사용한다.

### Terminal L4 — MoveIt + docking coordinator

> 아직 실제 hardware controller launch가 없으므로, 여기에는 명령을 적지 않는다.
> FakeSystem 기반 `apriltag_tool_docking.launch.py`는 실기에서 실행 금지다.

실기 controller를 구현한 뒤 이 terminal은 다음을 실행한다.

1. 실기 MoveIt `move_group`
2. MoveIt Servo
3. `apriltag_tool_docking_node.py`
4. `tool_attachment_manager.py`

### Terminal L5 — UI에서 tool 선택

예: tool 2 장착 요청.

```bash
ros2 topic pub --once /selected_tool_id std_msgs/msg/Int32 '{data: 2}'
```

### 예상 상태 순서

```text
coarse_moving
fine_xy_aligning
descending
lock_rotating
lock_complete:awaiting_scene_attach
ascending
retreating
returning_to_wait
mission_wait
ready:mission_wait
```

미션 대기 pose가 아직 비어 있으면 마지막 상태는 아래와 같다. 이 상태에서는 자동 미션을 시작하지 않는다.

```text
ready:mission_wait_pose_unconfigured
```

## 6. 즉시 중단 조건

아래 중 하나라도 발생하면 UI 요청을 반복하지 말고 물리 E-stop 또는 전원 차단 절차를 사용한다.

- 예상과 다른 관절 움직임
- soft limit 근접 또는 초과
- AprilTag 유실·오검출
- camera frame/TF 오류
- `/docking_status`의 `failed:*`
- `/tool_attachment_status`의 `attach_failed:*`
- 모터 통신 끊김·과전류·비정상 발열

실기 원인 확인 후에는 반드시 `/active_tool_id`와 planning scene 상태를 확인하고 재시작한다.

## 7. 현재 미완료 항목

자동 도킹을 실기에 허용하기 전에 아래를 완료한다.

1. RMD CAN + Dynamixel TTL/RS-485 통합 `ros2_control` hardware controller
2. tool 1·2·3 collision object 정의 및 attach 검증
3. 기존 툴 해제 상태기: 안전 retreat, wrist yaw 역회전, 물리 해제 확인
4. tool별 coarse/retreat pose, 상승 거리, 체결 각도
5. 전면 미션 대기 pose
6. aligned depth 기반 tag 거리 검증
