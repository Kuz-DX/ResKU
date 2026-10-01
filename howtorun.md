## Person Detection 비전 모델

주행/좌/우 카메라의 압축 컬러 영상을 `vision/models/person.pt` YOLO 모델
하나로 순차 추론한다. 모델의 학습 클래스 `pedestrian`은 출력 JSON에서 기존
인터페이스와 같은 `person`으로 발행된다.
사람 bbox가 그려진 `sensor_msgs/msg/CompressedImage` 출력은 다음과 같다.

- `/drive/person/detecion`
- `/left/person/detection`
- `/right/person/detection`

입력은 각각 `/drive/camera/color/image_raw/compressed`,
`/side/left/image_raw/compressed`, `/side/right/image_raw/compressed`이다.

```bash
source /home/shu/ResKU/install/setup.bash
ros2 launch vision person_detection.launch.py
```

기본 추론 크기는 640이고 CPU를 사용한다. 다른 장치를 쓰거나 속도를 우선할 때는
예를 들어 `device:=0 infer_size:=320`을 launch 인자로 전달한다.

JSON 검출 결과는 drive의 기존 `/person_detection/detections`와 side cam의
`/left/person/detections`, `/right/person/detections`로 발행된다. bbox 이미지가
필요 없고 JSON 발행 주기를 우선할 때는 `publish_visualization:=false`로 실행한다.

```bash
ros2 launch vision person_detection.launch.py publish_visualization:=false
```

## Supply box 인식

팔 카메라의 컬러·정렬 Depth·CameraInfo와 팔 TF를 실행한 후:

```bash
ros2 launch vision supply.launch.py
```

모델은 vision 패키지에 포함된 `supplyboxv3_int8_openvino_model`이다.
노드 구현은 `src/vision/vision/supply.py`이며, bbox는
`/arm/supply/detections`로 발행하여 DolBot_Center 로봇팔 카메라 패널에 표시한다.
기존 3D 목표 `/arm/target_point`와 거리 `/arm/target_depth_m`는 유지한다.
기존 미션 launch도 새 supply 실행 파일을 호출하므로 두 launch를 중복 실행하지 않는다.

## 3. 계절별 미션

### mission_manager_interfaces (신규 패키지, 임시)

`MissionResult.msg` 하나만 있는 msg 전용 패키지 (`ament_cmake`, 구 이름
`dolbotz_interfaces`). 미션 노드들이 공용으로 쓰는 결과 메시지 타입:
```
std_msgs/Header header
string mission_name
string state
geometry_msgs/Point target_point
bool valid
```

> **주의(임시 패키지)**: 미션 노드들이 아직 전부 스텁이라 이 스키마가
> 실제로 맞는지 검증된 적이 없다. 구현 진행하면서 안 맞으면 표준 메시지
> 타입(`std_msgs/String`, `geometry_msgs/PointStamped` 등)으로 대체하고
> 이 패키지 자체를 없애는 것도 고려할 것.

### 계절별 미션 스텁 노드 4개 (실제 인식 로직 미구현, 뼈대만 있음)

```bash
ros2 run dolbotz spring_ifof       # 봄 — 피아식별, /mission/spring_ifof/result
ros2 run dolbotz summer_traffic    # 여름 — 신호등 인식, /mission/summer_traffic/result
ros2 run dolbotz fall_marker       # 가을 — 비전마커 순차 인식, /mission/fall_marker/result
ros2 run dolbotz escort_follow     # 5구간 — 선도 정찰로봇 추종, /mission/escort_follow/result
```

넷 다 컬러 이미지(`/drive/camera/color/image_raw/compressed`, `camera_info`)를
구독하고(`escort_follow`만 뎁스도 message_filters로 동기화 구독), 각자
`/mission/<이름>/result`(`mission_manager_interfaces/MissionResult`)를 발행한다.
지금은 매 프레임 `valid=False`인 더미 결과만 나간다 — 각 파일 안
`# TODO: 실제 인식 로직 구현` 위치에 실제 모델/로직을 채워야 한다.

> **주의**: 각 노드의 배점/판정 기준은 아직 대회 규정 문서가 반영 안 됨 —
> docstring에 TODO로 남겨뒀으니 규정 확정되면 채울 것.

### launch 파일

```bash
ros2 launch dolbotz mission_spring.launch.py  # side camera + spring_ifof + LED
ros2 launch dolbotz mission_summer.launch.py  # side camera + summer_traffic + summer_supply
ros2 launch dolbotz mission_fall.launch.py    # side camera + fall_marker
```

### 자율 제어 (MPPI, 2단계 평가용 보존 — 지금 당장 쓰지 않음)

[2026 사용자 결정, 계절 미션 정리] 계절 미션 전용 노드(`slope_traverse_*`,
`dog_follow_node`, `summer_supply_drive_node`)와 `current_ramp_node`,
`stability_monitor_node`는 소스 자체가 삭제됐다 — 지금은 manual+return
미션(바로 아래 절)만 운용한다. MPPI(`nav2_mppi_controller`/`our_mppi_critics`)와
`path_relay`는 나중에 recorded return path와 비교하는 2단계 평가용으로
소스만 보존했다:

```bash
ros2 launch robot_bringup autonomous.launch.py
```

reduced_odom + Nav2 MPPI controller_server + path_relay_node만 남은
경량 구성이다. **manual_return_bringup.launch.py(아래)와 절대 동시에
띄우지 말 것** — 같은 CAN 버스/`/cmd_vel`을 두고 충돌한다.

# 메뉴얼 주행 + 자동 복귀 (manual+return 통합 아키텍처)

[manual+return 통합] 예전엔 manual 조종(can_driver_node)과 자율주행
(rmd_x8_driver_node)이 CAN을 각자 따로 소유해서 manual 주행 중엔 wheel
odometry가 아예 안 나왔다. 이제 rmd_x8_driver_node가 manual+return 공용
CAN 드라이버라 manual 주행 중에도 `/wheel/odom`/`/odometry/filtered`가
정상 발행되고, 사용자가 RETURN 트리거를 누르면 기록된 경로를 뒤집어
자동으로 출발지까지 복귀한다. 상세 아키텍처/토픽 그래프는
`src/drive/autonomous/robot_bringup/launch/manual_return_bringup.launch.py`
docstring과 `topic.md` 참고.

can_driver_node 기반 구 manual.launch.py는 더 이상 쓰지 않는다 (소스는
참고용으로 남아있지만, manual_stability_node의 안전 배선이 깨져 있어
그대로 실행하면 IMU 긴급정지가 동작하지 않는다).

### CAN_drive 인터페이스 활성화
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
ip -details link show can_drive

### 실행 (로봇 PC + 원격 PC 각 1줄)

[로봇 PC] rmd_x8_driver_node(CAN 유일 소유, /wheel/odom) + myahrs_driver_node
(/imu) + reduced_odom_node(/odometry/filtered) + drive_cmd_mux_node +
manual_path_recorder_node + return_state_machine_node +
return_path_follower_node 전부 포함:

[2026 경량화 결정] manual_stability_node(IMU pitch/roll 긴급정지)는 이
launch에 포함되지 않는다 -- 전복 위험이 없는 운용 환경으로 판단해 의도적으로
뺐다(제거 이유/재추가 방법은 manual_return_bringup.launch.py 상단 주석 참고).
```bash
source /home/shu/ResKU/install/setup.bash
ros2 launch robot_bringup manual_return_bringup.launch.py
```

[원격 PC, 조이스틱이 물린 쪽] joy_node + manual_joy_control_node
(출력: `/motor_speed_cmd_manual`, dps, Twist 변환은 drive_cmd_mux_node가 담당.
RETURN 트리거: `/path/return`, 구 `/mission/return/trigger`):
```bash
source /home/shu/ResKU/install/setup.bash
ros2 launch manual_joy_control manual_control.launch.py
```

두 PC 모두 같은 ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY=0 이어야 한다.

### rosbag 기록

```bash
bash src/drive/autonomous/robot_bringup/scripts/record_manual_drive.sh
```

## 확인용 (선택)

### 터미널 3 - 조이스틱 raw 입력 확인
source /home/shu/ResKU/install/setup.bash
ros2 topic echo /joy
 

### 터미널 4 - manual/최종 모터 명령 확인
source /home/shu/ResKU/install/setup.bash
ros2 topic echo /motor_speed_cmd_manual   # manual_joy_control_node 출력 (dps)
ros2 topic echo /cmd_vel                  # drive_cmd_mux_node 최종 출력(Twist) -> rmd_x8_driver_node

### 터미널 4-1 - 복귀 미션 상태/경로 확인
source /home/shu/ResKU/install/setup.bash
ros2 topic echo /mission/return/state    # IDLE/MANUAL_RECORDING/.../FINISHED
ros2 topic echo /recorded_path           # manual 주행 중 계속 쌓이는 기록 경로
ros2 topic echo /return_path             # RETURN 트리거 후 1회 발행되는 복귀 경로

### 터미널 5 - drive/arm 중 조이스틱이 지금 어디에 반응 중인지 확인
source /home/shu/ResKU/install/setup.bash
ros2 topic echo /control/active_target
# Options(9번) 버튼으로 drive <-> arm 토글. Arm 진입 시 기본은 MANUAL.
# Arm 활성 중 Share(8번) 버튼으로 MANUAL <-> AUTO 전환.
# active_target은 data: drive 또는 data: arm 으로 표시됨


cansend can_drive 141#7600000000000000
보호상태 해제


cansend can_drive 141#9C00000000000000
DATA[0]   0x9C          명령 코드
DATA[1]   int8_t        모터 온도, °C
DATA[2:3] int16_t       토크 전류
DATA[4:5] int16_t       출력축 속도, deg/s
DATA[6:7] uint16_t      출력축 엔코더 위치

모터상태 확인 명령

회전이 안 될 때, 
cansend can_drive 141#9C00000000000000
candump can_drive
이거 실행하고 로그 찍어두기


### CAN 인터페이스 활성화 can_arm
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm
ip -details link show can_arm
### 매니퓰레이터 원격 조이스틱 수동 구동

# 두 PC에서 동일한 ROS_DOMAIN_ID와 ROS_LOCALHOST_ONLY=0을 설정한다.
# 원격 PC: 물리 조이스틱의 /joy 발행
ros2 launch tool_manipulator_bringup remote_joy.launch.py joy_dev:=/dev/input/js0

# 로봇 PC: real_control + MoveIt Servo + TCP teleop
# real_control.launch.py를 별도로 동시에 실행하지 않는다.
ros2 launch tool_manipulator_bringup tcp_joy_teleop.launch.py \
  real_hardware:=true \
  launch_joy:=false

# 로봇 PC의 다른 터미널: controller active 확인 후 Servo 시작
ros2 control list_controllers
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger "{}"

# L1(button 4)을 누르는 동안만 이동한다. 상세 축 매핑과 종료 절차는
# src/arm/tool_manipulator_bringup/README.md의 조이스틱 TCP 수동 구동 절을 따른다.

### 실기 팔 RViz 미러링 (실측 각도/엔코더, 읽기 전용 - tool_manipulator_description)
# ~/arm_config/rmd_joint_state_bridge.py(외부 스크립트)와 동일한 역할을 하는
# 내장 노드(rmd_joint_state_bridge 패키지) - ros2_control 없이 RMD(CAN)/
# Dynamixel(TTL) 엔코더를 getter로만 읽어서 /joint_states + 진단 토픽으로
# 발행한다. 포지션 커맨드를 절대 안 보내므로 팔을 손으로 자유롭게 움직이며
# 자세(홈/스탠바이 등)를 캡처할 때 안전하게 쓸 수 있다.

# 터미널 1 - RViz (URDF만, MoveIt 없이 가볍게)
source /opt/ros/humble/setup.bash
source /home/shu/ResKU/install/setup.bash
ros2 launch tool_manipulator_description display.launch.py use_mesh:=true publish_joint_states:=false

# 터미널 2 - CAN 인터페이스 켜기 (위 "CAN 인터페이스 활성화 can_arm" 참고, 이미 켰으면 생략)
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm

# 터미널 3 - 브릿지 실행. tool_manipulator의 RMD 관절명 wrist_pitch_joint를 기본 사용한다.
# sign/q_offset은 hardware.yaml의 shoulder_joint/elbow_joint/wrist_pitch_joint 값과
# 맞춘다. CAN 인터페이스 기본값은 can_arm이다.
source /opt/ros/humble/setup.bash
source /home/shu/ResKU/install/setup.bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py
# 예: ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py wrist_sign:=-1.0 wrist_q_offset:=0.0523599

# 확인용 - 새 터미널에서
ros2 topic echo /arm/joint_angle_deg      # RMD 보정된 각도(deg), [shoulder, elbow, wrist] 순서
ros2 topic echo /arm/rmd_raw_angle_deg    # RMD 원본(보정 전) 각도(deg), 같은 순서
ros2 topic echo /arm/rmd_encoder          # RMD 원본 엔코더 카운트, 같은 순서
ros2 topic echo /arm/dxl_angle_deg        # Dynamixel 보정된 각도(deg), [base, gripper] 순서
ros2 topic echo /arm/dxl_encoder          # Dynamixel 원본 pulse, 같은 순서
# 실제 팔을 손으로 움직이면 RViz에서 그대로 따라 움직이고 위 토픽 값도 실시간으로 바뀜.

# === 카메라 4대 네이밍 정리 ===
# 주행용 depth D455  -> 네임스페이스 drive   (/drive/camera/...)
# 로봇팔용 depth D435i -> 네임스페이스 arm    (/arm/camera/...)
# 사이드캠 Logitech C920 -> 네임스페이스 left/camera  (/left/camera/...)
# 사이드캠 Logitech C922 -> 네임스페이스 right/camera (/right/camera/...)
# 기본은 아래 tool bringup 절의 `ros2 launch vision cameras.launch.py`로 4대를 함께 실행한다.
# 아래 개별 카메라 명령은 통합 launch를 쓰지 않을 때만 사용하며 동시에 실행하지 않는다.

# 로봇팔 D435i — summer_supply(구 arm_pickup) 전용
# -r __ns:=/arm 필수 — 안 주면 기본 네임스페이스(/camera/camera/...)로 나가서
# summer_supply의 기본 구독 토픽(/arm/camera/...)과 안 맞아 아무 프레임도 안 들어옴
# (주행 D455의 -r __ns:=/drive 패턴과 동일, 위 "카메라 4대 네이밍 정리" 참고)
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/arm -p serial_no:="'213622251385'" -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true -p enable_infra1:=false -p enable_infra2:=false -p enable_gyro:=false -p enable_accel:=false

ros2 run dolbotz summer_supply
# [2026-08-30] RF-DETR/YOLO 비교 끝나고 yolo(supplyboxv3.pt, 구 supplybest.pt에서
# 교체)로 확정 - detector_backend 파라미터/RF-DETR 코드는 제거됨(더 이상 존재하지 않음).
ros2 run dolbotz arm_visualizer

# 로봇팔 rosbag
ros2 bag record -o summer_supply_bag \
  /arm/camera/color/image_raw/compressed \
  /arm/camera/color/camera_info \
  /arm/camera/aligned_depth_to_color/image_raw/compressedDepth \
  /arm/camera/aligned_depth_to_color/camera_info

ros2 bag record -o summer_supply_bag -a


# 주행 D455 (align_depth: RGB 마스크와 depth를 같은 좌표계로 쓰기 위해 켬)
ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'117222251401'" -p enable_color:=true -p enable_depth:=true -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 -p enable_infra1:=false -p enable_infra2:=false -p align_depth.enable:=true


ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=drive \
  camera_name:=camera \
  serial_no:=_117222251401 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15



# 로봇팔 D435i — 시리얼은 위 "summer_supply 전용" 절과 동일 카메라(213622251385)
  ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=arm \
  camera_name:=camera \
  serial_no:=_213622251385 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15


# 단일 카메라 직접 실행 예시. cameras.launch.py와 동시에 실행하지 말 것.
# 사이드캠 (Logitech C920 -> left/camera, C922 -> right/camera, usb_cam 패키지)
# video_device 경로는 `ls /dev/video*` 또는 `v4l2-ctl --list-devices`로
# 실제 장치를 확인해서 채울 것 — 재부팅/재연결 시 번호가 바뀔 수 있으므로
# /dev/v4l/by-id/... 의 고정 심볼릭 경로를 쓰는 게 더 안전함.
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/left/camera \
  -p video_device:=<TODO: C920 장치 경로, 예: /dev/video0> \
  -p camera_name:=left_camera

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/right/camera \
  -p video_device:=<TODO: C922 장치 경로, 예: /dev/video2> \
  -p camera_name:=right_camera


  ros2 bag record -o ~/arm_pickup_bag_all \
  /left/camera/image_raw/compressed \
  /right/camera/image_raw/compressed \
  /arm/camera/color/camera_info \
  /arm/camera/color/image_raw \
  /arm/camera/color/image_raw/compressed \
  /arm/camera/color/image_raw/compressedDepth \
  /arm/camera/color/image_raw/theora \
  /arm/camera/color/metadata \
  /arm/camera/depth/camera_info \
  /arm/camera/depth/image_rect_raw \
  /arm/camera/depth/image_rect_raw/compressed \
  /arm/camera/depth/image_rect_raw/compressedDepth \
  /arm/camera/depth/image_rect_raw/theora \
  /arm/camera/depth/metadata \
  /arm/camera/extrinsics/depth_to_color \
  /arm/camera/extrinsics/depth_to_depth \
  /drive/camera/color/camera_info \
  /drive/camera/color/image_raw \
  /drive/camera/color/image_raw/compressed \
  /drive/camera/color/image_raw/compressedDepth \
  /drive/camera/color/image_raw/theora \
  /drive/camera/color/metadata \
  /drive/camera/depth/camera_info \
  /drive/camera/depth/image_rect_raw \
  /drive/camera/depth/image_rect_raw/compressed \
  /drive/camera/depth/image_rect_raw/compressedDepth \
  /drive/camera/depth/image_rect_raw/theora \
  /drive/camera/depth/metadata \
  /drive/camera/extrinsics/depth_to_color \
  /drive/camera/extrinsics/depth_to_depth

### 여름 자율 매니퓰레이터 실행 순서

아래 순서는 실제 로봇 기준이다. 모든 터미널에서 같은 워크스페이스를 source하고
`ROS_DOMAIN_ID=99`, `ROS_LOCALHOST_ONLY=0`, `rmw_cyclonedds_cpp`를 동일하게 써야
한다. 소스 수정 후 최초 1회는 관련 패키지를 빌드한 다음 새 터미널에서 실행한다.

```bash
cd /home/shu/ResKU
colcon build --packages-select \
  army_manipulator_bringup army_manipulator_description
source /home/shu/ResKU/install/setup.bash
```

**터미널 런처 권장 순서**

1. `CAN Arm`
2. `Arm Camera`
3. `summer_arm` — 팔 controller, MoveIt, `ik_node.py`

`summer_arm` 실행 후 `grasp_wait` 실제 도달 로그를 확인한다. 이 절에서는
`CAN Drive`, `Drive Camera`, `vision_summer`, `summer_drive` 실행 순서를 다루지
않는다.

런처를 사용하지 않을 때의 핵심 수동 명령은 다음과 같다.

```bash
# 1. 로봇팔 RealSense
#    [2026-09-05] 토픽은 /arm/camera/... 그대로, TF 프레임만 arm_camera_link /
#    arm_camera_color_optical_frame 로 발행된다(구동부 D455의 camera_* 프레임과
#    충돌 방지). 팔 URDF의 base_link도 arm_base_link로 바뀜(구동부 odom->base_link와
#    충돌 방지). 상세는 realsense_bringup.launch.py 상단 주석 참고.
ros2 launch army_manipulator_bringup realsense_bringup.launch.py \
  serial_no:=_243322074693 \
  camera_namespace:=arm \
  camera_name:=camera

# 2. 로봇팔 제어: 시작 즉시 grasp_wait로 이동 + 그리퍼 open (2026-09-05부터)
ros2 launch army_manipulator_bringup ik_node_bringup.launch.py \
  use_mock_hardware:=false
```

매니퓰레이터 확인용 명령:

```bash
ros2 control list_controllers
ros2 topic echo /arm/target_point
ros2 topic echo /arm/target_point_base
ros2 topic echo /joint_states
ros2 topic echo /arm/calculation_failed
ros2 topic echo /arm/picking_command
```

`/arm/target_point`는 외부 인지 노드가 발행하는 입력이고,
`/arm/picking_command`와 `/arm/calculation_failed`는 `ik_node.py`의 출력이다.
정지 완료 handshake 및 구동부 실행 방법은 아래 여름 구간 주행 노드 절에서
별도로 다룬다.


### 여름 구간 주행 노드 (summer_supply_drive_node) — 로봇팔 연동 정리

박스는 시작부터 팔 가동범위 안에 배치한다. 전진 신호와 2cm 접근 동작은 제거했다.

1. `summer_arm`의 IK 노드는 컨트롤러 준비 후 그리퍼를 열고 `grasp_wait`로 이동한다.
   준비자세 도착 전에는 검출 목표를 수락하지 않는다.
2. `vision_summer`는 유효 depth와 `base_actuator` 기준 `|X| <= 0.32m`를
   3프레임 연속 확인하면 `/arm/target_point`를 발행한다. 범위 밖이면 정지 상태로 대기한다.
3. IK 목표 Z는 -0.3080m(보정 -5.5mm)이다. 목표 이동 후 2초 대기하고 그리퍼를 닫는다.
4. 파지 성공 후 `hold` → `home`으로 이동하고 `/arm/picking_command`를 발행한 뒤 `bed`로 이동한다.
5. `summer_drive`는 파지 완료 전 `WAIT_PICK`에서 정지한다. 완료 신호 후 `cruise_vx=0.3927m/s`로
   주행하며 `/path`로 조향한다. 신호등 정지는 항상 우선한다.

**확인하면서 눈에 띈 점**

- 현재 파지 성공 판정은 그리퍼 닫기 trajectory 완료 기준이다. 실제 물체 접촉이나
  팔의 목표 도달 오차를 확인하지 않으므로 빈 파지도 성공 처리될 수 있다.
- 파지 완료 후 인지 모델은 종료되고 IK 노드는 새 좌표를 무시한다.
- `state_entered_time_`는 (아마 이전 재시도 루프 삭제 후) 갱신만 되고 어디서도
  읽히지 않는 죽은 변수다(`summer_supply_drive_node.cpp:137,233,263`).
- `utils/terminal_launcher/README.md`는 `Summer Drive`/`Mission Summer
  Perception`이라는 이름을 쓰는데, 실제 `commands.csv` 행 이름은
  `summer_drive`/`vision_summer`다.
- (해결) `commands.csv`의 `summer_arm` 행이 `control_bringup.launch.py`
  (`maru_ik_node.py` 경로)를 실행하던 걸 `ik_node_bringup.launch.py`
  (`ik_node.py`)로 맞췄다 — 위 "여름 자율 구동 매니퓰레이터" 절과 이제 일치.
  `ik_node.py`는 시작 시 자체적으로 GRASP_WAIT 자세로 들어가므로
  `/drive/supplybox_detected` 기반 프리포지셔닝이 원래 필요 없다.

**팔 치수 및 좌표 기준**

- 실제 핑거 부품 길이는 80mm다.
- 피니언 기어 원점에서 핑거 끝 TCP까지의 거리는 90mm이며, URDF의
  `pinion_gear -> tcp_link`도 `0.090m`다. IK의 wrist→TCP 보정은 링크 150mm와
  이 90mm를 합친 240mm를 사용하므로 핑거 80mm를 목표 Z에 다시 더하지 않는다.
- `base_actuator` 원점은 world 지면보다 350mm 높고, 95mm 박스 측면 중앙은
  지면보다 47.5mm 높다. 따라서 `base_actuator` 기준 TCP Z는 `-302.5mm`이고,
  여름 launch 기본 보정 `-5.5mm`(경사 아래쪽 박스를 위해 기존보다 2mm 하강)를 더한 최종
  목표는 `-308.0mm`다.
  새 높이는 반경 160~315mm, Y=-30/0/+30mm의 468개 조합에서 수치 IK 해를 확인했다.
  MoveIt 충돌검사와 실기 도달은 아직 확인하지 않았다.
- 이전 검증 높이 Z=-306.0mm에서
  중심선 기준 IK 가능 수평 반경은 약 157~436mm이며, 운용
  범위 반경 158~315mm, Y=±30mm는 거리별 seed로 모두 1회 시도에 풀린다
  (mock MoveIt, 충돌검사 on, timeout 0.5초 기준).
- 검출 Z는 IK 입력으로 사용하지 않지만 고정 Z와 50mm 이상 다르면 카메라
  TF/캘리브레이션 점검 경고를 출력한다.

---

# Tool Manipulator — 실기 제어·툴 교체 실행 절차

이 절은 팔과 툴 교체만 다룬다. `can_drive`와 `can_arm`은 다른 CAN 인터페이스다. Jetson은 모터·카메라를, 로컬 PC는 RViz·상태 감시를 담당한다. 두 PC는 같은 `ROS_DOMAIN_ID`와 `ROS_LOCALHOST_ONLY=0`을 사용한다.

> 안전 조건
>
> - `real_tool_change.launch.py`는 `hardware.yaml`, `tools.yaml`, `docking.yaml` 전체를 먼저 검사한다. 누락·`null`·음수 placeholder·형식 오류가 하나라도 있으면 controller manager와 모터 포트를 시작하지 않는다.
> - 실기 control 실행 중에는 `rmd_joint_state_bridge`, Dynamixel Wizard처럼 `can_arm` 또는 `/dev/ttyUSB0`을 여는 프로그램을 함께 실행하지 않는다.
> - 자율 tool/tag ID는 `0 = gripper`, `1 = drill`만 사용한다. ID 2 이상은 비전·도킹 대상에서 제외하며 `99`는 NO_TOOL 상태다.

## 단일 설정 원본과 실측 방법

값은 아래 세 파일에만 입력한다. 같은 값을 launch 명령에 중복 입력하지 않는다.

| 설정 경로 | 단위·좌표계 | 실측 방법 |
| --- | --- | --- |
| `hardware.yaml:joints.<joint>.zero_raw`, `sign`, `q_offset_rad` | raw, 부호, rad | 읽기 전용 bridge/Wizard에서 기계 영점 자세를 기록하고 ROS 관절 0 rad와 맞춘다. |
| `hardware.yaml:joints.<joint>.soft_limit_*`, 속도·전류·profile | raw 또는 rad, rad/s, mA/A | 저속 단일축 bench에서 기계 간섭 전 여유 위치와 허용 register 값을 기록한다. |
| `tools.yaml:tools.<id>.tag_id`, `coarse_joint_goal` | 정수, 6개 rad | 랙 태그를 읽고 `joint_names` 순서로 접근 자세를 티칭한다. |
| `tools.yaml:fixtures`, `collision.primitives`, `dock_fixture` | m, 해당 `frame` | 랙·tool 외곽을 측정해 box/cylinder와 실제 TF frame으로 등록한다. |
| `tools.yaml:tools.<id>.lock.attach_yaw_delta_rad` | 현재 yaw에서의 signed rad | 저속 체결 bench에서 시작/끝 joint state 차이를 기록한다. |
| `tools.yaml:tools.<id>.docking.{descent,retreat}_*` | `frame` 기준 단위 벡터, m, m/s, s | fixture 축을 기준으로 직선 하강·후퇴 거리와 속도·timeout을 티칭한다. |

tool case 1과 case 2(+Y)·case 3(-Y)는 모두 `arm_world`에서 world Y축 기준 `+55°` 기울어진다. 도킹 면은 case의 local YZ 면(outer local X=`±18.4 mm`)이며 TCP의 local XY 면과 일치해야 한다. 따라서 도킹 target에서 TCP `+Z`를 선택한 case face의 local `±X` 법선에 맞추고, case 1·2·3 모두 EE-facing local `+X` 면이 entry로 확정됐다. TCP XY는 해당 local +X YZ 면에 맞추며, 접근/후퇴 부호는 `tools.yaml`에 입력한다.
| `docking.yaml:vision_apriltag.*` | ROS topic, cm | 카메라의 실제 토픽과 인쇄한 tag 한 변 길이를 기록한다. |
| `docking.yaml:tag_pose_filter.*` | camera `frame_id`, m, s, px | 정상 검출 로그에서 frame, 지연, margin, PnP/depth 오차·범위를 정한다. |
| `docking.yaml:visual_servo_node.*` | expected tag frame의 m, m/s | docking 목표 tag 위치와 수렴 이득·허용오차를 저속 bench에서 정한다. |
| `docking.yaml:docking_motion_executor.*_joint_goal` | `joint_names` 순서의 6개 rad | `docking_wait`와 mission wait을 실제 팔로 티칭한다. ROS YAML 배열은 `0`과 `0.1`을 섞지 말고 모두 `0.0`처럼 float로 쓴다. |

`tools.yaml.real_docking.enabled_tool_ids`에는 모든 항목을 채운 tool만 넣고 그 tool의 `enabled: true`로 바꾼다. `physical_detach_supported`는 현재 반드시 `false`다. 물리 해제 절차는 구현되지 않았으므로 `Dock.mode=1`은 거부되며 scene detach를 성공으로 처리하지 않는다.

## 현재 attach 알고리즘

```text
/dock (mode=0, tool_id)
 -> DOCKING_WAIT -> tool별 COARSE 접근 자세
 -> tag_pose_filter: tag ID / timestamp / margin / aligned-depth 통과
 -> visual_servo_node: XY 수렴
 -> tool별 frame·direction·distance로 직선 DESCEND
 -> MoveIt wrist_yaw_joint의 attach_yaw_delta_rad 궤적
 -> /wrist_yaw_rotation_complete = true  (회전 action 성공 한 번만)
 -> tool별 frame·direction·distance로 직선 RETREAT
 -> /docking_complete = true             (후퇴 action 성공 뒤 한 번만)
 -> tool_scene_manager: attached collision + TCP/active_tool 갱신
 -> 설정된 wait / mission pose
```

`/wrist_yaw_rotation_complete`는 **yaw 궤적 성공만** 뜻한다. 체결 센서 검증이나 Planning Scene 부착 완료가 아니다. 이 설계는 별도 `/engagement_ok` 센서 판단을 사용하지 않으므로 실제 물리 체결을 독립적으로 보증하지 않는다.

회전 실패·timeout, tag/depth/filter 실패, 하강/후퇴 값 미설정, 후퇴 실패, cancel, hardware fault에서는 이후 이동과 두 완료 토픽을 모두 발행하지 않고 hold한다. 실패 중 fixture 안에 들어간 뒤에는 자동 후퇴 fallback도 하지 않는다. 통신/stale/limit fault는 모션 금지 hold이며, scene 갱신 실패 또는 물리 상태 불명은 `active_tool_id=-2 (UNKNOWN)`으로 격리한다. 실물이 비어 있음이 확인된 경우만 다음 service로 `99 (NO_TOOL)`로 복구한다.

```bash
ros2 service call /tool_scene_manager/operator_confirm_empty std_srvs/srv/Trigger "{}"
```

## Jetson J1 — 인터페이스 준비

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99       # 팀의 실제 공통 값
export ROS_LOCALHOST_ONLY=0
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm
ip -details link show can_arm
ls -l /dev/ttyUSB0
```

## 선택: 차량·회로박스·tool case RViz 확인

실기 control 또는 다른 `robot_state_publisher`와 동시에 실행하지 않는다. `world` fixed frame에서 차량 IMU 중심, `arm_world`, 대칭 tool case 두 개를 확인한다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_description display.launch.py use_mesh:=true tool_id:=0
```

## Jetson J2 — 포트를 열지 않는 사전검사

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run tool_manipulator_bringup config_validator.py \
  --hardware src/arm/tool_manipulator_bringup/config/hardware.yaml \
  --tools src/arm/tool_manipulator_bringup/config/tools.yaml \
  --docking src/arm/tool_manipulator_bringup/config/docking.yaml
```

`CONFIG READY`가 아니면 출력된 YAML 경로를 모두 채운 뒤 다시 검사한다. 이 명령은 하드웨어 포트·토크를 열지 않는다.

## Jetson J3/J4 — 카메라와 기존 AprilTag 노드

```bash
# J3: 기존 camera launch
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision cameras.launch.py

# J4: 기존 vision.apriltag를 docking.yaml 단일 원본으로 실행
ros2 launch tool_manipulator_bringup arm_tag_docking_vision.launch.py
```

J4는 `/arm/camera/color/image_raw/compressed`, `/arm/camera/color/camera_info`를 구독하고 `/arm/apriltag/centers`를 발행한다. `tag_size_cm`은 J2가 검사한 `docking.yaml.vision_apriltag.ros__parameters.tag_size_cm` 하나만 사용한다. `allowed_tag_ids: [0, 1]`에 따라 gripper와 drill tag만 발행하고 ID 2 이상은 무시한다. `tag_pose_filter`는 추가로 `/arm/camera/aligned_depth_to_color/image_raw`를 구독한다.

## Jetson J5 — 실제 팔과 도킹 stack

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tool_manipulator_bringup real_tool_change.launch.py
```

실기 launch는 preflight 성공 후에만 real `ros2_control`, joint state broadcaster, 6축 `arm_controller`, `ee_controller`, MoveIt, Servo, filter, docking manager, scene manager를 기동한다. 시작 상태는 UNKNOWN이다. 실제로 tool이 비어 있을 때만 앞의 `operator_confirm_empty` recovery를 실행한다.

## Local L1/L2 — RViz와 상태 감시

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo base_actuator tcp_link
ros2 control list_controllers
ros2 topic echo /tag_status
ros2 topic echo /servo_status
ros2 topic echo /docking_motion_status
ros2 topic echo /docking_status
ros2 topic echo /wrist_yaw_rotation_complete
ros2 topic echo /docking_complete
ros2 topic echo /tool_attachment_status
ros2 topic echo /active_tool_id
ros2 topic echo /control/hardware_fault
```

## Local L3 — UI 또는 수동 attach 요청

```bash
# tool 1 attach. mode=1 detach는 현재 명시적으로 거부된다.
ros2 action send_goal /dock tool_manipulator_bringup/action/Dock \
  "{tool_id: 1, mode: 0}" --feedback

# 취소: 현재 단계에서 hold, 이후 완료 토픽 없음
ros2 topic pub --once /tool_change/cancel std_msgs/msg/Bool "{data: true}"
```

## 인터페이스 계약

| 인터페이스 | 제공 | 사용 | 의미 |
| --- | --- | --- | --- |
| `/arm/apriltag/centers` | 기존 `vision.apriltag` | `tag_pose_filter` | 원본 tag JSON |
| `/tag_pose_valid` | `tag_pose_filter` | visual servo, executor | tag/depth gate를 통과한 JSON |
| `/servo_node/delta_twist_cmds` | `visual_servo_node` | MoveIt Servo | XY/직선 이동 Twist |
| `/arm_controller/follow_joint_trajectory` | MoveIt | `arm_controller` | 6축 approach·yaw·wait trajectory |
| `/wrist_yaw_rotation_complete` | motion executor | UI/로그 | yaw trajectory 성공 이벤트, scene 미반영 |
| `/docking_complete` | motion executor | `tool_scene_manager` | retreat 성공 이벤트; 이것만 scene attach trigger |
| `/tool_attachment_status`, `/active_tool_id` | scene manager | UI, manager | scene 갱신 결과와 attached tool |
| `/control/hardware_fault` | hardware interface | executor/UI | fault 시 hold; 이후 도킹 단계 금지 |

`apriltag_tool_docking.launch.py`는 FakeSystem 소프트웨어 검증용이다. 실기 모터 제어에는 사용하지 않는다.


## RMD 3축 영점·리밋 측정 (읽기 전용)

먼저 `rmd_joint_state_bridge`만 실행한다. 이 조합은 CAN getter만 사용하며
위치·속도·토크 명령을 보내지 않는다.

```bash
# 터미널 A: CAN 및 read-only bridge
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py

# 터미널 B: 캘리브레이션 캡처
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 run tool_manipulator_bringup capture_rmd_calibration.py
```

캡처 터미널에서 축 하나씩 다음 순서로 입력한다.

```text
snap
# 해당 축을 URDF 0 자세로 수동 배치
zero shoulder +              # 실제 raw 증가가 ROS 양의 방향이면 +, 반대면 -
# 안전 최소 위치로 수동 배치
limit shoulder min
# 안전 최대 위치로 수동 배치
limit shoulder max
show
```

`show`가 출력한 `sign`, `q_offset_rad`, `soft_limit_rad`만
`hardware.yaml`에 복사한다. RMD 속도·전류·후퇴 보호값은 이 읽기 전용
스크립트가 추정하지 않으며 별도 안전 시험 결과로 입력해야 한다.
