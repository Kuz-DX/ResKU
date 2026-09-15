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

### 자율 제어

**[하림 수정] drive 쪽은 더 이상 스텁이 아님** — `drive_auto` 패키지는 삭제됐고,
doldrive_ws에서 이관한 실제 파이프라인(`src/drive/autonomous/`)으로 완전히
대체됨. 조이스틱 없이 터미널만으로 실행 가능:

```bash
ros2 launch robot_bringup autonomous.launch.py
```

외부에서 공급되는 `/path`와 경사 신호를 받아 `path_relay`와
`slope_traverse_node`, MPPI(`nav2_mppi_controller`)를 거쳐 실제 CAN 모터
명령까지 이어진다. 세부 흐름은 `src/drive/autonomous/robot_bringup/launch/`
안 각 launch 파일의 설명을 참고한다.

# 메뉴얼 주행

### CAN_drive 인터페이스 활성화
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
ip -details link show can_drive

### [하림 수정] 실행 — 한 줄로 끝남 (joy_node+manual_joy_control_node+can_driver_node+
### myahrs_driver_node+manual_stability_node 전부 포함, IMU pitch/roll 25°/20° 넘으면
### 자동 긴급정지)
source install/setup.bash
ros2 launch can_driver manual.launch.py

### 예전 방식(터미널 2개로 따로 띄우기, 참고용 — 위 한 줄로 대체됨, IMU 안전장치 없음)
# source install/setup.bash
# ros2 run can_driver can_driver_node --ros-args -p can_channel:=can_drive
# source install/setup.bash
# ros2 launch manual_joy_control manual_control.launch.py

## 확인용 (선택)

### 터미널 3 - 조이스틱 raw 입력 확인
source ~/dolbotZ/install/setup.bash
ros2 topic echo /joy
 

### 터미널 4 - 최종 모터 속도 명령 확인
source ~/dolbotZ/install/setup.bash
ros2 topic echo /motor_speed_cmd

### 터미널 5 - drive/arm 중 조이스틱이 지금 어디에 반응 중인지 확인
source ~/dolbotZ/install/setup.bash
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
#
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py launch_joy:=false
# drive 쪽 manual_control.launch.py에서 joy_node를 이미 띄웠으므로 launch_joy:=false로
# 중복 실행 방지 (drive를 안 띄우고 arm만 단독으로 쓸 거면 이 인자 빼면 됨)
-> 선주야 ros2 launch robot_arm_bringup robot_arm_bringup.launch.py까지만 치면 됨.

### 실기 팔 RViz 미러링 (실측 각도/엔코더, 읽기 전용 - army_manipulator_description 내장)
# ~/arm_config/rmd_joint_state_bridge.py(외부 스크립트)와 동일한 역할을 하는
# 내장 노드(rmd_joint_state_bridge 패키지) - ros2_control 없이 RMD(CAN)/
# Dynamixel(TTL) 엔코더를 getter로만 읽어서 /joint_states + 진단 토픽으로
# 발행한다. 포지션 커맨드를 절대 안 보내므로 팔을 손으로 자유롭게 움직이며
# 자세(홈/스탠바이 등)를 캡처할 때 안전하게 쓸 수 있다.

# 터미널 1 - RViz (URDF만, MoveIt 없이 가볍게)
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
ros2 launch army_manipulator_description display.launch.py

# 터미널 2 - CAN 인터페이스 켜기 (위 "CAN 인터페이스 활성화 can_arm" 참고, 이미 켰으면 생략)
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm

# 터미널 3 - 브릿지 실행 (기본 파라미터 = army_manipulator_ros2_control.xacro 기본값과 동일.
# 실기 캘리브레이션(sign/q_offset, position_direction/position_zero_offset)이 다르면
# 아래처럼 launch argument로 덮어쓸 것 - 값은 army_manipulator_ros2_control.xacro의
# shoulder_sign/elbow_sign/wrist_sign, shoulder_q_offset/elbow_q_offset/wrist_q_offset과
# 그대로 맞추면 됨)
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
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
# 사이드캠 Logitech C920 -> 네임스페이스 leftview  (/leftview/...)
# 사이드캠 Logitech C922 -> 네임스페이스 rightview (/rightview/...)

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


# 사이드캠 (Logitech C920 -> leftview, C922 -> rightview, usb_cam 패키지)
# video_device 경로는 `ls /dev/video*` 또는 `v4l2-ctl --list-devices`로
# 실제 장치를 확인해서 채울 것 — 재부팅/재연결 시 번호가 바뀔 수 있으므로
# /dev/v4l/by-id/... 의 고정 심볼릭 경로를 쓰는 게 더 안전함.
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/leftview \
  -p video_device:=<TODO: C920 장치 경로, 예: /dev/video0> \
  -p camera_name:=leftview

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/rightview \
  -p video_device:=<TODO: C922 장치 경로, 예: /dev/video2> \
  -p camera_name:=rightview


  ros2 bag record -o ~/arm_pickup_bag_all \
  /leftview/image_raw/compressed \
  /rightview/image_raw/compressed \
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
cd ~/dolbotZ
colcon build --packages-select \
  army_manipulator_bringup army_manipulator_description
source install/setup.bash
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
