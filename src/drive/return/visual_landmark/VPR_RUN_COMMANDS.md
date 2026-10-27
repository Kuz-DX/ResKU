# Back-to-Base 주행 + VPR 실행 명령 (2026-10-27)

CAN 없는 RViz 가상 영상 검증: [VPR_VIRTUAL_TEST.md](../manual_return_sim/VPR_VIRTUAL_TEST.md).

팔 IK/Manual + 원격 조이스틱은 1-A 절부터 실행한다.
Auto 재개 없이 정지 MANUAL에서 VPR만 시험: [STATIONARY_VPR_DEBUG.md](STATIONARY_VPR_DEBUG.md). 기본 OFF인 임시 debug 기능.

로봇 PC에서 카메라·주행·기록을 실행하고 노트북에서 조이스틱·뷰어를 실행한다.
워크스페이스를 각 PC에 최신 상태로 준비한다. 아래 ~/ResKU는 실제 경로로 바꾼다.
ROS_DOMAIN_ID는 양쪽 모두 99로 예시를 통일했다. 기존 로봇이 다른 도메인을 쓰면
모든 터미널에 그 값을 동일하게 지정한다. RMW도 양쪽에서 동일하게 사용한다.

## 0. 공통: 새 터미널마다

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=0
```

처음 한 번, 변경된 코드 빌드(각 PC):

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup vision visual_landmark manual_joy_control tool_manipulator_bringup
source install/setup.bash
```

## 1. 로봇 PC: 장치 확인·CAN

```bash
ip -details link show can_drive
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-id/
ros2 pkg prefix realsense2_camera
rs-enumerate-devices
```

can_drive가 DOWN이고 기존 드라이버가 실행 중이지 않을 때:

```bash
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set can_drive up
```

IMU는 launch 기본 /dev/ttyACM0이다. 실제 장치가 다르면 3번 명령에
`imu_port:=/dev/serial/by-id/실제장치`를 추가한다.

## 1-A. 로봇 PC: 팔 IK/Manual 제어와 카메라 LOCK

주행 전에 팔 제어를 켜고 전방 카메라를 사용할 LOCK 자세로 고정한다.
현재 저장소의 원격 조이스틱 launch 이름은 `remote_joy.launch.py`다.
사용자가 부른 `remote.launch.py` 파일은 현재 main에는 없다.

팔 하드웨어 기본 설정은 CAN `can_arm`, Dynamixel `/dev/ttyUSB0`이다.
실제 장치/보정된 hardware.yaml을 확인한다. can_arm이 DOWN이고 기존 팔 driver가 없으면:

```bash
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set can_arm up
```

로봇 PC의 별도 터미널에서:

```bash
ros2 launch vision vpr_lock_arm_control.launch.py \
  real_hardware:=true launch_joy:=false
```

별도 실기 설정 파일을 쓰면 `hardware_config:=/absolute/path/hardware.yaml`을 추가한다.
이 launch는 실기 팔 제어를 실행한다. 기존 real_control/manual/tool1_attach 등
다른 팔 controller bringup은 동시에 실행하지 않는다.
terminal_launcher의 기존 `Arm_TF`(real_control)와 `Arm_Launch`도 이 구성에서는 실행하지 않는다.

노트북(실제 조이스틱이 연결된 PC)의 별도 터미널에서:

```bash
ros2 launch tool_manipulator_bringup remote_joy.launch.py
```

기존 팔 조작 방식으로 전방 카메라를 LOCK 자세에 놓고 그 자세를 유지한 뒤
Record/복귀 시험을 시작한다. launch 자체가 특정 LOCK 자세로 자동 이동한다는 뜻은 아니다.
이 `/joy`를 주행도 공유하므로 아래 주행 launch의 joy_node는 반드시 끈다.
이 wrapper는 팔 모델 TF를 `/arm_model/tf*`로 분리한다. 차량 TF와 팔 모델의 동명 base_link를 섞지 않는다.
전방 VPR TF는 다음 절의 별도 LOCK static publisher가 담당한다.

## 2. 로봇 PC 터미널 A: 전후방 카메라

실제 전방은 D435i serial 339222071362이다. 아래는 **각각 별도 터미널**에서 실행하고 유지한다.
같은 장치의 기존 driver/TF publisher와 중복 실행하지 않는다.

```bash
ros2 launch vision arm_camera.launch.py serial_no:=_339222071362
```

```bash
ros2 launch vision rear_vpr_trial.launch.py
```

```bash
ros2 launch vision front_d435i_vpr_tf.launch.py
```

마지막 명령은 사용자 확인 LOCK 자세에서만 유효한 시험용 `base_link -> arm_camera_cam_link`이다.
위치 (0.173603, 0, 0.650674)m, RPY=(0,0,0), y에 임의 가로 보정은 더하지 않는다.
RGB optical 내부 TF는 driver가 담당한다. 팔 전체 URDF TF는 차량 `/tf*`에 발행하지 않는다.
팔 제어가 필요하면 1번 wrapper만 사용하며, 팔이 움직이면 VPR을 중단한다.
측정 기준·불확실성·명령 전체: [FRONT_D435I_LOCK_TF.md](FRONT_D435I_LOCK_TF.md).

## 3. 로봇 PC 터미널 B: Back-to-Base + VPR

이 명령은 모터/CAN·IMU·odom·mux·경로 기록·복귀·VPR을 시작한다.
기존 같은 launch는 Ctrl+C로 종료한 뒤 실행한다. 별도로 reduced_odom_bringup,
can_driver manual.launch, autonomous.launch를 동시에 실행하지 않는다.

### 첫 시험: 추정 관찰, 실제 보정 OFF

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  can_interface:=can_drive imu_port:=/dev/ttyACM0 \
  reverse_return:=false \
  enable_auto_keyframes:=true enable_visual_localization:=true \
  enable_adaptive_visual_check:=true \
  visual_check_on_return_start:=true visual_check_on_auto_resume:=true \
  visual_debug_matches:=true \
  visual_apply_correction:=false enable_landmark_recovery:=false
```

시작 직후 gyro 초기화를 위해 최소 약 2.5초 정지하고 odom을 확인한다.
전방 팔을 LOCK 자세로 유지한다. 관찰 모드에서 LOCK 파일이 없으면 첫 처리 TF를
기준으로 삼는다. 복귀는 180도 회전 후 전진으로 설정했다(전방캠이 기록 장면을 바라봄).

### 실제 VPR 보정 ON 시험

관찰 결과·실제 이동량·TF/LOCK을 확인한 뒤 위 명령의 마지막 줄을 다음으로
바꿔 재시작한다. 파일 경로는 측정된 D435i LOCK 4×4 행렬 YAML이어야 한다.

```bash
  visual_apply_correction:=true enable_landmark_recovery:=false \
  visual_expected_lock_pose_file:=/absolute/path/d455_lock.yaml
```

재시작 후에는 새 Record로 reference를 다시 수집한다.
ON은 보정 추정치를 controller 좌표에 적용한다. 자동 yaw 검색은 계속 꺼 둔다.

## 4. 노트북 터미널 C/D: 조이스틱·시각화

각 터미널에서 0번 환경을 적용한다.

```bash
# C: 위 remote_joy의 /joy를 공유
ros2 launch manual_joy_control manual_control.launch.py launch_joy:=false
```

위 구성에서는 remote_joy만 joy_node를 실행한다. remote_joy를 사용하지 않고
주행만 실행하는 경우에만 launch_joy:=true로 한다. 조이스틱 발행자를 이중 실행하지 않는다.

```bash
# D: 전후방 영상 + 실제 특징점 매칭 + 검증 결과
ros2 run visual_landmark vpr_viewer
```

로봇 PC에 화면이 있으면 뷰어를 그 PC에서 실행해도 된다.
청색 점=현재 SIFT, 황색 선=mutual ratio 매칭, 녹색 선=PnP inlier.
Matches는 검사 중에만 갱신되며 마지막 후보일 수 있다.
`validated`=두 프레임 등 검증 통과, `localized`와 correction_applied=true=실제 보정 발행.
Q는 뷰어만 닫는다.

선택: 기존 주행 경로 RViz, 각각 별도 터미널:

```bash
ros2 run manual_return_sim sim_debug_viz
```

```bash
rviz2 -d install/manual_return_sim/share/manual_return_sim/config/sim_manual_return.rviz
```

이 선택 기능은 manual_return_sim 패키지가 설치돼 있어야 한다.

## 5. 로봇 PC 터미널 E: 시작 전 검사

```bash
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_trial_preflight.json
ros2 node list
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```

마지막 tf2_echo는 확인 후 Ctrl+C. preflight의 ready_for_observation=true와
영상/CameraInfo/depth/TF를 확인한다. false이면 JSON reason을 해결한 뒤 Record한다.
노트북에서도 ros2 topic list로 같은 토픽이 보이는지 확인한다.

## 6. 선택: 로봇 PC 터미널 F에서 시험 rosbag

rosbag은 선택 사항이다. 생략해도 경로 Record·VPR·시각화·복귀는 동작한다.
문제 발생 후 영상/TF/결과를 재확인하려면 기록한다.
사용할 때만 Record 버튼보다 먼저 시작한다. 종료는 Ctrl+C.

첫 줄의 `$(date +%Y%m%d_%H%M%S)`는 현재 시각을 저장 폴더 이름에 넣는
정상적인 Bash 구문이다. 예: `vpr_trial_20261027_143025`.
`date` 뒤 공백과 `+`를 유지한다. 고정 이름을 원하면
`ros2 bag record -o vpr_trial_01`로 바꾸되 기존 폴더와 겹치지 않게 한다.

```bash
ros2 bag record -o "vpr_trial_$(date +%Y%m%d_%H%M%S)" \
 /mission/return/state /mission/origin_pose /recorded_path /return_path \
 /mission/return/control_mode /path/record /path/return \
 /wheel/odom /odometry/filtered /odometry/diagnostics /imu/gyro/z_corrected \
 /visual_landmark/event /visual_landmark/check_request \
 /visual_landmark/scheduler_event /visual_landmark/localization_event \
 /visual_landmark/route_progress /visual_landmark/correction \
 /visual_landmark/debug/matches/compressed \
 /rear/camera/image_raw/compressed /rear/camera/camera_info \
 /arm/camera/color/image_raw/compressed /arm/camera/color/camera_info \
 /arm/camera/aligned_depth_to_color/image_raw /tf /tf_static /joint_states \
 /cmd_vel_return_path /cmd_vel
```

## 7. 실제 조작 순서

기존 UI/조이스틱으로 Record → 수동 출발 → Return → Manual → 수동 이동 → Auto.
PS(10)=Record, Share(8)=Return은 기존 주행 조이스틱 매핑이다.
명령행으로 트리거하려면 아래를 단계에 맞춰 **하나씩** 실행한다.
이 명령은 실제 미션/주행 상태를 바꾼다. UI와 동시에 반복 발행하지 않는다.

```bash
# 새 Record (false→true 상승 에지)
ros2 topic pub --once /path/record std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /path/record std_msgs/msg/Bool '{data: true}'
```

출발 후 reference_loaded를 확인한다. 복귀할 준비가 되면:

```bash
ros2 topic pub --once /path/return std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /path/return std_msgs/msg/Bool '{data: true}'
```

FOLLOW_RETURN_PATH에서 수동 전환:

```bash
ros2 topic pub --once /mission/return/control_mode std_msgs/msg/String '{data: MANUAL}'
```

수동 이동 후 Auto 재개:

```bash
ros2 topic pub --once /mission/return/control_mode std_msgs/msg/String '{data: AUTO}'
```

FOLLOW_RETURN_PATH 재진입 후 auto_resume 검사 요청이 나와야 한다.
진행 중 요청은 하나이며 retry cooldown 때문에 재개 직후 잠시 기다릴 수 있다.
시험 중 Record를 다시 누르지 않는다. 정상 odom에 반영된 수동 이동은 drift가
아니므로 보정량이 작을 수 있다. 실제 바닥 위치 기준과 VPR 추정값을 비교한다.

진단(각 명령 별도 터미널):

```bash
ros2 topic echo /mission/return/state
ros2 topic echo /visual_landmark/scheduler_event
ros2 topic echo /visual_landmark/localization_event
ros2 topic echo /visual_landmark/correction
```

OFF에서는 correction 토픽에 결과가 발행되지 않는 것이 정상이다.

## 8. 종료

긴급 종료(현재 구현은 0 명령 후 전체 manual_return launch 종료):

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
```

일반 종료는 먼저 주행을 멈추고 주행 launch Ctrl+C, bag Ctrl+C,
카메라 launch Ctrl+C, 조이스틱/뷰어 종료 순서로 한다.
