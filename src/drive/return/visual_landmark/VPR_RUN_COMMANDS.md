# Back-to-Base 주행 + VPR 실행 명령 (2026-10-27)

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
colcon build --packages-up-to robot_bringup vision visual_landmark manual_joy_control
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

## 2. 로봇 PC 터미널 A: 전후방 카메라

D455 시리얼을 실제 값으로 바꾼다. 밑줄 접두사를 유지한다.
다른 launch의 같은 카메라 driver는 먼저 종료한다.

```bash
ros2 launch vision vpr_observation.launch.py \
  rear_launch:=rear_vpr_trial.launch.py \
  start_arm:=true arm_serial_no:=_ACTUAL_D455_SERIAL \
  start_visual_nodes:=false start_viewer:=false
```

후방 장치 경로가 다르면 `device:=/dev/v4l/by-id/실제장치`를 추가한다.
이 명령은 전방 color+aligned depth, 후방 영상+시험 intrinsic+초기 optical TF를
시작한다. 주행/VPR 노드는 3번에서 시작하므로 start_visual_nodes=false를 유지한다.
후방 TF는 27mm/약 3도 하향/roll=0의 근사다.

**별도 필수 입력:** 실제 차체→팔 장착→관절→전방 color optical TF와 joint_states.
전방 RealSense driver는 카메라 내부 TF만 제공한다. 기존 검증된 팔 bringup을
실행하고 팔을 LOCK 자세에 둔다. 차체/팔 base_link 이름 충돌을 해결한 실제
장착 TF 실행 명령은 아직 이 저장소에서 확정되지 않았으므로 임의 0 TF로 대체하지 않는다.
이 연결이 없다면 5번 검사가 실패하며 전후방 VPR 위치 추정은 실행할 수 없다.

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
바꿔 재시작한다. 파일 경로는 측정된 D455 LOCK 4×4 행렬 YAML이어야 한다.

```bash
  visual_apply_correction:=true enable_landmark_recovery:=false \
  visual_expected_lock_pose_file:=/absolute/path/d455_lock.yaml
```

재시작 후에는 새 Record로 reference를 다시 수집한다.
ON은 보정 추정치를 controller 좌표에 적용한다. 자동 yaw 검색은 계속 꺼 둔다.

## 4. 노트북 터미널 C/D: 조이스틱·시각화

각 터미널에서 0번 환경을 적용한다.

```bash
# C: 기존 joy_node가 없다면
ros2 launch manual_joy_control manual_control.launch.py
```

팔 bringup 등이 이미 같은 /joy를 발행하면 `launch_joy:=false`를 추가한다.
조이스틱 명령 경로를 이중 실행하지 않는다.

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
