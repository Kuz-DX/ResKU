# 장착 카메라 관찰 시험 준비 (2026-10-26)

전체 구동 명령 모음: [VPR_RUN_COMMANDS.md](VPR_RUN_COMMANDS.md) (로봇/노트북별 터미널, Manual→Auto 시험, 기록·종료).

## 이번 PC에서 실제 확인한 상태

- 후방 Alcorlink USB 카메라 → `/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0`.
- 1280×720 MJPEG 15fps 단일 프레임 수신 성공. 원본: `/tmp/resku_mounted_rear.png`.
- 화면에 가까운 흰 면과 가로 봉이 크게 보인다. 원거리의 고정된 배경이 충분히 보이는지 확인한다.
- 전방 RealSense는 이 PC의 USB 목록에 없음.
- ROS_DOMAIN_ID=99에서 `/parameter_events`, `/rosout` 외 센서·odom·TF 토픽을 발견하지 못함.
- 사용자 설치로 usb_cam·compressed_image_transport·camera_calibration 설치 확인. 전방 realsense2_camera는 실행 PC에서 확인 필요.
- 따라서 현재 optical extrinsic / 양안 RGB-D VPR 준비 완료로 판정할 수 없다.

## 설치와 카메라 전용 실행

이 PC 터미널에서 필요한 패키지를 설치한다(비밀번호는 터미널에만 입력).

```bash
sudo apt-get install ros-humble-usb-cam ros-humble-compressed-image-transport ros-humble-camera-calibration
source /opt/ros/humble/setup.bash
colcon build --packages-select vision visual_landmark
source install/setup.bash
ros2 launch vision vpr_observation.launch.py
```

이 launch는 카메라·시각 노드만 시작한다. 모터·odometry·팔 controller는 기동하지 않는다.
후방은 저장된 시험 intrinsic(`apc850_rear_preview_trial_1280x720.yaml`)을
명시적으로 읽고 1280×720@15로 실행한다. trial 검증 RMS 0.783px, 최대 4.980px이므로
최종 보정 품질이 확정된 것은 아니다. optical static TF는 자동 발행하지 않는다.
VPR correction은 false, slip early trigger는 false로 설정된다.
원점/기록 상태/TF/return progress가 없으면 VPR은 실제 작업을 시작하지 않는다.

전방은 D455로 교체 예정. 전방 D455가 **이 PC에 실제 연결돼 있고** realsense2_camera가 설치돼 있다면:

```bash
ros2 launch vision vpr_observation.launch.py start_arm:=true arm_serial_no:=_ACTUAL_D455_SERIAL
```

전방 driver가 로봇 PC에 이미 실행돼 있으면 start_arm=false를 유지한다.
기존 robot_bringup이 시각 노드를 실행 중이라면 중복 실행하지 않는다:

```bash
ros2 launch vision vpr_observation.launch.py start_visual_nodes:=false
```

후방 driver도 이미 실행 중이면 start_rear:=false로 지정한다.
카메라 driver는 각 장치당 한 프로세스만 실행한다. 로봇 PC와 이 PC의
ROS_DOMAIN_ID·ROS_LOCALHOST_ONLY·RMW/네트워크 설정을 일치시켜야 원격 토픽을 확인할 수 있다.

## 영상·내부 보정·TF를 읽기 전용으로 검사

```bash
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_preflight.json
ros2 topic echo /rear/camera/camera_info --once
ros2 run tf2_tools view_frames
```

preflight는 5개 입력(후방 RGB/CameraInfo, 전방 RGB/CameraInfo/aligned depth),
최신 시각·frame·해상도·K/D·depth encoding/size, 영상 시각의 base_link/odom→각 optical TF를
검사하고 xyz/rpy/행렬을 JSON에 저장한다. 주행·보정 명령은 발행하지 않는다.
최신 depth/color 두 샘플의 시각 차이는 실제 synchronizer pair 성공률과 다를 수 있다.
TF 존재는 실제 장착 치수의 정확성·단일 발행자·LOCK 정확성을 증명하지 않는다.
correction_enabled=null은 현재 실행 노드의 설정을 이 도구가 검사하지 않았다는 뜻이다.

전방 설정은 `vpr_front_d455.yaml`을 사용하며 color/depth와 align_depth를 활성화한다.
기존 주행용 D455 설정은 depth/align이 꺼져 있으므로 VPR 설정으로 그대로 쓰지 않는다.
전방 교체 후 D455 자체 color CameraInfo를 쓰고 기존 D435i K/D·시리얼·LOCK TF를 재사용하지 않는다.
`rear_image_intrinsic_ready`는 후방 영상/내부 보정 연결만의 결과이며 전체 VPR 준비 상태와 다르다.

## 아직 반드시 필요한 실제 측정/확인

1. 전방 D455 연결 PC와 실제 실행 driver/토픽/frame 확인.
2. 두 카메라 뒷면의 정확한 측정점과 그 점→RGB optical 원점까지의 오프셋 확보.
3. 장착 roll/pitch/yaw와 전방 측정 당시 LOCK 관절 자세 확보.
4. 차체/팔 base_link 충돌 해결 및 실제 통합 TF 트리의 단일 발행자 확인.
5. 검증된 optical TF를 기존 URDF/broadcaster에 반영하고 LOCK 4×4 기준 파일 저장.
6. 새 mission에서 자동 기록 → 보정 OFF로 VPR 검증 → known-offset 시험.

뒷면 좌표는 전방 (0.152853, 0, 0.650674)m, 후방 (-0.316627, 0, 0.650674)m로
vpr_mount_measurements.yaml에 기록돼 있다. 여기에 임의의 optical 오프셋이나 회전을 더하지 않는다.
그 외 전체 실차 검증은 CAMERA_VPR_TODO.md 및 ADAPTIVE_VISUAL.md를 따른다.

뒷면→RGB optical 오프셋은 차체 위치 보정에 필요하지만 반드시 직접 자로 측정할 필요는 없다.
정확한 기구 도면 또는 차체 기준 자세를 아는 보드/표적을 이용한 외부 보정으로 전체 TF를 추정할 수 있다.
임의로 놓은 보드 한 장으로는 차체 기준 TF가 확정되지 않는다. 지금은 후방 영상·intrinsic 확인까지만 가능하며,
실제 전후방 RGB-D VPR은 D455와 양쪽 optical TF가 준비된 뒤 진행한다.

## 설치 후 실제 실행 확인

- usb_cam의 by-id 상대 심볼릭 링크 처리 문제를 launch에서 실제 장치 경로로 resolve해 해결했다.
- 후방 드라이버 실행 및 `/rear/camera/image_raw/compressed`, `/rear/camera/camera_info` 수신 성공.
- 영상 1280×720, frame `rear_camera_optical_frame`, plumb_bob K/D가 시험 YAML과 일치했다.
- preflight `rear_image_intrinsic_ready=true`. 전체 `ready_for_observation=false`는 전방 RGB-D와 base_link/odom TF 부재 때문이다.
- 최신 읽기 전용 점검 결과: `/tmp/resku_rear_ready.json`.
- 실사용 intrinsic 품질과 extrinsic이 검증됐다는 의미는 아니다.

## 후방 실차 VPR 초기 근사 설정 (2026-10-26)

사용자 추가 실측: 후방 케이스 뒷면→렌즈 약 27mm.
`rear_vpr_trial.launch.py`는 이를 차체 정후방 오프셋으로 가정해
(-0.343627, 0, 0.650674)m를 사용한다. 렌즈 표면을 optical 원점으로
근사하며, 기존 약 3도 하향 관찰과 housing roll=0을 가정한다.
이는 검증된 extrinsic이 아니다. 실제 기울기에 따른 오프셋의 z 성분도
현재 초기값에는 반영하지 않았다. 모든 xyz/optical RPY는 launch에서 덮어쓸 수 있다.

기존 후방 driver와 rear optical TF 발행자를 먼저 종료한 뒤:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vision rear_vpr_trial.launch.py
```

차체 base_link가 다른 이름이면 `parent_frame:=ACTUAL_FRAME`을 지정한다.
이 launch는 후방 driver와 초기 optical static TF만 실행한다.
실차 manual_return_bringup과 함께 사용하되 다른 후방 driver/TF는 중복 실행하지 않는다.
기존 vpr_observation을 함께 쓸 경우 `start_rear:=false`로 지정한다.
전방 D455의 RGB/aligned depth/CameraInfo 및 차체→팔→color optical TF는
별도로 필요하다. 후방만 준비돼도 전후방 VPR 전체가 준비된 것은 아니다.
`visual_apply_correction=false`, `enable_landmark_recovery=false`를 유지하고,
이 TF로 새 mission/랜드마크를 기록한 뒤 known-offset 관찰 시험을 수행한다.

## 로봇 PC: Record → 복귀 → Manual → Auto VPR 시각화 시험

이 절차는 로봇 PC에서 실행한다. 노트북 뷰어는 로봇과 ROS_DOMAIN_ID,
RMW 및 네트워크 discovery가 맞아야 원격 토픽을 볼 수 있다.
현재 노트북에서 로봇 PC 센서/TF 수신을 확인한 상태는 아니다.

### 빌드와 센서

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vision visual_landmark return_navigation drive_cmd_mux robot_bringup
source install/setup.bash
ros2 pkg prefix realsense2_camera
rs-enumerate-devices
```

실제 D455 시리얼을 다음 ACTUAL_D455_SERIAL 자리에 넣는다. 기존 후방 및
RealSense driver가 있으면 먼저 종료하거나 start_rear/start_arm을 false로 지정한다.
기존 rear optical TF와 trial TF를 중복 발행하지 않는다.

```bash
ros2 launch vision vpr_observation.launch.py \
  rear_launch:=rear_vpr_trial.launch.py \
  start_arm:=true arm_serial_no:=_ACTUAL_D455_SERIAL \
  start_visual_nodes:=false start_viewer:=true
```

위 launch는 센서·후방 초기 TF·읽기 전용 뷰어만 켠다.
전방은 D455 자체 color CameraInfo/내부 TF를 사용한다. 차체→팔→카메라 장착
TF와 joint_states는 실제 로봇에서 별도로 제공해야 한다. D435i LOCK 파일을
D455에 재사용하지 않는다. 후방 trial TF는 27mm/약 3도/roll=0의 초기 근사다.

### 실차 주행 스택

이미 실행 중인 manual_return_bringup은 중복 실행하지 말고 아래 옵션을 포함해
재시작한다. 이 명령은 모터/주행 스택을 시작한다.

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  enable_auto_keyframes:=true enable_visual_localization:=true \
  enable_adaptive_visual_check:=true \
  visual_check_on_return_start:=true visual_check_on_auto_resume:=true \
  visual_debug_matches:=true \
  visual_apply_correction:=false enable_landmark_recovery:=false
```

실제 장착에 맞는 measured LOCK 행렬이 있으면
`visual_expected_lock_pose_file:=/absolute/path/d455_lock.yaml`도 지정한다.
관찰 모드에서 파일을 생략하면 처음 처리된 TF가 LOCK 기준으로 고정되므로,
첫 검사 전에 팔을 실제 LOCK 자세에 둔다. 이것은 측정된 extrinsic을 대신하지 않는다.

```bash
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_trial_preflight.json
```

`ready_for_observation=true`와 전후방 영상/CameraInfo/depth/TF를 확인한 뒤
새 mission의 Record를 시작한다. preflight 통과도 실제 측정 정확도의 증명은 아니다.
reference_loaded가 나오는지 확인하고 기존 UI로 복귀를 시작한다.

### 화면과 시험 순서

- Rear live / Front live: 실제 영상. 수신이 끊기면 마지막 프레임의 경과 시간이 증가한다.
- Matches: 왼쪽은 저장된 후방 reference, 오른쪽은 현재 전방 영상.
  청색 점은 현재 SIFT 특징점, 황색 선은 mutual ratio 매칭,
  녹색 선은 그 중 유효 depth 대응점의 PnP RANSAC inlier다.
  모든 후보를 기존 방식으로 검증하며, 화면은 마지막 처리 후보일 수 있다.
- VPR status: mission, capture, scheduler, localization과 마지막 결과.
  녹색 선만으로 성공이 아니다. `validated`는 두 프레임·기하 검증 통과,
  `localized`와 `correction_applied=true`는 실제 보정 발행이다.
  영상 시각화는 구독자가 있을 때만 만들며 idle SIFT/PnP는 추가하지 않는다.

1. 새 Record로 배경 특징이 충분한 구간을 주행하고 reference_loaded를 확인한다.
2. 복귀 Auto에서 첫 check_requested와 Matches/validated를 확인한다.
3. Manual로 전환하고 낮은 속도로 위치/방향을 바꾼 뒤 Auto로 재개한다.
4. FOLLOW_RETURN_PATH에 들어간 뒤 `trigger=auto_resume` 요청을 확인한다.
   fresh odom이 필요하고 진행 중 요청은 하나만 허용하며 기존 retry cooldown을 지킨다.
5. `validated`의 planar_correction, 실제 차체 위치/각도를 비교한다.
   no_match/rejected/timeout이면 직전 보정을 유지하고 ordinary return이 계속된다.
6. Manual→Auto 전환을 반복한다. 새 Record를 누르면 새 mission이 되므로
   이 시험 도중 Record를 다시 누르지 않는다.

정상적으로 odom에 반영된 수동 우회는 추정 drift가 아니다. 큰 visual 보정이
안 나오는 것이 정상일 수 있다. 바닥 기준 표시/독립 위치 측정으로 실제 오차와
추정치를 비교한다. odom이나 TF를 임의 reset해서 시험하지 않는다.
첫 단계는 apply_correction=false여서 실제 경로를 VPR로 수정하지 않는다.
관찰과 TF/LOCK 검증 후 같은 시험을 true로 반복해야 실제 보정 효과를 확인할 수 있다.

### 시험 기록

별도 터미널에서 Record 시작 전에 실행하고 종료 시 Ctrl+C로 마무리한다.
매 실행마다 새 출력 디렉터리를 사용한다.

```bash
ros2 bag record -o "adaptive_vpr_trial_$(date +%Y%m%d_%H%M%S)" \
 /mission/return/state /mission/origin_pose /return_path \
 /wheel/odom /odometry/filtered /odometry/diagnostics /imu/gyro/z_corrected \
 /visual_landmark/event /visual_landmark/check_request \
 /visual_landmark/scheduler_event /visual_landmark/localization_event \
 /visual_landmark/route_progress /visual_landmark/correction \
 /visual_landmark/debug/matches/compressed \
 /rear/camera/image_raw/compressed /rear/camera/camera_info \
 /arm/camera/color/image_raw/compressed /arm/camera/color/camera_info \
 /arm/camera/aligned_depth_to_color/image_raw /tf /tf_static /joint_states \
 /cmd_vel_return_path
```

뷰어만 별도로 켜려면 `ros2 run visual_landmark vpr_viewer`.
Q는 뷰어만 종료하며 주행을 정지시키지 않는다.
영상 저장·시각화는 CPU/디스크 부하를 늘린다. processing latency와 stale/deadline
거부가 증가하면 실제 로봇에서 영상 부하를 낮추고 다시 측정한다.
