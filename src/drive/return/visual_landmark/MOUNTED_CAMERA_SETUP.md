# 장착 카메라 관찰 시험 준비 (2026-10-26)

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
