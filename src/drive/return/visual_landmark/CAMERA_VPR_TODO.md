# 카메라 VPR 재개 TODO

2026-10-25: 카메라 장착 완료(사용자 보고). 아래 좌표는 카메라 뒷면 기준이며 optical 좌표가 아니다.
아직 extrinsic 보정, 런타임 intrinsic 연결, 실차 VPR 검증은 완료되지 않았다.

## 보관한 결과

- 후방캠: Alcorlink USB 2.0 Camera, MJPEG 1280×720, 15fps 수신 확인.
- 이 PC에서 확인한 경로: `/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0`.
  다른 PC에서는 반드시 실제 경로를 다시 확인한다. 이 식별자가 개별 장치를 구별하는지도 확인한다.
- 체커보드: 10×7칸, 내부 코너 9×6, 실측 한 칸 0.015m.
- 관찰 시험용 후보: `src/vision/camera/config/calibration/apc850_rear_preview_trial_1280x720.yaml`.
- take 5 별도 검증: RMS 0.783px, 최대 4.980px, 29/30장 검출, 10/12구역 커버리지.
  이전 후보도 동일 검증 영상에서 평가했으며 RMS 1.229px였다.
- 숫자가 작은 이전 보고서들은 검증 영상이 다르므로 단순 순위 비교하지 않는다.
- 모든 후보는 시험용이다. launch 기본값이나 정식 `apc850_rear.yaml`로 자동 연결하지 않았다.
- 위치 기록: `src/vision/camera/config/vpr_mount_measurements.yaml`.
  차체 `base_link` 기준 카메라 뒷면 전방 LOCK 위치 (0.152853, 0, 0.650674)m,
  후방 위치 (-0.316627, 0, 0.650674)m. 회전은 모두 미측정이다.
  측정점과 optical 원점의 차이를 최종 확인해야 한다.
- 원본·전체 보고서·왜곡 보정 미리보기: `debug_report/camera_calibration/20261024_*/`.
  **Git 제외 로컬 파일이므로 이 커밋만으로 원본 영상이 백업되지 않는다.**

## 재개 순서

- [ ] 원본이 필요하면 위 로컬 폴더를 별도 저장장치/로봇 PC에 복사해 보관한다.
- [x] 카메라 장착 완료 및 뒷면 기준 x/z 확보(사용자 보고).
- [ ] 장착 강성·케이블 고정·최종 해상도·초점 설정을 점검한다.
- [ ] 차체와 팔의 `base_link` 이름 충돌을 먼저 해결한다.
  각 TF 자식 프레임의 부모와 발행자는 하나여야 한다. 팔 optical 프레임에 중복 static TF를 추가하지 않는다.
- [ ] 실제 PC에서 by-id 장치 경로와 지원 모드, 영상 수신을 재확인한다.
  이 촬영 PC에는 당시 ROS `usb_cam`, `camera_calibration` 패키지가 없었으므로 실행 PC에서 의존성을 확인한다.
- [ ] 두 카메라의 뒷면 측정점→RGB optical 원점 오프셋과 장착 회전을 측정해 optical x/y/z를 계산한다. 기존 y=0도 재확인한다.
- [ ] 전방 팔의 LOCK 관절 자세를 확정하고 반복 재현한다.
  차체→팔→D435i optical TF에서 측정 위치와 회전을 검증한다.
- [ ] 완전한 6DOF TF가 검증된 뒤에만 LOCK 기준 `base_from_camera` 4×4 행렬을 저장한다.
  현재 위치 기록 YAML은 이 행렬 파일을 대신하지 않는다.
- [ ] 최종 초점·해상도에서 시험 intrinsic을 검증한다.
  화면 가장자리·모서리, 별도 체커보드 영상, 직선 왜곡을 확인한다.
  초점·줌·해상도 변경 또는 품질 부족이면 평평한 보드로 재보정한다.
  장착 위치만 변한 것은 intrinsic 재보정을 자동으로 요구하지 않는다.
- [ ] 우선 아래 명령으로 시험 intrinsic을 명시적으로 연결한다. 실제 경로와 모드는 확인 후 사용한다.
- [ ] `/rear/camera/camera_info`의 K/D·1280×720·frame_id와 이미지가 일치하는지 확인한다.
- [ ] 검증된 후방 TF만 발행하고, 영상 시각의 후방·전방 optical TF 조회가 모두 성공하는지 확인한다.
- [ ] 기존 랜드마크 대신 검증된 CameraInfo/TF로 새 랜드마크를 기록한다.
- [ ] 정지 후방 촬영 → 180° 회전 → 전방 매칭을 수행하고 결과만 관찰한다.
- [ ] 실제 10cm·20cm·5° 이동과 추정량의 크기·방향·반복성을 비교한다.
- [ ] 여러 랜드마크, 반복 장면, depth 누락, TF 단절, 팔 자세 변화에서 잘못된 보정을 거부하는지 확인한다.
- [ ] 품질이 확인되면 정식 보정 파일을 확정하고 저속 복귀에서 보정 OFF/ON 실제 오차를 비교한다.
- [ ] 이후에만 자동 재탐색과 Control UI 연결을 검증한다.

## 시험 intrinsic 연결 예시 (TF 발행 없음)

워크스페이스 루트에서 필요한 ROS 의존성이 설치된 상태로 실행한다.

```bash
colcon build --packages-select vision visual_landmark
source install/setup.bash
ls -l /dev/v4l/by-id/
ros2 launch vision apc850_rear.launch.py \
  device:=/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0 \
  width:=1280 height:=720 fps:=15.0 \
  camera_info_url:=package://vision/config/camera/calibration/apc850_rear_preview_trial_1280x720.yaml \
  publish_extrinsic:=false
ros2 topic echo /rear/camera/camera_info --once
```

이 명령은 CameraInfo 연결용이며 extrinsic이 없는 상태에서 차체 위치 추정 준비가 완료되는 것은 아니다.

관찰 단계에서는 계속 유지:

```ini
landmark_capture_mode = stop
visual_apply_correction = false
enable_landmark_recovery = false
```

세부 TF 명령·시험 절차·PASS/FAIL 기준은 같은 폴더의 `HARDWARE_VALIDATION.md`를 따른다.

## 장착 후 실차 적용 상세 절차 (2026-10-25)

### 1. 뒷면 측정값을 optical 좌표로 변환

| 측정점 | x(m) | y(m, 기존 보고) | z(m) |
| --- | --- | --- | --- |
| 전방 팔 캠 뒷면, 측정 LOCK 자세 | 0.152853 | 0 | 0.650674 |
| 후방캠 뒷면 | -0.316627 | 0 | 0.650674 |

차체 좌표는 x 전방, y 좌측, z 상방으로 확인한다. 뒷면의 어느 점을
측정했는지(중심/장착면/케이스 모서리)를 명확히 하고, 해당 점에서
**RGB optical 원점**까지의 3축 오프셋을 도면 또는 실측으로 확보한다.
카메라가 바라보는 방향·상하 기울기·좌우 기울기도 확보한다.
D435i는 후방캠 YAML이 아니라 자기 color CameraInfo와 color optical TF를 쓴다.

```
p_base_optical = p_base_back + R_base_back * offset_back_to_optical
```

offset을 차체 축으로 직접 측정했다면 그 벡터를 p_base_back에 더한다.
전방/후방의 바라보는 방향이 다르므로 같은 케이스 두께를 두 x에 일괄
더하지 않는다. Optical 축은 x 오른쪽, y 아래, z 시선 전방이다.
후방 카메라의 optical yaw만 180도로 지정하는 방식은 충분하지 않다.
뒷면 측정치는 저장만 했으며 optical TF에는 자동 사용하지 않는다.

완료 기준: 두 카메라의 차체 기준 optical 위치 및 회전이 정해지고,
전방 측정에 사용한 팔 관절 자세가 기록되어 있어야 한다.

### 2. TF 트리 통합

- 차체 `base_link`와 팔 URDF의 동명 링크 충돌을 해결한다. 이름만 바꿀 경우
  MoveIt·컨트롤러·카메라 broadcaster 등 소비자도 함께 수정해야 한다.
- 차체→팔 루트 장착 TF는 실제 측정값을 쓴다. ground/world 사이에 임의의 0 TF를 넣지 않는다.
- 팔은 기존 관절→카메라 연결을 유지한다. 이미 부모가 있는 optical 자식에 차체 static TF를 중복 발행하지 않는다.
- 후방은 URDF 또는 static broadcaster 중 한 곳에서만 발행한다.
- 아래는 실제 실행 중 검증한다(전방 frame 이름은 실제 영상 header 기준으로 대체).

```bash
ros2 run tf2_tools view_frames
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link rear_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link arm_camera_color_optical_frame
```

완료 기준: 자식별 부모·발행자가 하나이며 차체 원점이 의도한 기준이다.
팔을 움직이면 전방 TF가 따라 움직이고, LOCK 복귀 시 측정 자세가 재현된다.

### 3. 두 카메라의 영상·내부 보정·depth 확인

후방 실행 예시는 위 시험 intrinsic 연결 명령을 사용한다.
전방 RealSense는 기존 launch를 쓰되 같은 장치를 두 driver로 동시에 열지 않는다.
`src/vision/camera/config/arm_d435i.yaml`은 color/depth와 aligned depth를 활성화하지만,
실제 실행 launch가 이 설정을 읽는지 반드시 확인한다.

```bash
ros2 topic list
ros2 topic hz /rear/camera/image_raw
ros2 topic echo /rear/camera/camera_info --once
ros2 topic echo /arm/camera/color/camera_info --once
ros2 topic hz /arm/camera/color/image_raw/compressed
ros2 topic hz /arm/camera/aligned_depth_to_color/image_raw
```

완료 기준:

- 후방은 시험 보정과 동일한 1280×720, 초점·줌·크롭 설정이다.
- 각 영상 header frame과 자기 CameraInfo/TF가 일치한다.
- D435i depth가 color 픽셀에 정렬되고, 실제 물체 거리와 depth가 맞는다.
- RGB/depth/TF 시간과 PC 간 시계가 일치하고 stale TF/영상이 없다.
- 실차 USB 부하에서도 영상이 끊기지 않는다. 실제 프레임과 거리 표적을 보며 확인한다.

### 4. 내부 보정 최종 검증과 LOCK 파일

시험 보정 RMS 0.783px/최대 4.980px 및 미완료 가장자리 구역을 알고 사용한다.
관찰 시험은 가능하나 최종 주행 보정 전에는 가장자리 직선·별도 보드 검증과
실제 위치 오차 시험을 통과해야 한다. 해상도나 초점을 바꾸면 재검증/재보정한다.
팔 LOCK 기준은 검증된 차체→RGB optical 4×4 행렬을 YAML의
`base_from_camera`로 저장하고 `visual_expected_lock_pose_file`에 지정한다.
위치 3개만으로 회전 포함 4×4 행렬을 만들지 않는다.

### 5. 새 랜드마크 기록 및 정지 VPR 시험

- `landmark_capture_mode=stop`, `visual_apply_correction=false`,
  `enable_landmark_recovery=false`를 유지한다.
- 검증된 CameraInfo와 영상 시각 TF로 새 경로/랜드마크를 기록한다.
  이전 잘못된 TF/내부 보정으로 저장한 랜드마크는 재사용하지 않는다.
- 후방 촬영 → 차체 회전 → 전방 팔 LOCK → 동일 장면 확인 순서로 시험한다.
  같은 높이라고 해서 180도 회전 후 카메라 위치가 같아지는 것은 아니다.
- 동일 자세에서 반복 추정하고, 알려진 +10cm 횡이동·+20cm 종이동·+5도 yaw에
  대해 보정 방향·크기와 반복성을 확인한다. 각 자세에서 최소 10회 반복한다.
- 초기 목표는 위치 오차 5cm 이내·yaw 2도 이내이며 실제 측정으로 판정한다.
- 매칭 없음, 잘못된 depth, 팔 이동, TF 단절, 반복 무늬에서 잘못된 결과를 거부해야 한다.

진단 토픽:

```bash
ros2 topic echo /visual_landmark/localization_event
ros2 topic echo /visual_landmark/correction
```

관찰 모드는 VPR 보정만 끈다. `manual_return_bringup` 자체는 모터와 복귀 제어를
기동하므로 정지 시험은 모터 구동을 차단하거나 visual 노드만 별도로 실행한다.
관찰 상태에서 `validated` 결과를 확인하되 correction 토픽은 발행하지 않아야 한다.

### 6. 저속 실제 복귀 및 선택 기능

위 단계가 통과한 뒤 `visual_apply_correction=true`를 켜고 LOCK 파일을 지정한다.
동일 경로에서 보정 OFF/ON을 반복 비교해 실제 Base 도착 위치·각도 오차와
보정 적용 순간의 불연속을 측정한다. wheel/IMU odometry를 reset하지 않는다.
Manual/Auto 전환·VPR 노드 종료·랜드마크 누락에도 기존 복귀 동작이 유지되는지 확인한다.
자동 재탐색은 마지막에 별도 검증한 뒤 활성화하고, ±30도 탐색 실패 시 수동 개입과
Auto 재개를 확인한다. UI에는 검색/검증/보정 적용/실패/재탐색 상태를 표시한다.
영상·CameraInfo·depth·TF·odometry·VPR 진단은 HARDWARE_VALIDATION.md의 bag 명령으로 보관한다.


## 자동 키프레임 / Adaptive Visual Correction 후속 검증

구현·전체 parameter·실행/rosbag 명령은 [ADAPTIVE_VISUAL.md](ADAPTIVE_VISUAL.md).
수동 stop 촬영은 유지하고, 자동 촬영은 moving으로만 수행한다.

- [ ] Phase 1: 자동 keyframe 1.25m/12도/6초+0.5m 조건·정지 중 중복 방지·blur 검증.
- [ ] Phase 2: 보정 OFF로 8m watchdog, 실패 cooldown, 후보 3개(+2 확장)·연속 두 프레임 확인.
- [ ] Phase 3: 실제 x/y +0.2m, yaw +5도 변화와 추정량·방향·반복성 비교.
- [ ] Phase 4: wheel/gyro 정상 직진·회전·제자리 회전·바닥별 mismatch 로그로 slip 임계값 튜닝.
  mismatch는 실제 global drift가 아니라 Visual Check를 앞당기는 risk indicator이다.
- [ ] Phase 5: 오보정·TF·LOCK·Manual/Auto 회귀 검증 후에만 실제 correction 적용.
- [ ] ROS bag에서 전체 check 요청→결과 평균/최대 지연과 CPU·처리 후보 개수 측정.
- [ ] 교차/평행 경로에서 outbound s와 단순화 return 경로 arc의 오차·분기 선택 검증.
