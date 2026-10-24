# 카메라 VPR 재개 TODO

2026-10-24: 최종 장착 위치 미확정으로 하드웨어 작업을 중단하고 시험 결과를 보관한다.
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
  차체 `base_link` 기준 전방 LOCK 위치 (0.152853, 0, 0.650674)m,
  후방 위치 (미측정, 0, 0.650674)m. 회전은 모두 미측정이다.
  측정점과 optical 원점의 차이를 최종 확인해야 한다.
- 원본·전체 보고서·왜곡 보정 미리보기: `debug_report/camera_calibration/20261024_*/`.
  **Git 제외 로컬 파일이므로 이 커밋만으로 원본 영상이 백업되지 않는다.**

## 재개 순서

- [ ] 원본이 필요하면 위 로컬 폴더를 별도 저장장치/로봇 PC에 복사해 보관한다.
- [ ] 후방캠 장착 위치와 방향, 케이블 고정, 최종 해상도·초점 설정을 확정한다.
- [ ] 차체와 팔의 `base_link` 이름 충돌을 먼저 해결한다.
  각 TF 자식 프레임의 부모와 발행자는 하나여야 한다. 팔 optical 프레임에 중복 static TF를 추가하지 않는다.
- [ ] 실제 PC에서 by-id 장치 경로와 지원 모드, 영상 수신을 재확인한다.
  이 촬영 PC에는 당시 ROS `usb_cam`, `camera_calibration` 패키지가 없었으므로 실행 PC에서 의존성을 확인한다.
- [ ] 후방캠 x·y·z와 optical 회전을 최종 측정한다. 장착 위치가 바뀌면 이전 y/z도 재확인한다.
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
