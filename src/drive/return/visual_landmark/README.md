# Back-to-Base visual localization

This package records outbound landmark RGB keyframes and localizes the returning
robot with the arm Intel D435i's RGB and aligned depth. No lidar, learned-model
download, rear depth camera, or odometry reset is required.

SIFT features are matched bidirectionally with a ratio test. Current aligned
depth lifts image features into metric current-camera 3D coordinates; PnP RANSAC
and nonlinear refinement estimate `reference_camera <- current_camera` against
the stored RGB pixels. This direction deliberately allows a future RGB-only rear
camera. Roll, pitch, yaw and translation are estimated together. It requires
sufficient overlapping, textured, static scenery and valid depth; arbitrary
viewpoint change, motion blur, occlusion and textureless ground are not guaranteed.

Image-time `odom <- camera` TF records chassis roll/pitch and the articulated
camera pose. The transform chain is:

```
mission <- current_base = (mission <- reference_camera)
                       * (reference_camera <- current_camera)
                       * (current_camera <- current_base)
```

The resulting planar `mission <- odom` transform is published on
`/visual_landmark/correction` (`PoseStamped`, frame `mission`; pose denotes the
transform, not the current robot pose). Both the return follower and mission
arrival check consume it. The transform persists while live odometry propagates
movement; matching failure does not reset it. New mission/path resets it. The
localization node never publishes drive commands or alters raw odometry.

## Running

Build and source the workspace as usual:

```bash
colcon build --packages-select visual_landmark return_navigation robot_bringup
source install/setup.bash
ros2 launch robot_bringup manual_return_bringup.launch.py landmark_capture_mode:=stop
```

Camera drivers and arm robot-state/TF publishers must already be running.
Default current inputs are:

- `/arm/camera/color/image_raw/compressed` (`CompressedImage`)
- `/arm/camera/aligned_depth_to_color/image_raw` (`Image`, 16UC1 mm or 32FC1 m)
- `/arm/camera/color/camera_info` (`CameraInfo`)

RGB and aligned depth must share the color optical frame and calibration size;
frame timestamps must agree within 40 ms. Standard OpenCV plumb-bob/rational
calibration is supported. Timestamped TF must connect `odom`, `base_link` and
both cameras. If the arm remains in its home pose throughout the mission, a measured full
`base_link <- camera optical frame` transform for that pose is sufficient;
individual joint measurements are not required by VPR. Publish that transform
through the existing TF tree (avoid duplicate TF publishers). If the arm moves,
the same transform must be updated from joint states and robot kinematics;
a fixed home-pose transform then gives incorrect robot localization.
The EKF currently publishes chassis roll/pitch from the IMU in odom TF. Altitude
is not mapped: this is planar route correction with full relative camera pose,
not a globally consistent 3D terrain map.

The planned operation is outbound manual driving with rear RGB capture, then
a 180-degree turn and forward autonomous return using the arm D435i. Mount the
rear camera at the same height and equivalent rear-facing pitch/roll as the arm
camera in its home pose. Measure both extrinsics: equal height/angles do not
remove the front/rear translation or 180-degree yaw difference.

Once a rear camera is available, pass its topics with `rear_image_topic` and
`rear_camera_info_topic`. Press D-pad up during manual route recording (MANUAL_RECORDING or
WAIT_RETURN_COMMAND) to capture a landmark. WAIT_RETURN_COMMAND means enough
route points exist to accept a return request; it does not end manual driving
or route/landmark recording. IDLE and return states do not permit capture. Stop mode is preferred for initial validation. `meta.yaml` schema 2
stores `mission_camera_transform` at image time. Missing TF/calibration allows
archiving the image but excludes it from localization. Old capture-only records
without this transform cannot provide tilt-aware localization.

Without a rear camera, a bench geometry check is possible by pointing the arm
camera at the same scene during recording and return and passing:

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  rear_image_topic:=/arm/camera/color/image_raw/compressed \
  rear_camera_info_topic:=/arm/camera/color/camera_info \
  landmark_capture_mode:=stop
```

This does not create overlap when a 180-degree return faces the opposite scene.
Arrange the arm/viewpoint appropriately or use a stationary bench sequence.

Only references captured during the current running mission are used. Start the
localization node before recording; restarting it loses its in-memory reference
set. References remain archived under `~/.ros/visual_landmarks`, but historical
missions are deliberately not automatically selected. Feature extraction occurs
once when each reference is loaded. Candidate search scans landmarks within
12 m of the corrected odometry prior, at most once per second. This is local
relocalization anchored to the recorded route, not worldwide place search.
Recorded landmark coordinates inherit outbound odometry error; they constrain
additional return drift, not absolute surveyed position.

## Acceptance and observability

- >=20 PnP inliers, >=45% inlier ratio, distributed image/3D points.
- <=2.5 px reprojection RMSE, positive reference-camera depth.
- Invalid/out-of-range/edge depth rejected.
- Competing landmarks with similar support rejected.
- Two consecutive frames must agree within 0.25 m and 0.1 rad.
- Correction step relative to the existing prior limited to 3 m and 0.7 rad.
- Input older than 0.5 s, computation older than 1.5 s, unsupported calibration
  and missing timestamped TF rejected.

Limits are initial engineering defaults, not field-calibrated confidence bounds.
For larger drift tune `candidate_radius_m`, `max_correction_m` and
`max_correction_yaw_rad` on the localization node after inspecting data. Common
launch topic overrides are `vpr_color_topic`, `vpr_depth_topic`, and
`vpr_camera_info_topic`. Disable using `enable_visual_localization:=false`.

Inspect `/visual_landmark/event` for captures/reference readiness and
`/visual_landmark/localization_event` for `reference_loaded`, `localized`,
`no_match`, or rejection reasons. Verify stationary pose accuracy, different
body/arm tilt, repeated scenery, depth holes, no match, and mission reset on the
robot before evaluating moving return. Synthetic tests validate geometry and
outlier handling but do not establish real-world recognition accuracy.

PnP convention: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

## 복귀 중 Manual / Auto 전환

복귀 임무의 경로·Base·랜드마크·시각 보정은 유지하면서 운전 권한을
전환한다. Manual에서는 원래 기록 경로를 덮어쓰지 않고 자유롭게 이동할 수
있으며, 자동 추종기의 진행 지점·코너 진행도는 정지한다. 완료 전에는
경로를 CLEAR하지 않는다. 프로세스 재시작 후 임무 복원은 지원하지 않는다.

Control UI는 `/mission/return/control_mode` (`std_msgs/String`)에 `MANUAL`
또는 `AUTO`를 발행한다. 화면 소스는 이 저장소에 없으므로 실제 선택 버튼은
UI 프로젝트에서 이 토픽에 연결해야 한다. 터미널 확인 방법:

```bash
# 자동 복귀 중 수동 전환
ros2 topic pub --once /mission/return/control_mode std_msgs/msg/String "{data: MANUAL}"
# 수동 작업 후 같은 복귀 임무 재개
ros2 topic pub --once /mission/return/control_mode std_msgs/msg/String "{data: AUTO}"
# UI 표시용 JSON 상태 (TRANSIENT_LOCAL)
ros2 topic echo /mission/return/control_status --qos-durability transient_local
```

- `RETURN_MANUAL`: 조이스틱 명령을 사용한다. 자동 명령은 차단한다.
- `RETURN_RESUME_STOP`: 두 입력을 모두 차단하고 신선한 odometry로 정지·잔진동
  대기 시간을 확인한다. 이전 전환의 명령은 재사용하지 않는다.
- Auto 재개 전에 추종기가 현재 위치와 남은 경로의 거리를 검사한다. 이미
  진행한 지점 바로 뒤부터 다음 미처리 코너 사이만 검사하므로 근처의 다른
  경로 분기로 바로 건너뛰지 않는다. 경로점/원점/보정·코너 진행도는 보존한다.
- 추종기의 `resume_max_path_distance_m` 기본값은 1 m이다. 허용 범위 안에서
  재개하면 기존 heading 정렬 및 경로 추종으로 합류한다. 장애물 우회 계획은
  없으므로 이 거리는 통행 가능한 시험 환경에 맞춰 설정해야 한다.
- odometry가 오래됐거나 허용 거리 밖이면 `RETURN_INTERVENTION`에서 정지를
  유지하고 JSON `status=manual_intervention_required`와 사유를 발행한다.
  운전자는 `MANUAL`을 선택해 이동한 뒤 `AUTO`로 재검사를 요청할 수 있다.
- 재개 정지/응답 제한은 상태 머신 `resume_timeout_s` 기본 5 s이다. 응답은
  요청 ID로 구분하여 취소된 이전 Auto 요청이 나중에 재개시키지 못한다.
- `FOLLOW_RETURN_PATH`에서만 자동 추종기가 진행도를 갱신한다. Auto 재개 시
  새로운 `/return_path`를 발행하거나 처음부터 180도 회전을 반복하지 않는다.
  최초 정지/회전 중 Manual로 바꿨다면 경로를 준비해 보존하고, Auto 재개 시
  경로 방향으로 정렬한다.
- 수동 작업 중에도 유효한 랜드마크가 보이면 시각 보정할 수 있다. 이는 경로
  기록이나 자동 운전 권한을 활성화하지 않는다.
- 정지·회전·추종 중 긴급정지는 기존 `/emergency_stop` 동작을 우선한다.

UI는 `manual`, `resume_check`, `auto`, `manual_intervention_required`를
JSON `status`에서 표시하고 실제 중재 상태는 `/mission/return/state`로
확인한다. `AUTO` 발행 자체를 자동 복귀 성공으로 표시하지 않는다.

### 랜드마크를 못 찾았을 때의 좌우 30도 탐색 — 제안, 아직 미구현

예상 랜드마크 구간을 놓친 것을 판단한 뒤 정지 → 정지 방향 기준 +30도 →
-30도 → 원래 방향 순으로 한 차례 탐색하고, 실패하면 정지 유지 및 수동
개입을 요청하는 흐름을 권장한다. yaw 오차로 장면이 시야 밖에 있는 경우
직접 수동으로 이동하지 않고 회복할 기회가 있다. 각 끝점에서 정지·안정화한
영상으로 검증하고, 회전 중 후보 발견 시에도 정지 후 확정해야 한다.

이는 시작 방향 기준 최대 편차 ±30도이며, +30도에서 -30도까지의 이동은
60도이다. 총 회전 이동량을 60도로 제한한다는 뜻이 아니다. 각도 도달은
시간이 아니라 odometry yaw로 확인하고 전체 제한시간도 별도로 둬야 한다.

현재 자동 탐색 회전, 예상 랜드마크 누락 감지, 자동 재탐색 실패 이벤트는
구현되지 않았다. 위 Manual/Auto 전환 및 수동 개입 표시 인터페이스는 구현돼
있다. 현재 VPR 매칭 실패만으로 자동 회전하거나 정지하지는 않는다.
