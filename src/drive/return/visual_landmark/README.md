# Back-to-Base visual localization

APC850/D435i 실차 설치·보정·A/B 평가·검증 절차는
[HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md)를 먼저 참고한다.
**현재 기본은 stop 촬영, 보정 관찰만 수행(`visual_apply_correction=false`),
자동 누락 정지/회전 탐색 비활성(`enable_landmark_recovery=false`)이다.**
관찰 모드는 일반 복귀 컨트롤러나 모터를 비활성화하는 설정이 아니다.
실제 측정 K/D·rear optical TF와 팔/차체의 `base_link` 이름 충돌 해결 전에는
실차 준비가 완료됐다고 간주하지 않는다.

This package records outbound landmark RGB keyframes and localizes the returning
robot with the arm Intel D435i's RGB and aligned depth. No lidar, learned-model
download, rear depth camera, or odometry reset is required.

SIFT features are matched bidirectionally with a ratio test. Current aligned
depth lifts image features into metric current-camera 3D coordinates; PnP RANSAC
and nonlinear refinement estimate `reference_camera <- current_camera` against
the stored RGB pixels. This direction supports an RGB-only rear
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
- 추종기의 `resume_max_path_distance_m` 기본값은 3 m이다. 허용 범위 안에서
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

### 랜드마크 누락 감지와 좌우 30도 탐색

유효한 `reference_loaded` 랜드마크를 기록 경로에 투영하고, 복귀 경로의
진행 거리를 기준으로 예상 구간을 판정한다. 기본은 랜드마크 1 m 전부터
검색 구간으로 보고, 1 m를 지나도록 인식하지 못하거나 해당 구간 진입 후 4 s
동안 인식하지 못하면 전진을 멈춘다. Base 근처 랜드마크는 도착 판정 전에
검사한다. 경로는 다음 미처리 코너까지의 진행 구간으로 제한하여 주변 다른
경로 분기로 갑자기 진행도가 건너뛰는 것을 줄인다. 거리와 시간은 추정값이며
반복되는 장소/자기 교차 경로에서는 추가 현장 검증이 필요하다.

`RETURN_LANDMARK_SEARCH`에서 최초 정지 방향을 기준으로
`0° → +30° → -30° → 0°` 한 차례 탐색한다. 각도는 odometry yaw 피드백으로
판정하며 기본 회전 속도는 0.15 rad/s, 각도 도달 허용오차는 2°이다. 각 위치에서
3 s 관찰하고 초기 정지/안정화(기본 0.5 s) 이후의 영상만 보정 확정에 사용한다.
+30°에서 -30°까지의 이동은 60°이고, 전체 회전 이동량은 약 120°이다.
제한은 시작 방향에 대한 편차 ±30°이며, 총 이동량 60°라는 의미가 아니다.

탐색 중에도 candidate radius를 유지하며 지정된 대상 랜드마크를 검색한다. 후보 발견 시
회전을 멈추고 새 정지 영상 두 장의 매칭을 검증한다. 이때 일반 VPR의 inlier,
재투영 오차 및 보정량 검사는 그대로 유지한다. 보정 적용 모드에서 성공하면 제어기가 해당
보정을 수신한 것을 확인하고 기존 Auto 재합류 검사(기본 3 m) 후 복귀한다.
보정이 너무 커서 검증을 통과하지 못하면 자동 재개하지 않는다.

실패, 60 s 시간 초과, 허용 각도 초과, odometry 단절 또는 RGB-D/카메라 TF
단절 시 `RETURN_INTERVENTION`에서 정지를 유지하고
`/mission/return/control_status`에 `manual_intervention_required`와 사유를
발행한다. 일반 탐색 진행은 `landmark_search`로 표시한다. 수동 전환은 탐색을
즉시 중단하고 경로/진행도를 보존한다. 이후 Manual로 장면을 찾거나 경로
가까이 이동한 뒤 Auto를 요청할 수 있다. 동일 랜드마크의 자동 탐색은 임무당
한 번으로 제한해 수동 개입 후 같은 지점에서 무한 탐색하지 않는다.

캡처되지 않았거나 TF/보정값/특징이 없어 참조로 로딩되지 않은 이미지는
누락 감지 대상이 아니다. 시작 시 유효 랜드마크가 없으면 기존 odometry 복귀를
유지한다. 노드 재시작 시 참조/누락 감지 상태의 자동 복원은 지원하지 않는다.
제자리 회전은 장애물 회피 기능을 추가하는 것이 아니므로 회전 공간을 확보해야
한다. 3 m 재합류 제한도 주변 지형의 통행 가능성을 판단하지 않는다.

기본값은 launch 인자로 조정할 수 있다:

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  visual_apply_correction:=true enable_landmark_recovery:=true \
  resume_max_path_distance_m:=3.0 \
  landmark_miss_window_m:=1.0 landmark_miss_wait_s:=4.0 \
  landmark_search_angle_deg:=30.0 landmark_search_speed_radps:=0.15 \
  landmark_search_dwell_s:=3.0 landmark_search_timeout_s:=60.0
```

내부 인터페이스:

- `/visual_landmark/missed`: 추종기가 누락한 랜드마크 ID와 시각을 발행한다.
- `/visual_landmark/search_request`: 검색 대상, 요청 ID와 보정 허용 시각.
- `/visual_landmark/localization_event`: `frame`, `candidate`, `localized` 및
  참조 로딩/실패 이벤트. 검색 결과는 요청 ID·랜드마크 ID·영상 시각으로 검증한다.
- `/mission/return/state`: 탐색 중에는 `RETURN_LANDMARK_SEARCH`이며, 일반
  경로 추종은 정지하고 상태 머신의 회전 명령만 mux를 통과한다.


### 현재 조이스틱 버튼맵 (기본 파라미터)

주행 포커스(`/control/active_target=drive`)에서 다음 입력을 사용한다.
`manual_control.launch.py`는 아래 기본 버튼 번호를 덮어쓰지 않는다.

| 입력 | 기능 | ROS Joy 인덱스 |
|---|---|---|
| 왼쪽 스틱 상하 | 왼쪽 바퀴 속도 | axes[1] |
| 오른쪽 스틱 상하 | 오른쪽 바퀴 속도 | axes[4] |
| △ / × | 누르는 동안 전진 / 후진 | buttons[2] / [0] |
| □ / ○ | 누르는 동안 제자리 좌회전 / 우회전 | buttons[3] / [1] |
| L1 / L2 | 왼쪽 기본속도 증가 / 감소 | buttons[4] / [6] |
| R1 / R2 | 오른쪽 기본속도 증가 / 감소 | buttons[5] / [7] |
| L3 / R3 | 차동주행 시험: 왼쪽 / 오른쪽 바퀴가 빠름 | buttons[11] / [12] |
| PS | 기록 시작 `/path/record` | buttons[10] |
| Share | 최초 복귀 요청 `/path/return` | buttons[8] |
| D-pad ↑ | 랜드마크 촬영 `/path/landmark` | axes[7] > 0.5 |
| D-pad ↓ | Manual 전환 요청 | axes[7] < -0.5 |
| D-pad → | Auto 재개 요청 | axes[6] < -0.5 |
| D-pad ← | 예비 (주행 포커스) | axes[6] > 0.5 |
| Options | 팔 / 주행 포커스 전환 (팔 텔레옵이 관리할 때) | buttons[9] |

PS는 IDLE에서 기록을 시작한다. 다시 눌러 기록을 끝내는 토글이 아니다.
Share는 복귀 가능한 수동 기록 상태에서 복귀를 시작한다. 복귀 중 Manual/Auto
전환 버튼으로는 사용하지 않는다. Options는 조이스틱을 팔/주행 중 어디에
전달할지 선택하는 것이며 자동/수동 운전 권한 전환과는 별개이다.

D-pad ↓/→는 주행 포커스에서 `/mission/return/control_mode`에 `MANUAL`/`AUTO`를
한 번 누를 때 한 번 발행한다. Control UI와 터미널도 같은 토픽을 사용할 수 있다.
팔 포커스에서는 발행하지 않으며, 팔 조작 중 D-pad를 누른 채 주행으로
전환했으면 중립으로 놓았다가 다시 눌러야 모드 전환 요청이 발생한다.
↓와 →를 동시에 누르면 Manual을 우선하고, 계속 누르고 있어도 재발행하지 않는다.
Auto는 즉시 주행이 아니라 정지·경로 재합류 검사 요청이며, 최초 복귀는 Share를
사용한다. 축이 반대인 장치는 `manual_mode_axis_sign` / `auto_mode_axis_sign`
(기본 각각 -1.0)을 조정한다.
현재 주행 노드의 PS 버튼은 비상정지로 사용하지 않으며, 비상정지는 별도
`/emergency_stop` 인터페이스이다. 장치/드라이버가 달라지면 실제 `/joy`의
인덱스와 D-pad 부호를 확인해야 한다.
