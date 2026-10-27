> 현재 기본 운용은 [CHECKPOINT_VPR.md](CHECKPOINT_VPR.md)의 Record 2초 촬영 및 역순 체크포인트 검사와 정지 사진 UI입니다. 아래 기존 8m/실시간 preview 설명보다 해당 문서가 우선합니다.

# VPR 품질·최신 프레임·Dashboard 개선 (2026-10-27)

기준 main: `b4fe9d6` (원격 main과 일치 확인). 작업 시작 시 변경사항 없음.
실차 카메라/로봇 PC에 접속하거나 모터/Record/Return 명령을 실행하지 않았다.
현재 단일 Dashboard의 배치를 유지하며 생산 VPR 품질 진단을 추가한다.

## 변경 전/후 파이프라인

이전: raw SIFT 최대 2000 → mutual ratio → 3×3 depth → PnP RANSAC/LM →
geometry/cheirality/RMSE → correction bound → 두 프레임 합의.
RGB-D callback에서 이 작업을 직접 처리했고 viewer는 별도 최대 400점 SIFT를 5Hz로 계산했다.

현재:

```text
rear reference: SIFT 최대 800 → response 순 6×4 grid → cell당 최대 20 / 전체 최대 480 → cache
current RGB-D: depth-1 DDS + sync queue 2 → latest slot 1개 → 단일 처리 worker
  → SIFT 최대 800 → 동일 grid → mutual Lowe ratio 0.65
  → 각 K/D로 왜곡 제거 → F-RANSAC → 5×5 안정 depth → metric 3D
  → PnP RANSAC + LM → inlier grid/bbox → cheirality/RMSE
  → 차량 body 축 pose prior → 기존 correction bound/ambiguity → 두 프레임 합의
  → observation validated (보정 OFF 유지)
```

raw/filtered 특징 수는 장면에 따라 목표보다 적을 수 있다. 최소 개수를 채우려고
약한 특징을 추가하지 않는다. descriptor는 선택된 원래 index와 동일한 행을 사용한다.

## 단일 Dashboard

- 위 왼쪽/오른쪽: 후방/전방 live preview. cyan은 **preview SIFT**, 생산 매칭/검증 결과가 아니다.
- 아래 왼쪽: 생산 VPR debug image. amber=mutual descriptor pair, green=PnP RANSAC inlier.
  green만으로 prior/두 프레임 합의까지 통과한 것은 아니다. 최종 validated/localized를 확인한다.
- 연결선은 `debug_max_lines=60`으로 제한해 겹침을 줄이고 녹색 inlier 양쪽 끝점을 표시한다.
  모든 correspondence는 검사하며 이 제한은 그림에만 적용한다.
- 아래 오른쪽: 생산 raw/grid 특징 수, mutual/geometry/depth/PnP 단계별 수,
  inlier ratio/grid cells/RMSE, prior translation/yaw/roll/pitch, SIFT·matching/depth·PnP·전체 ms,
  RGB-D skew/slot wait/drop 누계, reject reason. cyan 점 수를 이 표에 대입하지 않는다.
- `F`: viewer의 live preview SIFT만 ON/OFF. `Q`: viewer만 종료한다.
- 생산 check 시작 이벤트를 받으면 preview SIFT를 일시 중지한다. 종료 이벤트 또는
  request timeout+1초에 복귀한다. 영상 표시와 STALE 검사는 계속된다.
- frame/result는 후보 한 장 기준이다. 모든 bounded 후보는 실제 검증하고 ambiguity gate를 유지한다.
- WAITING/STALE/RX count/source age 표시 유지. 시계 불일치도 source age에 나타난다.

## 초기 parameter before/after

모두 현장 초기 후보이며 최종 보정값이 아니다. 공통 설정:
`config/vpr_quality.yaml`. launch argument `visual_quality_config`로 다른 파일을 지정할 수 있다.
각 노드가 선언하는 ROS parameter도 직접 지정할 수 있다. **변경 후 capture/localizer를 재시작**한다.
현재 임계값은 초기화 시 읽으며 runtime ros2 param set으로 detector가 즉시 재생성되지는 않는다.

| parameter | 이전 | 현재 |
|---|---:|---:|
| viewer live_features | true | true (저부하 preview 유지, F로 OFF 가능) |
| preview_rate_hz | 5 | 2 |
| preview_max_width | 640 | 480 |
| preview_max_features | 400 | 120 (표시 상한) |
| pause_preview_during_check | 없음 | true |
| debug_max_lines (표시만) | 150 고정 | 60 |
| sift_nfeatures | 2000 | 800 |
| sift_contrast_threshold | OpenCV 0.04 | 0.06 |
| sift_edge_threshold | OpenCV 10 | 8 |
| feature_grid_rows / cols | 없음 | 4 / 6 |
| feature_grid_max_per_cell / max_total | 없음 | 20 / 480 |
| match.ratio_test | 0.70 | 0.65 |
| match.geometric_filter | 없음 | true |
| match.geometric_ransac_px / min_ratio | 없음 | 1.5px / 0.5 |
| match.depth_patch_radius | 1 (3×3) | 2 (5×5) |
| match.depth_edge_ratio | 0.08 | 0.05 |
| match.min_inliers | 20 | 30 |
| match.min_inlier_ratio | 0.45 | 0.60 |
| match.ransac_px / max_rmse_px | 3 / 2.5 | 2.5 / 2.0 |
| match.inlier_grid_rows / cols | 없음 | 4 / 6 |
| match.min_inlier_grid_cells | 없음 | 8 |
| match.min_inlier_image_coverage | 없음 | 0.08 (bbox面積比) |
| pose_prior.enabled | 없음 | true |
| pose_prior.max_translation_m | 없음 | 0.5m |
| pose_prior.max_yaw_deg | 없음 | 12° |
| pose_prior.max_roll_deg / max_pitch_deg | 없음 | 5° / 5° |
| agreement_translation_m / yaw_rad | 0.25 / 0.1 | 0.15 / 0.05 |
| RGB-D subscription depth | 5 | 1 |
| sync_queue_size | 5 | 2 |
| sync_slop_s | 0.04 | **0.04 유지** |
| apply_correction / landmark recovery | false / false | false / false 유지 |

실제 장치 timestamp 분포는 아직 측정하지 않았다. `rgbd_stamp_skew_s`를 먼저 기록하고
0.02/0.015s slop에서 pair 수신률을 비교한다. 단순히 slop을 줄이는 것만으로 지연 개선을
입증할 수 없으며 실제 color/depth 시차가 크면 pair가 사라진다.

## 새 검증과 좌표계

F-RANSAC은 reference/current 각각 자신의 K/D로 undistort한 pixel 좌표에서 수행한다.
서로 다른 해상도·초점거리를 같다고 가정하지 않는다. F 추정 실패/부족한 inlier는
`geometric_ransac`으로 거부하며 descriptor만으로 통과시키는 fallback은 없다.
순수 회전/작은 baseline/평면 장면에서는 F가 약하거나 퇴화하므로 downstream gate를 유지한다.

PnP는 여전히 **reference camera <- current camera**다. current depth로 back-project한
3D를 reference K/D로 project한다. 기존 parent<-child convention과 correction 수식은 동일하다.
coverage는 reference 영상의 실제 width/height로 검사하며 3D spread와 기존 60px span도 유지한다.
생산 node와 offline evaluator/benchmark는 실제 reference shape를 넘긴다. 이전 standalone
`estimate_pose()` API에서 shape를 안 넘기면 coverage는 측정 불가(`inlier_coverage_checked=false`)다.
새 production acceptance를 검증하려면 standalone에서도 반드시 shape를 넘긴다.

예상 상대 pose:

```text
prior = applied mission<-odom correction, 없으면 inverse(mission origin)
predicted reference<-current = inverse(mission<-reference_camera) * prior * odom<-current_camera
```

PnP와 predicted 차이는 current optical 좌표의 회전 축을 그대로 yaw라고 해석하지 않는다.
`base_from_camera * inverse(predicted) * estimated * inverse(base_from_camera)`로 차량 body
축의 잔차를 계산해 translation norm과 roll/pitch/yaw를 검사한다. diagnostics 각도 단위는 rad,
UI는 deg다. 실패 사유는 `pose_prior_mismatch`다.

**prior 제한 0.5m/12°를 넘는 실제 누적오차도 거부한다.** 따라서 이 값으로 임의 크기의
험지 drift를 복구한다고 주장하지 않는다. 알려진 offset 시험과 실제 drift 분포를 바탕으로
조절한다. observation 모드에서 validated된 correction을 내부 prior에 누적 적용하지 않는
기존 정책도 유지한다. 이 때문에 observation 누적오차가 커지면 prior gate에 걸릴 수 있다.

합의 강화는 기존 두 프레임 방식으로 유지한다. 첫 성공은 `agreement_pending`,
불일치는 `temporal_agreement_mismatch`; 요청의 max frames/deadline을 넘기면 종료한다.
3-frame consensus와 candidate 병렬화는 이번에 추가하지 않았다.

## 최신 프레임 처리와 CPU

ROS 수신 콜백은 JPEG decode/SIFT/PnP를 수행하지 않고 lock이 있는 latest slot만 덮어쓴다.
동일 slot에 더 오래된 source stamp가 들어오면 현재 최신 pair를 보존한다.
DDS image depth=1, sync queue=2, worker handoff depth=1로 중간 frame을 버린다.

Executor는 2 threads지만 **프레임 처리 worker는 하나**다. 수신 callback group만 분리하며,
worker timer와 mission/origin/check/pending/correction/reference callback은 동일한
MutuallyExclusive default group으로 직렬화한다. 후보도 순차 처리해 shared diagnostic/debug
race를 만들지 않는다. TF listener는 기존 buffer/listener를 사용한다.
입력 0.5초 freshness, image-time TF, commit stale/deadline, LOCK, 두 프레임 검사는 유지한다.

production OpenCV는 thread=1로 중첩 내부 병렬화를 제한한다. 후보 matching 병렬화는
실기 profiling 후에만 검토한다. 현재 bounded candidates 전체를 검사하고 deterministic
count/error/id 순으로 정렬한다. debug flag=false 또는 subscriber=0이면 draw/copy/JPEG가 없다.

cache는 기존 points/descriptors 키를 유지하며 feature 설정 signature와 raw count만 추가한다.
동일 signature는 재사용한다. 기존 schema2/3, unversioned/다른 설정 cache는 이미지에서
한 번 다시 추출한다. archive나 mission metadata를 자동 수정하지 않으며 동기화된 새
reference/current feature 설정을 사용한다. 잘못된 cache shape/nonfinite 데이터는 거부한다.

## diagnostics

기존 current_sift_s, attempts[].candidate_total_s, matching_and_depth_s, pnp_s,
processing_latency_s를 유지한다. 추가:

- raw_features / filtered_features, feature_count_raw / feature_count_grid_filtered
- attempts[].reference_feature_count_raw, reference_features/current_features
- raw_matches, mutual_ratio_matches, geometric_inliers, depth_valid_matches
- pnp_points, pnp_inliers (기존 inliers도 유지), inlier_ratio, reprojection_rmse
- inlier_grid_cells, inlier_bbox_width_ratio, inlier_bbox_height_ratio
- pose_prior_checked, pose_prior_translation_error, pose_prior_yaw_error,
  pose_prior_roll_error, pose_prior_pitch_error
- rgbd_stamp_skew_s, input_frame_age_s, pair_queue_wait_s, pairs_overwritten (process 누계)

측정되지 않은 prior/RMSE는 null/--이며 0 오차로 표시하지 않는다. no camera/timeout은
해당 stage 자체가 실행되지 않을 수 있다. 거부 stage는 reject_reason/reason으로 구분한다.

## 빌드·실행·실차 검증

기존 프로세스를 정지하고 수정 코드를 평소 방법으로 로봇 PC에 반영한 뒤:

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
si
cbr --packages-select visual_landmark robot_bringup
si
```

노트북은 visual_landmark만 다시 빌드한다. 모든 터미널은 현재 동작하는 DDS/domain99 환경을
적용한다. 기존 팔 LOCK TF·전후방 카메라·조이스틱 실행 절차는 그대로 유지한다.
차체 launch 예시(실제 모터 스택을 시작하므로 사용자 운용 절차에 따라 실행):

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  can_interface:=can_drive imu_port:=/dev/ttyACM1 reverse_return:=false \
  enable_auto_keyframes:=true enable_visual_localization:=true \
  enable_adaptive_visual_check:=true visual_debug_matches:=true \
  visual_apply_correction:=false enable_landmark_recovery:=false \
  visual_check_on_return_start:=false visual_check_on_auto_resume:=false \
  enable_stationary_vpr_debug:=true
```

다른 YAML 시험 시 위 명령에 `visual_quality_config:=/absolute/path/vpr_quality.yaml`을 추가한다.
feature 설정은 capture/localizer 양쪽에 동일하게 지정한다.

노트북 단일 Dashboard:

```bash
ros2 run visual_landmark vpr_viewer
# 더 낮은 부하 / live cyan 완전히 OFF
ros2 run visual_landmark vpr_viewer --ros-args -p live_features:=false
```

둘 중 하나만 실행한다. UI에서 F로 preview를 전환할 수도 있다.

```bash
ros2 topic echo /visual_landmark/localization_event
ros2 param get /visual_localization_node sift_nfeatures
ros2 param get /visual_localization_node match.min_inliers
ros2 param get /visual_localization_node pose_prior.max_yaw_deg
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_quality_preflight.json
```

새 mission으로 reference_loaded raw/grid/cache 상태를 확인한다. Return 후 Manual로
완전 정지해 RETURN_MANUAL을 확인하고 기존 정지 진단 요청으로 매칭을 확인한다.

```bash
ros2 topic pub --once /visual_landmark/debug_check std_msgs/msg/Bool '{data: true}'
```

정상/반복 패턴/깊이 경계/가림/프레임 끊김/알려진 횡이동·yaw를 비교한다.
검사 중 cyan pause, actual geometry/depth/inlier 표, prior error와 rejection을 확인한다.
viewer 종료 전후 mission/control이 유지되는지 확인한다. 관찰 동안 correction publish가
없어야 한다. 이미 켜진 correction 소비자/제어기는 변경하지 않았다.

## 테스트·측정 결과

관련 회귀 **170개 통과**, visual_landmark/robot_bringup 별도 빌드 성공.
로컬 domain119에서 실제 노드+latest worker에 합성 RGB-D/TF를 공급해 두 프레임 validated,
478 inliers/24 cells, correction_applied=false를 확인했다. 실기 시험을 대신하지 않는다.

로컬 unit/regression은 실제 OpenCV synthetic scene을 사용한다. grid cap/전체 cap/index 유지,
F outlier 제거, 정상 cross-camera/distortion, 5×5 depth 경계, spatial coverage,
optical/body yaw prior, 큰 yaw 거부/기존 correction 보존, legacy cache migration/reuse,
stale input pre-SIFT rejection, concurrent latest-slot overwrite, debug flag=false의 no-render,
observation no-publish 및 기존 mission/control 회귀를 검사한다.

합성 벤치마크 (같은 로컬 CPU, OpenCV4.5.4, 1 thread, 640×480, random texture,
3 candidates, 각각 5회; **실제 인식 정확도/rosbag/full request latency 아님**):

| 항목 | 이전 평균 | 이후 평균 |
|---|---:|---:|
| usable current features | 1296 | 478 |
| current_sift_s | 48.41ms | 46.39ms |
| matching_and_depth_s (3후보 합) | 247.96ms | 68.19ms |
| pnp_s (3후보 합) | 4.30ms | 1.72ms |
| frame total | 300.71ms | 116.36ms |
| process CPU | 300.69ms | 116.34ms |

로컬 JSON: `debug_report/vpr_optimization/{before,after}.json` (Git 제외).
측정은 deterministic scene microbenchmark이며 실기 timestamp skew/지연·인식 성공률은 미측정이다.
그리드로 descriptor matching 수가 줄었지만 SIFT 검출 자체가 비례해 빨라진 것은 아니다.
새 gate 때문에 거부/성공 경로가 달라질 수 있어 timing만으로 정확도 개선을 주장하지 않는다.

```bash
python3 -m visual_landmark.benchmark_adaptive --iterations 10 --output /tmp/vpr_quality_benchmark.json
```

실기 튜닝 대상: preview 부하, SIFT contrast/edge/grid, ratio/F threshold,
depth radius/edge, inlier count/ratio/RMSE/coverage, prior 6DOF 한계, agreement, sync slop.
필드 데이터를 보며 조절하고 gate를 단순히 끄거나 완화해 성공률만 높이지 않는다.

## 수정 파일과 변경 이유

- localization.py: bounded/grid SIFT, F/depth/PnP coverage/prior 및 stage 진단.
- latest_pair.py: thread-safe depth-1 handoff.
- visual_localization_node.py: serialized latest worker, prior/coverage 연결, cache 검증,
  agreement/timing 이벤트. 좌표 수식·기존 gate 유지.
- visual_landmark_node.py: reference에도 동일 feature 설정/서명 저장.
- viewer.py: 기존 단일 UI에 실제 pipeline metrics, 낮은 preview 예산·pause·F.
- config/vpr_quality.yaml, setup.py: 조절 가능한 초기값·패키징.
- manual_return_bringup.launch.py: 시각 노드에만 quality YAML 전달.
- evaluate_matching.py, benchmark_adaptive.py: 실제 reference 해상도 전달.
- test_quality_pipeline.py 및 기존 synthetic fixture: 새 gate/인덱스/슬롯/기존 제어 격리 회귀.
- 본 문서와 ADAPTIVE_VISUAL/HARDWARE_VALIDATION/IMPLEMENTATION_REPORT/STATIONARY_VPR_DEBUG:
  현재 pipeline·UI·초기값·한계·측정 절차.

return state machine, Pure Pursuit, drive_cmd_mux, reduced odometry, mission origin,
카메라 TF와 D435i calibration은 수정하지 않았다. correction/recovery 자동 활성화도 없다.

공식 API 참고:
[OpenCV calib3d](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
[ROS2 Humble callback groups source](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/callback_groups.py).


## Record / Return 상단 표시와 UI 재시작

- Record (`MANUAL_RECORDING`, `WAIT_RETURN_COMMAND`): 왼쪽 실시간 후방, 오른쪽 실시간 전방.
- Return (Manual 전환 포함): 왼쪽 저장 후방 참조, 오른쪽 실시간 전방.
  실제 검사에서 선택한 참조 ID가 이미지에 표시되며 하단 Matches와 같은 참조이다.
  통과 후보 중 순위가 가장 높은 참조를 표시한다. 모두 실패하면 가장 많은 PnP
  inlier를 가진 시도 참조를 표시하므로, 표시 자체가 검증 성공을 뜻하지 않는다.
- 저장 참조는 역순 경로의 후보 선택 결과이며 영상 뒤집기나 연속 역재생이 아니다.
  검사 사이에는 마지막 검사 참조를 유지하고, 첫 검사 전에는 대기 화면을 표시한다.
- `/visual_landmark/debug/reference/compressed`는 진단 전용 CompressedImage이며
  transient_local depth 1이다. UI를 재시작해도 마지막 참조를 받는다.
  새 미션에서는 저장 표시를 비운다. `visual_debug_matches:=true`가 필요하다.

수정 반영은 **로봇 PC와 노트북 각각** 다음을 실행한다. 로봇의 기존 주행 launch는
차체를 정지한 후 종료하고 빌드 후 다시 실행한다. 아래 명령은 노드를 시작하지 않는다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_landmark
source install/setup.bash
```

**노트북**: 기존 정상 통신 중인 ROS 환경(domain 99, 올바른 DDS 설정)을 적용한
터미널에서 UI만 실행한다. UI를 닫아도 주행 및 카메라 노드는 종료되지 않는다.

```bash
ros2 run visual_landmark vpr_viewer
```

종료: UI에 포커스를 주고 `Q`, 또는 이 명령을 실행한 터미널에서 `Ctrl+C`.
재실행: 같은 명령을 다시 실행한다. 잘못된 인터페이스를 지정하는 `sr`은 실행하지 않는다.


## 현재 운용 기본값

캡처 거리 1.25m, `visual_apply_correction=true` / `apply_correction=true`.
연속 UI SIFT preview는 `live_features=false`가 기본이다. 실시간 영상에는 특징점을
다시 계산하지 않는다. 하단은 실제 요청 검사에서 생성한 정지 비교 사진을 다음 결과까지
유지한다. F 키로 연속 preview를 켜면 UI SIFT 연산이 다시 발생한다.
보정 활성화 시에도 기존 TF/LOCK/기하/두 프레임/보정량 제한은 유지한다.
`stationary_debug`는 observation 전용이므로 보정이 켜져 있으면 debug_check가 거부된다.
실제 보정 검증은 AUTO 복귀의 adaptive check에서 한다. 이 변경은 실행 중인 노드의
파라미터를 바꾸지 않으며 수정 코드 빌드와 launch 재시작이 필요하다.

表示更新: Record에서 마지막으로 로드한 참조 사진에 캐시된 SIFT 점을 표시한다.
Return에서는 route_progress를 이용해 표시용 후보 사진을 갱신한다. 이는 검사 트리거와
독립적이며 추가 SIFT/PnP를 실행하지 않는다. 참조 ID/표시 종류가 바뀔 때만 인코딩한다.
ROUTE CANDIDATE 표시는 검증 성공이 아니다. 하단 Matches는 실제 검사 결과만 유지한다.
이 동작은 위의 '첫 검사 전 대기' 설명을 대체한다.
