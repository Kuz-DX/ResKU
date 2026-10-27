# Adaptive Visual Correction

2026-10-27 품질/UI 후속 변경: [VPR_QUALITY_UI.md](VPR_QUALITY_UI.md)를 우선한다.
SIFT/grid/F-RANSAC/depth/PnP coverage/prior/latest-slot 및 단일 Dashboard를 추가했다.
아래 기존 pipeline/초기값 기록 중 해당 항목은 새 문서의 before/after 표로 대체한다.


Route-constrained Visual Landmark Matching adds an optional visual layer to the
existing wheel vx + myAHRS gyro wz → reduced_odom → /odometry/filtered pipeline.
Corner-aware Pure Pursuit, the estimator, mission state machine and raw odometry
reset policy are unchanged. Visual failure ≠ Back-to-Base failure.

## Before/after audit

Already present: APC850 reference RGB, D435i color/aligned depth, cached reference
SIFT, mutual ratio matches, depth back-projection, PnP RANSAC + LM, reprojection,
geometry/depth/cheirality gates, two-frame agreement, correction step limits,
ambiguity rejection, image-time TF, arm LOCK guard, persistent mission←odom
correction, observation-only mode, Manual/Auto pause/resume and opt-in yaw search.

Previously outbound capture required a button; return ran matching every second
against every radius/route-eligible reference. New automatic moving keyframes and
request-gated localization reuse those existing geometric gates. Manual stop
capture remains available and does not share an automatic hold request.

## Responsibilities and topics

- visual_landmark_node: automatic outbound capture only in MANUAL_RECORDING or
  WAIT_RETURN_COMMAND; manual /path/landmark behavior unchanged.
- visual_check_scheduler_node: separate light process, distance-based visual
  watchdog and wheel/gyro disagreement based early trigger. No motion publisher,
  controller dependency, EKF input or mission transition authority.
- return_path_follower_node: additionally publishes /visual_landmark/route_progress
  (String JSON arc, stamp_ns) using its existing corner/floor-constrained segment.
  No changes to its control law, corner handling or rejoin policy.
- visual_localization_node: listens to /visual_landmark/check_request (String JSON)
  and performs expensive work only during a check. Existing /visual_landmark/search_request
  remains the separately enabled stop/yaw recovery interface.
- /visual_landmark/scheduler_event: String JSON request/success/failure/timeout,
  request UUID, trigger, distance and slip score.
- /visual_landmark/localization_event: existing events plus check_request,
  check_complete, trigger, candidate IDs, timing and correction_applied.
- /visual_landmark/correction remains the only visual correction output.

The scheduler reuses /wheel/odom angular.z, /imu/gyro/z_corrected (Float64) and
/odometry/diagnostics from reduced_odom. Bias correction, IMU→base conversion and
calibration are not reimplemented. Float64 has no timestamp: corrected gyro
freshness is arrival-based; wheel/odom/diagnostics also require their source stamp.

## Automatic reference capture

Defaults: 1.25m distance OR 12° heading difference OR 6s with at least 0.5m
travel since the last accepted keyframe. One initial frame may be recorded at
rest to establish the anchor. Stationary timeout never creates repeated images.
A failed frame does not advance the valid anchor; retries are limited to 1Hz.
Automatic capture always moves; it never publishes /path/landmark or hold.
Manual stop capture keeps its existing sequencer and mux behavior.

Checks: image freshness (existing image_stale_s), odometry/image skew <=0.5s,
CameraInfo/frame/size/model, image-time camera/base TF, >=20 usable SIFT features.
Optional Laplacian blur gate is disabled by default (threshold 0), pending field
calibration. Automatic descriptors are calculated once and saved to features.npz;
reference loading validates and reuses this cache. Manual/old records compute
features once at load. Failed TF archives may remain but no captured/reference
activation event is published for them.

Schema 3 adds capture_source, automatic_capture_reason (initial/distance/yaw/timeout),
capture_quality and odom_sample_stamp to existing order, s_along_path, mission
pose, image timestamp/file, CameraInfo, image-time transforms and latest odom
sample twist. Twist and travelled s are nearest available callback data, not
interpolated at image time; sample skew is recorded/limited. Localization uses
image-time TF for geometry. Old schema 2 references remain readable.
New mission resets automatic state. RETURN_MANUAL never records a new outbound
path/keyframe; Manual/Auto resume retains the current mission and reference set.

## Scheduler, request lifetime and validity

First entry into return initializes distance=0 and score=0. Default start check
is false. The watchdog sums finite raw odometry XY steps (jumps >1m are excluded).
It includes manual detours during an active return, and is preserved across
Manual/Auto transitions. Checks are requested only in FOLLOW_RETURN_PATH with
fresh odometry; manual/turn/search states never start adaptive checks.

At 8m since the last validated observation request a check. This odometry-derived
distance is NOT true drift. No covariance or accumulated-angle trigger is used.

Early trigger when healthy:

```
excess = max(0, abs(wheel_wz - corrected_gyro_wz) - deadband)
score = max(0, score + (excess - decay) * dt)
```

Require gyro_calibrated=1, wz_source_gyro=1, imu_stale=0, fresh diagnostics,
fresh finite wheel/gyro. Missing/fallback/stale signals clear risk accumulation
and disable the early trigger, without disabling the distance watchdog. Tick
intervals outside (0,0.5]s do not integrate across gaps.
Mismatch is only a skid/slip risk indicator, NOT measured global drift.

One UUID request at a time, up to 5s / 4 processed frames. Two independent
agreeing frames remain mandatory. Maximum-frame no-match/rejection or timeout
ends the check; previous correction persists. No camera, no references or dead
visual process also times out. Failed checks never reset valid distance, and a
5s retry cooldown prevents a frame-by-frame request storm. Success resets
watchdog and slip score even in observation mode or correction deadband.
Unknown/old result UUIDs cannot reset the watchdog. Manual transition cancels
in-flight adaptive checks, preserving distance and existing correction.

A check request is JSON, e.g. manual field diagnostic:

```
{"active":true,"request":"UNIQUE_ID","trigger":"manual/debug",
 "requested_at":CURRENT_ROS_TIME_SECONDS,"timeout_s":5.0}
```

Use current ROS time (same use_sim_time/clock domain), unique IDs and send only
during FOLLOW_RETURN_PATH. Manual debug requests bypass the scheduler and do not
reset its anchor unless their UUID matches its pending request. `active:false`
with the matching request cancels. Do not reuse a historical timestamp example.

## Candidate selection and correction

Primary limit 3, selected by distance along RETURN path, not a global XY scan.
Reference metadata is projected in reversed outbound order. Recorded outbound
s and spatial projection disambiguate crossings; references already traversed
beyond route_backtrack_m are excluded. Return progress comes from the actual
controller's remaining corner-constrained interval and must be <=0.5s old.
Arcs are cached until path/reference changes. After a failed frame expand to at
most primary+2 (hard bound 7) within the same request. No global brute-force
fallback. Path simplification and outbound odom drift can bias s; complex routes
still require field tests. Legacy records missing order/s have weaker geometry
fallback. Process restart does not reload the mission.

Current SIFT runs once per processed check frame, references reuse descriptors.
Retain geometric verification for ALL bounded candidates to preserve existing
ambiguity rejection; choosing the highest descriptor count alone then running
one PnP would weaken that gate. Matching any nearby frame is not success:
depth, geometry, inliers, ratio, cheirality, reprojection, correction bound,
LOCK, freshness and two-frame agreement must all pass.

Validated corrections below 0.03m AND 0.01rad skip publication but reset the
watchdog. apply_correction=false always suppresses publication. Last validated
route arc advances in observation mode, but internal applied correction changes
only on actual publication. Explicit legacy yaw recovery retains its correction
commit behavior (deadband does not suppress its required commit).

## Launch parameters

All are exposed by manual_return_bringup.launch.py. **initial engineering defaults;
field calibration required**. Slip constants are provisional, not measured hardware values.

| Parameter | Default |
| --- | --- |
| enable_auto_keyframes | true |
| auto_keyframe_distance_m | 1.25 |
| auto_keyframe_yaw_deg | 12.0 |
| auto_keyframe_max_interval_s | 6.0 |
| auto_keyframe_min_motion_for_timeout_m | 0.5 |
| auto_keyframe_retry_s | 1.0 |
| auto_keyframe_min_features | 20 |
| auto_keyframe_blur_min | 0.0 (disabled) |
| enable_adaptive_visual_check | true |
| visual_check_distance_m | 8.0 |
| max_visual_candidates | 3 (configurable 1..7) |
| visual_check_retry_cooldown_s | 5.0 |
| visual_check_timeout_s | 5.0 |
| visual_check_max_frames | 4 |
| visual_check_on_return_start | false |
| enable_slip_early_trigger | true |
| slip_wz_deadband_radps | 0.15 |
| slip_score_trigger | 0.35 |
| slip_score_decay | 0.02 |
| wheel_timeout_s / gyro_timeout_s | 0.5 / 0.5 |
| diagnostics_timeout_s | 2.0 |
| correction_min_translation_m / correction_min_yaw_rad | 0.03 / 0.01 |

Correction stays false, recovery stays false, manual landmark_capture_mode stays
stop. enable_adaptive_visual_check=false explicitly opts into legacy periodic
localization (with bounded candidates); this is not the compute-off switch.
enable_visual_localization=false disables localization and scheduler processes.

## Execution and field verification

Build without starting motors:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_landmark return_navigation drive_cmd_mux robot_bringup
source install/setup.bash
```

Camera drivers and verified TF must be started separately. APC850 intrinsic and
both optical extrinsics are not invented or activated by this implementation.
Camera housing-back measurements are NOT optical translations; see CAMERA_VPR_TODO.md.
The launch below starts the existing motor/return stack, not a stationary harness.

Phase 1, outbound automatic capture only (ordinary return still available):

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  enable_auto_keyframes:=true enable_visual_localization:=false \
  visual_apply_correction:=false enable_landmark_recovery:=false
```

Start a NEW mission with localization enabled before recording for phase 2;
references are loaded only from live events (not historical archives):

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  enable_auto_keyframes:=true enable_adaptive_visual_check:=true \
  visual_apply_correction:=false enable_landmark_recovery:=false
```

Phase 3: verify 8m request and known x/y +0.2m, yaw +5° offsets, repeatability and
sign; no-match/camera disconnect/node shutdown must not interrupt ordinary return.
Phase 4: log straight/normal turn/in-place turn/high- and low-friction surfaces;
compare healthy disagreement distributions and tune deadband/score/decay before
relying on slip triggers. Do not force hazardous slip to tune a threshold.
Phase 5: only after TF, recognition and false-positive tests pass, enable
visual_apply_correction=true and a measured visual_expected_lock_pose_file.
Explicit yaw recovery remains disabled unless separately tested.

```bash
ros2 bag record -o adaptive_visual_bag \
 /wheel/odom /imu/gyro/z_corrected /odometry/diagnostics /odometry/filtered \
 /visual_landmark/event /visual_landmark/check_request /visual_landmark/scheduler_event \
 /visual_landmark/route_progress /visual_landmark/localization_event /visual_landmark/correction \
 /return_path /cmd_vel_return_path /mission/return/state /mission/origin_pose \
 /rear/camera/image_raw/compressed /rear/camera/camera_info \
 /arm/camera/color/image_raw/compressed /arm/camera/color/camera_info \
 /arm/camera/aligned_depth_to_color/image_raw /tf /tf_static
```

Performance: current_sift_s, attempts[].candidate_total_s,
attempts[].matching_and_depth_s, attempts[].pnp_s, processing_latency_s and
candidate_count are logged. Individual-frame latency is distinct from the full
request round-trip, which includes two-frame agreement and retries.
Synthetic benchmark (not camera accuracy or full-check latency):

```bash
python3 -m visual_landmark.benchmark_adaptive --iterations 10 --output /tmp/adaptive_benchmark.json
```

Measure request→terminal scheduler-event latency mean/max on rosbag too. CPU can
be recorded with pidstat on the two visual processes. When idle there must be no
current feature extraction/matching/PnP; RGB-D sync callbacks and scheduler are
still active. No real camera timing, slip-floor calibration, optical TF,
self-crossing recognition or final vehicle accuracy is established by unit tests.
