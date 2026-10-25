# APC850 rear RGB + locked arm D435i: deployment and validation

2026-10-24 hardware pause and next steps: [CAMERA_VPR_TODO.md](CAMERA_VPR_TODO.md).

## Existing design audit (local main a925e18)

The existing design is preserved: wheel vx + myAHRS gyro wz -> reduced_odom ->
continuous `/odometry/filtered`. SIFT mutual ratio matching lifts CURRENT aligned
D435i pixels into metric 3D, then PnP RANSAC + LM projects into REFERENCE APC850
pixels with the APC850's own K/D. Neither equal resolutions nor equal intrinsics
are required. Raw grayscale is still the production preprocessing.

No EKF redesign/reset, roll/pitch correction, SLAM, learned VPR download or MPPI
change is made. Relative camera pose is 6DOF but control correction is x,y,yaw.

Adaptive moving capture / request-gated checks are documented in [ADAPTIVE_VISUAL.md](ADAPTIVE_VISUAL.md).
Manual button capture remains stop mode; automatic capture never holds motion.

Two pre-existing behaviors needed changes for hardware validation: launch capture
was `moving`, and missing-landmark recovery could stop an odometry return even
when the visual node died. Defaults are now `stop`, observation-only visual
localization, and opt-in landmark recovery. A rejected visual match never clears
the last applied correction. With recovery disabled, no visual output is needed
for continuing local odometry return. Explicit recovery remains available for the
previously requested stop/search/manual-intervention behavior.

## Build and rear launch

Build in the usual sourced workspace (robot_bringup needs its existing dependencies):

```bash
colcon build --packages-select vision visual_landmark return_navigation drive_cmd_mux robot_bringup
source install/setup.bash
ls -l /dev/v4l/by-id/
v4l2-ctl --device /dev/v4l/by-id/ACTUAL_APC850-video-index0 --list-formats-ext
ros2 launch vision apc850_rear.launch.py \
  device:=/dev/v4l/by-id/ACTUAL_APC850-video-index0 \
  width:=1280 height:=720 fps:=15.0
```

`ACTUAL_APC850` is a placeholder to replace with the observed persistent device
name, not a fabricated device identity. APC850's stated maximum is 1920x1080 @30;
this is not a statement that a particular JECS USB setup supports every mode.
Select a listed MJPEG mode on the actual camera/USB bus. The provisional runtime
default 1280x720 @15 must be checked. `pixel_format=mjpeg2rgb` follows the JECS
side-camera configuration; no duplicate image_transport republisher is added.
Install compressed_image_transport and verify a single publisher of:

- `/rear/camera/image_raw`
- `/rear/camera/image_raw/compressed`
- `/rear/camera/camera_info`

Camera name is `apc850_rear`, frame is `rear_camera_optical_frame`. Missing
calibration warns but still permits image archival; localization rejects it.
Launch validates resolution for an existing `file://` calibration. Runtime also
validates resolution for package URLs or calibration loaded later by the driver.

## Intrinsic calibration: measure, do not fill placeholders

1. Secure the camera on a stable support (vehicle installation is not required
   for intrinsics), choose final runtime resolution, and keep the same
   focus/zoom/runtime image processing thereafter. Use the above launch.
2. Use a 9x6 **inner-corner** checkerboard. Measure square edge length in metres;
   the 0.030 example below is only valid for an actually measured 30 mm board.
3. Collect at least 20-30 varied frames: centre, all four corners and edges,
   several distances, and different board yaw/pitch/roll. Avoid blurred frames.
4. ROS calibration option:

```bash
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.030 \
  image:=/rear/camera/image_raw camera:=/rear/camera
```

Save/export the measured camera_info YAML, set camera_name to `apc850_rear`, and
verify the saved resolution. usb_cam's set-camera-info service availability can
vary; export/save a YAML rather than assuming Commit updated the runtime file.
Copy the **measured** YAML to:

```
src/vision/camera/config/calibration/apc850_rear.yaml
```

Rebuild vision; installed location:
`install/vision/share/vision/config/camera/calibration/apc850_rear.yaml`.
No numerical template is provided. A directory README is installed even before
hardware calibration exists.

5. Restart the rear driver with the measured file:

```bash
ros2 launch vision apc850_rear.launch.py \
  device:=/dev/v4l/by-id/ACTUAL_APC850-video-index0 \
  width:=1280 height:=720 fps:=15.0 \
  camera_info_url:=package://vision/config/camera/calibration/apc850_rear.yaml
ros2 topic echo /rear/camera/camera_info --once
```

Verify frame_id, width/height, nonzero K[0]/K[4], finite D and the intended model.
Inspect undistorted images for straight physical edges, especially near borders.
Approximate 110-degree FOV does not establish a fisheye model. Start with
`plumb_bob`. `rational_polynomial` is also supported; `equidistant`/fisheye is
explicitly rejected, not silently interpreted as Brown distortion.

### Held-out intrinsic validation and rational-model comparison

Save raw checkerboard frames as PNG, partition into `fit/` and separate
`validation/` sets. Do not compare training RMS alone. Offline fitting is available:

```bash
ros2 run visual_landmark validate_camera_calibration fit \
  --images 'dataset/fit/*.png' --square 0.030 --board 9x6 \
  --model plumb_bob --calibration /tmp/apc850_plumb.yaml --output-dir /tmp/plumb_fit
ros2 run visual_landmark validate_camera_calibration fit \
  --images 'dataset/fit/*.png' --square 0.030 --board 9x6 \
  --model rational_polynomial --calibration /tmp/apc850_rational.yaml --output-dir /tmp/rational_fit
ros2 run visual_landmark validate_camera_calibration validate \
  --images 'dataset/validation/*.png' --square 0.030 --board 9x6 \
  --calibration /tmp/apc850_plumb.yaml --output-dir /tmp/plumb_validation
# Repeat validate with /tmp/apc850_rational.yaml and a different output-dir.
```

Outputs include fx/fy/cx/cy, resolution/model, reprojection RMS, per-image and
max errors, per-image edge RMS, coverage hull fraction and occupied 4x3 bins,
plus up to five undistorted previews. Each held-out board pose is fitted with
fixed K/D; K/D are not re-estimated during validation. Prefer a simpler model
when held-out/edge errors and physical straightness are satisfactory.
RMS <0.5 px is recommended, 0.5-0.8 px is trial quality, >=1 px warns. These are
engineering indicators, not hard safety guarantees. Check undetected frames and
coverage too; low RMS on only central images is insufficient.

## Extrinsics and the existing arm TF tree: deployment blocker to resolve

The current active arm wrapper is
`tool_manipulator_moveit_config/config/tool_manipulator.urdf.xacro`, which includes
`tool_manipulator_description/urdf/tool_manipulator.urdf.xacro`.
The source chain is:

```
ground -> world -> arm_world -> base_actuator -> base_link (arm base_joint)
       -> arm joints -> tcp_link -> cam_link
       -> arm_camera_link -> arm_camera_color_optical_frame
```

The final two edges are owned respectively by the calibrated
`arm_camera_extrinsics` broadcaster and RealSense driver. `real_control.launch.py`
starts joint_state_broadcaster and robot_state_publisher; it does not itself
start RealSense or the separate camera mounting broadcaster.
`arm_camera_extrinsics.yaml` already contains repository mounting measurements;
this change does not replace them. RealSense configs differ: the vision arm
config uses `camera_name=arm_camera`, while dolbotz's arm_cam config uses
`camera_name=camera` and documents potential drive-camera frame collisions.
Do not run both arm camera drivers at once. Verify actual optical names.

**Important:** drive odometry also uses `base_link` as the vehicle body. The arm
URDF uses `base_link` as a revolute arm link. Starting both unchanged may give
that frame two parents/authorities. A mathematically valid lookup is not proof
that it references the vehicle body. Do not patch this with guessed zero TFs.
Before vehicle tests, separate the arm link names (updating all arm consumers as
an integration change) and attach the arm root to the uniquely owned vehicle
frame using measured mounting geometry. Or provide an already integrated robot
TF tree with unique frame ownership. This task does not rename the whole arm
MoveIt/URDF system or invent that bridge. It is a real deployment prerequisite.

Verify on the actual combined system:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link arm_camera_color_optical_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link rear_camera_optical_frame
ros2 topic info /tf --verbose
```

`base_link` in these commands must be the **vehicle**, not the arm joint link.
Record a bag of /tf and /tf_static and inspect authorities if names conflict.
Do not add a base->arm optical shortcut while the same optical child already
has a parent. There must be exactly one authoritative chain per child.

Rear mounting option: `apc850_rear.launch.py` publishes **no TF by default**.
2026-10-25 housing-back measurements (vehicle `base_link` origin, metres; NOT optical origin):

| Camera | x | y | z | optical roll/pitch/yaw |
| --- | --- | --- | --- | --- |
| Rear APC850 | -0.316627 | 0.0 | 0.650674 | unmeasured |
| Front arm D435i at intended LOCK pose | 0.152853 | 0.0 | 0.650674 | unmeasured |

These partial measurements are also recorded in
`src/vision/camera/config/vpr_mount_measurements.yaml` (installed under
`share/vision/config/camera`). This is a measurement record, not an input to the
TF broadcaster or `visual_expected_lock_pose_file`: unknown rotation prevents
forming a complete LOCK matrix. The arm's existing articulated TF remains the
runtime source of its extrinsics.

All rear optical translation defaults are now empty: the user clarified that
these positions refer to the camera housing back, not the optical origin.
Measure the back-reference-to-RGB-optical offset and mounting rotation first.
Do not publish the housing coordinates as an optical TF.

If rear optical TF is not already in URDF, supply the remaining measured
values and opt in. RPY is fixed-axis radians for parent<-optical-child:

```bash
ros2 launch vision apc850_rear.launch.py \
  device:=/dev/v4l/by-id/ACTUAL_APC850-video-index0 \
  camera_info_url:=package://vision/config/camera/calibration/apc850_rear.yaml \
  publish_extrinsic:=true parent_frame:=base_link \
  x:=OPTICAL_X y:=OPTICAL_Y z:=OPTICAL_Z \
  roll:=MEASURED_ROLL pitch:=MEASURED_PITCH yaw:=MEASURED_YAW
```

Replace all OPTICAL/MEASURED values; empty/invalid values refuse TF publication. Optical
axes are x right, y down, z forward. Rear-facing yaw alone is insufficient to
specify the optical rotation. Equal front/rear height does not remove their
translation difference or substitute for mounting measurement.

### D435i LOCK pose

The recorded front housing-back position is (0.152853, 0, 0.650674)m.
Convert the housing reference to the D435i RGB optical origin before comparing
against vehicle<-D435i optical TF. Do not use the housing position directly in
a LOCK matrix or overwrite the TCP-to-camera mounting transform with it.

Physically lock/command a repeatable arm pose for the entire VPR return. The
optional guard compares image-time vehicle<-D435i TF with expected LOCK TF,
rejecting >2 cm translation or >0.035 rad rotation. By default it snapshots the
first valid return frame; this detects later motion but does **not** establish
that the first pose is the desired LOCK pose. For repeatable deployment supply
`visual_expected_lock_pose_file` with a measured YAML `base_from_camera` 4x4
matrix for the intended pose. No invented numeric matrix is supplied.
A stable but incorrectly calibrated TF, duplicate authority, or joint encoder
bias cannot be detected from this difference check alone.

## Transform convention and quality gates

All matrices denote parent<-child. PnP object points are in the CURRENT camera,
image points in the REFERENCE camera, so OpenCV returns reference<-current:

```
T_M_B = T_M_Cref * T_Cref_Ccurrent * inverse(T_B_Ccurrent)
C_M_O = planar(T_M_B) * inverse(planar(T_O_B))
corrected_current = C_M_O * planar(current_T_O_B)
```

`T_M_Cref` and capture extrinsic use the reference image time; current TF uses
current RGB time. Body roll/pitch and the optical mounting geometry participate
in relative SE3, but only x,y,yaw is applied to the persistent mission<-odom
correction. The EKF and raw odometry remain continuous.

Default matching/depth gates are retained:

- SIFT 2000 features; mutual Lowe ratio <0.7 in both directions.
- 3x3 depth neighbourhood: all finite/in range 0.25-8 m, median depth;
  max-min <=8% of median rejects foreground boundaries/discontinuities.
- Current K/D undistorts rays before depth back-projection; reference K/D goes
  to PnP and reprojection. Neither image is implicitly resized.
- RANSAC 300 iterations, 3 px threshold, confidence 0.999; >=20 inliers and >=45%.
- Second singular value of centred 3D support >=0.1 m and image span >=60 px
  in both axes; refinement RMSE <=2.5 px and positive reference-camera depth.
- Competing score >=85% of best is ambiguous and rejected.
- Two consecutive frames within 2.5 s agree within 0.25 m and 0.1 rad;
  duplicated/reversed timestamps cannot count as a second observation.
- Step relative to previous prior <=3 m and 0.7 rad.
- Current RGB age <=0.5 s, processing completion age <=1.5 s; synchronized
  color/aligned depth within 40 ms, same optical frame, same current resolution.
- TF is queried at the image time; no latest-TF fallback. Static transforms
  are timeless; dynamic extrapolation failure rejects the reference/match.
- Candidate radius 12 m is retained even in explicit search. Once a landmark
  is accepted, route arc cannot backtrack by more than 2 m. This prior does not
  solve all self-crossing/identical-scene ambiguities.

Matching/depth thresholds are node parameters under `match.*`. Agreement,
ambiguity, pose-lock and correction limits are also parameters. Log-based
changes must not casually weaken false-positive rejection. Existing tests and
synthetic geometry remain the baseline, not evidence of field accuracy.

JSON localization events preserve existing keys and add diagnostics: feature
counts, raw/mutual matches, depth correspondence and inlier counts/ratio,
reprojection RMSE, relative xyz/rpy, planar correction, step size, TF matrices,
models/resolutions, rejection reason and processing latency. `validated` means
observation-only; only `localized` implies a correction message was published.
The logged planar correction is the absolute persistent mission<-odom transform;
`diagnostics.correction_step_*` describes the change at the current robot pose.

## Offline A/B/C/D matching evaluation

Create a JSON list manifest. Paths are relative to the manifest file:

```json
[
  {"reference_image":"rear/L000.jpg", "reference_calibration":"rear.yaml",
   "current_image":"return/000.jpg", "current_calibration":"arm.yaml",
   "depth":"return/000_depth.png", "repeat_group":"stationary_pose_1"}
]
```

Use uint16 PNG depth in millimetres, or `.npy` float depth in metres. All pairs
in a repeat_group must be the **same physical camera pose**. Different groups
represent different offsets. No fabricated dataset/results are supplied.

```bash
ros2 run visual_landmark evaluate_landmark_matching \
  --manifest dataset/pairs.json --output /tmp/ab_matching.json
```

A=raw grayscale; B=undistorted; C=undistorted+CLAHE; D=C with central 60% reference
width. Rectified current feature coordinates are mapped back to original pixels
for aligned-depth sampling; rectified reference uses zero D in PnP. Compare
raw/mutual matches, PnP inliers/ratio, RMSE and per-group xyz/rotation repeatability.
Choose the simplest successful variant. The evaluation never changes production
preprocessing automatically, and color similarity is not an acceptance criterion.

## Staged vehicle validation and PASS/FAIL

A. Intrinsic only: held-out reports and straight-line previews as above. PASS:
adequate edge/corner coverage, correct resolution/frame, recommended/trial RMS
with physical straightness. FAIL: wrong size, zero K, nonfinite D, large edge bias.

B. Stationary single landmark: keep controller disconnected/disabled or wheels
unpowered for bench capture. Use actual APC850 outbound capture, rotate the
robot 180 degrees, lock D435i pose. Start with defaults:

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  visual_apply_correction:=false enable_landmark_recovery:=false
ros2 topic echo /visual_landmark/localization_event
ros2 topic echo /visual_landmark/correction
```

The bringup also starts motors and ordinary return logic; observation-only does
**not** disable those components. Do not issue `/path/return` to powered wheels
for an intended stationary test. To evaluate without controller/motor actuation,
run just the visual nodes with existing sensor/TF/origin/state publishers or
replay a bag remapping `/visual_landmark/correction` and drive command topics
away from live controllers. A bench harness may publish the required mission
state/origin with no robot controller running.

PASS: `validated` after two consistent frames, no correction topic publication,
stable reference/current frame/model/resolution/TF diagnostics. FAIL: wrong TF
chain, arm lock guard trips, ambiguous scenes or unstable poses. No control
correction is applied in this phase.

C. Known offsets: mark physical references and move +0.10 m lateral, +0.20 m
longitudinal, +5 degrees yaw from a measured return pose. Compare relative pose
and correction *step*, not absolute C_M_O translation alone. Repeat each pose
at least ten times and include same-pose repeats. Provisional PASS: correct
sign/direction, translation error <=5 cm and yaw error <=2 degrees; these are
initial project acceptance targets, not proven sensor specifications. FAIL:
wrong sign, systematic bias, insufficient repeatability or any false positive.

D. Several landmarks: verify expected route order, no competing-landmark
acceptance, depth holes/no-match/stale TF reject without changing correction.
Kill the visual node with recovery disabled; local odometry return must remain
operational. FAIL if the visual node resets odometry or an old mission is reused.

E. Powered Back-to-Base only after A-D pass and TF naming conflict is resolved:

```bash
ros2 launch robot_bringup manual_return_bringup.launch.py \
  visual_apply_correction:=true visual_lock_pose_guard:=true \
  visual_expected_lock_pose_file:=/absolute/path/measured_lock_pose.yaml \
  enable_landmark_recovery:=false landmark_capture_mode:=stop
```

Compare matched route trials with visual correction on/off, surveyed final Base
error, correction discontinuities, missed landmarks and latency. PASS: repeated
improvement against the measured baseline, no false acceptance and no regression
in Manual/Auto switching. Stop if offsets are not reproducible or TF is unstable.
Only then optionally enable `enable_landmark_recovery:=true` for bounded ±30°
search; this changes the no-match behavior to an intentional stop and may request
manual intervention. Never infer that correction mode adds obstacle avoidance.

Recommended bag inputs:

```bash
ros2 bag record /rear/camera/image_raw/compressed /rear/camera/camera_info \
  /arm/camera/color/image_raw/compressed /arm/camera/color/camera_info \
  /arm/camera/aligned_depth_to_color/image_raw /odometry/filtered /tf /tf_static \
  /mission/origin_pose /mission/return/state /return_path \
  /visual_landmark/event /visual_landmark/localization_event /visual_landmark/correction
```

## Hardware values deliberately absent

Actual APC850 by-id identity and supported modes; measured APC850 K/D and
validation dataset; full rear optical extrinsic; intended D435i LOCK joints and
measured vehicle<-optical LOCK matrix; resolved combined arm/vehicle TF ownership;
measured cross-camera accuracy, depth reliability and JECS processing latency.
No code-only test can supply these values or certify field readiness.

Primary references: [OpenCV calibration/PnP convention](https://docs.opencv.org/4.11.0/d9/d0c/group__calib3d.html),
[ROS monocular calibration tutorial source](https://github.com/ros-perception/image_pipeline/blob/rolling/camera_calibration/doc/tutorial_mono.rst).

## Modified files

New:

- `src/vision/camera/launch/apc850_rear.launch.py`
- `src/vision/camera/config/calibration/README.md`
- `src/drive/return/visual_landmark/visual_landmark/calibration.py`
- `src/drive/return/visual_landmark/visual_landmark/calibration_tool.py`
- `src/drive/return/visual_landmark/visual_landmark/evaluate_matching.py`
- `src/drive/return/visual_landmark/test/test_cross_camera.py`
- `src/drive/return/visual_landmark/test/test_rear_launch.py`
- `src/drive/return/visual_landmark/HARDWARE_VALIDATION.md`

Updated:

- `src/vision/setup.py`, `src/vision/package.xml`
- `src/drive/return/visual_landmark/setup.py`, `README.md`
- `src/drive/return/visual_landmark/visual_landmark/localization.py`
- `src/drive/return/visual_landmark/visual_landmark/visual_landmark_node.py`
- `src/drive/return/visual_landmark/visual_landmark/visual_localization_node.py`
- `src/drive/return/visual_landmark/test/test_recording_states.py` (fixture updated,
  existing regression assertions retained)
- `src/drive/return/return_navigation/return_navigation/return_path_follower_node.py`
- `src/drive/autonomous/robot_bringup/launch/manual_return_bringup.launch.py`

Validation in this environment: 83 tests passed; return_navigation,
visual_landmark, drive_cmd_mux and vision built in an isolated /tmp install.
Installed calibration/evaluation CLI help and APC850 launch --show-args passed.
No hardware performance result, measured calibration, commit or push is claimed.
