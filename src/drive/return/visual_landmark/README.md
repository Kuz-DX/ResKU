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
both cameras. The arm-camera mounting calibration and joint TF must be valid;
using a fixed mounting transform while the arm actually moves invalidates pose.
The EKF currently publishes chassis roll/pitch from the IMU in odom TF. Altitude
is not mapped: this is planar route correction with full relative camera pose,
not a globally consistent 3D terrain map.

Once a rear camera is available, pass its topics with `rear_image_topic` and
`rear_camera_info_topic`. Press D-pad up in MANUAL_RECORDING to capture a
landmark. Stop mode is preferred for initial validation. `meta.yaml` schema 2
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
