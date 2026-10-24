# Measured camera calibration only

Save measured APC850 ROS camera_info YAML as `apc850_rear.yaml` here, then rebuild
`vision`. It installs to `share/vision/config/camera/calibration/apc850_rear.yaml`.
No runtime YAML is supplied because no physical calibration has been performed.
Use camera_name `apc850_rear`; YAML image_width/image_height MUST equal runtime.
Do not substitute example identity K/D or zero mounting geometry.

See `src/drive/return/visual_landmark/HARDWARE_VALIDATION.md` for collection,
held-out validation, calibration model comparison and mounting TF instructions.
