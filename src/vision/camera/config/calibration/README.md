# Measured camera calibration only

Resume checklist: `src/drive/return/visual_landmark/CAMERA_VPR_TODO.md`.

Save measured APC850 ROS camera_info YAML as `apc850_rear.yaml` here, then rebuild
`vision`. It installs to `share/vision/config/camera/calibration/apc850_rear.yaml`.
The canonical `apc850_rear.yaml` is not supplied until validation is complete.
Use camera_name `apc850_rear`; YAML image_width/image_height MUST equal runtime.
Do not substitute example identity K/D or zero mounting geometry.

See `src/drive/return/visual_landmark/HARDWARE_VALIDATION.md` for collection,
held-out validation, calibration model comparison and mounting TF instructions.

## 2026-10-24 trial measurement

`apc850_rear_trial_1280x720.yaml` is a measured **trial** plumb_bob calibration,
not automatically selected by any launch. 9x6 inner corners, measured 15 mm
squares, MJPEG 1280x720 at 15 fps. Fit: 51/60 detected frames, RMS 0.616 px.
Separate validation: 30/30 detected, RMS 0.658 px, maximum residual 6.537 px,
9/12 coverage bins. The first shaken take was excluded entirely.

See `apc850_rear_trial_1280x720_report.json` for device identity and metrics.
Raw images, full reports and undistorted previews are preserved locally under
`debug_report/camera_calibration/20261024_take2/` (git ignored).
Board flatness, edge quality and final mounting/focus still need verification;
do not treat this trial as completed vehicle localization validation.
No extrinsic TF was calibrated or enabled by this collection.

## Flat-board repeat (2026-10-24, take 3)

`apc850_rear_flat_trial_1280x720.yaml` preserves a separate candidate measured
after the user secured the board flat, with the same resolution and board size.
Fit: 52/60 detected, RMS 0.439 px, 8/12 coverage bins, hull fraction 0.269.
Held-out validation: 30/30 detected, RMS 0.440 px, maximum residual 1.832 px,
6/12 coverage bins, hull fraction 0.189. Residuals improved over take 2, but
edge/corner coverage is incomplete: the report's RMS-only `recommended` label
does not establish complete calibration quality. Collect additional diverse
edge/corner poses before final promotion. Neither candidate is auto-activated.

Metrics: `apc850_rear_flat_trial_1280x720_report.json`. Raw frames and full
reports: `debug_report/camera_calibration/20261024_flat_take3/` (git ignored).

## Additional edge collection (2026-10-24, take 4)

`apc850_rear_edges_trial_1280x720.yaml` was fitted from take 3 fit images plus
60 new images; take 3 validation images were not added to fitting. Fit:
108/120 detected, RMS 0.420 px, 8/12 bins, hull fraction 0.316.
A separate new validation sequence detected 27/30 boards, RMS 0.401 px,
maximum residual 1.333 px, 6/12 bins, hull fraction 0.211.
Residuals improved, but edge coverage remains incomplete. This is another trial
candidate, not the canonical runtime calibration or proof of vehicle accuracy.
Raw images and full reports are preserved locally under
`debug_report/camera_calibration/20261024_edges_take4/` (git ignored).
Session metrics are in `apc850_rear_edges_trial_1280x720_report.json`.

## Live-preview guided collection (2026-10-24, take 5)

`apc850_rear_preview_trial_1280x720.yaml` uses take 3 fit images sampled every
third frame and take 5 fit images every second frame (47/50 detected). Sampling
is temporal, not based on residuals. Fit RMS 0.552 px, 9/12 bins.
New independent validation: 29/30 detected, RMS 0.783 px, maximum 4.980 px,
10/12 bins. On these same validation frames the take 4 candidate gives RMS
1.229 px and maximum 15.781 px, exposing limitations not evident in its earlier
central validation. Take 5 improves on that comparison but is still trial
quality with incomplete coverage; no canonical/runtime calibration is promoted.
Preview grid/text were drawn only on the display, never saved into raw frames.
Raw data/full reports: `debug_report/camera_calibration/20261024_preview_take5/`.
Metrics and selection provenance: `apc850_rear_preview_trial_1280x720_report.json`.
