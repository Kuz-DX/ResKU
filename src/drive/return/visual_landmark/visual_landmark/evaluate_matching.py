"""Offline A/B/C/D preprocessing comparison; production remains raw SIFT."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from visual_landmark.calibration import load_calibration, camera_model
from visual_landmark.localization import features, match, pose_fields


def prepared(image, k, d, variant, current=False):
    processed = image if variant == 'A' else cv2.undistort(image, k, d)
    if variant in ('C', 'D'):
        processed = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8)).apply(processed)
    points, descriptors = features(processed)
    if variant == 'D' and not current:
        # Central 60% overlap hypothesis, with original image coordinates preserved.
        keep = ((points[:, 0] >= image.shape[1]*.2) & (points[:, 0] <= image.shape[1]*.8))
        points = points[keep]
        descriptors = descriptors[keep] if descriptors is not None else None
    if current and variant != 'A' and len(points):
        # Features detected on rectified RGB must sample ORIGINAL aligned depth.
        rays = np.c_[points, np.ones(len(points))] @ np.linalg.inv(k).T
        points = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2).astype(np.float32)
    return points, descriptors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, help='JSON list of image/calibration/depth pairs')
    parser.add_argument('--output', required=True, help='JSON report')
    args = parser.parse_args()
    root = Path(args.manifest).resolve().parent
    pairs = json.loads(Path(args.manifest).read_text())
    records, groups = [], {}
    for index, pair in enumerate(pairs):
        path = lambda key: root/pair[key]
        rd, rk, distortion = load_calibration(path('reference_calibration'))
        cd, ck, current_d = load_calibration(path('current_calibration'))
        reference = cv2.imread(str(path('reference_image')), cv2.IMREAD_GRAYSCALE)
        current = cv2.imread(str(path('current_image')), cv2.IMREAD_GRAYSCALE)
        if reference is None or current is None:
            raise ValueError('unreadable pair image')
        camera_model(rd['image_width'], rd['image_height'], rk, distortion, rd['distortion_model'], reference.shape)
        camera_model(cd['image_width'], cd['image_height'], ck, current_d, cd['distortion_model'], current.shape)
        if path('depth').suffix == '.npy':
            depth = np.load(path('depth'), allow_pickle=False).astype(np.float32)
        else:
            raw = cv2.imread(str(path('depth')), cv2.IMREAD_UNCHANGED)
            if raw is None or raw.dtype != np.uint16:
                raise ValueError('PNG depth must be uint16 millimetres; .npy must be metres')
            depth = raw.astype(np.float32)*.001
        if depth.shape != current.shape:
            raise ValueError('aligned depth/current resolution mismatch')
        for variant in 'ABCD':
            rp, descriptors = prepared(reference, rk, distortion, variant)
            current_features = prepared(current, ck, current_d, variant, current=True)
            diagnostics = dict(pair=index, variant=variant)
            result = match((rp, descriptors, rk, distortion if variant == 'A' else np.zeros_like(distortion)),
                           current_features, depth, ck, current_d, diagnostics=diagnostics)
            if result is not None:
                diagnostics['reference_from_current'] = result[0].tolist()
                if 'repeat_group' in pair:
                    groups.setdefault((variant, str(pair['repeat_group'])), []).append(result[0])
            records.append(diagnostics)
    repeatability = []
    for (variant, group), transforms in groups.items():
        xyz = np.asarray([t[:3, 3] for t in transforms])
        # Relative SO3 distance to first successful sample avoids Euler wrap artifacts.
        angles = [float(np.arccos(np.clip((np.trace(transforms[0][:3, :3].T@t[:3, :3])-1)/2, -1, 1))) for t in transforms]
        repeatability.append(dict(variant=variant, repeat_group=group, samples=len(transforms),
                                  xyz_std_m=xyz.std(axis=0).tolist(), rotation_rms_rad=float(np.sqrt(np.mean(np.square(angles))))))
    Path(args.output).write_text(json.dumps(dict(records=records, repeatability=repeatability), indent=2))
    print(f'{len(records)} results written to {args.output}; compare success and residuals on held-out real data')


if __name__ == '__main__':
    main()
