"""Offline measured checkerboard calibration and held-out quality validation."""
import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from visual_landmark.calibration import load_calibration


def observations(pattern, square, filenames):
    board = np.zeros((pattern[0]*pattern[1], 3), np.float32)
    board[:, :2] = np.mgrid[:pattern[0], :pattern[1]].T.reshape(-1, 2)*square
    found, size = [], None
    for filename in filenames:
        image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('cannot read '+filename)
        if size is not None and size != image.shape[::-1]:
            raise ValueError('mixed image resolutions')
        size = image.shape[::-1]
        ok, corners = cv2.findChessboardCornersSB(image, pattern)
        if ok:
            found.append((filename, corners.astype(np.float32)))
    if not found:
        raise ValueError('no checkerboards detected')
    return board, found, size


def validate(board, found, size, k, d, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    errors, per_image, all_corners = [], [], []
    for index, (filename, corners) in enumerate(found):
        ok, r, t = cv2.solvePnP(board, corners, k, d)
        if not ok:
            raise ValueError('validation solvePnP failed: '+filename)
        predicted = cv2.projectPoints(board, r, t, k, d)[0]
        residual = np.linalg.norm((predicted-corners).reshape(-1, 2), axis=1)
        errors.extend(residual.tolist())
        pixels = corners.reshape(-1, 2)
        edge = ((pixels[:, 0] < size[0]*.15) | (pixels[:, 0] > size[0]*.85)
                | (pixels[:, 1] < size[1]*.15) | (pixels[:, 1] > size[1]*.85))
        per_image.append(dict(file=filename, rms_px=float(np.sqrt(np.mean(residual**2))),
                              max_px=float(residual.max()),
                              edge_rms_px=float(np.sqrt(np.mean(residual[edge]**2))) if edge.any() else None))
        all_corners.extend(pixels.tolist())
        if index < 5:
            image = cv2.imread(filename)
            cv2.imwrite(str(output_dir/f'undistorted_{index:03d}.png'), cv2.undistort(image, k, d))
    pixels = np.asarray(all_corners, np.float32)
    hull = cv2.convexHull(pixels)
    bins = {(min(3, max(0, int(x/size[0]*4))), min(2, max(0, int(y/size[1]*3)))) for x, y in pixels}
    rms = float(np.sqrt(np.mean(np.square(errors))))
    return dict(resolution=list(size), fx=float(k[0, 0]), fy=float(k[1, 1]),
                cx=float(k[0, 2]), cy=float(k[1, 2]), reprojection_rms_px=rms,
                max_reprojection_px=max(errors), per_image=per_image,
                coverage_hull_fraction=float(cv2.contourArea(hull)/(size[0]*size[1])),
                coverage_bins_4x3=len(bins), detected_images=len(found),
                quality='recommended' if rms < .5 else 'trial' if rms <= .8 else 'review',
                warnings=['RMS >=1 px: recalibration recommended'] if rms >= 1 else [])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['fit', 'validate'])
    parser.add_argument('--images', required=True, help='Quoted image glob, keep fit and validation sets separate')
    parser.add_argument('--calibration', required=True, help='Measured ROS YAML input (validate) or output (fit)')
    parser.add_argument('--square', type=float, required=True, help='Measured square edge in metres')
    parser.add_argument('--board', default='9x6', help='Inner corners, columns x rows')
    parser.add_argument('--model', choices=['plumb_bob', 'rational_polynomial'], default='plumb_bob')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    if not np.isfinite(args.square) or args.square <= 0:
        parser.error('square must be measured and positive')
    pattern = tuple(map(int, args.board.split('x')))
    if len(pattern) != 2 or min(pattern) < 3:
        parser.error('invalid board')
    board, found, size = observations(pattern, args.square, sorted(glob.glob(args.images)))
    if args.mode == 'fit':
        if len(found) < 20:
            parser.error('at least 20 detected diverse fit images required')
        flags = cv2.CALIB_RATIONAL_MODEL if args.model == 'rational_polynomial' else 0
        training_rms, k, d, _, _ = cv2.calibrateCamera([board]*len(found), [x[1] for x in found], size, None, None, flags=flags)
        d = d.ravel()[:8 if args.model == 'rational_polynomial' else 5]
        matrix = lambda a: dict(rows=a.shape[0], cols=a.shape[1], data=a.ravel().tolist())
        data = dict(image_width=size[0], image_height=size[1], camera_name='apc850_rear',
                    camera_matrix=matrix(k), distortion_model=args.model,
                    distortion_coefficients=matrix(d.reshape(1, -1)),
                    rectification_matrix=matrix(np.eye(3)), projection_matrix=matrix(np.c_[k, np.zeros(3)]))
        Path(args.calibration).write_text(yaml.safe_dump(data))
        print(json.dumps(dict(training_rms_px=training_rms, warning='Fit-set RMS is not held-out validation. Run validate on separate images.')))
    else:
        data, k, d = load_calibration(args.calibration)
        if size != (data['image_width'], data['image_height']):
            parser.error('validation/calibration resolution mismatch')
    report = validate(board, found, size, k, d, args.output_dir)
    report.update(distortion_model=data['distortion_model'], input_images=len(glob.glob(args.images)),
                  dataset_role='training' if args.mode == 'fit' else 'held_out_validation')
    Path(args.output_dir, 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
