"""Checkerboard optical extrinsic candidates. All transforms are parent<-child."""
import cv2
import numpy as np
from visual_landmark.localization import valid_transform


def board_points(columns=9, rows=6, square=.015):
    if columns < 3 or rows < 3 or not np.isfinite(square) or square <= 0:
        raise ValueError('invalid measured board dimensions')
    points = np.zeros((columns*rows, 3), np.float64)
    points[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2)*square
    return points


def surveyed_board(p0, px, py, columns=9, rows=6, square=.015, tolerance=.005):
    """Measure the exact labelled inner corners 0, columns-1, (rows-1)*columns.

    Camera pose is NOT observable in base coordinates without this independent
    board survey. Metres, vehicle axes x forward/y left/z up.
    """
    p0, px, py = (np.asarray(v, dtype=float) for v in (p0, px, py))
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (p0,px,py)):
        raise ValueError('all three surveyed XYZ points must be finite and measured')
    x, y = px-p0, py-p0
    lx, ly = np.linalg.norm(x), np.linalg.norm(y)
    if abs(lx-(columns-1)*square) > tolerance or abs(ly-(rows-1)*square) > tolerance:
        raise ValueError('survey lengths disagree with measured checkerboard dimensions')
    if min(lx, ly) < 1e-6 or abs(np.dot(x/lx, y/ly)) > .05:
        raise ValueError('survey axes must be nonzero and perpendicular')
    x = x/lx
    y = y-x*np.dot(x, y)
    y = y/np.linalg.norm(y)
    result = np.eye(4)
    result[:3, :3] = np.column_stack([x, y, np.cross(x,y)])
    result[:3, 3] = p0
    return result


def optical_candidates(corners, k, d, base_from_board=None, columns=9, rows=6, square=.015):
    points = board_points(columns, rows, square)
    pixels = np.asarray(corners, dtype=float).reshape(-1,2)
    if pixels.shape != (len(points),2) or not np.all(np.isfinite(pixels)):
        raise ValueError('invalid detected corners')
    if base_from_board is not None and not valid_transform(base_from_board):
        raise ValueError('invalid surveyed base_from_board')
    ok, rotations, translations, _ = cv2.solvePnPGeneric(points, pixels, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if not ok:
        return candidates
    for r,t in zip(rotations, translations):
        rotation = cv2.Rodrigues(r)[0]
        if np.min((points@rotation.T+t.reshape(3))[:,2]) <= 0:
            continue
        tf = np.eye(4)
        tf[:3,:3],tf[:3,3] = rotation,t.reshape(3)
        projected = cv2.projectPoints(points,r,t,k,d)[0].reshape(-1,2)
        rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
        candidates.append(dict(reprojection_rms_px=rms,camera_from_board=tf.tolist(),
            base_from_camera=None if base_from_board is None else (base_from_board@np.linalg.inv(tf)).tolist()))
    return sorted(candidates,key=lambda c:c['reprojection_rms_px'])
