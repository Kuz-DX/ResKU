"""Metric RGB-D to RGB localization. Matrices use parent <- child convention."""
from dataclasses import dataclass

import time
import cv2
import numpy as np


@dataclass(frozen=True)
class MatchOptions:
    min_inliers: int = 20
    min_inlier_ratio: float = 0.45
    ratio_test: float = 0.7
    ransac_px: float = 3.0
    max_rmse_px: float = 2.5
    min_depth_m: float = 0.25
    max_depth_m: float = 8.0
    depth_edge_ratio: float = 0.08

    def __post_init__(self):
        if (self.min_inliers < 20 or not .45 <= self.min_inlier_ratio <= 1
                or not 0 < self.ratio_test < 1 or not 0 < self.max_rmse_px <= self.ransac_px
                or not 0 < self.min_depth_m < self.max_depth_m or not 0 < self.depth_edge_ratio < 1):
            raise ValueError('invalid match thresholds; minimum 20 inliers and 45% ratio retained')


def valid_transform(t):
    t = np.asarray(t, dtype=float)
    return (t.shape == (4, 4) and np.all(np.isfinite(t))
            and np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
            and np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            and abs(np.linalg.det(t[:3, :3])-1) < 1e-5)


def pose_fields(t):
    r = t[:3, :3]
    return dict(xyz=t[:3, 3].tolist(), rpy=[float(np.arctan2(r[2, 1], r[2, 2])),
        float(np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))), float(np.arctan2(r[1, 0], r[0, 0]))])


def transform(xyz, xyzw):
    q = np.asarray(xyzw, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12 or not np.all(np.isfinite(xyz)):
        raise ValueError('invalid transform quaternion/translation')
    q /= np.linalg.norm(q)
    x, y, z, w = q
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = xyz
    return t


def planar(t):
    yaw = np.arctan2(t[1, 0], t[0, 0])
    return transform([t[0, 3], t[1, 3], 0], [0, 0, np.sin(yaw/2), np.cos(yaw/2)])


def features(image):
    keypoints, descriptors = cv2.SIFT_create(nfeatures=2000).detectAndCompute(image, None)
    return np.asarray([k.pt for k in keypoints], dtype=np.float32).reshape(-1, 2), descriptors


def estimate_pose(points3d, pixels, k, distortion, min_inliers=20, options=None, diagnostics=None):
    """Return reference_camera <- current_camera and geometric quality, or None."""
    options = options or MatchOptions(min_inliers=min_inliers)
    min_inliers = options.min_inliers
    diagnostics = {} if diagnostics is None else diagnostics
    diagnostics.update(pnp_points=len(points3d), inliers=0, inlier_ratio=0., reject_reason='insufficient_depth_correspondences')
    if len(points3d) < min_inliers:
        return None
    xyz = np.ascontiguousarray(points3d, dtype=np.float64)
    uv = np.ascontiguousarray(pixels, dtype=np.float64)
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(uv)):
        diagnostics['reject_reason'] = 'nonfinite_correspondences'
        return None
    pnp_started = time.perf_counter()
    ok, r, t, indices = cv2.solvePnPRansac(
        xyz, uv, k, distortion, iterationsCount=300, reprojectionError=options.ransac_px,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    diagnostics['pnp_s'] = time.perf_counter()-pnp_started
    diagnostics['reject_reason'] = 'pnp_inliers'
    if indices is not None:
        diagnostics.update(inliers=len(indices), inlier_ratio=len(indices)/len(xyz))
    if not ok or indices is None or len(indices) < min_inliers or len(indices)/len(xyz) < options.min_inlier_ratio:
        return None
    idx = indices.ravel()
    diagnostics['reject_reason'] = 'clustered_geometry'
    # Avoid clustered correspondences giving a poorly constrained pose.
    if np.linalg.svd(xyz[idx] - xyz[idx].mean(axis=0), compute_uv=False)[1] < 0.1:
        return None
    if np.min(np.ptp(uv[idx], axis=0)) < 60:
        return None
    refine_started = time.perf_counter()
    r, t = cv2.solvePnPRefineLM(xyz[idx], uv[idx], k, distortion, r, t)
    diagnostics['pnp_s'] += time.perf_counter()-refine_started
    rot = cv2.Rodrigues(r)[0]
    projected = cv2.projectPoints(xyz[idx], r, t, k, distortion)[0].reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum((projected - uv[idx])**2, axis=1))))
    diagnostics.update(reprojection_rmse=error, reject_reason='reprojection_or_cheirality')
    if error > options.max_rmse_px or np.any((xyz[idx] @ rot.T + t.reshape(3))[:, 2] <= 0):
        return None
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rot, t.ravel()
    diagnostics.update(reject_reason=None, relative_pose=pose_fields(result))
    return result, len(idx), error


def _match(reference, current, depth, current_k, current_d, options=None, diagnostics=None):
    options = options or MatchOptions()
    diagnostics = {} if diagnostics is None else diagnostics
    rp, rd, rk, distortion = reference
    cp, cd = current
    diagnostics.update(reference_features=len(rp), current_features=len(cp), raw_matches=0, mutual_ratio_matches=0, reject_reason='missing_descriptors')
    if rd is None or cd is None or len(rd) < 2 or len(cd) < 2:
        return None
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(cd, rd, k=2)
    diagnostics['raw_matches'] = len(forward)
    reverse = matcher.knnMatch(rd, cd, k=2)
    back = {a.queryIdx: a.trainIdx for pair in reverse if len(pair) == 2
            for a, b in [pair] if a.distance < options.ratio_test*b.distance}
    pairs = [(a.queryIdx, a.trainIdx) for pair in forward if len(pair) == 2
             for a, b in [pair] if a.distance < options.ratio_test*b.distance and back.get(a.trainIdx) == a.queryIdx]
    diagnostics['mutual_ratio_matches'] = len(pairs)
    xyz, pixels = [], []
    for ci, ri in pairs:
        u, v = cp[ci]
        x, y = int(round(u)), int(round(v))
        if not (1 <= x < depth.shape[1]-1 and 1 <= y < depth.shape[0]-1):
            continue
        patch = depth[y-1:y+2, x-1:x+2]
        if not np.all(np.isfinite(patch)) or patch.min() < options.min_depth_m or patch.max() > options.max_depth_m:
            continue
        z = float(np.median(patch))
        if np.ptp(patch) > options.depth_edge_ratio*z:  # depth discontinuity / foreground boundary
            continue
        ray = cv2.undistortPoints(np.array([[[u, v]]], np.float64), current_k, current_d)[0, 0]
        xyz.append([ray[0]*z, ray[1]*z, z])
        pixels.append(rp[ri])
    return estimate_pose(xyz, pixels, rk, distortion, options=options, diagnostics=diagnostics)


def match(reference, current, depth, current_k, current_d, options=None, diagnostics=None):
    """Time bounded candidate work without changing geometric acceptance gates."""
    diagnostics = {} if diagnostics is None else diagnostics
    started = time.perf_counter()
    try:
        return _match(reference, current, depth, current_k, current_d, options, diagnostics)
    finally:
        diagnostics['candidate_total_s'] = time.perf_counter()-started
        diagnostics['matching_and_depth_s'] = max(0., diagnostics['candidate_total_s']-diagnostics.get('pnp_s', 0.))
