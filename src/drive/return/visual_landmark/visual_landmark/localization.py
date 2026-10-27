"""Metric RGB-D to RGB localization. Matrices use parent <- child convention."""
from dataclasses import dataclass, asdict
import json
import math

import time
import cv2
import numpy as np


@dataclass(frozen=True)
class MatchOptions:
    min_inliers: int = 30
    min_inlier_ratio: float = 0.60
    ratio_test: float = 0.65
    ransac_px: float = 2.5
    max_rmse_px: float = 2.0
    min_depth_m: float = 0.25
    max_depth_m: float = 8.0
    depth_edge_ratio: float = 0.05
    depth_patch_radius: int = 2
    geometric_filter: bool = True
    geometric_ransac_px: float = 1.5
    geometric_min_ratio: float = 0.5
    inlier_grid_rows: int = 4
    inlier_grid_cols: int = 6
    min_inlier_grid_cells: int = 8
    min_inlier_image_coverage: float = 0.08

    def __post_init__(self):
        if not all(np.isfinite(v) for v in vars(self).values()):
            raise ValueError('nonfinite match thresholds')
        if (self.min_inliers < 20 or not .45 <= self.min_inlier_ratio <= 1
                or not 0 < self.ratio_test < 1 or not 0 < self.max_rmse_px <= self.ransac_px
                or not 0 < self.min_depth_m < self.max_depth_m or not 0 < self.depth_edge_ratio < 1):
            raise ValueError('invalid match thresholds; minimum 20 inliers and 45% ratio retained')
        if (not isinstance(self.depth_patch_radius, int) or not 1 <= self.depth_patch_radius <= 5
                or not 0 < self.geometric_ransac_px <= 5 or not 0 < self.geometric_min_ratio <= 1
                or not 1 <= self.inlier_grid_rows <= 16 or not 1 <= self.inlier_grid_cols <= 16
                or not 1 <= self.min_inlier_grid_cells <= self.inlier_grid_rows*self.inlier_grid_cols
                or not 0 <= self.min_inlier_image_coverage <= 1):
            raise ValueError('invalid geometry/depth/coverage thresholds')


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


@dataclass(frozen=True)
class FeatureOptions:
    sift_nfeatures: int = 800
    sift_contrast_threshold: float = 0.06
    sift_edge_threshold: float = 8.0
    feature_grid_rows: int = 4
    feature_grid_cols: int = 6
    feature_grid_max_per_cell: int = 20
    feature_max_total: int = 480

    def __post_init__(self):
        for name, value in vars(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError('invalid feature option '+name)
            if name not in ('sift_contrast_threshold', 'sift_edge_threshold') and not isinstance(value, int):
                raise ValueError('feature count/grid options must be integers')
        if self.sift_nfeatures > 5000 or self.feature_max_total > 5000:
            raise ValueError('feature budget exceeds 5000')

    def signature(self):
        return json.dumps(asdict(self), sort_keys=True)


def grid_select(points, responses, image_shape, options=None):
    """Return original indices; descriptor rows must use exactly the same indices."""
    options = options or FeatureOptions()
    height, width = image_shape[:2]
    points = np.asarray(points).reshape(-1, 2)
    responses = np.asarray(responses)
    if len(points) != len(responses) or min(height, width) <= 0:
        raise ValueError('invalid grid features/shape')
    counts = {}
    selected = []
    # Stable tie order makes cache creation/ranking reproducible.
    for i in np.argsort(-responses, kind='stable'):
        x, y = points[i]
        if not np.isfinite(responses[i]) or not (0 <= x < width and 0 <= y < height):
            continue
        cell = (int(y*options.feature_grid_rows/height), int(x*options.feature_grid_cols/width))
        if counts.get(cell, 0) >= options.feature_grid_max_per_cell:
            continue
        selected.append(i)
        counts[cell] = counts.get(cell, 0)+1
        if len(selected) >= options.feature_max_total:
            break
    return np.asarray(selected, dtype=np.int64)


def features(image, options=None, diagnostics=None):
    options = options or FeatureOptions()
    detector = cv2.SIFT_create(nfeatures=options.sift_nfeatures,
        contrastThreshold=options.sift_contrast_threshold, edgeThreshold=options.sift_edge_threshold)
    keypoints, descriptors = detector.detectAndCompute(image, None)
    points = np.asarray([k.pt for k in keypoints], dtype=np.float32).reshape(-1, 2)
    indices = grid_select(points, [k.response for k in keypoints], image.shape, options)
    if diagnostics is not None:
        diagnostics.update(feature_count_raw=len(points), feature_count_grid_filtered=len(indices),
                           raw_features=len(points), filtered_features=len(indices))
    return points[indices], None if descriptors is None else descriptors[indices]


def spatial_coverage(pixels, image_shape, options):
    height, width = image_shape[:2]
    uv = np.asarray(pixels).reshape(-1, 2)
    valid = uv[(uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)]
    if len(valid) == 0:
        return dict(inlier_grid_cells=0, inlier_bbox_width_ratio=0., inlier_bbox_height_ratio=0.)
    cells = np.floor(valid / [width, height] * [options.inlier_grid_cols, options.inlier_grid_rows]).astype(int)
    span = np.ptp(valid, axis=0)/[width, height]
    return dict(inlier_grid_cells=len(np.unique(cells, axis=0)),
                inlier_bbox_width_ratio=float(span[0]), inlier_bbox_height_ratio=float(span[1]))


def geometric_mask(current_pixels, reference_pixels, current_k, current_d, reference_k, reference_d, options):
    """F-RANSAC on independently undistorted pixels; different K/size are supported.

    F alone is weak for planar/pure-rotation scenes: downstream depth/PnP/coverage,
    pose prior and temporal gates remain mandatory. No unverified fallback.
    """
    count = len(current_pixels)
    if count < 8:
        return np.zeros(count, dtype=bool)
    cp = cv2.undistortPoints(np.asarray(current_pixels, np.float64).reshape(-1, 1, 2),
                             current_k, current_d, P=current_k).reshape(-1, 2)
    rp = cv2.undistortPoints(np.asarray(reference_pixels, np.float64).reshape(-1, 1, 2),
                             reference_k, reference_d, P=reference_k).reshape(-1, 2)
    fundamental, mask = cv2.findFundamentalMat(cp, rp, cv2.FM_RANSAC,
        options.geometric_ransac_px, .999, 2000)
    if (fundamental is None or fundamental.shape != (3, 3) or not np.all(np.isfinite(fundamental))
            or np.linalg.matrix_rank(fundamental) != 2 or mask is None or mask.size != count):
        return np.zeros(count, dtype=bool)
    return mask.ravel().astype(bool)


@dataclass(frozen=True)
class PriorOptions:
    enabled: bool = True
    max_translation_m: float = .5
    max_yaw_deg: float = 12.
    max_roll_deg: float = 5.
    max_pitch_deg: float = 5.

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for k, v in vars(self).items() if k != 'enabled'):
            raise ValueError('invalid pose prior thresholds')


def check_pose_prior(estimated, predicted, base_from_camera, options, diagnostics):
    # Relative matrices are reference<-current. Conjugate residual into vehicle
    # body axes so yaw means rotation around body Z, not optical Z (roll).
    residual = base_from_camera @ np.linalg.inv(predicted) @ estimated @ np.linalg.inv(base_from_camera)
    fields = pose_fields(residual)
    translation = float(np.linalg.norm(residual[:3, 3]))
    roll, pitch, yaw = [abs(float(v)) for v in fields['rpy']]
    diagnostics.update(pose_prior_translation_error=translation, pose_prior_yaw_error=yaw,
                       pose_prior_roll_error=roll, pose_prior_pitch_error=pitch,
                       pose_prior_checked=bool(options.enabled))
    return (not options.enabled or (translation <= options.max_translation_m
        and yaw <= math.radians(options.max_yaw_deg) and roll <= math.radians(options.max_roll_deg)
        and pitch <= math.radians(options.max_pitch_deg)))


def estimate_pose(points3d, pixels, k, distortion, min_inliers=30, options=None, diagnostics=None, debug=None, image_shape=None):
    """Return reference_camera <- current_camera and geometric quality, or None."""
    options = options or MatchOptions(min_inliers=min_inliers)
    min_inliers = options.min_inliers
    diagnostics = {} if diagnostics is None else diagnostics
    diagnostics.update(pnp_points=len(points3d), pnp_inliers=0, inliers=0, inlier_ratio=0., reject_reason='insufficient_depth_correspondences')
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
    if debug is not None and indices is not None:
        debug['inlier_indices'] = indices.ravel().tolist()
    diagnostics['pnp_s'] = time.perf_counter()-pnp_started
    diagnostics['reject_reason'] = 'pnp_inliers'
    if indices is not None:
        diagnostics.update(inliers=len(indices), pnp_inliers=len(indices), inlier_ratio=len(indices)/len(xyz))
    if not ok or indices is None or len(indices) < min_inliers or len(indices)/len(xyz) < options.min_inlier_ratio:
        return None
    idx = indices.ravel()
    diagnostics['reject_reason'] = 'clustered_geometry'
    # Avoid clustered correspondences giving a poorly constrained pose.
    if np.linalg.svd(xyz[idx] - xyz[idx].mean(axis=0), compute_uv=False)[1] < 0.1:
        return None
    if np.min(np.ptp(uv[idx], axis=0)) < 60:
        return None
    if image_shape is not None:
        coverage = spatial_coverage(uv[idx], image_shape, options)
        diagnostics.update(coverage, inlier_coverage_checked=True)
        if (coverage['inlier_grid_cells'] < options.min_inlier_grid_cells
                or coverage['inlier_bbox_width_ratio']*coverage['inlier_bbox_height_ratio'] < options.min_inlier_image_coverage):
            diagnostics['reject_reason'] = 'insufficient_inlier_spatial_coverage'
            return None
    else:
        diagnostics['inlier_coverage_checked'] = False  # legacy standalone API; production always supplies shape
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


def _match(reference, current, depth, current_k, current_d, options=None, diagnostics=None, debug=None, reference_shape=None):
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
    if debug is not None:
        debug['pairs'] = [(cp[ci].tolist(), rp[ri].tolist()) for ci, ri in pairs]
        debug['depth_pairs'] = []
    if options.geometric_filter:
        mask = geometric_mask([cp[ci] for ci, _ in pairs], [rp[ri] for _, ri in pairs],
                              current_k, current_d, rk, distortion, options)
        filtered = [pair for pair, keep in zip(pairs, mask) if keep]
    else:
        filtered = pairs
    diagnostics.update(geometric_inliers=len(filtered), geometric_filter_applied=options.geometric_filter)
    if options.geometric_filter and (len(filtered) < options.min_inliers
                                     or len(filtered)/max(1, len(pairs)) < options.geometric_min_ratio):
        diagnostics['reject_reason'] = 'geometric_ransac'
        return None
    if debug is not None:
        debug['geometric_pairs'] = [(cp[ci].tolist(), rp[ri].tolist()) for ci, ri in filtered]
    xyz, pixels = [], []
    radius = options.depth_patch_radius
    for ci, ri in filtered:
        u, v = cp[ci]
        x, y = int(round(u)), int(round(v))
        if not (radius <= x < depth.shape[1]-radius and radius <= y < depth.shape[0]-radius):
            continue
        patch = depth[y-radius:y+radius+1, x-radius:x+radius+1]
        if not np.all(np.isfinite(patch)) or patch.min() < options.min_depth_m or patch.max() > options.max_depth_m:
            continue
        z = float(np.median(patch))
        if np.ptp(patch) > options.depth_edge_ratio*z:  # depth discontinuity / foreground boundary
            continue
        ray = cv2.undistortPoints(np.array([[[u, v]]], np.float64), current_k, current_d)[0, 0]
        xyz.append([ray[0]*z, ray[1]*z, z])
        pixels.append(rp[ri])
        if debug is not None:
            debug['depth_pairs'].append((cp[ci].tolist(), rp[ri].tolist()))
    diagnostics['depth_valid_matches'] = len(xyz)
    return estimate_pose(xyz, pixels, rk, distortion, options=options, diagnostics=diagnostics, debug=debug, image_shape=reference_shape)


def match(reference, current, depth, current_k, current_d, options=None, diagnostics=None, debug=None, reference_shape=None):
    """Time candidate matching, geometry, depth and PnP work with stage diagnostics."""
    diagnostics = {} if diagnostics is None else diagnostics
    diagnostics.update({key: 0 for key in ('raw_matches', 'mutual_ratio_matches', 'geometric_inliers',
        'depth_valid_matches', 'pnp_points', 'pnp_inliers', 'inlier_grid_cells')})
    diagnostics.update(pnp_s=0., inlier_ratio=0., reprojection_rmse=None,
                       inlier_bbox_width_ratio=0., inlier_bbox_height_ratio=0.)
    for key in ('translation', 'yaw', 'roll', 'pitch'):
        diagnostics['pose_prior_'+key+'_error'] = None
    started = time.perf_counter()
    try:
        return _match(reference, current, depth, current_k, current_d, options, diagnostics, debug, reference_shape)
    finally:
        diagnostics['candidate_total_s'] = time.perf_counter()-started
        diagnostics['matching_and_depth_s'] = max(0., diagnostics['candidate_total_s']-diagnostics.get('pnp_s', 0.))
