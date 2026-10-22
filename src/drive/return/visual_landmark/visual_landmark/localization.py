"""Metric RGB-D to RGB localization. Matrices use parent <- child convention."""
import cv2
import numpy as np


def transform(xyz, xyzw):
    q = np.asarray(xyzw, dtype=float)
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


def estimate_pose(points3d, pixels, k, distortion, min_inliers=20):
    """Return reference_camera <- current_camera and geometric quality, or None."""
    if len(points3d) < min_inliers:
        return None
    xyz = np.ascontiguousarray(points3d, dtype=np.float64)
    uv = np.ascontiguousarray(pixels, dtype=np.float64)
    ok, r, t, indices = cv2.solvePnPRansac(
        xyz, uv, k, distortion, iterationsCount=300, reprojectionError=3.0,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or indices is None or len(indices) < min_inliers or len(indices)/len(xyz) < 0.45:
        return None
    idx = indices.ravel()
    # Avoid clustered correspondences giving a poorly constrained pose.
    if np.linalg.svd(xyz[idx] - xyz[idx].mean(axis=0), compute_uv=False)[1] < 0.1:
        return None
    if np.min(np.ptp(uv[idx], axis=0)) < 60:
        return None
    r, t = cv2.solvePnPRefineLM(xyz[idx], uv[idx], k, distortion, r, t)
    rot = cv2.Rodrigues(r)[0]
    projected = cv2.projectPoints(xyz[idx], r, t, k, distortion)[0].reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum((projected - uv[idx])**2, axis=1))))
    if error > 2.5 or np.any((xyz[idx] @ rot.T + t.reshape(3))[:, 2] <= 0):
        return None
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rot, t.ravel()
    return result, len(idx), error


def match(reference, current, depth, current_k, current_d):
    rp, rd, rk, distortion = reference
    cp, cd = current
    if rd is None or cd is None or len(rd) < 2 or len(cd) < 2:
        return None
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward = matcher.knnMatch(cd, rd, k=2)
    reverse = matcher.knnMatch(rd, cd, k=2)
    back = {a.queryIdx: a.trainIdx for pair in reverse if len(pair) == 2
            for a, b in [pair] if a.distance < 0.7*b.distance}
    pairs = [(a.queryIdx, a.trainIdx) for pair in forward if len(pair) == 2
             for a, b in [pair] if a.distance < 0.7*b.distance and back.get(a.trainIdx) == a.queryIdx]
    xyz, pixels = [], []
    for ci, ri in pairs:
        u, v = cp[ci]
        x, y = int(round(u)), int(round(v))
        if not (1 <= x < depth.shape[1]-1 and 1 <= y < depth.shape[0]-1):
            continue
        patch = depth[y-1:y+2, x-1:x+2]
        if not np.all(np.isfinite(patch)) or patch.min() < 0.25 or patch.max() > 8:
            continue
        z = float(np.median(patch))
        if np.ptp(patch) > 0.08*z:  # depth discontinuity / foreground boundary
            continue
        ray = cv2.undistortPoints(np.array([[[u, v]]], np.float64), current_k, current_d)[0, 0]
        xyz.append([ray[0]*z, ray[1]*z, z])
        pixels.append(rp[ri])
    return estimate_pose(xyz, pixels, rk, distortion)
