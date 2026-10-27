import cv2
import numpy as np

from visual_landmark.localization import estimate_pose, match, planar, transform


def scene():
    rng = np.random.default_rng(42)
    xyz = rng.uniform([-1.3, -0.9, 2.5], [1.3, 0.9, 6.0], (120, 3))
    k = np.array([[520., 0, 320], [0, 520, 240], [0, 0, 1]])
    r = np.array([0.25, -0.17, 0.12])  # substantial roll and pitch, not yaw-only
    t = np.array([0.31, -0.12, 0.22])
    uv = cv2.projectPoints(xyz, r, t, k, None)[0].reshape(-1, 2)
    expected = np.eye(4)
    expected[:3, :3] = cv2.Rodrigues(r)[0]
    expected[:3, 3] = t
    return xyz, uv, k, expected


def test_metric_pnp_with_tilt_noise_and_outliers():
    xyz, uv, k, expected = scene()
    rng = np.random.default_rng(7)
    uv += rng.normal(0, 0.15, uv.shape)
    uv[:25] = rng.uniform([0, 0], [640, 480], (25, 2))
    result = estimate_pose(xyz, uv, k, None)
    assert result is not None
    actual, count, error = result
    np.testing.assert_allclose(actual, expected, atol=0.006)
    assert count >= 90 and error < 0.5


def test_reject_insufficient_and_clustered_correspondences():
    xyz, uv, k, _ = scene()
    assert estimate_pose(xyz[:10], uv[:10], k, None) is None
    assert estimate_pose(xyz, np.full_like(uv, 200), k, None) is None


def test_full_transform_chain_with_moving_arm_and_tilt():
    _, _, _, relative = scene()
    mc = transform([4, 2, 0.8], [0.1, 0.2, 0.3, 0.9])
    ob = transform([8, 3, 0], [0.15, -0.1, 0.4, 0.9])
    bc = transform([0.4, 0.2, 0.8], [-0.2, 0.5, 0.1, 0.8])
    oc = ob @ bc
    localized_base = mc @ relative @ np.linalg.inv(oc) @ ob
    np.testing.assert_allclose(localized_base, mc @ relative @ np.linalg.inv(bc), atol=1e-10)
    correction = planar(localized_base) @ np.linalg.inv(planar(ob))
    np.testing.assert_allclose(correction @ planar(ob), planar(localized_base), atol=1e-10)
    # A subsequent odometry increment continues moving after localization.
    step = transform([0.4, 0, 0], [0, 0, 0, 1])
    np.testing.assert_allclose(correction @ planar(ob) @ step,
                               planar(localized_base) @ step, atol=1e-10)


def test_rgbd_matching_recovers_metric_tilt_and_rejects_invalid_depth():
    xyz, uv, k, expected = scene()
    cp = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), k, None)[0].reshape(-1, 2)
    depth = np.zeros((480, 640), np.float32)
    # Sparse projected points; reject overlaps to keep neighbourhoods consistent.
    keep = []
    for i, (u, v) in enumerate(cp):
        x, y = np.round([u, v]).astype(int)
        if 3 <= x < 637 and 3 <= y < 477 and not np.any(depth[y-3:y+4, x-3:x+4]):
            # Supply the configured 5x5 stable depth support.
            depth[y-2:y+3, x-2:x+3] = xyz[i, 2]
            keep.append(i)
    rng = np.random.default_rng(1)
    descriptors = rng.normal(size=(len(keep), 128)).astype(np.float32)
    current = (cp[keep].astype(np.float32), descriptors)
    reference = (uv[keep].astype(np.float32), descriptors.copy(), k, None)
    debug = {}
    result = match(reference, current, depth, k, None, debug=debug)
    assert result is not None
    assert len(debug['pairs']) == len(keep)
    assert len(debug['inlier_indices']) == result[1]
    assert all(0 <= i < len(debug['depth_pairs']) for i in debug['inlier_indices'])
    np.testing.assert_allclose(result[0], expected, atol=0.001)
    assert match(reference, current, depth*0, k, None) is None
    assert match(reference, (cp[keep], None), depth, k, None) is None
