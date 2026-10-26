"""Geometry and configuration for optional, measured AprilTag docking."""
import math

import cv2
import numpy as np
import yaml


def transform(xyz, xyzw):
    xyz, q = np.asarray(xyz, dtype=float), np.asarray(xyzw, dtype=float)
    if (xyz.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(xyz))
            or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9):
        raise ValueError('expected finite xyz and nonzero xyzw quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = xyz
    return result


def pose_errors(current, target):
    return (float(np.linalg.norm(target[:3, 3]-current[:3, 3])),
            float(np.linalg.norm(cv2.Rodrigues(target[:3, :3] @ current[:3, :3].T)[0])))


def bounded_target(current, target, translation_step, rotation_step):
    """One simultaneous bounded translation/rotation increment in base frame."""
    result = current.copy()
    delta = target[:3, 3]-current[:3, 3]
    rotation = cv2.Rodrigues(target[:3, :3] @ current[:3, :3].T)[0].reshape(3)
    result[:3, 3] += delta * min(1., translation_step/max(np.linalg.norm(delta), 1e-12))
    rotation *= min(1., rotation_step/max(np.linalg.norm(rotation), 1e-12))
    result[:3, :3] = cv2.Rodrigues(rotation)[0] @ current[:3, :3]
    return result


def tag_pose(detection, camera_matrix, distortion, settings):
    """PnP uses detector corner ordering, matching measure_apriltag_docking.py."""
    if (detection.get('hamming') != 0
            or not math.isfinite(float(detection.get('decision_margin', float('nan'))))
            or float(detection['decision_margin']) < settings['min_decision_margin']):
        raise ValueError('tag quality rejected')
    pixels = np.asarray([[p['u'], p['v']] for p in detection['corners_px']], dtype=float)
    if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
        raise ValueError('invalid tag corners')
    edges = np.linalg.norm(pixels-np.roll(pixels, -1, axis=0), axis=1)
    if np.min(edges) < settings['min_tag_edge_px']:
        raise ValueError('tag image is too small for docking')
    half = settings['tag_size_m']/2
    square = np.asarray([[-half, half, 0], [half, half, 0],
                         [half, -half, 0], [-half, -half, 0]], dtype=float)
    result = cv2.solvePnPGeneric(
        square, pixels, camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    ok, rotations, translations = result[:3]
    if not ok:
        raise ValueError('invalid tag pose')
    candidates = []
    for rotation, translation in zip(rotations, translations):
        rotation = np.asarray(rotation, dtype=float).reshape(3, 1)
        translation = np.asarray(translation, dtype=float).reshape(3, 1)
        if (not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rotation))
                or translation[2, 0] <= 0):
            continue
        projected = cv2.projectPoints(
            square, rotation, translation, camera_matrix, distortion)[0]
        residual = pixels-projected.reshape(4, 2)
        rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
        candidates.append((rmse, rotation, translation))
    if not candidates:
        raise ValueError('tag behind camera or invalid PnP candidates')
    candidates.sort(key=lambda candidate: candidate[0])
    error, rotation, translation = candidates[0]
    if error > settings['max_reprojection_px']:
        raise ValueError('tag reprojection error too large')
    if (len(candidates) > 1
            and candidates[1][0]-error < settings['min_pnp_solution_gap_px']):
        raise ValueError('ambiguous planar tag pose')
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = cv2.Rodrigues(rotation)[0], translation.reshape(3)
    return result


def load_settings(filename):
    with open(filename, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('tools'), dict):
        raise ValueError('docking config must contain tools mapping')
    result = {}
    for tool_id, raw in data['tools'].items():
        if type(tool_id) is not int or tool_id not in (0, 1, 2) or not isinstance(raw, dict):
            raise ValueError('invalid docking tool entry')
        if raw.get('enabled') is not True:
            continue
        item = dict(raw)
        if type(item.get('tag_id')) is not int or item['tag_id'] < 0:
            raise ValueError('enabled docking requires tag_id')
        # Physical transforms and timing are deliberately mandatory.
        limits = dict(tag_size_m=(0, 1), tag_max_age_s=(0, .5),
                      min_decision_margin=(0, 1000), max_reprojection_px=(0, 5),
                      min_tag_edge_px=(5, 1000), min_pnp_solution_gap_px=(0, 5),
                      tf_timeout_s=(0, .5), max_tag_position_jump_m=(0, .05),
                      max_tag_angle_jump_rad=(0, .5),
                      position_tolerance_m=(0, .01), angle_tolerance_rad=(0, .1),
                      step_m=(0, .01), step_rad=(0, .05),
                      max_position_error_m=(0, .25), max_angle_error_rad=(0, .5),
                      joint_step_rad=(0, .10), joint_speed_rad_s=(0, .05),
                      linear_speed_m_s=(0, .02), angular_speed_rad_s=(0, .1),
                      stopped_speed_rad_s=(0, .02), joint_tolerance_rad=(0, .02),
                      acquire_timeout_s=(0, 30), sequence_timeout_s=(0, 600),
                      lock_timeout_s=(0, 120))
        for name, (low, high) in limits.items():
            value = item.get(name)
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not low < value <= high):
                raise ValueError(f'tool {tool_id}: {name} must be in ({low}, {high}]')
        if type(item.get('stable_samples')) is not int or not 3 <= item['stable_samples'] <= 100:
            raise ValueError('stable_samples must be 3..100 distinct observations')
        if type(item.get('lock_enabled')) is not bool:
            raise ValueError('explicit lock_enabled required')
        stages = item.get('stages')
        if not isinstance(stages, list) or len(stages) != 3:
            raise ValueError('three tag-relative stages required: coarse, fine, dock')
        item['targets'] = [transform(stage['xyz_m'], stage['xyzw']) for stage in stages]
        result[tool_id] = item
    return result
