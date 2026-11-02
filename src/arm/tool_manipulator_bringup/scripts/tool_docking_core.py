"""Geometry and configuration for optional, measured AprilTag docking."""
import math

import cv2
import numpy as np
import yaml

from tool_ids import TOOL_IDS, tool_label


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


def direction_angle(first, second):
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    if (first.shape != (3,) or second.shape != (3,)
            or not np.all(np.isfinite(first)) or not np.all(np.isfinite(second))
            or np.linalg.norm(first) < 1e-9 or np.linalg.norm(second) < 1e-9):
        raise ValueError('expected finite nonzero directions')
    first, second = first/np.linalg.norm(first), second/np.linalg.norm(second)
    return math.atan2(float(np.linalg.norm(np.cross(first, second))), float(first @ second))


def interpolate_direction(first, second, fraction):
    angle = direction_angle(first, second)
    if not 0 <= fraction <= 1 or angle >= math.pi-1e-6:
        raise ValueError('invalid direction interpolation')
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    first, second = first/np.linalg.norm(first), second/np.linalg.norm(second)
    if angle < 1e-9:
        return second.copy()
    return (math.sin((1-fraction)*angle)*first+math.sin(fraction*angle)*second)/math.sin(angle)


def bounded_target(current, target, translation_step, rotation_step):
    """One simultaneous bounded translation/rotation increment in base frame."""
    result = current.copy()
    delta = target[:3, 3]-current[:3, 3]
    rotation = cv2.Rodrigues(target[:3, :3] @ current[:3, :3].T)[0].reshape(3)
    result[:3, 3] += delta * min(1., translation_step/max(np.linalg.norm(delta), 1e-12))
    rotation *= min(1., rotation_step/max(np.linalg.norm(rotation), 1e-12))
    result[:3, :3] = cv2.Rodrigues(rotation)[0] @ current[:3, :3]
    return result


def camera_normal_target(base_tag, base_camera, reference_x, stand_off=None):
    """Centre the optical axis on the tag, preserving roll and normal distance.

    Optical +Z looks toward the tag. Choose the tag-normal sign from the
    observed camera side rather than assuming a particular PnP normal sign.
    """
    normal, measured_distance = tag_normal_from_camera_side(base_tag, base_camera)
    distance = measured_distance if stand_off is None else float(stand_off)
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('camera must remain in front of the tag plane')
    x_axis = np.asarray(reference_x)-normal*np.dot(reference_x, normal)
    norm = float(np.linalg.norm(x_axis))
    if norm < .1:
        raise ValueError('camera roll reference nearly parallel to tag normal')
    x_axis /= norm
    target = np.eye(4)
    target[:3, :3] = np.column_stack((x_axis, np.cross(normal, x_axis), normal))
    target[:3, 3] = base_tag[:3, 3]-normal*distance
    return target, distance


def tag_normal_from_camera_side(base_tag, base_camera):
    """Choose the observed tag normal pointing from the camera toward the tag."""
    normal = base_tag[:3, 2].copy()
    separation = base_tag[:3, 3]-base_camera[:3, 3]
    if np.dot(normal, separation) < 0:
        normal = -normal
    distance = float(np.dot(normal, separation))
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('camera must remain in front of the tag plane')
    return normal, distance


def tag_pose(detection, camera_matrix, distortion, settings, reference=None):
    """Solve planar tag pose, optionally preserving temporal pose continuity.

    IPPE's two planar solutions can exchange reprojection-error order under
    sub-pixel corner noise.  Once a stationary rack tag has an accepted pose,
    ``reference`` selects the valid candidate continuous with that pose rather
    than allowing a mirror-solution flip.
    """
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
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = cv2.Rodrigues(rotation)[0], translation.reshape(3)
        candidates.append((rmse, rotation, translation, pose))
    if not candidates:
        raise ValueError('tag behind camera or invalid PnP candidates')
    candidates.sort(key=lambda candidate: candidate[0])
    error, rotation, translation, result = candidates[0]
    if error > settings['max_reprojection_px']:
        raise ValueError('tag reprojection error too large')
    if reference is not None:
        reference = np.asarray(reference, dtype=float)
        if reference.shape != (4, 4) or not np.all(np.isfinite(reference)):
            raise ValueError('invalid tag pose continuity reference')
        valid = [candidate for candidate in candidates
                 if candidate[0] <= settings['max_reprojection_px']]
        error, rotation, translation, result = min(
            valid, key=lambda candidate: pose_errors(reference, candidate[3])[1])
    elif (len(candidates) > 1
            and candidates[1][0]-error < settings['min_pnp_solution_gap_px']):
        # At the requested front-on alignment IPPE can return the same pose
        # twice. Reject distinct mirror solutions, not equivalent duplicates.
        _, other_rotation, other_translation, _ = candidates[1]
        turn = float(np.linalg.norm(cv2.Rodrigues(
            cv2.Rodrigues(other_rotation)[0] @ cv2.Rodrigues(rotation)[0].T)[0]))
        shift = float(np.linalg.norm(other_translation-translation))
        if (turn > settings.get('angle_tolerance_rad', .02)
                or shift > settings.get('position_tolerance_m', .002)):
            raise ValueError('ambiguous planar tag pose')
    return result


def load_settings(filename):
    with open(filename, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('tools'), dict):
        raise ValueError('docking config must contain tools mapping')
    result = {}
    for tool_id, raw in data['tools'].items():
        if type(tool_id) is not int or tool_id not in TOOL_IDS or not isinstance(raw, dict):
            raise ValueError('invalid docking tool entry')
        if raw.get('enabled') is not True:
            continue
        item = dict(raw)
        if type(item.setdefault('require_l2_hold', True)) is not bool:
            raise ValueError('require_l2_hold must be a boolean')
        if type(item.get('axial_sampled_path', False)) is not bool:
            raise ValueError('axial_sampled_path must be a boolean')
        mode = item.setdefault('alignment_mode', 'tag_tcp_stages')
        if mode not in ('tag_tcp_stages', 'camera_normal_advance',
                        'fixed_tcp_axis_advance', 'fixed_joint_goal'):
            raise ValueError('unknown docking alignment_mode')
        if ('maintain_entry_orientation' in item
                and type(item['maintain_entry_orientation']) is not bool):
            raise ValueError('maintain_entry_orientation must be a boolean')
        if ('measured_pre_dock_alignment_enabled' in item
                and type(item['measured_pre_dock_alignment_enabled']) is not bool):
            raise ValueError('measured_pre_dock_alignment_enabled must be a boolean')
        if 'tag_family' in item and (not isinstance(item['tag_family'], str)
                                     or not item['tag_family'].strip()):
            raise ValueError('tag_family must be a nonempty string')
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
                      acquire_timeout_s=(0, 30), sequence_timeout_s=(0, 600))
        if mode in ('camera_normal_advance', 'fixed_tcp_axis_advance'):
            limits['advance_distance_m'] = (0, .10)
        if mode == 'tag_tcp_stages' or item.get('actuation_enabled') is True:
            limits['actuation_timeout_s'] = (0, 120)
        for name, (low, high) in limits.items():
            value = item.get(name)
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not low < value <= high):
                raise ValueError(f'{tool_label(tool_id)}: {name} must be in ({low}, {high}]')
        if type(item.get('stable_samples')) is not int or not 3 <= item['stable_samples'] <= 100:
            raise ValueError('stable_samples must be 3..100 distinct observations')
        if type(item.get('actuation_enabled')) is not bool:
            raise ValueError('explicit actuation_enabled required')
        if mode == 'fixed_tcp_axis_advance':
            axis = np.asarray(item.get('advance_axis_tcp'), dtype=float)
            if (axis.shape != (3,) or not np.all(np.isfinite(axis))
                    or float(np.linalg.norm(axis)) < 1e-9):
                raise ValueError(
                    f'{tool_label(tool_id)}: finite nonzero advance_axis_tcp required')
            item['advance_axis_tcp'] = axis/np.linalg.norm(axis)
            if (item['actuation_enabled']
                    and type(item.get('desired_docked')) is not bool):
                raise ValueError('fixed-axis actuation requires desired_docked state')
            result[tool_id] = item
            continue
        if mode == 'fixed_joint_goal':
            correction = item.get('tag_translation_correction', {})
            if not isinstance(correction, dict):
                raise ValueError('tag_translation_correction must be a mapping')
            if correction.get('enabled', False):
                matrix = np.asarray(correction.get('reference_camera_matrix'), dtype=float)
                distortion = np.asarray(correction.get('reference_distortion'), dtype=float)
                if (matrix.shape != (9,) or not np.all(np.isfinite(matrix))
                        or distortion.ndim != 1 or distortion.size not in (4, 5, 8)
                        or not np.all(np.isfinite(distortion))):
                    raise ValueError('tag correction requires recorded camera calibration')
                for key in ('reference_joint_rad', 'outer_entry_joint_rad'):
                    values = np.asarray(correction.get(key), dtype=float)
                    if values.shape != (6,) or not np.all(np.isfinite(values)):
                        raise ValueError(f'tag correction requires six finite {key}')
                pixels = np.asarray(correction.get('reference_corners_px'), dtype=float)
                if pixels.shape != (4, 2) or not np.all(np.isfinite(pixels)):
                    raise ValueError('tag correction requires four reference corners')
                for key in ('max_translation_m', 'sample_spread_m'):
                    value = float(correction.get(key, 0))
                    if not math.isfinite(value) or value <= 0:
                        raise ValueError(f'invalid tag correction {key}')
                if not 0 < correction['sample_spread_m'] <= correction['max_translation_m'] <= .010:
                    raise ValueError('single-frame tag reference permits at most 10 mm correction')
                if type(correction.get('sample_count')) is not int or correction['sample_count'] < 3:
                    raise ValueError('tag correction requires at least three distinct samples')
            goal = np.asarray(item.get('final_joint_goal_rad'), dtype=float)
            if goal.shape != (6,) or not np.all(np.isfinite(goal)):
                raise ValueError(
                    f'{tool_label(tool_id)}: six finite final_joint_goal_rad values required')
            item['final_joint_goal'] = goal
            intermediate = np.asarray(item.get('intermediate_joint_goals_rad', []), dtype=float)
            if intermediate.size == 0:
                intermediate = np.empty((0, 6))
            if intermediate.ndim != 2 or intermediate.shape[1] != 6 or not np.all(np.isfinite(intermediate)):
                raise ValueError('intermediate_joint_goals_rad requires finite six-joint rows')
            item['intermediate_joint_goals'] = intermediate
            prefix = np.asarray(item.get('axial_approach_joint_goals_rad', []), dtype=float)
            if prefix.size == 0:
                prefix = np.empty((0, 6))
            if prefix.ndim != 2 or prefix.shape[1] != 6 or not np.all(np.isfinite(prefix)):
                raise ValueError('axial_approach_joint_goals_rad requires finite six-joint rows')
            if len(prefix) and not item.get('axial_sampled_path', False):
                raise ValueError('axial approach prefix requires axial_sampled_path')
            item['axial_approach_joint_goals'] = prefix
            if (item['actuation_enabled']
                    and type(item.get('desired_docked')) is not bool):
                raise ValueError('fixed-joint actuation requires desired_docked state')
            result[tool_id] = item
            continue
        if mode == 'camera_normal_advance':
            if (item['actuation_enabled']
                    and type(item.get('desired_docked')) is not bool):
                raise ValueError('camera-normal actuation requires desired_docked state')
            # The final tag-to-TCP measurement supplies the missing lateral
            # docking geometry. The coordinator derives pre-dock by backing
            # off exactly advance_distance_m along the observed tag normal.
            try:
                item['measured_tag_tcp_target'] = transform(
                    item['measured_tag_tcp_xyz_m'], item['measured_tag_tcp_xyzw'])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    f'{tool_label(tool_id)}: measured tag-to-TCP transform required') from exc
            result[tool_id] = item
            continue
        if type(item.get('desired_docked')) is not bool:
            raise ValueError('explicit desired_docked state required')
        stages = item.get('stages')
        if not isinstance(stages, list) or len(stages) != 3:
            raise ValueError('three tag-relative stages required: coarse, fine, dock')
        item['targets'] = [transform(stage['xyz_m'], stage['xyzw']) for stage in stages]
        result[tool_id] = item
    return result
