"""Read-only YOLO foot-ray / floor intersection and object placement geometry.

Requires a complete, unobscured person box and a calibrated ground plane.
The box bottom is an estimate of foot contact, not measured depth.
"""
import math

import cv2
import numpy as np

from tool_docking_core import transform


def highest_confidence_person(payload, min_confidence):
    """Select the highest-confidence person above the configured threshold."""
    people = [
        (float(d['confidence']), d)
        for d in payload['detections']
        if d.get('class_name') == 'person'
        and float(d['confidence']) >= min_confidence
    ]
    if not people:
        raise ValueError('at least one confident person required')
    # max() preserves the first detection when confidence values are equal.
    return max(people, key=lambda item: item[0])[1]


def person_on_floor(payload, camera_matrix, distortion, floor_camera, min_confidence):
    person = highest_confidence_person(payload, min_confidence)
    box = np.asarray(person['xyxy'], dtype=float)
    width, height = payload['image_size']['width'], payload['image_size']['height']
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not 1 < box[0] < box[2] < width-2
            or not 1 < box[1] < box[3] < height-2):
        raise ValueError('person box clipped or invalid; feet cannot be inferred')
    pixel = np.array([[(box[0]+box[2])/2, box[3]]], dtype=float)
    xy = cv2.undistortPoints(pixel.reshape(1, 1, 2), camera_matrix, distortion).reshape(2)
    ray = floor_camera[:3, :3] @ np.r_[xy, 1.]
    origin = floor_camera[:3, 3]
    if ray[2] >= -1e-6 or origin[2] <= 0:
        raise ValueError('person foot ray does not intersect floor in front of camera')
    target = origin-ray*origin[2]/ray[2]
    if not np.isfinite(target).all():
        raise ValueError('invalid floor intersection')
    return target


def person_direction_on_floor(payload, camera_matrix, distortion, floor_camera,
                              min_confidence):
    """Horizontal bearing from the visible bbox extent; feet are unnecessary.

    Use the box centre when both horizontal edges are in the image.  If only
    one edge is visible, aim at that edge instead of averaging it with an
    off-image coordinate.  A box spanning both sides has no observed
    horizontal edge and cannot provide this directional cue.
    """
    person = highest_confidence_person(payload, min_confidence)
    box = np.asarray(person['xyxy'], dtype=float)
    width = payload['image_size']['width']
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not box[0] < box[2] or width <= 0):
        raise ValueError('person box has no valid horizontal extent')
    # Detectors clamp off-image coordinates to 0/width (or width-1), so an
    # edge touching the two-pixel image margin is still treated as clipped.
    left_visible = 1 < box[0] < width-2
    right_visible = 1 < box[2] < width-2
    if left_visible and right_visible:
        aim_x = (box[0]+box[2])/2
    elif left_visible:
        aim_x = box[0]
    elif right_visible:
        aim_x = box[2]
    else:
        raise ValueError('person box has no visible horizontal edge')
    # Only horizontal image position is requested. Use the calibrated optical
    # centre vertically so clipped head/feet do not bias the bearing.
    pixel = np.array([[aim_x, camera_matrix[1, 2]]], dtype=float)
    xy = cv2.undistortPoints(pixel.reshape(1, 1, 2), camera_matrix, distortion).reshape(2)
    ray = floor_camera[:3, :3] @ np.r_[xy, 1.]
    direction = ray[:2]
    length = float(np.linalg.norm(direction))
    if not np.isfinite(direction).all() or length < 1e-6:
        raise ValueError('person horizontal bearing is undefined')
    return direction/length


def placement_targets(person_floor, floor_base, config, floor_chassis=None):
    """Object bottom reaches floor z=0; TCP height follows calibrated offset."""
    robot = floor_base[:3, 3]
    delta = np.asarray(person_floor)[:2]-robot[:2]
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        raise ValueError('undefined person bearing')
    direction = delta/distance
    if config.get('placement_mode') == 'chassis_edge':
        if floor_chassis is None or np.dot(floor_chassis[:3, 2], [0., 0., 1.]) < .999:
            raise ValueError('horizontal chassis TF required')
        chassis_floor = np.linalg.inv(floor_chassis)
        origin = (chassis_floor @ np.r_[robot, 1.])[:2]
        ray = (chassis_floor[:3, :3] @ np.r_[direction, 0.])[:2]
        half = np.asarray(config['chassis_half_extents_m'])
        if half.shape != (2,) or not np.isfinite(half).all() or np.any(half <= 0) or np.any(np.abs(origin) > half):
            raise ValueError('arm base must lie inside valid chassis footprint')
        exits = [(math.copysign(h, d)-o)/d for h, d, o in zip(half, ray, origin) if abs(d) > 1e-9]
        radius = min(exits)+config['edge_clearance_m']
    else:
        radius = config['place_radius_m']
    if distance < radius+config['person_clearance_m']:
        raise ValueError('person too close to placement area')
    yaw = math.atan2(delta[1], delta[0])
    turn = transform([0, 0, 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
    orientation = transform([0, 0, 0], config['place_orientation_xyzw'])
    target = turn @ orientation
    bottom = np.r_[robot[:2]+direction*radius, 0.]
    target[:3, 3] = bottom-target[:3, :3] @ config['tcp_to_object_bottom_m']
    preplace = target.copy()
    preplace[2, 3] += config['preplace_height_m']
    return preplace, target


def placement_targets_for_direction(direction_floor, floor_base, config,
                                    floor_chassis=None):
    """Place at the chassis boundary using bearing only, with no distance claim."""
    direction = np.asarray(direction_floor, dtype=float)
    if direction.shape != (2,) or not np.isfinite(direction).all():
        raise ValueError('invalid person direction')
    length = float(np.linalg.norm(direction))
    if length < 1e-6:
        raise ValueError('undefined person bearing')
    direction /= length
    # Reuse the reviewed chassis/radius and TCP-offset geometry. The synthetic
    # point is deliberately beyond every configured radius; human clearance is
    # not evaluated because this mode does not estimate distance.
    robot = floor_base[:3, 3]
    synthetic_distance = 10.0
    local = dict(config, person_clearance_m=0.)
    return placement_targets(np.r_[robot[:2]+direction*synthetic_distance, 0.],
                             floor_base, local, floor_chassis)


def aligned_base(model, measured, person_base, margin):
    """Rotate only base_joint toward the person, respecting its actual URDF axis."""
    q = np.asarray(measured, dtype=float).copy()
    pose = model.fk(q)
    axis = model.jacobian(q)[3:, 0]
    if abs(axis[2]) < .999:
        raise ValueError('base axis is not vertical in base_actuator')
    # Recover the horizontal pivot from v = axis x (tcp - pivot).
    pivot = pose[:3, 3]+np.cross(axis, model.jacobian(q)[:3, 0])
    start = pose[:2, 3]-pivot[:2]
    desired = np.asarray(person_base)[:2]-pivot[:2]
    if min(np.linalg.norm(start), np.linalg.norm(desired)) < .01:
        raise ValueError('undefined base bearing')
    angle = math.atan2(desired[1], desired[0])-math.atan2(start[1], start[0])
    q[0] += math.atan2(math.sin(angle), math.cos(angle))/axis[2]
    if not model.within_limits(q, margin):
        raise ValueError('person bearing exceeds base joint limits')
    return q
