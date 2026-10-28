"""Read-only YOLO foot-ray / floor intersection and object placement geometry.

Requires a complete, unobscured person box and a calibrated ground plane.
The box bottom is an estimate of foot contact, not measured depth.
"""
import math

import cv2
import numpy as np

from tool_docking_core import transform


def person_on_floor(payload, camera_matrix, distortion, floor_camera, min_confidence):
    people = [d for d in payload['detections']
              if d.get('class_name') == 'person' and float(d['confidence']) >= min_confidence]
    if len(people) != 1:
        raise ValueError('exactly one confident person required')
    box = np.asarray(people[0]['xyxy'], dtype=float)
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
