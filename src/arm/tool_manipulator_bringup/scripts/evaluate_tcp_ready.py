#!/usr/bin/env python3
"""Offline candidate evaluation. Never publishes motion or opens hardware.

Optional collision_checker(q) must validate the full robot in its scene.
Without it, collision and approach safety remain explicitly unverified.
"""
import json
from pathlib import Path
import sys

import numpy as np

source = Path(__file__).resolve().parents[2]/'tool_change_min'
if source.is_dir():
    sys.path.insert(0, str(source))
try:
    from tool_change_min.kinematics import ArmKinematics, _rotation
except ImportError:
    from arm_shared_kinematics import ArmKinematics, _rotation


CANDIDATES = {
    'recovery_reference': [0., 1.4831, 1.5071, -1.5571, 0., 0.],
    'candidate_a': [0., 1.2, 1.2, -1.2, 0., 0.],
    'candidate_b': [0., .9, 1., -1., 0., 0.],
    'candidate_c': [0., .6, .8, -.8, 0., 0.],
}


def condition(model, q):
    jac = model.jacobian(q).copy()
    jac[:3] /= .2
    return float(np.linalg.cond(jac))


def evaluate(model, q, start=None, collision_checker=None):
    q = np.asarray(q, dtype=float)
    if not model.within_limits(q, .02):
        raise ValueError('candidate outside IK margin')
    pose = model.fk(q)
    margins = [min(v-lo, hi-v) for v, (lo, hi) in zip(q, model.limits.values())]
    result = dict(positions_rad=q.tolist(), limit_margins_rad=margins,
                  condition=condition(model, q), directions={},
                  collision_free=None if collision_checker is None else bool(collision_checker(q)),
                  approach=None, acceleration_verified=False)
    for axis, name in enumerate(('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')):
        for sign in (-1, 1):
            seed = q.copy()
            entry = dict(ik=False, collision_free=None, max_condition=condition(model, q))
            valid = True
            try:
                for k in range(1, 11):
                    target = pose.copy()
                    if axis < 3:
                        target[axis, 3] += sign*.01*k/10
                    else:
                        target[:3, :3] = pose[:3, :3] @ _rotation(
                            np.eye(3)[axis-3], sign*np.deg2rad(5)*k/10)
                    solved = model.solve(target, seed, .02)
                    # Subsample every joint segment at <=0.005 rad.
                    count = max(1, int(np.ceil(np.max(np.abs(solved-seed))/.005)))
                    for sample in np.linspace(seed, solved, count+1):
                        entry['max_condition'] = max(entry['max_condition'], condition(model, sample))
                        if collision_checker is not None:
                            valid = bool(collision_checker(sample)) and valid
                    seed = solved
                entry['ik'] = True
                entry['collision_free'] = valid if collision_checker is not None else None
            except ValueError as exc:
                entry['reason'] = str(exc)
            result['directions'][name+('+' if sign > 0 else '-')] = entry
    if start is not None:
        start = np.asarray(start, dtype=float)
        if not model.within_limits(start):
            raise ValueError('approach reference outside physical soft limits')
        count = max(1, int(np.ceil(np.max(np.abs(q-start))/.005)))
        samples = np.linspace(start, q, count+1)
        result['approach'] = dict(
            reference_positions_rad=start.tolist(), reference_is_live_feedback=False,
            limits_ok=all(model.within_limits(s) for s in samples),
            collision_free=(None if collision_checker is None else
                            all(collision_checker(s) for s in samples)),
            # Smooth quintic q=q0+dq*(10u^3-15u^4+6u^5), T>=10s.
            # Derivative maxima are 1.875 and 10/sqrt(3).
            duration_sec=max(10., float(1.875*np.max(np.abs(q-start))/.12)))
        duration = result['approach']['duration_sec']
        result['approach']['peak_velocity_rad_s'] = (1.875*np.abs(q-start)/duration).tolist()
        result['approach']['peak_acceleration_rad_s2'] = (10/np.sqrt(3)*np.abs(q-start)/duration**2).tolist()
    return result


def main():
    arm = Path(__file__).resolve().parents[2]
    description = arm/'tool_manipulator_description'
    bringup = arm/'tool_manipulator_bringup'
    if not (description/'urdf/tool_manipulator.urdf.xacro').is_file():
        from ament_index_python.packages import get_package_share_directory
        description = Path(get_package_share_directory('tool_manipulator_description'))
        bringup = Path(get_package_share_directory('tool_manipulator_bringup'))
    model = ArmKinematics(description/'urdf/tool_manipulator.urdf.xacro',
                          bringup/'config/hardware.yaml')
    print(json.dumps({name: evaluate(model, q, CANDIDATES['recovery_reference'])
                      for name, q in CANDIDATES.items()}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
