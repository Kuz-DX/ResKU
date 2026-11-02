"""AprilTag correction of existing manual motion; no joystick remapping.

The operator owns approach motion. Corrections act only when the existing
manual velocity approaches the dock; neutral input produces zero motion.
Explicit manual joint/tool commands retain priority in the adapter.
"""
import math

import cv2
import numpy as np
import yaml

from tool_docking_core import transform, pose_errors


def load_config(filename):
    with open(filename, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    control = data['control']
    limits = {
        'assist_full_speed_m_s': .05,
        'lateral_gain_s_inv': 10., 'angular_gain_s_inv': 10.,
        'max_lateral_correction_m_s': .02, 'max_angular_correction_rad_s': .2,
        'joint_speed_rad_s': .12, 'damping': 1., 'rotation_weight_m': 1.,
        'max_target_distance_m': .3, 'max_target_angle_rad': 1.,
        'lateral_tolerance_m': .02, 'angle_tolerance_rad': .2,
        'axial_tolerance_m': .01, 'slowdown_distance_m': .1,
        'tag_max_age_s': .5, 'acquire_timeout_s': 30.,
        'max_tag_position_jump_m': .05, 'max_tag_angle_jump_rad': .5,
    }
    for key, upper in limits.items():
        value = control[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f'{key} must be finite and in (0, {upper}]')
    if type(control['stable_samples']) is not int or not 2 <= control['stable_samples'] <= 100:
        raise ValueError('stable_samples must be 2..100')
    if control['slowdown_distance_m'] <= control['axial_tolerance_m']:
        raise ValueError('slowdown distance must exceed axial tolerance')
    vision = data['vision']
    for key in ('min_decision_margin', 'max_reprojection_px', 'min_tag_edge_px',
                'min_pnp_solution_gap_px', 'position_tolerance_m', 'angle_tolerance_rad'):
        value = vision[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'vision.{key} must be finite and positive')
    if set(data['tools']) != {0, 1}:
        raise ValueError('manual docking requires exactly tool0 and tool1')
    for item in data['tools'].values():
        if (type(item['tag_id']) is not int or item['tag_id'] < 0 or
                not isinstance(item['tag_family'], str) or not item['tag_family'] or
                type(item['tag_size_m']) not in (int, float) or
                not 0 < item['tag_size_m'] <= 1):
            raise ValueError('invalid tag identity/size')
        item['tag_to_tcp'] = transform(item['tag_to_tcp_xyz_m'], item['tag_to_tcp_xyzw'])
    if data['tools'][0]['tag_id'] == data['tools'][1]['tag_id']:
        raise ValueError('tool tag IDs must differ')
    return data


def checked_pose(pose):
    pose = np.asarray(pose, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all() or
            not np.allclose(pose[3], [0., 0., 0., 1.], atol=1e-9) or
            not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-6)):
        raise ValueError('invalid rigid pose')
    return pose


def capped(vector, maximum):
    return vector * min(1., maximum / max(float(np.linalg.norm(vector)), 1e-12))


class DockingAssist:
    def __init__(self, settings):
        self.settings = settings
        self.cfg = settings['control']
        self.tool = None
        self.state = 'idle'
        self.reason = 'not selected'
        self.observation_reason = 'no observations'
        self.base_tag = None
        self.target = None
        self.last_stamp = -1
        self.expires = -math.inf
        self.stable = 0
        self.errors = {}

    @property
    def selected(self):
        return self.state in ('acquiring', 'active')

    def begin(self, tool, now):
        if tool not in self.settings['tools']:
            raise ValueError('unsupported tool')
        self.tool, self.state, self.reason = tool, 'acquiring', 'waiting for fresh tag samples'
        self.started = now
        self.base_tag = self.target = None
        self.last_stamp, self.expires, self.stable = -1, -math.inf, 0
        self.errors = {}
        self.observation_reason = 'waiting for post-selection image'

    def cancel(self, reason):
        self.state, self.reason = 'idle', reason
        self.base_tag = self.target = None
        self.stable = 0

    def reject(self, reason):
        self.observation_reason = reason
        if self.state == 'acquiring':
            self.stable = 0

    def observe(self, pose, stamp_ns, now, age_s):
        if not self.selected:
            return
        if not math.isfinite(age_s) or not 0 <= age_s < self.cfg['tag_max_age_s']:
            raise ValueError('stale/future tag timestamp')
        if stamp_ns <= self.last_stamp:
            raise ValueError('tag timestamp did not advance')
        if self.state == 'active' and now >= self.expires:
            raise ValueError('tag input timeout; select assist again')
        pose = checked_pose(pose)
        if self.base_tag is not None:
            shift, angle = pose_errors(self.base_tag, pose)
            if (shift > self.cfg['max_tag_position_jump_m'] or
                    angle > self.cfg['max_tag_angle_jump_rad']):
                raise ValueError('tag pose jump')
        if now >= self.expires:
            self.stable = 0
        self.base_tag = pose.copy()
        self.target = pose @ self.settings['tools'][self.tool]['tag_to_tcp']
        self.last_stamp = stamp_ns
        # Receipt time must not give an old image a second full timeout period.
        self.expires = now + self.cfg['tag_max_age_s'] - age_s
        self.stable += 1
        self.observation_reason = 'fresh tag accepted'

    def check(self, current, now):
        if self.state == 'acquiring':
            if now-self.started >= self.cfg['acquire_timeout_s']:
                raise ValueError('tag acquisition timeout')
            if self.stable < self.cfg['stable_samples'] or now >= self.expires:
                return False
        elif self.state != 'active':
            return False
        if now >= self.expires:
            raise ValueError('tag input timeout')
        distance, angle = pose_errors(checked_pose(current), self.target)
        if distance > self.cfg['max_target_distance_m'] or angle > self.cfg['max_target_angle_rad']:
            raise ValueError('dock target outside assist envelope; approach manually')
        self.state = 'active'
        return True

    def assist_twist(self, current, manual_twist, now, dt):
        """Correct ROOT-frame physical velocity from the original manual mapping."""
        manual_twist = np.asarray(manual_twist, dtype=float)
        if manual_twist.shape != (6,) or not np.isfinite(manual_twist).all() or not 0 < dt <= .1:
            raise ValueError('invalid manual velocity or interval')
        if not self.check(current, now):
            return manual_twist.copy()
        cfg = self.cfg
        rotation = current[:3, :3]
        normal = self.target[:3, 2]
        error = self.target[:3, 3]-current[:3, 3]
        axial = float(error @ normal)
        lateral = error-normal*axial
        rot_error = cv2.Rodrigues(self.target[:3, :3] @ rotation.T)[0].reshape(3)
        self.errors = dict(axial_m=axial, lateral_m=float(np.linalg.norm(lateral)),
                           angle_rad=float(np.linalg.norm(rot_error)))
        self.reason = 'manual hold' if not np.any(manual_twist) else 'manual motion'
        linear, angular = manual_twist[:3].copy(), manual_twist[3:].copy()
        forward = float(linear @ normal)
        if forward > 0:
            strength = min(1., forward/cfg['assist_full_speed_m_s'])
            aligned = (self.errors['lateral_m'] <= cfg['lateral_tolerance_m'] and
                       self.errors['angle_rad'] <= cfg['angle_tolerance_rad'])
            gap = max(0., axial-cfg['axial_tolerance_m'])
            if gap < 1e-9:
                gap = 0.
            allowed = (min(forward * max(0., min(1., axial/cfg['slowdown_distance_m'])),
                           gap/dt) if aligned else 0.)
            # Slow/gate the WHOLE original movement, preserving its direction.
            # Removing only its normal component could leave a large sideways
            # drift when the original radial manual direction is oblique to
            # the dock. Explicit joint/O inputs bypass this assist in the adapter.
            scale = allowed/forward
            linear *= scale
            angular *= scale
            linear += capped(lateral*cfg['lateral_gain_s_inv'], cfg['max_lateral_correction_m_s'])*strength
            angular += capped(rot_error*cfg['angular_gain_s_inv'], cfg['max_angular_correction_rad_s'])*strength
            if not aligned:
                self.reason = 'aligning; forward travel gated'
            elif allowed == 0:
                self.reason = 'dock reference reached; retreat available'
        return np.concatenate((linear, angular))

    def snapshot(self):
        return dict(state=self.state, tool=self.tool, reason=self.reason,
                    observation_reason=self.observation_reason,
                    stable_samples=self.stable, **self.errors)


def joint_velocity(jacobian, twist, speeds, cfg, axial_normal, manual_velocity=None):
    jacobian, twist = np.asarray(jacobian), np.asarray(twist)
    if jacobian.shape != (6, 6) or twist.shape != (6,) or not np.isfinite(jacobian).all() or not np.isfinite(twist).all():
        raise ValueError('invalid full-arm differential IK input')
    weights = np.array([1.]*3 + [cfg['rotation_weight_m']]*3)
    base = np.zeros(6) if manual_velocity is None else np.asarray(manual_velocity, dtype=float)
    if base.shape != (6,) or not np.isfinite(base).all():
        raise ValueError('invalid manual joint velocity')
    delta = twist-jacobian @ base
    if np.max(np.abs(delta)) < 1e-12:
        return base.copy()
    matrix, requested = weights[:, None]*jacobian, weights*delta
    # Give commanded axial speed an exact first-order constraint. Ordinary
    # damped 6D IK can turn a lateral/tilt correction into unintended insertion.
    row = np.asarray(axial_normal) @ jacobian[:3]
    norm_sq = float(row @ row)
    if norm_sq < 1e-12:
        return np.zeros(6)
    primary = row * float(np.asarray(axial_normal) @ delta[:3]) / norm_sq
    nullspace = np.eye(6) - np.outer(row, row)/norm_sq
    reduced = matrix @ nullspace
    velocity = base + primary + reduced.T @ np.linalg.solve(
        reduced @ reduced.T + cfg['damping']**2*np.eye(6), requested-matrix @ primary)
    caps = np.minimum(speeds, cfg['joint_speed_rad_s'])
    return velocity / max(1., float(np.max(np.abs(velocity)/caps)))
