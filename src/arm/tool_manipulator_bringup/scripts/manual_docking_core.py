"""One requested angular alignment, followed by measured orientation monitoring."""
import math
import cv2
import numpy as np
import yaml
from tool_docking_core import transform, pose_errors, bounded_target

def load_config(filename):
    with open(filename, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    control = data['control']
    limits = {
        'joint_speed_rad_s': .12, 'max_target_angle_rad': 1.,
        'tag_max_age_s': .5, 'acquire_timeout_s': 30.,
        'max_tag_position_jump_m': .05, 'max_tag_angle_jump_rad': .5,
    }
    for key, upper in limits.items():
        value = control[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f'{key} must be finite and in (0, {upper}]')
    if type(control['stable_samples']) is not int or not 2 <= control['stable_samples'] <= 100:
        raise ValueError('stable_samples must be 2..100')
    alignment = data['alignment']
    if alignment.get('mode') not in ('rmd_pitch',):
        raise ValueError('alignment.mode must be rmd_pitch')
    tolerance = alignment.get('angle_tolerance_rad')
    if tolerance is not None and (type(tolerance) not in (int, float) or
            not math.isfinite(tolerance) or not 0 < tolerance <= .2):
        raise ValueError('invalid alignment.angle_tolerance_rad')
    for key in ('position_tolerance_m', 'orientation_step_rad', 'joint_sample_rad',
                'acceleration_rad_s2', 'planning_timeout_s', 'feedback_timeout_s'):
        value = alignment[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid alignment.' + key)
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


def alignment_target(current, dock, mode, base_yaw):
    """Keep TCP position. Align its +Z insertion axis without commanded axial roll."""
    current, dock = checked_pose(current), checked_pose(dock)
    source, desired = current[:3, 2], dock[:3, 2]
    if mode == 'axis':
        cross = np.cross(source, desired)
        sine, cosine = float(np.linalg.norm(cross)), float(source @ desired)
        if sine < 1e-9:
            if cosine < 0:
                raise ValueError('opposed insertion axes; approach manually')
            vector = np.zeros(3)
        else:
            vector = cross/sine * math.atan2(sine, cosine)
    elif mode == 'rmd_pitch':
        axis = np.array([-math.sin(base_yaw), math.cos(base_yaw), 0.])
        first, second = source-axis*(source@axis), desired-axis*(desired@axis)
        if min(np.linalg.norm(first), np.linalg.norm(second)) < 1e-6:
            raise ValueError('insertion axis has undefined pitch in the base plane')
        first, second = first/np.linalg.norm(first), second/np.linalg.norm(second)
        vector = axis*math.atan2(float(axis@np.cross(first, second)), float(first@second))
    else:
        raise ValueError('alignment axes not confirmed')
    target = current.copy()
    target[:3, :3] = cv2.Rodrigues(vector)[0] @ current[:3, :3]
    return target


class RmdModel:
    """Reuse ArmKinematics.solve with only the three RMD columns/unknowns."""
    def __init__(self, model, fixed):
        self.model, self.fixed = model, fixed.copy()
        self.joint_names = model.joint_names[1:4]
        self.limits = {name: model.limits[name] for name in self.joint_names}

    def expand(self, q):
        full = self.fixed.copy()
        full[1:4] = q
        return full

    def fk(self, q):
        return self.model.fk(self.expand(q))

    def pose_and_jacobian(self, q):
        pose, jacobian = self.model.pose_and_jacobian(self.expand(q))
        return pose, jacobian[:, 1:4]

    def _pose_error_from_pose(self, current, target):
        return self.model._pose_error_from_pose(current, target)

    def within_limits(self, q, margin_rad=0.):
        return self.model.within_limits(self.expand(q), margin_rad)

    def solve(self, target, seed, margin):
        return type(self.model).solve(self, target, seed, margin, max_time_s=.1)


def dxl_reference(start, model, margin):
    reference = start.copy()
    reference[0] = min((-math.pi, math.pi), key=lambda angle: abs(angle-start[0]))
    reference[4:6] = 0.
    if not model.within_limits(reference, margin):
        raise ValueError('requested DXL reference outside hardware limits')
    return reference


def plan_alignment(model, start, dock, cfg, speed_limits, margin, cancelled):
    """Plan DXL preparation, then position-preserving RMD-only pitch alignment.

    Worker-only math; no ROS state or commands. Every path sample is returned
    for collision validation before any motion, including the DXL preparation.
    """
    reference = dxl_reference(start, model, margin)
    initial = model.fk(reference)
    camera_rotation = np.asarray(cfg.get('tcp_camera_rotation', np.eye(3)), dtype=float)
    if camera_rotation.shape != (3, 3) or not np.isfinite(camera_rotation).all():
        raise ValueError('invalid TCP-to-optical rotation')
    alignment_frame = initial.copy()
    alignment_frame[:3, :3] = initial[:3, :3] @ camera_rotation
    frame_target = alignment_target(alignment_frame, dock, cfg['mode'], reference[0])
    target = frame_target.copy()
    target[:3, :3] = frame_target[:3, :3] @ camera_rotation.T
    residual = math.acos(float(np.clip(frame_target[:3, 2] @ dock[:3, 2], -1., 1.)))
    if residual > cfg['angle_tolerance_rad']:
        raise ValueError('tag insertion axis outside fixed-DXL RMD plane; reposition manually')
    angle = pose_errors(initial, target)[1]
    if angle > cfg['max_angle_rad']:
        raise ValueError('required RMD pitch change too large; approach manually')
    count = max(1, math.ceil(angle/cfg['orientation_step_rad']))
    rmd = RmdModel(model, reference)
    seed = start.copy()
    points, durations, samples, stages = [], [], [seed.copy()], []

    def append(goal, stage):
        nonlocal seed
        delta = np.abs(goal-seed)
        duration = max(.02, float(np.max(1.875*delta/speed_limits)),
                       math.sqrt(float(np.max((10/math.sqrt(3))*delta/cfg['acceleration_rad_s2']))))
        divisions = max(1, math.ceil(float(np.max(delta))/cfg['joint_sample_rad']))
        if len(samples)+divisions > 2048:
            raise ValueError('alignment path sample budget exceeded')
        for fraction in np.linspace(0., 1., divisions+1)[1:]:
            q = seed+(goal-seed)*fraction
            if not model.within_limits(q, margin):
                raise ValueError('interpolated alignment point outside limits')
            if stage == 'RMD_ALIGN' and np.linalg.norm(model.fk(q)[:3, 3]-initial[:3, 3]) > cfg['position_tolerance_m']:
                raise ValueError('RMD alignment would translate TCP beyond position tolerance')
            samples.append(q.copy())
        points.append(goal.copy())
        durations.append(duration)
        stages.append(stage)
        seed = goal.copy()

    append(reference, 'DXL_PREPARE')
    for index in range(1, count+1):
        if cancelled.is_set():
            raise ValueError('alignment planning cancelled')
        waypoint = bounded_target(initial, target, 1., angle*index/count)
        goal = rmd.expand(rmd.solve(waypoint, seed[1:4], margin))
        append(goal, 'RMD_ALIGN')
    return dict(start=start.copy(), points=points, durations=durations, samples=samples,
                stages=stages, target=target.copy(), reference=reference)


def trajectory_position(plan, elapsed):
    start = plan['start']
    for goal, duration in zip(plan['points'], plan['durations']):
        if elapsed < duration:
            s = max(0., elapsed/duration)
            fraction = s*s*s*(10+s*(-15+6*s))
            return start+(goal-start)*fraction
        elapsed -= duration
        start = goal
    return start.copy()


class DockingAssist:
    def __init__(self, settings):
        self.settings, self.cfg = settings, settings['control']
        self.tool, self.state, self.reason = None, 'idle', 'not requested'
        self.observation_reason = 'no observations'
        self.base_tag = self.observed_target = self.target = None
        self.last_stamp, self.expires, self.stable = -1, -math.inf, 0
        self.errors = {}
        self.monitoring = False

    @property
    def selected(self):
        return self.state not in ('idle', 'ALIGNMENT_FAILED')

    def begin(self, tool, now):
        if tool not in self.settings['tools']:
            raise ValueError('unsupported tool')
        self.tool, self.state, self.reason = tool, 'TAG_NOT_DETECTED', 'waiting for fresh tag samples'
        self.started = now
        self.base_tag = self.observed_target = self.target = None
        self.last_stamp, self.expires, self.stable = -1, -math.inf, 0
        self.errors, self.monitoring = {}, False
        self.observation_reason = 'waiting for post-request image'

    def cancel(self, reason):
        self.state, self.reason, self.monitoring = 'idle', reason, False
        self.base_tag = self.observed_target = self.target = None
        self.stable = 0

    def fail(self, reason):
        self.state, self.reason, self.monitoring = 'ALIGNMENT_FAILED', reason, False

    def reject(self, reason):
        self.observation_reason = reason
        if self.target is None:
            self.stable = 0

    def observe(self, pose, stamp_ns, now, age_s):
        if not self.selected:
            return
        if not math.isfinite(age_s) or not 0 <= age_s < self.cfg['tag_max_age_s']:
            raise ValueError('stale/future tag timestamp')
        if stamp_ns <= self.last_stamp:
            raise ValueError('tag timestamp did not advance')
        pose = checked_pose(pose)
        if self.base_tag is not None:
            shift, angle = pose_errors(self.base_tag, pose)
            if shift > self.cfg['max_tag_position_jump_m'] or angle > self.cfg['max_tag_angle_jump_rad']:
                raise ValueError('tag pose jump')
        if now >= self.expires:
            self.stable = 0
        self.base_tag = pose.copy()
        self.observed_target = pose @ self.settings['tools'][self.tool]['tag_to_tcp']
        self.last_stamp, self.expires = stamp_ns, now+self.cfg['tag_max_age_s']-age_s
        self.stable += 1
        self.observation_reason = 'fresh tag accepted; alignment target remains latched'

    def capture(self, current, now, base_yaw):
        if self.target is not None:
            return True
        if now-self.started >= self.cfg['acquire_timeout_s']:
            raise ValueError('tag acquisition timeout')
        if self.stable < self.cfg['stable_samples'] or now >= self.expires:
            return False
        self.target = self.observed_target.copy()  # Frozen once per request.
        self.state, self.reason = 'ALIGNMENT_READY', 'target captured; checking complete path'
        return True

    def monitor(self, measured_pose):
        if self.target is None:
            return
        distance, angle = pose_errors(checked_pose(measured_pose), self.target)
        self.errors = dict(angle_rad=angle, tcp_displacement_m=distance)
        if self.monitoring:
            self.state = ('ALIGNMENT_OK' if angle <= self.settings['alignment']['angle_tolerance_rad']
                          else 'ALIGNMENT_READY')
            self.reason = (('camera plane aligned; insertion angle NOT verified' if self.settings['tools'][self.tool].get('alignment_reference') == 'camera_plane' else 'measured insertion orientation aligned; manual docking only') if self.state == 'ALIGNMENT_OK'
                           else 'orientation changed; press tool combination to align again')

    def snapshot(self):
        return dict(state=self.state, tool=self.tool, reason=self.reason,
                    observation_reason=self.observation_reason, stable_samples=self.stable,
                    monitoring_only=self.monitoring, target_latched=self.target is not None, **self.errors)
