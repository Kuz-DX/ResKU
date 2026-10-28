"""Incremental Cartesian teleoperation using tool_change_min's shared FK/IK."""
import math
import time
from collections import Counter, deque
import numpy as np


class LimitBlocked(ValueError):
    """All bounded attempts hit a joint limit; keep the last accepted target."""


class EeDpad:
    """PS axes 6/7: left CCW, right CW; up/down adjust once per press."""
    def __init__(self):
        self.speed = 3.
        self.vertical = 0
        self.direction = 0

    def update(self, axes, enabled=True):
        if not enabled or len(axes) < 8 or not np.all(np.isfinite(axes)):
            self.vertical = self.direction = 0
            return
        vertical = int(axes[7] > .5)-int(axes[7] < -.5)
        if vertical and vertical != self.vertical:
            self.speed = round(float(np.clip(self.speed+vertical, .02, 16.7)), 2)
        self.vertical = vertical
        # Do not rotate on diagonal presses used to change speed.
        self.direction = 0 if vertical else int(axes[6] > .5)-int(axes[6] < -.5)


def nearest_tcp_entry(model, measured, reference, margin=.121,
                      condition_limit=20., samples=100):
    """Nearest point on the limit-projection -> reviewed-ready corridor.

    Collision checking remains the managed workflow's responsibility. This
    function only establishes joint margin and numerical IK conditioning.
    The default leaves 0.02 rad of settling room beyond the 0.101-rad
    admission check, so a boundary target does not immediately fail re-entry.
    """
    measured = np.asarray(measured, dtype=float)
    reference = np.asarray(reference, dtype=float)
    if measured.shape != (6,) or reference.shape != (6,) or not (
            np.isfinite(measured).all() and np.isfinite(reference).all()):
        raise ValueError('nearest TCP entry requires finite six-joint poses')
    limits = np.asarray(list(model.limits.values()), dtype=float)
    lower, upper = limits[:, 0]+margin, limits[:, 1]-margin
    if np.any(lower >= upper) or not model.within_limits(reference, margin):
        raise ValueError('reviewed TCP ready pose does not satisfy entry margin')
    projected = np.clip(measured, lower, upper)

    def acceptable(q):
        if not model.within_limits(q, margin):
            return False
        jacobian = model.jacobian(q).copy()
        jacobian[:3] /= .2
        condition = float(np.linalg.cond(jacobian))
        return math.isfinite(condition) and condition <= condition_limit

    if acceptable(projected):
        return projected
    previous_fraction = 0.
    for index in range(1, samples+1):
        fraction = index/samples
        candidate = projected+fraction*(reference-projected)
        if acceptable(candidate):
            low, high = previous_fraction, fraction
            for _ in range(20):
                middle = (low+high)/2
                if acceptable(projected+middle*(reference-projected)):
                    high = middle
                else:
                    low = middle
            return projected+high*(reference-projected)
        previous_fraction = fraction
    raise ValueError('no TCP entry pose with condition <= %.1f on ready corridor' % condition_limit)


class SafetyError(ValueError):
    """Invalid feedback/command state: stop immediately, not after five IK failures."""
    def __init__(self, reason, fault=True):
        super().__init__(reason)
        self.fault = fault


class TcpMetrics:
    """Bounded timing windows plus lifetime outcome/overrun counters."""
    def __init__(self, window=512):
        self.samples = {name: deque(maxlen=window) for name in ('ik', 'jacobian', 'callback', 'interval')}
        self.counts = Counter()
        self.failures = Counter()

    def observe(self, name, milliseconds):
        self.samples[name].append(float(milliseconds))
        self.counts[name+'_samples'] += 1
        if name in ('ik', 'callback', 'interval') and milliseconds > 20.:
            self.counts[name+'_over_20ms'] += 1

    def snapshot(self):
        result = dict(counts=dict(self.counts), failures_by_reason=dict(self.failures))
        for name, values in self.samples.items():
            result[name+'_ms'] = (None if not values else dict(
                window_samples=len(values), mean=float(np.mean(values)), max=max(values),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99))))
        return result


def joy_twist(axes, buttons, deadzone=.12):
    """PS layout: L1 deadman, R1 rotation modifier, left XY, right vertical Z."""
    if len(axes) < 5 or len(buttons) < 6 or not np.all(np.isfinite(axes)):
        return False, np.zeros(6)
    if not buttons[4]:
        return False, np.zeros(6)
    def axis(i):
        v = float(np.clip(axes[i], -1., 1.))
        return 0. if abs(v) <= deadzone else math.copysign((abs(v)-deadzone)/(1-deadzone), v)
    xyz = np.array([axis(1), axis(0), axis(4)])
    xyz /= max(1., np.linalg.norm(xyz))
    result = np.zeros(6)
    result[3:6] = xyz if buttons[5] else 0.
    result[:3] = xyz if not buttons[5] else 0.
    return True, result


def joy_joint_jog(axes, buttons, deadzone=.12):
    """L1+R1: right vertical base, left vertical pitch, horizontals roll/yaw."""
    direction = np.zeros(6)
    if (len(axes) < 5 or len(buttons) < 6 or not buttons[4] or not buttons[5]
            or not np.all(np.isfinite(axes))):
        return direction
    for joint, axis in ((0, 4), (3, 1), (4, 0), (5, 3)):
        value = float(np.clip(axes[axis], -1., 1.))
        if abs(value) > deadzone:
            direction[joint] = math.copysign((abs(value)-deadzone)/(1.-deadzone), value)
    return direction


class TcpStepper:
    def __init__(self, model, speeds, linear=.02, angular=.15, margin=.02,
                 lead=.08, iterations=30, budget=.030):
        self.model = model
        self.speeds = np.minimum(np.asarray(speeds), .12)
        self.linear, self.angular, self.margin = linear, angular, margin
        self.lead, self.iterations, self.budget = lead, iterations, budget
        if (self.speeds.shape != (6,) or not np.all(np.isfinite(self.speeds)) or
                np.any(self.speeds <= 0) or not all(math.isfinite(v) and v > 0 for v in
                (linear, angular, margin, lead, budget)) or iterations < 1):
            raise ValueError('invalid TCP limits')
        self.q = self.pose = None
        self.condition = math.inf
        self.scale = 0.
        self.velocity_scale = 1.
        self.last_jacobian_duration_ms = None

    def reset(self, measured):
        if not self.model.within_limits(measured, self.margin):
            raise SafetyError('measured seed violates soft limit margin')
        self.q = np.asarray(measured).copy()
        self.pose = self.model.fk(self.q)

    def joint_jog_candidate(self, measured, direction, dt):
        """Direct base/pitch/roll/yaw proposal; caller collision-checks it."""
        direction = np.asarray(direction, dtype=float)
        if (direction.shape != (6,) or not np.all(np.isfinite(direction))
                or np.any(direction[[1, 2]]) or not 0 < dt <= .1):
            raise ValueError('invalid joint jog increment')
        if self.q is None or not self.model.within_limits(measured, self.margin):
            raise SafetyError('joint jog feedback violates soft limits')
        q = self.q.copy()
        q += np.clip(direction, -1., 1.) * self.speeds * dt
        if not self.model.within_limits(q, self.margin):
            raise LimitBlocked('joint jog limit')
        if np.max(np.abs(q-measured)) > self.lead:
            raise LimitBlocked('joint jog feedback lead limit')
        return q, self.model.fk(q)

    def wrist_candidate(self, measured, direction, dt):
        """Keep the existing roll/yaw API for callers of direct wrist jog."""
        direction = np.asarray(direction, dtype=float)
        if direction.shape != (2,) or not np.all(np.isfinite(direction)):
            raise ValueError('invalid wrist increment')
        jog = np.zeros(6)
        jog[4:6] = direction
        return self.joint_jog_candidate(measured, jog, dt)

    def candidate(self, measured, twist, dt):
        """Propose base-frame translation and target-TCP-local rotation."""
        deadline = time.monotonic() + self.budget
        self.last_jacobian_duration_ms = None
        try:
            from tool_change_min.kinematics import _rotation
        except ImportError:
            from arm_shared_kinematics import _rotation
        if self.q is None or not 0 < dt <= .1:
            raise ValueError('uninitialized target or invalid timestep')
        if not self.model.within_limits(measured, self.margin):
            raise SafetyError('feedback violates soft limits')
        if (np.asarray(self.q).shape != (6,) or not np.all(np.isfinite(self.q))
                or not self.model.within_limits(self.q, self.margin)):
            raise SafetyError('cached IK seed invalid')
        twist = np.asarray(twist, dtype=float)
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            raise SafetyError('invalid TCP increment')
        if self.pose is None or self.pose.shape != (4, 4) or not np.all(np.isfinite(self.pose)):
            raise SafetyError('cached TCP pose invalid')
        seed_error = np.asarray(self.model.pose_error(self.q, self.pose))
        if (seed_error.shape != (6,) or not np.all(np.isfinite(seed_error))
                or np.linalg.norm(seed_error[:3]) >= 1e-5
                or np.linalg.norm(seed_error[3:]) >= 1e-4):
            raise SafetyError('cached seed/pose FK mismatch')
        self.last_jacobian_duration_ms = None
        jac_started = time.perf_counter()
        jac = self.model.jacobian(np.asarray(measured))
        self.last_jacobian_duration_ms = (time.perf_counter()-jac_started)*1000.
        if jac.shape != (6, 6) or not np.all(np.isfinite(jac)):
            raise SafetyError('invalid Jacobian')
        # Characteristic length 0.2 m balances translational/rotational units.
        jac = jac.copy()
        jac[:3] /= .2
        singular = np.linalg.svd(jac, compute_uv=False)
        self.condition = (math.inf if singular[-1] < 1e-8 else float(singular[0]/singular[-1]))
        self.scale = float(np.clip((100.-self.condition)/80., 0., 1.))
        if self.scale <= 0:
            raise ValueError('singularity stop (condition >= 100)')
        increment = np.asarray(twist)*np.array([self.linear]*3+[self.angular]*3)*dt*self.scale
        # Predict joint demand, then shrink the entire Cartesian increment so its
        # direction is retained. Never clip individual joints after solving IK.
        scaled_increment = increment.copy()
        scaled_increment[:3] /= .2
        # The geometric Jacobian is spatial (base-frame). Map the local
        # angular increment using the accepted target orientation, which also
        # defines the rotation below, even when measured feedback lags.
        scaled_increment[3:] = self.pose[:3, :3] @ increment[3:]
        predicted = np.linalg.solve(jac, scaled_increment)
        ratio = float(np.max(np.abs(predicted)/(self.speeds*dt)))
        self.velocity_scale = min(1., .9/max(ratio, 1e-12))
        attempts = min(4, self.iterations)
        error = 'joint velocity limit'
        limit_rejections = 0
        for _ in range(attempts):
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise ValueError('IK time budget exceeded')
            target = self.pose.copy()
            target[:3, 3] += increment[:3]*self.velocity_scale
            rot = increment[3:]*self.velocity_scale
            angle = np.linalg.norm(rot)
            if angle > 1e-12:
                target[:3, :3] = target[:3, :3] @ _rotation(rot/angle, angle)
            try:
                # self.pose was produced by self.q.  Seeding the next small
                # increment from that matching command state avoids repeatedly
                # solving the accumulated TCP lead from lagging feedback.  The
                # measured state is still enforced by the lead/limit checks.
                solved = self.model.solve(target.copy(), self.q.copy(), self.margin,
                    max_iterations=self.iterations//attempts, max_time_s=remaining)
            except ValueError as exc:
                error = str(exc)
                if error.startswith('IK iteration would violate soft limits'):
                    limit_rejections += 1
                self.velocity_scale *= .5
                continue
            if time.monotonic() >= deadline:
                raise ValueError('IK time budget exceeded')
            solved = np.asarray(solved, dtype=float)
            if (solved.shape != (6,) or not np.all(np.isfinite(solved))
                    or not self.model.within_limits(solved, self.margin)):
                raise SafetyError('invalid IK solution')
            residual = np.asarray(self.model.pose_error(solved, target))
            if residual.shape != (6,) or not np.all(np.isfinite(residual)):
                raise SafetyError('invalid FK residual')
            if np.linalg.norm(residual[:3]) >= 1e-5 or np.linalg.norm(residual[3:]) >= 1e-4:
                raise ValueError('IK FK residual exceeds tolerance')
            if time.monotonic() >= deadline:
                raise ValueError('IK time budget exceeded')
            ratio = float(np.max(np.abs(solved-self.q)/(self.speeds*dt)))
            if ratio > 1.+1e-9:
                error = 'joint velocity limit'
                self.velocity_scale *= min(.5, .9/ratio)
                continue
            if np.max(np.abs(solved-measured)) > self.lead:
                raise SafetyError('joint feedback lead limit', fault=False)
            return solved, target
        if limit_rejections == attempts:
            raise LimitBlocked('soft limit: increment blocked; reverse stick direction')
        raise ValueError(error)

    def commit(self, q, pose):
        self.q, self.pose = q.copy(), pose.copy()
