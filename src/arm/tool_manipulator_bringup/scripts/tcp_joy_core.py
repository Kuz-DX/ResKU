"""Incremental Cartesian teleoperation using tool_change_min's shared FK/IK."""
import math
import time
import numpy as np


class LimitBlocked(ValueError):
    """All bounded attempts hit a joint limit; keep the last accepted target."""


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

    def reset(self, measured):
        if not self.model.within_limits(measured, self.margin):
            raise ValueError('measured seed violates soft limit margin')
        self.q = np.asarray(measured).copy()
        self.pose = self.model.fk(self.q)

    def candidate(self, measured, twist, dt):
        """Propose base-frame translation and target-TCP-local rotation."""
        deadline = time.monotonic() + self.budget
        try:
            from tool_change_min.kinematics import _rotation
        except ImportError:
            from arm_shared_kinematics import _rotation
        if self.q is None or not 0 < dt <= .1:
            raise ValueError('uninitialized target or invalid timestep')
        if not self.model.within_limits(measured, self.margin):
            raise ValueError('seed violates soft limits')
        jac = self.model.jacobian(np.asarray(measured))
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
                solved = self.model.solve(target, self.q, self.margin,
                    max_iterations=self.iterations//attempts, max_time_s=remaining)
            except ValueError as exc:
                error = str(exc)
                if error == 'IK iteration would violate soft limits':
                    limit_rejections += 1
                self.velocity_scale *= .5
                continue
            if time.monotonic() >= deadline:
                raise ValueError('IK time budget exceeded')
            if not np.all(np.isfinite(solved)) or not self.model.within_limits(solved, self.margin):
                raise ValueError('invalid IK solution')
            ratio = float(np.max(np.abs(solved-self.q)/(self.speeds*dt)))
            if ratio > 1.+1e-9:
                error = 'joint velocity limit'
                self.velocity_scale *= min(.5, .9/ratio)
                continue
            if np.max(np.abs(solved-measured)) > self.lead:
                raise ValueError('joint feedback lead limit')
            return solved, target
        if limit_rejections == attempts:
            raise LimitBlocked('soft limit: increment blocked; reverse stick direction')
        raise ValueError(error)

    def commit(self, q, pose):
        self.q, self.pose = q.copy(), pose.copy()
