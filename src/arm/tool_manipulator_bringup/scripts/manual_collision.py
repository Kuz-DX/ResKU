#!/usr/bin/env python3
"""Main manual control: rate-limited collision lookahead, 50 Hz position output."""
import math

import numpy as np
from moveit_msgs.srv import GetStateValidity
from manual import Manual, main
from manual_control_common import JOINTS

CHECK_JOINTS = (*JOINTS, 'ee_joint')
CHECK_GROUP = 'manual_collision'


def path_samples(measured, start, end, steps, budget=96):
    """Sample the command segment and its connections to measured feedback."""
    out, seen = [], set()
    for a, b in ((measured, start), (measured, end), (start, end)):
        count = max(1, math.ceil(float(np.max(np.abs(b-a)/steps))))
        if count+1 > budget:
            raise ValueError('collision sample budget exceeded')
        for q in np.linspace(a, b, count+1):
            key = tuple(map(float, q))
            if key not in seen:
                seen.add(key)
                out.append(key)
                if len(out) > budget:
                    raise ValueError('collision sample budget exceeded')
    return out


def segment_error(position, a, b):
    delta = b-a
    length_sq = float(delta@delta)
    fraction = 0. if length_sq < 1e-20 else float(np.clip((position-a)@delta/length_sq, 0., 1.))
    return float(np.max(np.abs(position-(a+fraction*delta))))


class CollisionWindow:
    """Only emit steps on an approved line; never wait inside a ROS callback.

    A batch is submitted at most once per period. Position output pauses while
    a batch is pending. Between batches, small steps consume the checked line.
    Invalid/stale/failed checks discard the line and capture measured HOLD.
    """
    def __init__(self, client, rate=10., horizon=.35, timeout=.15,
                 step=.005, ee_step=.02, tolerance=.01):
        values = (rate, horizon, timeout, step, ee_step, tolerance)
        if (not all(math.isfinite(v) for v in values) or not 1 <= rate <= 20 or
                not .02 <= timeout <= .5 or not 0 < step <= .01 or
                not 0 < ee_step <= .05 or not 0 < tolerance <= .02 or
                not 1/rate+timeout+.04 <= horizon <= 1.):
            raise ValueError('invalid collision window parameters')
        self.client, self.period, self.horizon, self.timeout = client, 1/rate, horizon, timeout
        self.steps = np.array([step]*6+[ee_step])
        self.tolerance = tolerance
        self.pending = self.plan = self.context = None
        self.next_check = -math.inf
        self.reason = 'waiting for collision check'
        self.contacts = []
        self.latency_ms = None
        self.batches = self.accepted = self.rejected = self.timeouts = 0

    def cancel(self):
        if self.pending is not None:
            for future in self.pending['futures']:
                if not future.done():
                    future.cancel()
        self.pending = self.plan = None

    def stop(self, measured, reason):
        self.cancel()
        self.reason = reason
        return measured[:6].copy(), 0.

    def _submit(self, now, measured, command, velocity, bounds, lead, goal):
        duration = self.horizon
        lo, hi = bounds
        for i, v in enumerate(velocity[:6]):
            if abs(v) < 1e-12:
                continue
            # Startup may begin just outside soft bounds. Its existing inward
            # recovery policy supplies an explicit goal; no bound is widened.
            bound = goal[i] if goal is not None else (hi[i] if v > 0 else lo[i])
            duration = min(duration, max(0., (bound-command[i])/v))
            lead_bound = measured[i]+math.copysign(lead, v)
            duration = min(duration, max(0., (lead_bound-command[i])/v))
        if duration < 1e-8:
            self.reason = 'motion boundary; holding position'
            self.plan = None
            return
        start = np.r_[command, measured[6]]
        end = start+velocity*duration
        samples = path_samples(measured, start, end, self.steps)
        futures = []
        try:
            for sample in samples:
                req = GetStateValidity.Request()
                req.group_name = CHECK_GROUP
                req.robot_state.joint_state.name = list(CHECK_JOINTS)
                req.robot_state.joint_state.position = list(sample)
                req.robot_state.is_diff = True
                futures.append(self.client.call_async(req))
        except Exception:
            for future in futures:
                if not future.done():
                    future.cancel()
            raise
        self.pending = dict(futures=futures, start=start, end=end, velocity=velocity.copy(),
                            measured=measured.copy(), sent=now, context=self.context)
        self.plan = None
        self.batches += 1
        self.reason = 'checking collision lookahead'

    def step(self, now, dt, measured, command, velocity, context, bounds, lead, goal=None):
        measured, command, velocity = map(lambda a: np.asarray(a, dtype=float),
                                         (measured, command, velocity))
        if (measured.shape != (7,) or command.shape != (6,) or velocity.shape != (7,) or
                not all(np.all(np.isfinite(a)) for a in (measured, command, velocity)) or
                not 0 < dt <= .05):
            raise ValueError('invalid collision window input')
        if self.context != context:
            changed = self.context is not None
            self.context = context
            self.cancel()
            if changed:
                return self.stop(measured, 'input changed; measured hold')
        if not np.any(np.abs(velocity) > 1e-10):
            self.cancel()
            self.reason = 'idle'
            # Explicit one-shot targets still need to settle at their endpoint.
            return (command.copy() if goal is not None else measured[:6].copy()), 0.
        if not self.client.service_is_ready():
            return self.stop(measured, 'collision service unavailable; measured hold')

        if self.pending is not None:
            p = self.pending
            if now-p['sent'] > self.timeout:
                self.timeouts += 1
                self.next_check = now+self.period
                return self.stop(measured, 'collision timeout; measured hold')
            if not all(f.done() for f in p['futures']):
                return command.copy(), 0.
            self.latency_ms = (now-p['sent'])*1000.
            self.pending = None
            self.next_check = now+self.period
            try:
                results = [f.result() for f in p['futures']]
                invalid = [r for r in results if r is None or not r.valid]
            except Exception:
                self.rejected += 1
                return self.stop(measured, 'collision service error; measured hold')
            if invalid:
                self.contacts = sorted({
                    ' <-> '.join(sorted((c.contact_body_1, c.contact_body_2)))
                    for r in invalid if r is not None for c in r.contacts})[:6]
                self.rejected += 1
                return self.stop(measured, 'collision/state invalid; measured hold')
            if (p['context'] != context or not np.allclose(command, p['start'][:6], atol=1e-10, rtol=0.) or
                    np.max(np.abs(measured-p['measured'])) > self.tolerance):
                self.rejected += 1
                return self.stop(measured, 'feedback changed during collision check; measured hold')
            self.contacts = []
            self.plan = dict(**p, expires=now+self.horizon)
            self.accepted += 1

        if now >= self.next_check:
            self.next_check = now+self.period
            try:
                self._submit(now, measured, command, velocity, bounds, lead, goal)
            except Exception as exc:
                self.rejected += 1
                return self.stop(measured, f'collision request failed ({exc}); measured hold')
            return command.copy(), 0.
        if self.plan is None:
            return command.copy(), 0.
        p = self.plan
        if now > p['expires']:
            return self.stop(measured, 'collision approval expired; measured hold')
        # A position/velocity controller may track different joints at different
        # rates. Reject feedback that leaves the sampled paths instead of using
        # a collision result for an unrelated actual pose.
        deviation = min(segment_error(measured, a, b) for a, b in (
            (p['measured'], p['start']), (p['measured'], p['end']), (p['start'], p['end'])))
        if deviation > self.tolerance:
            return self.stop(measured, 'feedback left checked path; measured hold')
        # Follow the approved joint line exactly. Scale it down if the current
        # requested speeds decrease; changed directions require a new check.
        active = np.abs(p['velocity']) > 1e-10
        if (np.any(velocity[active]*p['velocity'][active] <= 0.) or
                np.any(np.abs(velocity[~active]) > 1e-5)):
            return self.stop(measured, 'motion direction changed; measured hold')
        scale = min(1., float(np.min(velocity[active]/p['velocity'][active])))
        position = np.r_[command, measured[6]]
        delta = p['velocity']*dt*scale
        for i in np.flatnonzero(np.abs(delta) > 1e-12):
            scale = min(scale, max(0., (p['end'][i]-position[i])/(p['velocity'][i]*dt)))
        delta = p['velocity']*dt*scale
        candidate = command+delta[:6]
        if np.max(np.abs(candidate-measured[:6])) > lead+1e-10:
            return self.stop(measured, 'collision-window tracking error; measured hold')
        self.reason = 'following collision-checked segment'
        return candidate, float(p['velocity'][6]*scale)


class CollisionManual(Manual):
    collision_guarded = True
    extra_parameters = dict(Manual.extra_parameters, collision_check_rate_hz=10.,
                            collision_lookahead_sec=.35, collision_timeout_sec=.15,
                            collision_step_rad=.005, ee_collision_step_rad=.02,
                            collision_feedback_tolerance_rad=.01)

    def __init__(self):
        self.ee_position, self.ee_state_time = None, -math.inf
        self.window = None
        self.last_collision_log = -math.inf
        super().__init__()
        self.window = CollisionWindow(self.create_client(GetStateValidity, '/manual/check_state_validity'),
            rate=self.params['collision_check_rate_hz'], horizon=self.params['collision_lookahead_sec'],
            timeout=self.params['collision_timeout_sec'], step=self.params['collision_step_rad'],
            ee_step=self.params['ee_collision_step_rad'],
            tolerance=self.params['collision_feedback_tolerance_rad'])

    def on_state(self, message):
        super().on_state(message)
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            self.ee_state_time = -math.inf
        elif 'ee_joint' in message.name:
            q = message.position[message.name.index('ee_joint')]
            if math.isfinite(q):
                self.ee_position, self.ee_state_time = float(q), self.now_seconds()
            else:
                self.ee_state_time = -math.inf

    def _cancel(self):
        if self.window is not None:
            self.window.cancel()

    def on_focus(self, message):
        if message.data != self.focus:
            self._cancel()
        super().on_focus(message)

    def on_hardware_fault(self, message):
        if message.data.strip():
            self._cancel()
        super().on_hardware_fault(message)

    def run_startup_recovery(self, now, dt):
        # Scene startup does not consume the existing recovery motion timeout.
        if (not self.window.client.service_is_ready() or self.ee_position is None or
                not 0 <= now-self.ee_state_time <= self.params['joint_state_timeout_sec']):
            self._cancel()
            self.publish_ee_velocity(0.)
            self.status = 'startup waiting for collision service and full joint feedback'
            self.hold()
            return
        super().run_startup_recovery(now, dt)

    def stop(self, reason, dt):
        self._cancel()
        self.stop_pose_active = False
        self.publish_ee_velocity(0.)
        self.neutral_required = True
        self.status = reason
        if self.target is not None and self.fresh_state(self.now_seconds()):
            self.target = self.measured()
        self.stop_target = None
        self.hold()

    def apply_motion(self, candidate, ee_velocity, source, dt, goal=None):
        now = self.now_seconds()
        if self.ee_position is None or not 0 <= now-self.ee_state_time <= self.params['joint_state_timeout_sec']:
            self.stop('ee_joint feedback unavailable', dt)
            return
        velocity = np.r_[(candidate-self.target)/dt, ee_velocity]
        axes = ('forward_axis', 'wrist_yaw_axis', 'wrist_roll_axis', 'ee_velocity_axis',
                'wrist_pitch_axis', 'vertical_axis')
        buttons = ('base_ccw_button', 'base_cw_button', 'ee_enable_button')
        context = (source, tuple(int(np.sign(self.axis(n))) for n in axes),
                   tuple(self.button(n) for n in buttons)) if source == 'operator' else (source,)
        before = self.target.copy()
        self.target, ee_output = self.window.step(now, dt, np.r_[self.measured(), self.ee_position],
            self.target, velocity, context,
            (self.lower+self.params['limit_margin_rad'], self.upper-self.params['limit_margin_rad']),
            self.params['max_target_lead_rad'], goal)
        self.rmd_applied_velocity = (self.target[1:4]-before[1:4])/dt
        self.status = self.window.reason
        self.publish_ee_velocity(ee_output)
        self.hold()
        if (source == 'stop_pose' and 'measured hold' in self.status and
                not self.status.startswith(('input changed', 'motion direction changed'))):
            self.stop_pose_active = False
            self.neutral_required = True
        if ('hold' in self.status and self.status != 'idle' and now-self.last_collision_log >= 2.):
            self.last_collision_log = now
            self.get_logger().warning(self.status + (': '+', '.join(self.window.contacts)
                                                     if self.window.contacts else ''))

    def emit_diagnostics(self, dt):
        if self.window is not None:
            w = self.window
            self.collision_status = dict(reason=w.reason, pending=w.pending is not None,
                check_rate_hz=1/w.period, lookahead_sec=w.horizon, latency_ms=w.latency_ms,
                batches=w.batches, accepted=w.accepted, rejected=w.rejected,
                timeouts=w.timeouts, contacts=w.contacts)
        super().emit_diagnostics(dt)


if __name__ == '__main__':
    raise SystemExit(main(CollisionManual))
