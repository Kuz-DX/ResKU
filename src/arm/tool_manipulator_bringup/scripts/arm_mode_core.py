"""ROS-independent command ownership and bounded waypoint execution."""
import math
import uuid


MODES = ('manual', 'tcp_ik', 'tool_auto', 'hold', 'fault')


class ModeCore:
    def __init__(self, limits, speeds, startup_recovery_enabled=False):
        self.limits, self.speeds = limits, speeds
        self.mode = 'hold'
        self.requested = 'hold'
        self.transitioning = False
        self.epoch = uuid.uuid4().hex
        self.reason = ''
        self.focus = 'drive'
        self.ready = False
        self.position = None
        self.velocity = None
        self.state_time = -math.inf
        self.target = None
        self.goal = None
        self.ee = 0.
        self.command_time = -math.inf
        self.queue = []
        self.execution_id = None
        self.started = 0.
        self.settled = 0
        self.startup_enabled = startup_recovery_enabled
        self.startup_complete = not startup_recovery_enabled
        self.startup_recovering = False
        self.startup_goal = None
        self.startup_started = None
        self.startup_progress_time = None
        self.startup_best_error = math.inf
        self.startup_margin = math.radians(2.)
        self.startup_allowance = math.radians(5.)
        self.startup_speed = .03
        # Loaded RMD joints can start moving several seconds after the bounded
        # position ramp begins.  Permit slow, measurable recovery progress, but
        # never wait indefinitely on a stalled joint.
        self.startup_timeout = 60.
        self.startup_stall_timeout = 15.
        self.startup_progress_epsilon = .0005
        self.tcp_entry_required = False
        self.tcp_entry = None

    def tcp_entry_blocker(self, now):
        if not self.tcp_entry_required:
            return ''
        proof = self.tcp_entry
        if proof is None or proof['epoch'] != self.epoch or not 0 <= now-proof['time'] < .5:
            return 'TCP entry assessment missing/stale; remain HOLD'
        if not proof['safe']:
            return proof['reason']
        if not self.fresh(now) or not self.valid(proof['positions']):
            return 'TCP entry feedback invalid'
        if any(min(v-lo, hi-v) < .1 for v, (lo, hi) in zip(self.position, self.limits)):
            return 'TCP entry requires 0.10 rad joint margin; request tcp_ready assessment'
        if max(abs(a-b) for a, b in zip(self.position, proof['positions'])) > .001:
            return 'TCP entry feedback changed; recheck required'
        if max(map(abs, self.velocity)) > .02:
            return 'TCP entry requires stopped feedback'
        return ''

    def feedback_allowed(self, position):
        if self.startup_enabled and not self.startup_complete:
            return (len(position) == len(self.limits) and all(
                math.isfinite(v) and lo-(self.startup_allowance if 1 <= i <= 3 else 0.) <= v
                <= hi+(self.startup_allowance if 1 <= i <= 3 else 0.)
                for i, (v, (lo, hi)) in enumerate(zip(position, self.limits))))
        return self.valid(position)

    def begin_startup(self, now):
        """Capture measured seed once, immediately before controller activation."""
        if self.mode == 'fault' or not self.fresh(now) or not self.feedback_allowed(self.position):
            raise ValueError('startup requires valid fresh feedback')
        self.target = list(self.position)
        if not self.startup_enabled:
            return
        self.invalidate()
        self.mode = self.requested = 'hold'
        self.startup_goal = list(self.target)
        for i in (1, 2, 3):
            lo, hi = self.limits[i]
            self.startup_goal[i] = min(hi-self.startup_margin, max(lo+self.startup_margin, self.target[i]))
        self.startup_recovering = any(abs(a-b) > 1e-9 for a,b in zip(self.target, self.startup_goal))
        self.startup_complete = not self.startup_recovering
        self.transitioning = self.startup_recovering
        self.goal = list(self.startup_goal)
        self.startup_started = None
        self.startup_progress_time = None
        self.startup_best_error = max(
            abs(self.position[i]-self.startup_goal[i]) for i in (1, 2, 3))
        self.reason = 'startup RMD recovery: inward to 2-degree inset' if self.startup_recovering else ''

    def run_startup(self, now, dt):
        self.ee = 0.
        if not self.fresh(now) or dt <= 0 or dt > .5:
            if (self.startup_started is not None and
                    now-self.startup_progress_time > self.startup_stall_timeout):
                self.fault('startup recovery stalled timeout (15 seconds); inspect hardware before restart')
                return list(self.target)
            self.reason = 'startup recovery paused: stale feedback or timer discontinuity'
            return list(self.target)
        if not self.feedback_allowed(self.position):
            self.fault('startup feedback outside 5-degree recovery envelope')
            return list(self.target)
        if max(abs(a-b) for a,b in zip(self.target, self.position)) > .10:
            self.fault('startup recovery following error')
            return list(self.target)
        if self.startup_started is None:
            self.startup_started = now
            self.startup_progress_time = now
        measured_error = max(
            abs(self.position[i]-self.startup_goal[i]) for i in (1, 2, 3))
        if measured_error < self.startup_best_error-self.startup_progress_epsilon:
            self.startup_best_error = measured_error
            self.startup_progress_time = now
        if now-self.startup_started > self.startup_timeout:
            self.fault('startup recovery total timeout; inspect hardware before restart')
            return list(self.target)
        if now-self.startup_progress_time > self.startup_stall_timeout:
            self.fault('startup recovery stalled timeout (15 seconds); inspect hardware before restart')
            return list(self.target)
        candidate = list(self.target)
        for i in (1, 2, 3):
            step = min(self.speeds[i], self.startup_speed)*min(dt, .02)
            candidate[i] += max(-step, min(step, self.startup_goal[i]-candidate[i]))
        if max(abs(candidate[i]-self.position[i]) for i in (1, 2, 3)) <= .10:
            self.target = candidate
        self.reason = 'startup RMD recovery: inward to 2-degree inset'
        if (max(abs(self.target[i]-self.startup_goal[i]) for i in (1, 2, 3)) < 1e-9
                and self.valid(self.position)
                and max(abs(self.position[i]-self.startup_goal[i]) for i in (1, 2, 3)) <= .005
                and max(map(abs, self.velocity)) <= .02):
            self.startup_recovering = False
            self.startup_complete = True
            self.transitioning = False
            self.reason = 'startup recovery verified; HOLD'
        return list(self.target)

    def valid(self, values):
        return (len(values) == len(self.limits) and all(
            math.isfinite(v) and lo <= v <= hi
            for v, (lo, hi) in zip(values, self.limits)))

    def feedback(self, position, velocity, now):
        if not self.feedback_allowed(position):
            names = ('base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
                     'wrist_roll_joint', 'wrist_yaw_joint')
            detail = []
            for i, (v, (lo, hi)) in enumerate(zip(position, self.limits)):
                allowance = (self.startup_allowance if self.startup_enabled and
                             not self.startup_complete and 1 <= i <= 3 else 0.)
                if not math.isfinite(v) or not lo-allowance <= v <= hi+allowance:
                    detail.append(f'{names[i]}={v:.6f}, allowed=[{lo-allowance:.6f}, {hi+allowance:.6f}]')
            self.fault('invalid joint position: '+('; '.join(detail) or 'joint count mismatch'))
            return
        if len(velocity) != len(position) or not all(map(math.isfinite, velocity)):
            self.fault('invalid joint velocity: missing or nonfinite values')
            return
        self.position, self.velocity, self.state_time = list(position), list(velocity), now
        if self.target is None:
            self.target = list(position)

    def fresh(self, now):
        return self.position is not None and 0 <= now-self.state_time < .5

    def invalidate(self):
        self.epoch = uuid.uuid4().hex
        self.goal, self.queue, self.execution_id, self.ee = None, [], None, 0.
        self.command_time = -math.inf
        self.settled = 0

    def fault(self, reason):
        self.invalidate()
        # ForwardCommandController retains its last sample.  Replace an
        # in-flight recovery/trajectory sample with the measured position so a
        # fault means HOLD, rather than continuing toward the previous goal.
        if self.position is not None:
            self.target = list(self.position)
            self.goal = list(self.position)
        self.startup_recovering = False
        self.mode = self.requested = 'fault'
        self.transitioning = False
        self.reason = reason

    def request(self, mode, now):
        if mode not in MODES or mode == 'fault':
            raise ValueError('invalid requested mode')
        if self.mode == 'fault':
            raise ValueError('fault is latched; reset required')
        if self.startup_enabled and not self.startup_complete:
            if mode == 'hold' and self.startup_recovering:
                self.fault('startup recovery cancelled; restart required')
                return
            raise ValueError('startup recovery must complete before mode requests')
        if mode != 'hold' and (self.focus != 'arm' or not self.ready or not self.fresh(now)):
            raise ValueError('arm focus, active controller and fresh feedback required')
        if mode == 'tcp_ik':
            blocker = self.tcp_entry_blocker(now)
            if blocker:
                self.request('hold', now)
                self.reason = blocker
                raise ValueError(blocker)
        self.invalidate()
        if mode == 'tcp_ik' and self.tcp_entry is not None:
            self.tcp_entry = dict(self.tcp_entry, epoch=self.epoch)
        self.mode, self.requested = 'hold', mode
        self.transitioning = True
        self.started = now
        self.reason = 'stopping before mode change'
        self.goal = list(self.position) if self.fresh(now) else None

    def reset(self, now):
        if self.startup_enabled and not self.startup_complete:
            raise ValueError('incomplete startup recovery requires inspection and restart')
        if not self.ready or not self.fresh(now) or max(map(abs, self.velocity)) > .02:
            raise ValueError('healthy controllers and stopped fresh feedback required')
        self.mode = 'hold'
        self.request('hold', now)

    def command(self, source, epoch, positions, ee, now):
        if (self.mode != source or source not in ('manual', 'tcp_ik') or
                epoch != self.epoch or self.transitioning or not self.ready or
                self.focus != 'arm' or not self.fresh(now)):
            return False
        if not self.valid(positions) or not math.isfinite(ee) or abs(ee) > .12:
            raise ValueError('invalid command or limits exceeded')
        if max(abs(a-b) for a, b in zip(positions, self.position)) > .10:
            raise ValueError('streaming target exceeds 0.10 rad feedback lead')
        self.goal, self.ee, self.command_time = list(positions), ee, now
        return True

    def path(self, epoch, execution_id, waypoints, now):
        if (self.mode != 'tool_auto' or epoch != self.epoch or self.transitioning or
                self.focus != 'arm' or not self.ready or not self.fresh(now)):
            return False
        if self.execution_id is not None:
            raise ValueError('execution already active; cancel first')
        if not isinstance(execution_id, str) or not execution_id or not 1 <= len(waypoints) <= 1000:
            raise ValueError('execution_id and 1..1000 waypoints required')
        if not all(self.valid(p) for p in waypoints):
            raise ValueError('invalid waypoint')
        self.queue = [list(p) for p in waypoints]
        self.execution_id = execution_id
        self.goal = self.queue.pop(0)
        self.started = now
        return True

    def tick(self, now, dt):
        if self.target is None:
            return None
        if self.startup_enabled and not self.startup_complete:
            if self.mode == 'fault':
                return list(self.target) if self.ready and self.target is not None else None
            if not self.ready:
                return None
            if self.startup_recovering:
                return self.run_startup(now, dt)
            return None
        if not self.fresh(now):
            self.fault('joint feedback timeout')
            return None
        if not self.ready:
            return None
        if dt <= 0 or dt > .2:
            self.fault('control timer discontinuity')
        if self.mode in ('manual', 'tcp_ik') and now-self.command_time > .3:
            self.request('hold', now)
            self.reason = 'command timeout'
        if max(abs(a-b) for a,b in zip(self.target, self.position)) > .15:
            self.fault('following error')
        if self.mode == 'fault':
            # Capture one hold target, rather than following drifting feedback.
            if self.goal is None:
                self.goal = list(self.position)
        goal = self.goal if self.goal is not None else self.target
        step = min(max(dt, 0.), .04)
        self.target = [a+max(-s*step, min(s*step,b-a))
                       for a,b,s in zip(self.target, goal, self.speeds)]
        stopped = (max(map(abs, self.velocity)) <= .02 and
                   max(abs(a-b) for a,b in zip(self.position, goal)) <= .01 and
                   max(abs(a-b) for a,b in zip(self.target, goal)) < 1e-6)
        self.settled = self.settled+1 if stopped else 0
        if self.transitioning:
            if now-self.started > 15.:
                self.fault('transition stop timeout')
            elif self.settled >= 5:
                if self.requested == 'tcp_ik':
                    blocker = self.tcp_entry_blocker(now)
                    if blocker:
                        self.request('hold', now)
                        self.reason = blocker
                        return list(self.target)
                self.mode = self.requested
                self.transitioning = False
                self.command_time = now
                if self.mode != 'hold' or not self.reason.startswith('TCP entry'):
                    self.reason = ''
                self.settled = 0
        elif self.mode == 'tool_auto' and self.execution_id is not None:
            if now-self.started > 120.:
                self.fault('waypoint timeout')
            elif self.settled >= 5:
                if self.queue:
                    self.goal = self.queue.pop(0)
                    self.started, self.settled = now, 0
                else:
                    self.request('hold', now)
                    self.reason = 'path completed'
        return list(self.target)
