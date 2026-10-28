"""ROS-independent command ownership and bounded waypoint execution."""
import math
import uuid


MODES = ('manual', 'tcp_ik', 'tool_auto', 'hold', 'fault')
# DXL feedback hunts by several encoder counts while physically stopped.  Keep
# the tighter RMD bound, but accept only a small 0.02-rad DXL neighborhood around
# the collision-checked snapshot so entry cannot retry forever on quantization.
TCP_ENTRY_TOLERANCE = (.02, .001, .001, .001, .02, .02)
# MX-106(2.0): one Goal Velocity count is 0.229 rpm (0.0239691227 rad/s).
# Tool-specific commanded speeds remain separate from this manual safety ceiling.
MANUAL_EE_MAX_VELOCITY_RAD_S = 400 * 0.0239691227


def non_arm_feedback_changed(current, snapshot, arm_joints):
    """EE is also a DXL encoder; unknown joints retain the tight fallback."""
    return (current.keys() != snapshot.keys() or any(
        abs(value-snapshot[joint]) > (.02 if joint == 'ee_joint' else .001)
        for joint, value in current.items() if joint not in arm_joints))


class ModeCore:
    def __init__(self, limits, speeds, startup_recovery_enabled=False, path_accelerations=None,
                 tcp_accelerations=None, joint_names=None, path_speeds=None,
                 path_feedback_leads=None, path_jerks=None):
        self.limits, self.speeds = limits, speeds
        self.joint_names = (list(joint_names) if joint_names is not None else
                            [f'joint_{index}' for index in range(len(speeds))])
        if len(self.joint_names) != len(speeds) or len(set(self.joint_names)) != len(speeds):
            raise ValueError('invalid joint names')
        self.path_accelerations = (list(path_accelerations) if path_accelerations is not None
                                   else [0.25]*len(speeds))
        if (len(self.path_accelerations) != len(speeds)
                or not all(math.isfinite(a) and 0 < a <= 2. for a in self.path_accelerations)):
            raise ValueError('invalid path acceleration limits')
        self.path_jerks = (list(path_jerks) if path_jerks is not None
                           else [1.0]*len(speeds))
        if (len(self.path_jerks) != len(speeds)
                or not all(math.isfinite(j) and 0 < j <= 20. for j in self.path_jerks)):
            raise ValueError('invalid path jerk limits')
        self.path_speed_limits = (list(path_speeds) if path_speeds is not None
                                  else list(speeds))
        if (len(self.path_speed_limits) != len(speeds)
                or not all(math.isfinite(v) and 0 < v <= s
                           for v, s in zip(self.path_speed_limits, speeds))):
            raise ValueError('invalid path speed limits')
        self.tcp_accelerations = (list(tcp_accelerations) if tcp_accelerations is not None
                                  else [0.25]*len(speeds))
        if (len(self.tcp_accelerations) != len(speeds)
                or not all(math.isfinite(a) and 0 < a <= 2. for a in self.tcp_accelerations)):
            raise ValueError('invalid TCP acceleration limits')
        self.tcp_velocity = [0.]*len(speeds)
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
        self.ee_position = self.ee_velocity = None
        self.ee_state_time = -math.inf
        self.ee_goal = None
        self.ee_best_error = math.inf
        self.ee_progress_time = None
        self.command_time = -math.inf
        self.queue = []
        self.execution_id = None
        self.path_durations = []
        self.path_stops = []
        self.path_stop = False
        self.path_speeds = list(speeds)
        self.path_velocity = [0.]*len(speeds)
        self.path_acceleration = [0.]*len(speeds)
        self.path_reference_error = 0.
        self.path_command_error = 0.
        self.path_clamp_amount = 0.
        self.path_goal_error = 0.
        # A waypoint clock must never run the command far ahead of loaded
        # hardware.  Keep command lead small and let measured feedback pace it.
        self.path_feedback_leads = (list(path_feedback_leads)
                                    if path_feedback_leads is not None
                                    else [0.05]*len(speeds))
        if (len(self.path_feedback_leads) != len(speeds)
                or not all(math.isfinite(v) and 0.005 <= v <= 0.05
                           for v in self.path_feedback_leads)):
            raise ValueError('invalid path feedback leads')
        self.path_stall_timeout = 2.0
        self.path_stall_since = [None]*len(speeds)
        self.path_stall_position = [0.]*len(speeds)
        self.path_progress_time = None
        self.path_best_error = math.inf
        self.last_execution_id = None
        self.last_execution_result = None
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
        # Managed first arm entry uses the same DXL zero ramp as manual.py.
        self.entry_alignment_enabled = False
        self.entry_alignment_complete = True
        self.entry_alignment_started = None
        self.entry_alignment_deadline = None
        self.entry_alignment_speed = .09
        self.entry_alignment_timeout_margin = 15.
        self.tcp_entry_required = False
        self.tcp_entry = None
        self.entry_stable_since = None
        self.entry_anchor = None

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
        if any(abs(a-b) > tol for a, b, tol in
               zip(self.position, proof['positions'], TCP_ENTRY_TOLERANCE)):
            return 'TCP entry feedback changed; recheck required'
        if max(map(abs, self.velocity)) > .02:
            return 'TCP entry requires stopped feedback'
        if self.entry_stable_since is None or now-self.entry_stable_since < .2:
            return 'TCP entry waiting for 0.20 sec stable feedback'
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

    def run_entry_alignment(self, now, dt):
        """First arm focus: ramp only DXL base/roll/yaw to zero in HOLD."""
        indexes = (0, 4, 5)
        self.ee = 0.
        if not self.fresh(now) or not self.valid(self.position) or dt <= 0 or dt > .2:
            self.fault('DXL entry alignment requires fresh valid feedback and timer')
            return list(self.target)
        if self.entry_alignment_started is None:
            if any(not self.limits[i][0] <= 0 <= self.limits[i][1] for i in indexes):
                self.fault('DXL zero outside joint limits')
                return list(self.target)
            self.target = list(self.position)
            self.entry_alignment_started = now
            self.entry_alignment_deadline = (now + max(
                abs(self.position[i])/min(self.speeds[i], self.entry_alignment_speed)
                for i in indexes) + self.entry_alignment_timeout_margin)
        if now > self.entry_alignment_deadline:
            self.fault('DXL entry alignment timed out; inspect feedback')
            return list(self.target)
        if max(abs(a-b) for a, b in zip(self.target, self.position)) > .10:
            self.fault('DXL entry alignment following error')
            return list(self.target)
        candidate = list(self.target)
        for i in indexes:
            step = min(self.speeds[i], self.entry_alignment_speed)*min(dt, .02)
            candidate[i] += max(-step, min(step, -candidate[i]))
        if max(abs(candidate[i]-self.position[i]) for i in indexes) <= .10:
            self.target = candidate
        self.goal = list(self.target)
        self.reason = 'aligning DXL base/roll/yaw to zero'
        if (max(abs(self.target[i]) for i in indexes) <= 1e-9 and
                max(abs(self.position[i]) for i in indexes) <= .005 and
                max(map(abs, self.velocity)) <= .02 and
                self.entry_stable_since is not None and
                now-self.entry_stable_since >= .2):
            self.entry_alignment_complete = True
            self.entry_alignment_started = None
            self.entry_alignment_deadline = None
            self.reason = 'DXL entry alignment verified; HOLD'
        return list(self.target)

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
        for index, (value, speed, (lower, upper)) in enumerate(
                zip(position, velocity, self.limits)):
            if ((value > upper-.05 and speed > .02)
                    or (value < lower+.05 and speed < -.02)):
                self.fault(
                    'joint moving outward near soft limit: %s position=%.6f '
                    'velocity=%.6f allowed=[%.6f, %.6f]' %
                    (self.joint_names[index], value, speed, lower, upper))
                return
        if (not self.fresh(now) or self.entry_anchor is None
                or max(map(abs, velocity)) > .02
                or any(abs(a-b) > tol for a, b, tol in
                       zip(position, self.entry_anchor, TCP_ENTRY_TOLERANCE))):
            self.entry_anchor = list(position)
            self.entry_stable_since = now if max(map(abs, velocity)) <= .02 else None
        elif self.entry_stable_since is None:
            self.entry_stable_since = now
        self.position, self.velocity, self.state_time = list(position), list(velocity), now
        if self.target is None:
            self.target = list(position)

    def fresh(self, now):
        return self.position is not None and 0 <= now-self.state_time < .5

    def ee_feedback(self, position, velocity, now):
        if not all(math.isfinite(v) for v in (position, velocity)):
            self.ee_state_time = -math.inf
            return
        self.ee_position, self.ee_velocity, self.ee_state_time = position, velocity, now

    def close_ee(self, epoch, execution_id, target, now, speed=.12):
        """Execute a collision-checked gripper goal under tool_auto ownership."""
        if (self.mode != 'tool_auto' or epoch != self.epoch or self.transitioning or
                self.focus != 'arm' or not self.ready or not self.fresh(now)):
            return False
        if self.execution_id is not None:
            raise ValueError('execution already active; cancel first')
        if type(speed) not in (int, float) or not math.isfinite(speed) or not 0 < speed <= .12:
            raise ValueError('ee speed must be in (0, 0.12] rad/s')
        if (not isinstance(execution_id, str) or not execution_id or
                type(target) not in (int, float) or not math.isfinite(target) or
                not -2.351592547829 <= target <= 2.839398438376):
            raise ValueError('invalid ee goal')
        if not 0 <= now-self.ee_state_time < .3:
            raise ValueError('fresh ee feedback required')
        if max(map(abs, self.velocity)) > .02:
            raise ValueError('arm must be stopped before ee motion')
        self.target = self.goal = list(self.position)
        self.execution_id, self.ee_goal = execution_id, float(target)
        self.ee_speed = float(speed)
        self.started = self.ee_progress_time = now
        self.ee_best_error = abs(target-self.ee_position)
        self.settled = 0
        return True

    def run_ee(self, now):
        """Hold all arm joints; stop on stale feedback, drift, stall or timeout."""
        self.ee = 0.
        if not 0 <= now-self.ee_state_time < .3:
            self.abort_path_to_hold('ee feedback timeout', now)
        elif any(abs(a-b) > tol for a, b, tol in zip(
                self.position, self.goal, (.02, .005, .005, .005, .02, .02))):
            self.abort_path_to_hold('arm moved during ee motion', now)
        elif now-self.started > 60.:
            self.abort_path_to_hold('ee motion timeout', now)
        else:
            error = self.ee_goal-self.ee_position
            if abs(error) < self.ee_best_error-.001:
                self.ee_best_error, self.ee_progress_time = abs(error), now
            if abs(error) <= .01 and abs(self.ee_velocity) <= .02:
                self.settled += 1
                if self.settled >= 5:
                    completed = self.execution_id
                    self.request('hold', now)
                    self.last_execution_id = completed
                    self.last_execution_result = 'succeeded'
                    self.reason = 'ee goal reached'
            elif now-self.ee_progress_time > 3.:
                self.abort_path_to_hold('ee motion stalled', now)
            else:
                self.settled = 0
                if abs(error) > .01:
                    self.ee = max(-self.ee_speed, min(self.ee_speed, error))
        return list(self.target)

    def invalidate(self):
        if self.execution_id is not None:
            self.last_execution_id = self.execution_id
            self.last_execution_result = 'aborted'
        self.epoch = uuid.uuid4().hex
        self.goal, self.queue, self.execution_id, self.ee = None, [], None, 0.
        self.ee_goal = None
        self.command_time = -math.inf
        self.settled = 0
        self.path_durations = []
        self.path_stops = []
        self.path_stop = False
        self.path_speeds = list(self.path_speed_limits)
        self.path_velocity = [0.]*len(self.speeds)
        self.path_acceleration = [0.]*len(self.speeds)
        self.path_reference_error = 0.
        self.path_command_error = 0.
        self.path_clamp_amount = 0.
        self.path_goal_error = 0.
        self.path_progress_time = None
        self.path_best_error = math.inf

    def abort_path_to_hold(self, reason, now):
        """Abort only managed path ownership; healthy stopped hardware remains reusable."""
        self.invalidate()
        if self.position is None or not self.fresh(now):
            self.fault(reason+'; feedback unavailable')
            return
        self.target = list(self.position)
        self.goal = list(self.position)
        self.mode = self.requested = 'hold'
        self.transitioning = False
        self.reason = reason

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
        if mode != 'hold' and self.entry_alignment_enabled and not self.entry_alignment_complete:
            raise ValueError('DXL entry alignment must complete before mode requests')
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
        ee_speed_limit = MANUAL_EE_MAX_VELOCITY_RAD_S if source == 'manual' else .12
        if not self.valid(positions) or not math.isfinite(ee) or abs(ee) > ee_speed_limit:
            raise ValueError('invalid command or limits exceeded')
        if max(abs(a-b) for a, b in zip(positions, self.position)) > .10:
            raise ValueError('streaming target exceeds 0.10 rad feedback lead')
        self.goal, self.ee, self.command_time = list(positions), ee, now
        return True

    def pause_tcp(self, epoch, now):
        """Drop command lead at measured position while retaining TCP ownership."""
        if self.mode != 'tcp_ik' or epoch != self.epoch or self.transitioning:
            return False
        self.ee = 0.
        if not self.ready or not self.fresh(now) or not self.valid(self.position):
            self.fault('TCP pause without healthy controller/feedback')
            return True
        self.target = list(self.position)
        self.goal = list(self.position)
        self.tcp_velocity = [0.]*len(self.speeds)
        self.command_time = now
        self.reason = 'TCP paused; press L1 to resume from measured pose'
        return True

    def stop_tcp(self, epoch, now):
        """Epoch-bound Deadman stop; replace remaining lead with measured HOLD."""
        if self.mode != 'tcp_ik' or epoch != self.epoch or self.transitioning:
            return False
        if not self.ready or not self.fresh(now) or not self.valid(self.position):
            self.fault('TCP deadman stop without healthy controller/feedback')
            return True
        self.request('hold', now)
        # Like FAULT HOLD, stop the position controller following an old
        # forward target. Do not ramp back from the outstanding command lead.
        self.target = list(self.position)
        self.goal = list(self.position)
        self.tcp_velocity = [0.]*len(self.speeds)
        self.reason = 'TCP deadman stop; explicit mode request required'
        return True

    def path(self, epoch, execution_id, waypoints, now, durations=None,
             stop_at_waypoints=None):
        if (self.mode != 'tool_auto' or epoch != self.epoch or self.transitioning or
                self.focus != 'arm' or not self.ready or not self.fresh(now)):
            return False
        if self.execution_id is not None:
            raise ValueError('execution already active; cancel first')
        if not isinstance(execution_id, str) or not execution_id or not 1 <= len(waypoints) <= 1000:
            raise ValueError('execution_id and 1..1000 waypoints required')
        if not all(self.valid(p) for p in waypoints):
            raise ValueError('invalid waypoint')
        if durations is not None and (len(durations) != len(waypoints) or
                not all(type(d) in (int, float) and math.isfinite(d) and 0 < d < 120.
                        for d in durations)):
            raise ValueError('invalid waypoint durations')
        if stop_at_waypoints is not None and (len(stop_at_waypoints) != len(waypoints)
                or not all(type(stop) is bool for stop in stop_at_waypoints)):
            raise ValueError('invalid waypoint stop flags')
        self.queue = [list(p) for p in waypoints]
        self.path_durations = list(durations) if durations is not None else [None]*len(waypoints)
        self.path_stops = (list(stop_at_waypoints) if stop_at_waypoints is not None
                           else [False]*len(waypoints))
        # Carry velocity through a sample only when the next segment continues
        # in the same direction. Otherwise the old velocity can drive a tiny
        # reverse segment away from its goal for many seconds.
        starts = [self.target, *waypoints[:-1]]
        for i in range(len(waypoints)-1):
            if any(abs(b-a) > 1e-9 and (b-a)*(d-b) <= 0.
                   for a, b, d in zip(starts[i], waypoints[i], waypoints[i+1])):
                self.path_stops[i] = True
        self.execution_id = execution_id
        self.path_velocity = [0.]*len(self.speeds)
        self.path_acceleration = [0.]*len(self.speeds)
        self._next_waypoint()
        self.started = now
        self.path_progress_time = now
        self.path_best_error = math.inf
        return True

    def _next_waypoint(self):
        self.goal = self.queue.pop(0)
        duration = self.path_durations.pop(0)
        self.path_stop = self.path_stops.pop(0)
        if duration is None:
            self.path_speeds = list(self.path_speed_limits)
        else:
            # Preserve synchronized joint-line geometry, extending time if
            # necessary instead of speeding past any existing joint bound.
            duration = max(duration, max(abs(b-a)/s for a, b, s in
                                         zip(self.target, self.goal, self.path_speed_limits)))
            self.path_speeds = [abs(b-a)/duration for a, b in zip(self.target, self.goal)]
        self.path_progress_time = self.started
        self.path_best_error = math.inf
        self.path_stall_since = [None]*len(self.speeds)

    def stalled_path_joint(self, now):
        """Detect a loaded joint failing to follow, not a slow command ramp.

        A small final error already inside the completion tolerance is not a
        stall. Start each joint's watchdog only after a meaningful command lead
        builds up; progress of another joint cannot hide a blocked actuator.
        """
        for i, (measured, target, goal, lead) in enumerate(zip(
                self.position, self.target, self.goal, self.path_feedback_leads)):
            if abs(goal-measured) <= .01 or abs(target-measured) < lead*.5:
                self.path_stall_since[i] = None
                continue
            epsilon = min(.001, max(1e-6, self.path_speeds[i]*self.path_stall_timeout*.25))
            if (self.path_stall_since[i] is None or
                    abs(goal-measured) < abs(goal-self.path_stall_position[i])-epsilon):
                self.path_stall_since[i] = now
                self.path_stall_position[i] = measured
            elif now-self.path_stall_since[i] > self.path_stall_timeout:
                return i
        return None

    def cancel_path(self, epoch, execution_id, now):
        if (self.mode != 'tool_auto' or epoch != self.epoch
                or self.execution_id != execution_id or execution_id is None):
            return False
        self.request('hold', now)
        if self.fresh(now):
            self.target = self.goal = list(self.position)
        return True

    @staticmethod
    def _jerk_braking_distance(velocity, acceleration, acceleration_limit, jerk):
        """Conservative continuous distance needed to stop along one direction."""
        velocity = max(0., velocity)
        acceleration = max(-acceleration_limit, min(acceleration_limit, acceleration))
        ramp_time = max(0., (acceleration+acceleration_limit)/jerk)
        velocity_after_ramp = velocity+acceleration*ramp_time-.5*jerk*ramp_time**2
        if velocity_after_ramp <= 0.:
            # Stop during the jerk ramp. Bisection avoids a fragile cubic root.
            lower, upper = 0., ramp_time
            for _ in range(24):
                middle = .5*(lower+upper)
                vm = velocity+acceleration*middle-.5*jerk*middle**2
                if vm > 0.:
                    lower = middle
                else:
                    upper = middle
            ramp_time = upper
            velocity_after_ramp = 0.
        ramp_distance = (velocity*ramp_time+.5*acceleration*ramp_time**2
                         -jerk*ramp_time**3/6.)
        return max(0., ramp_distance)+velocity_after_ramp**2/(2.*acceleration_limit)

    def _tool_path_step(self, goal, dt):
        """Jerk/acceleration-bounded path command with explicit stop waypoints."""
        result = []
        reference = []
        must_stop = not self.queue or self.path_stop
        for index, (position, target, speed, velocity, acceleration, jerk,
                    current_acceleration) in enumerate(zip(
                self.target, goal, self.path_speeds, self.path_velocity,
                self.path_accelerations, self.path_jerks, self.path_acceleration)):
            distance = target-position
            if abs(distance) < 1e-12:
                next_velocity = 0. if must_stop else velocity
                result.append(target)
                self.path_velocity[len(result)-1] = next_velocity
                self.path_acceleration[len(result)-1] = 0. if must_stop else current_acceleration
                reference.append(target)
                continue
            desired_speed = speed
            if must_stop:
                # v^2 <= 2*a*d guarantees enough distance to stop at the goal.
                # Do not subtract an acceleration-period margin here: below
                # 0.5*a*dt^2 that becomes zero and strands the command just
                # short of the waypoint. The jerk-aware brake below supplies
                # the discrete stopping margin without creating a dead zone.
                desired_speed = min(
                    desired_speed, math.sqrt(2.*acceleration*abs(distance)))
                direction = math.copysign(1., distance)
                braking_distance = self._jerk_braking_distance(
                    velocity*direction, current_acceleration*direction,
                    acceleration, jerk)
                # Once directional speed is within one jerk quantum, forcing a
                # zero-speed target can create a one-cycle stop just short of
                # the goal. Allow the bounded final approach instead.
                directional_velocity = max(0., velocity*direction)
                if (directional_velocity > jerk*dt*dt and
                        abs(distance) <= braking_distance+directional_velocity*dt):
                    desired_speed = 0.
            desired_velocity = math.copysign(desired_speed, distance)
            velocity_error = desired_velocity-velocity
            # At cruise velocity the demanded acceleration is exactly zero.
            # Using copysign() directly for a zero error would select positive
            # acceleration and create a small accelerate/decelerate limit cycle.
            desired_acceleration = (0. if abs(velocity_error) < 1e-12 else
                                    math.copysign(acceleration, velocity_error))
            # Begin removing acceleration early enough to meet the requested
            # velocity without an acceleration discontinuity (dv=a^2/2j).
            if (current_acceleration*velocity_error > 0.
                    and abs(velocity_error) <= current_acceleration**2/(2.*jerk)):
                desired_acceleration = 0.
            acceleration_change = jerk*dt
            next_acceleration = current_acceleration+max(
                -acceleration_change,
                min(acceleration_change, desired_acceleration-current_acceleration))
            next_velocity = velocity+next_acceleration*dt
            step = .5*(velocity+next_velocity)*dt
            if must_stop and step*distance <= 0:
                # A conservative jerk brake may stop just before the target;
                # approach again from zero instead of reversing or snapping.
                result.append(position)
                self.path_velocity[len(result)-1] = 0.
                self.path_acceleration[len(result)-1] = 0.
            elif abs(step) >= abs(distance):
                result.append(target)
                # Preserve velocity through an intermediate sample. At the final
                # goal the measured-settle gate still requires actual standstill.
                self.path_velocity[len(result)-1] = 0. if must_stop else next_velocity
                self.path_acceleration[len(result)-1] = 0. if must_stop else next_acceleration
            else:
                result.append(position+step)
                self.path_velocity[len(result)-1] = next_velocity
                self.path_acceleration[len(result)-1] = next_acceleration
            reference.append(result[-1])
            # Feedback-paced command envelope. This preserves the taught path
            # and acceleration profile while preventing a slow/stalled motor
            # from accumulating a large following error.
            measured = self.position[index]
            lead = self.path_feedback_leads[index]
            result[-1] = max(measured-lead, min(measured+lead, result[-1]))
        self.path_reference_error = max(
            abs(value-measured) for value, measured in zip(reference, self.position))
        self.path_command_error = max(
            abs(value-measured) for value, measured in zip(result, self.position))
        self.path_clamp_amount = max(abs(a-b) for a, b in zip(reference, result))
        self.path_goal_error = max(abs(a-b) for a, b in zip(goal, self.position))
        return result

    def _tcp_step(self, goal, dt):
        """Acceleration-bound streamed IK goals without dropping input packets."""
        result = []
        for index, (position, target, speed, velocity, acceleration) in enumerate(zip(
                self.target, goal, self.speeds, self.tcp_velocity,
                self.tcp_accelerations)):
            distance = target-position
            if abs(distance) < 1e-12:
                self.tcp_velocity[index] = 0.
                result.append(target)
                continue
            braking_speed = max(0., math.sqrt(2.*acceleration*abs(distance))-acceleration*dt)
            desired_velocity = math.copysign(min(speed, braking_speed), distance)
            change = acceleration*dt
            next_velocity = velocity+max(-change, min(change, desired_velocity-velocity))
            step = .5*(velocity+next_velocity)*dt
            if step*distance <= 0 or abs(step) >= abs(distance):
                result.append(target)
                self.tcp_velocity[index] = 0.
            else:
                result.append(position+step)
                self.tcp_velocity[index] = next_velocity
        return result

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
        if (self.entry_alignment_enabled and not self.entry_alignment_complete and
                self.focus == 'arm' and self.ready and self.mode == 'hold' and
                not self.transitioning):
            return self.run_entry_alignment(now, dt)
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
        following = [abs(a-b) for a, b in zip(self.target, self.position)]
        if max(following) > .15:
            index = max(range(len(following)), key=following.__getitem__)
            reason = ('following error: %s target=%.6f measured=%.6f error=%.6f rad' %
                      (self.joint_names[index], self.target[index], self.position[index],
                       following[index]))
            if self.mode == 'tool_auto' and self.execution_id is not None:
                self.abort_path_to_hold(reason, now)
            else:
                self.fault(reason)
        if self.mode == 'fault':
            # Capture one hold target, rather than following drifting feedback.
            if self.goal is None:
                self.goal = list(self.position)
        if self.mode == 'tool_auto' and self.ee_goal is not None:
            return self.run_ee(now)
        goal = self.goal if self.goal is not None else self.target
        step = min(max(dt, 0.), .04)
        if self.mode == 'tool_auto' and self.execution_id:
            self.target = self._tool_path_step(goal, step)
        elif self.mode == 'tcp_ik':
            self.target = self._tcp_step(goal, step)
        else:
            self.tcp_velocity = [0.]*len(self.speeds)
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
                if self.mode != 'hold' or not self.reason.startswith(('TCP entry', 'TCP deadman')):
                    self.reason = ''
                self.settled = 0
        elif self.mode == 'tool_auto' and self.execution_id is not None:
            stalled_joint = self.stalled_path_joint(now)
            error = max(abs(a-b) for a, b in zip(self.position, self.goal))
            if error < self.path_best_error-.001:
                self.path_best_error = error
                self.path_progress_time = now
            if now-self.started > 120.:
                self.abort_path_to_hold('waypoint timeout; path aborted to HOLD', now)
            elif stalled_joint is not None:
                index = stalled_joint
                self.abort_path_to_hold(
                    'waypoint stalled: %s goal=%.6f measured=%.6f error=%.6f rad; '
                    'command lead persisted without feedback progress; inspect hardware' %
                    (self.joint_names[index], self.goal[index], self.position[index],
                     abs(self.goal[index]-self.position[index])), now)
            # Intermediate trajectory samples describe one continuous motion,
            # not individual stop goals. Advance as soon as the command ramp
            # reaches each sample; require measured standstill only at the
            # final waypoint. Waiting for five stopped feedback cycles at every
            # MoveIt sample caused repeated stop/start chatter.
            elif (self.queue and ((self.path_stop and self.settled >= 5) or
                  (not self.path_stop and
                   max(abs(a-b) for a, b in zip(self.target, self.goal)) < 1e-6))):
                self._next_waypoint()
                self.started, self.settled = now, 0
                self.path_progress_time = now
            elif not self.queue and self.settled >= 5:
                completed = self.execution_id
                self.request('hold', now)
                self.last_execution_id = completed
                self.last_execution_result = 'succeeded'
                self.reason = 'path completed'
        return list(self.target)
