"""Shared preparation/return workflow; motor ownership remains in the mode manager."""
import json
import math
import uuid

import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity, GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from arm_mode_core import TCP_ENTRY_TOLERANCE, non_arm_feedback_changed
try:
    from tool_change_min.kinematics import ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ARM_JOINTS


class TcpMotionWorkflow:
    def request_named_pose(self, home, now, goal=None, label=None):
        """Explicit recovery request: stop first, then validate each leg."""
        s = self.status
        if s.get('mode') == 'fault':
            return False, 'manager fault: '+s.get('reason', 'inspect control_status')
        if not s.get('entry_alignment_complete', True):
            return False, 'DXL entry alignment incomplete: '+s.get('blocker', '')
        if (not self.ready_enabled or now-self.status_time >= .3
                or not s.get('ready') or not s.get('startup_complete')
                or s.get('focus') != 'arm' or s.get('mode') == 'fault'):
            return False, 'named pose requires ready arm focus, completed startup and no fault'
        startup_goal = np.asarray(s.get('startup_goal_rad'), dtype=float)
        if home and (startup_goal.shape != (6,) or not np.all(np.isfinite(startup_goal))):
            return False, 'startup home pose is unavailable'
        if not self.cancel.service_is_ready():
            return False, 'HOLD service unavailable'
        self._reset_ready()
        self.returning_home = home
        self.home_goal = startup_goal.copy() if home else None
        if home:
            # Home retains recovered RMD joints; the three Dynamixels align
            # to joint zero regardless of their pose at process startup.
            self.home_goal[[0, 4, 5]] = 0.
        self.home_leg = 'prepare' if home else 'ready'
        self.home_settle_since = None
        self.explicit_ready = True
        self.named_one_shot = True
        self.ready_goal = (self.default_ready_goal.copy() if goal is None
                           else np.asarray(goal, dtype=float).copy())
        if (self.ready_goal.shape != (6,) or not np.all(np.isfinite(self.ready_goal))
                or (goal is not None and not self.model.within_limits(self.ready_goal, .101))):
            self.returning_home = self.explicit_ready = self.named_one_shot = False
            return False, 'named pose goal violates TCP entry margin'
        self.auto_entry_armed = True
        self.discard()
        self.blocked = True
        self.ready_future = self.cancel.call_async(Trigger.Request())
        self.ready_started = now
        self.ready_state = 'stopping'
        self.reason = 'stopping before '+(label or 'TCP ready')+(' then startup home' if home else '')
        return True, self.reason

    def retry_ready(self, valid, was_pressed, home, now):
        """A fresh neutral L1 press retries validation, never bypasses it."""
        s = self.status
        if (valid and self.deadman and not was_pressed and not home
                and vars(self).get('ready_enabled', False)
                and self.ready_state in ('failed', 'idle', 'await_entry')
                and not vars(self).get('returning_home', False)
                and s.get('mode') == 'hold' and not s.get('transitioning')
                and s.get('focus') == 'arm' and s.get('ready')
                and s.get('startup_complete') and now-self.status_time < .3
                and s.get('entry_alignment_complete', True)
                and now-self.state_time < .3 and not np.any(self.twist)):
            self._reset_ready()
            self.ready_goal = self.default_ready_goal.copy()
            self.explicit_ready = True
            self.named_one_shot = False
            self.entry_retry_count = 0
            self.auto_entry_armed = True
            self.reason = 'L1 requested fresh TCP entry/ready validation'

    def _reset_ready(self):
        pending = vars(self).get('ready_future')
        if vars(self).get('ready_state') == 'planning_home' and pending is not None and not pending.done():
            pending.cancel()
        for future in self.ready_checks:
            if not future.done():
                future.cancel()
        self.ready_state, self.ready_checks = 'idle', []
        self.ready_snapshot = self.ready_started = self.ready_future = None
        self.ready_execution_id = self.ready_safe_at = None
        self.entry_deadline = None
        self.entry_retry_after = 0.

    def _fail_ready(self, reason):
        self._reset_ready()
        self.ready_state = 'failed'
        self.auto_entry_armed = False
        self.reason = reason
        self.explicit_ready = False
        if vars(self).get('returning_home', False):
            self.returning_home = False
            self.ready_goal = self.default_ready_goal.copy()

    def drive_ready(self, now):
        """Collision-check and execute one explicit-focus TCP-ready entry."""
        s = self.status
        if not self.ready_enabled or not self.auto_entry_armed:
            return
        if s.get('mode') == 'fault':
            self._fail_ready('named pose cancelled by manager fault')
            return
        if self.ready_state == 'stopping':
            if now-self.ready_started > 15.:
                self._fail_ready('HOLD timed out before named pose')
            elif self.ready_future.done():
                response = self.ready_future.result()
                if response is None or not response.success:
                    self._fail_ready('HOLD rejected before named pose')
                elif s.get('mode') == 'hold' and not s.get('transitioning'):
                    if self.returning_home:
                        if self.measured is None or now-self.state_time >= .3:
                            return
                        # Plan the complete route; fixed ready-posture base
                        # rotation has collided with the circuit box.
                        self.ready_goal = self.home_goal.copy()
                        self.home_leg = 'planned'
                    self.ready_state = 'idle'
            return
        if vars(self).get('returning_home', False):
            velocities = np.asarray(s.get('velocities_rad_s', []), dtype=float)
            fresh = now-self.status_time < .3 and now-self.state_time < .3
            if self.home_leg == 'planned' and self.ready_state == 'executing':
                distance = math.inf
                if fresh and self.measured is not None and np.isfinite(self.measured).all():
                    previous = self.ready_snapshot
                    for waypoint in self.home_waypoints:
                        target = np.asarray(waypoint)
                        delta = target-previous
                        fraction = (0. if np.dot(delta, delta) < 1e-16 else
                                    np.clip(np.dot(self.measured-previous, delta)/np.dot(delta, delta), 0., 1.))
                        distance = min(distance, float(np.max(np.abs(
                            self.measured-(previous+fraction*delta)))))
                        previous = target
                if distance > .01:
                    if self.cancel.service_is_ready():
                        self.cancel.call_async(Trigger.Request())
                    self._fail_ready('HOME stopped: measured posture left checked path or feedback stale')
                    return
            if self.home_leg == 'rotate':
                drift = (self.measured is None or not fresh or
                         not np.isfinite(self.measured).all() or
                         np.max(np.abs(self.measured[1:]-self.ready_goal[1:])) > .01)
                if drift:
                    if self.cancel.service_is_ready():
                        self.cancel.call_async(Trigger.Request())
                    self._fail_ready('HOME rotation stopped: arm posture drift or stale feedback')
                    return
            if self.ready_state == 'home_settle':
                settled = (fresh and self.measured is not None and
                           np.isfinite(self.measured).all() and
                           np.max(np.abs(self.measured-self.ready_goal)) <= .01 and
                           velocities.shape == (6,) and np.isfinite(velocities).all() and
                           np.max(np.abs(velocities)) <= .01)
                if now-self.ready_started > 5.:
                    self._fail_ready('HOME posture did not settle; base rotation blocked')
                    return
                if not settled:
                    self.home_settle_since = None
                    return
                if self.home_settle_since is None:
                    self.home_settle_since = now
                if now-self.home_settle_since < .5:
                    return
                self._reset_ready()
                if self.home_leg == 'prepare':
                    self.home_leg = 'rotate'
                    self.ready_goal[0] = self.home_goal[0]
                elif self.home_leg == 'rotate':
                    self.home_leg = 'home'
                    self.ready_goal = self.home_goal.copy()
                else:
                    self.returning_home = self.explicit_ready = self.auto_entry_armed = False
                    self.ready_goal = self.default_ready_goal.copy()
                self.reason = 'HOME stage: '+self.home_leg
                return
        deadline = vars(self).get('entry_deadline')
        if (self.ready_state in ('await_entry', 'request_tcp')
                and deadline is not None and now >= deadline):
            self._fail_ready(
                'TCP entry did not settle within 3.0 sec; remain HOLD; '
                'release and press L1 to retry')
            return
        if self.ready_state in ('request_tcp', 'wait_tcp'):
            if now-self.ready_started > 3.:
                self._fail_ready('TCP transition timed out; release and press L1 to retry')
                return
        if (now-self.status_time >= .3 or now-self.state_time >= .3 or self.measured is None
                or not s.get('ready') or not s.get('startup_complete')
                or not s.get('entry_alignment_complete', True) or s.get('focus') != 'arm'):
            return
        if self.ready_state == 'idle':
            if s.get('mode') != 'hold' or s.get('transitioning'):
                return
            if not self.check.service_is_ready():
                self.reason = 'waiting for MoveIt collision service before TCP ready'
                return
            if vars(self).get('returning_home', False):
                if not self.home_planner.service_is_ready():
                    self._fail_ready('HOME planner unavailable; remain HOLD')
                    return
                request = MotionPlanRequest()
                request.group_name = 'arm'
                request.num_planning_attempts = 5
                request.allowed_planning_time = 10.
                request.max_velocity_scaling_factor = .1
                request.max_acceleration_scaling_factor = .1
                request.start_state.joint_state.name = list(self.full_state)
                request.start_state.joint_state.position = list(self.full_state.values())
                request.start_state.is_diff = True
                goal = Constraints()
                for name, value in zip(ARM_JOINTS, self.ready_goal):
                    goal.joint_constraints.append(JointConstraint(joint_name=name,
                        position=float(value), tolerance_above=.001,
                        tolerance_below=.001, weight=1.))
                request.goal_constraints = [goal]
                self.ready_snapshot = self.measured.copy()
                self.ready_full_state = dict(self.full_state)
                self.ready_check_epoch = s.get('epoch')
                self.ready_started = now
                self.ready_future = self.home_planner.call_async(
                    GetMotionPlan.Request(motion_plan_request=request))
                self.ready_state = 'planning_home'
                self.reason = 'planning HOME around circuit box'
                return
            differences = np.abs(self.ready_goal-self.measured)
            limiting_joint = int(np.argmax(differences))
            delta = float(differences[limiting_joint])
            limit = (self.home_max_delta if vars(self).get('returning_home', False)
                     else self.ready_max_delta)
            if delta > limit and not vars(self).get('explicit_ready', False):
                self._fail_ready(
                    '%s move blocked: %s %.3f rad exceeds %.3f rad limit '
                    '(measured %.3f, goal %.3f rad)' %
                    ('manual Stop_pose' if vars(self).get('returning_home', False)
                     else 'TCP ready automatic', ARM_JOINTS[limiting_joint], delta, limit,
                     self.measured[limiting_joint], self.ready_goal[limiting_joint]))
                return
            count = max(1, math.ceil(float(np.max(np.abs(self.ready_goal-self.measured))) /
                                     self.ready_step))
            self.ready_snapshot = self.measured.copy()
            self.ready_check_epoch = s.get('epoch')
            self.ready_full_state = dict(self.full_state)
            self.ready_started = now
            self.ready_checks = []
            self.ready_requests = []
            for q in np.linspace(self.measured, self.ready_goal, count+1):
                req = GetStateValidity.Request()
                values = dict(self.full_state)
                values.update(zip(ARM_JOINTS, map(float, q)))
                req.robot_state.joint_state.name = list(values)
                req.robot_state.joint_state.position = list(values.values())
                req.robot_state.is_diff = True
                self.ready_requests.append(req)
            self.ready_checks.append(self.check.call_async(self.ready_requests[0]))
            self.ready_state = 'checking'
            self.reason = 'checking path to TCP ready'
            return
        if self.ready_state == 'planning_home':
            if now-self.ready_started > 12.:
                self.ready_future.cancel()
                self._fail_ready('HOME planning timed out; remain HOLD')
                return
            if not self.ready_future.done():
                return
            try:
                from tool_trajectory_joy import planned_entry
                response = self.ready_future.result().motion_plan_response
                if response.error_code.val != 1:
                    raise ValueError('MoveIt error %s' % response.error_code.val)
                path = planned_entry(dict(start=self.ready_goal, waypoints=[], durations=[]),
                                     response.trajectory.joint_trajectory, self.ready_snapshot)
                if len(path['waypoints']) > 1000:
                    raise ValueError('too many HOME waypoints')
                self.home_waypoints = [q.tolist() for q in path['waypoints']]
                self.home_durations = path['durations']
                self.ready_requests = []
                previous = self.ready_snapshot
                for target in path['waypoints']:
                    if not self.model.within_limits(target):
                        raise ValueError('HOME waypoint outside limits')
                    count = max(1, math.ceil(float(np.max(np.abs(target-previous)))/self.ready_step))
                    if len(self.ready_requests)+count+1 > 20000:
                        raise ValueError('HOME verification too large')
                    for q in np.linspace(previous, target, count+1):
                        req = GetStateValidity.Request()
                        values = dict(self.ready_full_state)
                        values.update(zip(ARM_JOINTS, map(float, q)))
                        req.robot_state.joint_state.name = list(values)
                        req.robot_state.joint_state.position = list(values.values())
                        req.robot_state.is_diff = True
                        self.ready_requests.append(req)
                    previous = target
                self.ready_checks = [self.check.call_async(self.ready_requests[0])]
                self.ready_state = 'checking'
                self.ready_started = now
                self.reason = 'verifying planned HOME segments'
            except (ValueError, RuntimeError, AttributeError, IndexError) as exc:
                self._fail_ready('HOME plan rejected: '+str(exc))
            return
        if self.ready_state == 'checking':
            if (s.get('mode') != 'hold' or s.get('transitioning')
                    or s.get('epoch') != vars(self).get('ready_check_epoch', s.get('epoch'))):
                self._fail_ready('arm ownership changed during named pose collision check')
                return
            if non_arm_feedback_changed(self.full_state, self.ready_full_state, ARM_JOINTS):
                self._fail_ready('non-arm feedback changed during TCP ready check; remain HOLD')
                return
            if (not all(f.done() for f in self.ready_checks)
                    and now-self.ready_started > self.ready_timeout):
                completed = sum(f.done() for f in self.ready_checks)
                self._fail_ready(
                    'TCP ready collision check timeout (%d/%d complete); remain HOLD' %
                    (completed, len(self.ready_requests)))
                return
            tolerances = np.array(TCP_ENTRY_TOLERANCE)
            # During a sampled path check, use the same one-sample RMD drift
            # budget as the tool-path validator. Final TCP entry stays tighter.
            tolerances[1:4] = self.ready_step
            if np.any(np.abs(self.measured-self.ready_snapshot) > tolerances):
                self._fail_ready('feedback changed during TCP ready check; remain HOLD')
                return
            if not all(f.done() for f in self.ready_checks):
                return
            for index, future in enumerate(self.ready_checks):
                response = future.result()
                if response is None or not response.valid:
                    contacts = sorted({
                        ' <-> '.join(sorted((contact.contact_body_1, contact.contact_body_2)))
                        for contact in getattr(response, 'contacts', [])})
                    detail = (': '+', '.join(contacts[:6]) if contacts else
                              '; no contact pairs reported (check joint bounds/scene)')
                    self._fail_ready(
                        'TCP ready path collision/state invalid at sample %d/%d%s; remain HOLD' %
                        (index+1, len(self.ready_requests), detail))
                    return
            if len(self.ready_checks) < len(self.ready_requests):
                self.reason = 'checking path to TCP ready (%d/%d complete)' % (
                    len(self.ready_checks), len(self.ready_requests))
                self.ready_checks.append(self.check.call_async(
                    self.ready_requests[len(self.ready_checks)]))
                self.ready_started = now
                return
            if not (vars(self).get('returning_home', False) or vars(self).get('named_one_shot', False)) and (not self.deadman or now-self.joy_time >= .25):
                self.reason = 'TCP ready path verified; hold L1 to move'
                return
            if not self.tool_mode.service_is_ready():
                self._fail_ready('tool_auto mode service unavailable; remain HOLD')
                return
            self.ready_future = self.tool_mode.call_async(Trigger.Request())
            self.ready_state = 'request_tool'
            self.reason = 'requesting managed TCP ready motion'
            return
        if self.ready_state == 'request_tool':
            if not self.ready_future.done():
                return
            result = self.ready_future.result()
            if result is None or not result.success:
                self._fail_ready('tool_auto request rejected; remain HOLD')
                return
            self.ready_state = 'wait_tool'
            return
        if self.ready_state == 'wait_tool':
            if s.get('mode') != 'tool_auto' or s.get('transitioning'):
                return
            if not (vars(self).get('returning_home', False) or vars(self).get('named_one_shot', False)) and (not self.deadman or now-self.joy_time >= .25):
                self._fail_ready('L1 released before TCP ready motion; remain HOLD')
                if self.cancel.service_is_ready():
                    self.cancel.call_async(Trigger.Request())
                return
            self.ready_execution_id = 'tcp-ready-'+uuid.uuid4().hex
            self.tool_pub.publish(String(data=json.dumps(dict(
                epoch=s['epoch'], execution_id=self.ready_execution_id,
                waypoints=(self.home_waypoints if vars(self).get('returning_home', False)
                           else [self.ready_goal.tolist()]),
                durations=(self.home_durations if vars(self).get('returning_home', False)
                           else [self.ready_duration])))))
            self.ready_state = 'executing'
            self.reason = 'moving to TCP ready'
            return
        if self.ready_state == 'executing':
            # L1 authorizes this collision-checked one-shot move at its start.
            # Releasing it afterward must not turn a reached ready pose into a
            # failed state; TCP motion still requires a fresh release/re-press.
            if (s.get('last_execution_id') == self.ready_execution_id and
                    s.get('last_execution_result') == 'succeeded' and s.get('mode') == 'hold'):
                self.ready_state = 'await_entry'
                self.ready_safe_at = None
                self.entry_retry_count = 0
                self.reason = 'TCP ready reached; verifying entry'
                if vars(self).get('returning_home', False):
                    self.ready_state = 'home_settle'
                    self.ready_started = now
                    self.home_settle_since = None
                    self.reason = 'HOME '+self.home_leg+' reached; verifying stopped feedback'
                else:
                    self.explicit_ready = False
            elif s.get('mode') == 'fault' or (s.get('last_execution_id') == self.ready_execution_id
                                              and s.get('last_execution_result') == 'aborted'):
                self._fail_ready('TCP ready execution failed: '+s.get('reason', 'inspect arm'))
            return
        if self.ready_state == 'await_entry' and self.ready_safe_at is not None:
            # Allow the manager subscription to consume the published proof first.
            if (now-self.ready_safe_at < .1
                    or now < vars(self).get('entry_retry_after', 0.)
                    or not self.tcp_mode.service_is_ready()):
                return
            if deadline is None:
                self.entry_deadline = now+3.
            self.ready_future = self.tcp_mode.call_async(Trigger.Request())
            self.ready_started = now
            self.ready_request_epoch = s.get('epoch')
            self.ready_state = 'request_tcp'
            self.reason = 'requesting TCP_IK after verified TCP ready'
            return
        if self.ready_state == 'request_tcp' and self.ready_future.done():
            result = self.ready_future.result()
            if result is not None and result.success:
                self.ready_state = 'wait_tcp'
                self.reason = 'waiting for confirmed TCP_IK mode'
            else:
                self.auto_entry_armed = False
                detail = getattr(result, 'message', '') if result is not None else ''
                if detail in ('TCP entry feedback changed; recheck required',
                              'TCP entry waiting for 0.20 sec stable feedback',
                              'TCP entry requires stopped feedback'):
                    self.entry_retry_count += 1
                    # Settling is expected after a completed path. Bound it by
                    # elapsed time, not three fast service round trips. Each
                    # retry still needs a fresh collision proof and all manager
                    # guards, including 0.20 sec of stopped feedback.
                    self.ready_state = 'await_entry'
                    self.ready_safe_at = None
                    self.entry_retry_after = now+.25
                    self.auto_entry_armed = True
                    self.reason = 'TCP entry settling; recheck %d: %s' % (
                        self.entry_retry_count, detail)
                else:
                    self.ready_state = 'failed'
                    self.reason = 'TCP_IK request rejected: '+detail
            return
        if self.ready_state == 'wait_tcp':
            if s.get('mode') == 'tcp_ik' and not s.get('transitioning'):
                self.ready_state = 'idle'
                self.auto_entry_armed = False
                self.reason = 'TCP_IK confirmed; release L1 to rearm'
            elif (s.get('epoch') != self.ready_request_epoch
                  and not s.get('transitioning')):
                self._fail_ready('TCP transition did not complete: '+s.get('reason', '')+
                                 '; release and press L1 to retry')


