"""Shared preparation/return workflow; motor ownership remains in the mode manager."""
import json
import math
import uuid

import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity
from arm_mode_core import TCP_ENTRY_TOLERANCE, non_arm_feedback_changed
try:
    from tool_change_min.kinematics import ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ARM_JOINTS


class TcpMotionWorkflow:
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
            self.entry_retry_count = 0
            self.auto_entry_armed = True
            self.reason = 'L1 requested fresh TCP entry/ready validation'

    def _reset_ready(self):
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
        if vars(self).get('returning_home', False):
            self.returning_home = False
            self.ready_goal = self.default_ready_goal.copy()

    def drive_ready(self, now):
        """Collision-check and execute one explicit-focus TCP-ready entry."""
        s = self.status
        if not self.ready_enabled or not self.auto_entry_armed:
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
            differences = np.abs(self.ready_goal-self.measured)
            limiting_joint = int(np.argmax(differences))
            delta = float(differences[limiting_joint])
            limit = (self.home_max_delta if vars(self).get('returning_home', False)
                     else self.ready_max_delta)
            if delta > limit:
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
        if self.ready_state == 'checking':
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
            if not vars(self).get('returning_home', False) and (not self.deadman or now-self.joy_time >= .25):
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
            if not vars(self).get('returning_home', False) and (not self.deadman or now-self.joy_time >= .25):
                self._fail_ready('L1 released before TCP ready motion; remain HOLD')
                if self.cancel.service_is_ready():
                    self.cancel.call_async(Trigger.Request())
                return
            self.ready_execution_id = 'tcp-ready-'+uuid.uuid4().hex
            self.tool_pub.publish(String(data=json.dumps(dict(
                epoch=s['epoch'], execution_id=self.ready_execution_id,
                waypoints=[self.ready_goal.tolist()], durations=[self.ready_duration]))))
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
                    self.returning_home = False
                    self.ready_goal = self.default_ready_goal.copy()
                    # HOME is intentionally outside the TCP-entry joint margin.
                    # Prepare the fixed, bounded ready path again and wait for L1.
                    self.auto_entry_armed = True
                    self.ready_state = 'idle'
                    self.reason = 'manual Stop_pose reached; release then hold L1 for TCP ready'
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


