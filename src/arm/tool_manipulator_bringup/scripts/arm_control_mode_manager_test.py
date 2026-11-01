#!/usr/bin/env python3
"""Test-only manager extension for bounded timed gripper motion."""
import json
import math
import time
import arm_control_mode_manager as manager
from arm_mode_core import ModeCore


class TestModeCore(ModeCore):
    def timed_ee(self, epoch, execution_id, velocity, duration, now):
        if (type(velocity) not in (float, int) or not math.isfinite(velocity)
                or velocity != -1. or duration != 3.2):
            raise ValueError('test EE opening requires -1 rad/s for 3.2 seconds')
        # Timed velocity control of continuous EE: no position-goal bounds.
        # Keep ownership, freshness and stopped-arm checks independent of close_ee.
        if (self.mode != 'tool_auto' or epoch != self.epoch or self.transitioning or
                self.focus != 'arm' or not self.ready or not self.fresh(now)):
            return False
        if self.execution_id is not None:
            raise ValueError('execution already active; cancel first')
        if not isinstance(execution_id, str) or not execution_id:
            raise ValueError('invalid ee execution id')
        if (not 0 <= now-self.ee_state_time < .3 or self.ee_position is None
                or not math.isfinite(self.ee_position)):
            raise ValueError('fresh finite ee feedback required')
        if max(map(abs, self.velocity)) > .02:
            raise ValueError('arm must be stopped before ee motion')
        self.target = self.goal = list(self.position)
        self.execution_id = execution_id
        self.ee_goal = self.ee_position + velocity*duration
        self.started = now
        self.timed_ee_id = execution_id
        self.timed_ee_velocity = velocity
        self.timed_ee_end = now+duration
        self.timed_ee_stable_since = None
        return True

    def run_ee(self, now):
        if getattr(self, 'timed_ee_id', None) != self.execution_id or self.execution_id is None:
            return super().run_ee(now)
        self.ee = 0.
        if not 0 <= now-self.ee_state_time < .3:
            self.abort_path_to_hold('timed ee feedback timeout', now)
        elif any(abs(a-b) > tol for a,b,tol in zip(
                self.position, self.goal, (.02,.005,.005,.005,.02,.02))):
            self.abort_path_to_hold('arm moved during timed ee', now)
        elif now >= self.timed_ee_end:
            # Confirm standstill from fresh velocity feedback; EE position is free.
            self.reason = 'timed ee opening stopped; waiting for stable feedback'
            if now-self.timed_ee_end > 3.:
                self.abort_path_to_hold('timed ee failed to settle after stop', now)
                return
            if not math.isfinite(self.ee_velocity) or abs(self.ee_velocity) > .02:
                self.timed_ee_stable_since = None
                return
            if self.timed_ee_stable_since is None:
                self.timed_ee_stable_since = now
            elif now-self.timed_ee_stable_since >= .3:
                completed = self.execution_id
                self.request('hold', now)
                self.last_execution_id, self.last_execution_result = completed, 'succeeded'
                self.reason = 'timed ee opening settled; HOLD'
        else:
            self.ee = self.timed_ee_velocity


class TestControlModeManager(manager.ControlModeManager):
    def on_command(self, source, msg):
        try:
            data = json.loads(msg.data)
            if (isinstance(data, dict) and 'independent_ee_velocity' in data
                    and self.core.execution_id is not None
                    and getattr(self.core, 'timed_ee_id', None) == self.core.execution_id):
                self.last_error = 'independent EE jog blocked during timed tool opening'
                return
            if source == 'tool_auto' and isinstance(data, dict) and 'ee_duration_sec' in data:
                accepted = self.core.timed_ee(data['epoch'], data['execution_id'],
                    data['ee_velocity_rad_s'], data['ee_duration_sec'], time.monotonic())
                if accepted:
                    self.independent_ee_velocity = 0.
                    self.independent_ee_command_time = -float('inf')
                self.last_error = '' if accepted else 'timed ee rejected: inactive owner'
                return
        except (ValueError, TypeError, KeyError) as exc:
            self.last_error = 'invalid timed ee: '+str(exc)
            return
        super().on_command(source, msg)


if __name__ == '__main__':
    manager.ModeCore = TestModeCore
    manager.ControlModeManager = TestControlModeManager
    manager.main()
