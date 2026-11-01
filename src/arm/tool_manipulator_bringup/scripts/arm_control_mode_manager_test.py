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
                or velocity != -.48 or duration != 2.):
            raise ValueError('test EE opening requires -0.48 rad/s for 2 seconds')
        target = self.ee_position + velocity*duration if self.ee_position is not None else float('nan')
        accepted = self.close_ee(epoch, execution_id, target, now, speed=.12)
        if accepted:
            self.timed_ee_id = execution_id
            self.timed_ee_velocity = velocity
            self.timed_ee_end = now+duration
        return accepted

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
            completed = self.execution_id
            self.request('hold', now)
            self.last_execution_id, self.last_execution_result = completed, 'succeeded'
            self.reason = 'timed ee opening completed; HOLD'
        elif self.ee_position <= -2.351592547829:
            self.abort_path_to_hold('timed ee reached lower limit', now)
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
