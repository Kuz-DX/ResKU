"""Source-config replay with ideal feedback, without ROS nodes or motor access."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore
from tool_trajectory_joy import (load_path, execution_plan,
                                 isolate_shoulder_zero_crossings, execution_timeout)


def test_tcp_ready_then_recorded_path_zero_completes():
    config = Path(__file__).resolve().parents[1]/'config'
    hardware = yaml.safe_load((config/'hardware.yaml').read_text())['joints']
    ready = yaml.safe_load((config/'tcp_ready.yaml').read_text())['tcp_ready']
    limits = [hardware[j]['soft_limit_rad'] for j in ready['joint_names']]
    c = ModeCore(limits, [.12]*6,
                 path_speeds=[.08, .04, .06, .06, .08, .08],
                 path_accelerations=[.08, .03, .06, .06, .08, .08],
                 path_jerks=[.30, .04, .15, .15, .30, .30],
                 path_feedback_leads=[.04, .015, .025, .025, .04, .04])
    c.ready, c.focus = True, 'arm'
    c.feedback(np.radians([0., 85., 87., -89., 0., 0.]), [0.]*6, 0.)
    now = 0.

    def replay(name, points, durations, stops=None):
        nonlocal now
        c.mode = c.requested = 'tool_auto'
        c.transitioning = False
        assert c.path(c.epoch, name, points, now, durations, stops)
        start = now
        for _ in range(60000):
            now += .01
            # One-sample feedback delay; velocity is the actual command delta.
            velocity = [(t-p)/.01 for t, p in zip(c.target, c.position)]
            c.feedback(c.target, velocity, now)
            c.tick(now, .01)
            if c.last_execution_id == name:
                break
        assert c.last_execution_id == name
        assert c.last_execution_result == 'succeeded', c.reason
        return now-start

    replay('ready', [ready['positions_rad']], [ready['duration_sec']])
    def within_limits(q, margin):
        return all(lo+margin <= v <= hi-margin for v, (lo, hi) in zip(q, limits))
    taught = load_path(config/'tool0_trajectory.yaml',
                       SimpleNamespace(within_limits=within_limits))
    plan = execution_plan(taught, c.position, .02, .35, .03)
    plan = isolate_shoulder_zero_crossings(plan, .08, .015)
    budget = execution_timeout(plan, c.position, c.path_speed_limits)
    elapsed = replay('path0', [p.tolist() for p in plan['waypoints']],
                     plan['durations'], plan['stop_at_waypoints'])
    assert elapsed < budget, (elapsed, budget)
