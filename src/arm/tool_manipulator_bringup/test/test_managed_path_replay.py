"""Source-config replay with ideal feedback, without ROS nodes or motor access."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore
from tool_trajectory_joy import (load_path, execution_plan,
                                 isolate_shoulder_zero_crossings, execution_timeout)


def test_tcp_ready_manager_motion_completes():
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
    np.testing.assert_allclose(c.position, ready['positions_rad'], atol=.01)


def test_tool1_recorded_pause_does_not_slow_execution_or_exceed_joint_speed_limits():
    config = Path(__file__).resolve().parents[1]/'config'
    data = yaml.safe_load((config/'tool1_trajectory.yaml').read_text())
    hardware = yaml.safe_load((config/'hardware.yaml').read_text())['joints']
    limits = [hardware[j]['soft_limit_rad'] for j in data['joint_names']]
    c = ModeCore(limits, [.12]*6,
                 path_speeds=[.08, .04, .06, .06, .08, .08],
                 path_accelerations=[.08, .03, .06, .06, .08, .08],
                 path_jerks=[.30, .04, .15, .15, .30, .30],
                 path_feedback_leads=[.04, .015, .025, .025, .04, .04])
    def within_limits(q, margin):
        return all(lo+margin <= v <= hi-margin for v, (lo, hi) in zip(q, limits))
    taught = load_path(config/'tool1_trajectory.yaml',
                       SimpleNamespace(within_limits=within_limits))
    c.ready, c.focus = True, 'arm'
    c.mode = c.requested = 'tool_auto'
    c.feedback(taught['start'], [0.]*6, 0.)
    plan = isolate_shoulder_zero_crossings(execution_plan(taught, c.position))
    budget = execution_timeout(plan, c.position, c.path_speed_limits)
    assert c.path(c.epoch, 'tool1', plan['waypoints'], 0.,
                  plan['durations'], plan['stop_at_waypoints'])
    now, final_started, final_cruise_time = 0., None, None
    for _ in range(int(budget/.01)+1):
        now += .01
        if not c.queue and final_started is None:
            final_started = now
            final_cruise_time = max(abs(g-t)/v for g, t, v in
                                   zip(c.goal, c.target, c.path_speeds) if v > 0.)
        velocity = [(t-p)/.01 for t, p in zip(c.target, c.position)]
        c.feedback(c.target, velocity, now)
        before = np.asarray(c.target)
        c.tick(now, .01)
        assert np.all(np.abs(np.asarray(c.target)-before)/.01 <=
                      np.asarray(c.path_speed_limits)+1e-8)
        if c.last_execution_id == 'tool1':
            break
    assert c.last_execution_result == 'succeeded', c.reason
    # The same final joint segment needs ~15 s at the existing speed caps,
    # plus acceleration/settling, rather than the recorded 102 s pause.
    assert final_cruise_time == pytest.approx(15.0713612409768)
    assert now-final_started < 30.
    assert now < budget
    np.testing.assert_allclose(c.position, data['points'][-1]['positions_rad'], atol=.01)
