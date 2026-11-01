from pathlib import Path
import sys
import pytest
import yaml
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_control_mode_manager_test import TestModeCore as TimedCore
from tool_trajectory_joy_test import load_path


def core():
    c = TimedCore([(-4.,4.)]*6, [.12]*6)
    c.focus, c.ready, c.mode = 'arm', True, 'tool_auto'
    c.feedback([0.]*6, [0.]*6, 0.)
    c.ee_position, c.ee_velocity, c.ee_state_time = 1., 0., 0.
    assert c.timed_ee(c.epoch, 'open', -.48, 2., 0.)
    return c


def test_two_seconds_then_zero_and_hold():
    c = core()
    for t in [.01, 1., 1.99]:
        c.ee_state_time = t
        c.run_ee(t)
        assert c.ee == -.48
    c.ee_state_time = 2.
    c.run_ee(2.)
    assert c.ee == 0. and c.mode == 'hold'
    assert c.last_execution_id == 'open' and c.last_execution_result == 'succeeded'


def test_stale_feedback_stops_without_success():
    c = core()
    c.run_ee(.4)
    assert c.ee == 0. and c.last_execution_result == 'aborted'


def test_arm_drift_stops():
    c = core()
    c.position[1] = .006
    c.run_ee(.01)
    assert c.ee == 0. and c.last_execution_result == 'aborted'


def test_split_at_source_point_14():
    p = Path(__file__).resolve().parents[1]/'config/test/resampled/tool0_trajectory.yaml'
    d = yaml.safe_load(p.read_text())
    m = Mock(); m.within_limits.return_value = True
    path = load_path(p, m)
    i = path['ee_split_index']
    assert i == d['source_point_indices'][13]
    assert list(path['waypoints'][i-1]) == d['points'][i]['positions_rad']


def test_middle_action_precedes_tail():
    import numpy as np
    from types import SimpleNamespace
    from tool_trajectory_joy_test import ToolTrajectoryJoy, ARM_JOINTS
    q = np.zeros(6)
    n = SimpleNamespace(ee_mid_phase='approach', path_id=0,
        full_state=dict(zip(ARM_JOINTS, q), ee_joint=1.),
        report=Mock(), begin=Mock(), start_tolerance=.03, max_start_delta=.1,
        approach_speed=.05,
        ee_tail=dict(start=q, waypoints=[q+.01], durations=[1.], total=1.))
    assert ToolTrajectoryJoy.handle_mid_path_ee(n)
    assert n.ee_mid_phase == 'opening'
    command = n.begin.call_args.args[1]
    assert command['ee_duration_sec'] == 2. and command['ee_velocity_rad_s'] == -.48
    assert command['ee_target_rad'] == pytest.approx(.04)
    assert ToolTrajectoryJoy.handle_mid_path_ee(n)
    assert n.ee_mid_phase == 'tail'
    assert 'ee_duration_sec' not in n.begin.call_args.args[1]
    assert not ToolTrajectoryJoy.handle_mid_path_ee(n)
