"""Offline validation of captured path loading and joystick request edges."""
from pathlib import Path
import sys
from unittest.mock import Mock
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
from tool_trajectory_joy import (collision_check_budget, execution_plan, load_path,
                                 nearest_path_suffix, planned_entry, requested_path)


def write_path(tmp_path, enabled=True, times=(0., 1., 3.)):
    path = tmp_path/'path.yaml'
    path.write_text(yaml.safe_dump(dict(
        execution_enabled=enabled,
        joint_names=['base_joint', 'shoulder_joint', 'elbow_joint',
                     'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint'],
        points=[dict(time_from_start_sec=t, positions_rad=[i*.01]*6)
                for i, t in enumerate(times)])))
    return path


def test_enabled_path_uses_zero_point_as_start_and_segment_durations(tmp_path):
    model = Mock()
    model.within_limits.return_value = True
    loaded = load_path(write_path(tmp_path), model)
    assert loaded['start'].tolist() == [0.]*6
    assert [q.tolist() for q in loaded['waypoints']] == [[.01]*6, [.02]*6]
    assert loaded['durations'] == [1., 2.]


def test_disabled_path_is_not_loadable_for_execution(tmp_path):
    assert load_path(write_path(tmp_path, enabled=False), Mock()) is None


@pytest.mark.parametrize('times', [(1., 2.), (0., 0.), (0., 120.)])
def test_invalid_timing_is_rejected(tmp_path, times):
    model = Mock()
    model.within_limits.return_value = True
    with pytest.raises(ValueError, match='times'):
        load_path(write_path(tmp_path, times=times), model)


def test_l2_plus_first_button_has_one_rising_edge():
    axes, buttons = [0., 0., -1.], [0, 0, 1, 0, 0]
    request, held = requested_path(axes, buttons, [False]*3)
    assert request == 0
    assert requested_path(axes, buttons, held)[0] is None
    request, held = requested_path([0., 0., 1.], buttons, held)
    assert request is None and held == [False]*3
    assert requested_path(axes, buttons, held)[0] == 0


def test_simultaneous_path_buttons_are_ignored():
    request, held = requested_path([0., 0., -1.], [0, 0, 1, 1, 0], [False]*3)
    assert request is None and held == [True, True, False]


def test_ui_request_uses_one_rising_edge_and_nearest_entry():
    node = SimpleNamespace(ui_held={(0, True): False}, request_path=Mock())
    from tool_trajectory_joy import ToolTrajectoryJoy
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=True), 0, True)
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=True), 0, True)
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=False), 0, True)
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=True), 0, True)
    assert node.request_path.call_count == 2
    node.request_path.assert_called_with(0, True, nearest_entry=True)


def test_docking_trigger_is_immediately_returned_to_opposite_level():
    node = SimpleNamespace(docking_command=Mock())
    from tool_trajectory_joy import ToolTrajectoryJoy
    ToolTrajectoryJoy.send_docking_pulse(node, True)
    assert [call.args[0].data for call in node.docking_command.publish.call_args_list] == [True, False]
    node.docking_command.reset_mock()
    ToolTrajectoryJoy.send_docking_pulse(node, False)
    assert [call.args[0].data for call in node.docking_command.publish.call_args_list] == [False, True]


def test_report_uses_distinct_logger_calls_for_different_severities():
    node = SimpleNamespace(diagnostics=Mock(), path_id=0, get_logger=Mock())
    logger = node.get_logger.return_value
    from tool_trajectory_joy import ToolTrajectoryJoy
    ToolTrajectoryJoy.report(node, 'checking', 'checking')
    ToolTrajectoryJoy.report(node, 'failed', 'failed')
    logger.info.assert_called_once_with('checking')
    logger.error.assert_called_once_with('failed')


def test_nearby_start_gets_collision_checked_timed_approach():
    path = dict(start=np.zeros(6), waypoints=[np.ones(6)*.1],
                durations=[2.], total=2.)
    plan = execution_plan(path, np.ones(6)*.07)
    assert len(plan['waypoints']) == 2
    np.testing.assert_array_equal(plan['waypoints'][0], path['start'])
    assert plan['durations'] == pytest.approx([1.4, 2.])
    assert plan['total'] == pytest.approx(3.4)


def test_far_start_remains_blocked():
    path = dict(start=np.zeros(6), waypoints=[np.ones(6)*.1],
                durations=[2.], total=2.)
    with pytest.raises(ValueError, match='automatic approach limit'):
        execution_plan(path, np.ones(6)*.101)


def test_nearest_entry_keeps_only_forward_suffix():
    path = dict(start=np.zeros(6),
                waypoints=[np.ones(6), np.ones(6)*2, np.ones(6)*3],
                durations=[1., 2., 3.], total=6.)
    result = nearest_path_suffix(path, np.ones(6)*1.8)
    assert result['entry_index'] == 2
    np.testing.assert_array_equal(result['start'], np.ones(6)*2)
    assert [q.tolist() for q in result['waypoints']] == [[3.]*6]
    assert result['durations'] == [3.]
    assert result['total'] == 3.


def test_nearest_final_point_still_creates_safe_hold_target():
    path = dict(start=np.zeros(6), waypoints=[np.ones(6)], durations=[2.], total=2.)
    suffix = nearest_path_suffix(path, np.ones(6))
    plan = execution_plan(suffix, np.ones(6))
    assert len(plan['waypoints']) == 1
    np.testing.assert_array_equal(plan['waypoints'][0], np.ones(6))
    assert plan['durations'] == [.5]


def test_collision_budget_scales_for_long_serial_moveit_checks():
    assert collision_check_budget(100, 10.) == 10.
    assert collision_check_budget(1068, 10.) == pytest.approx(23.36)
    assert collision_check_budget(10000, 10.) == 120.


def test_moveit_entry_plan_is_prefixed_to_taught_path():
    names = ['wrist_yaw_joint', 'base_joint', 'shoulder_joint',
             'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
    def point(positions, sec, nanosec=0):
        return SimpleNamespace(positions=positions,
                               time_from_start=SimpleNamespace(sec=sec, nanosec=nanosec))
    # Positions follow the deliberately shuffled trajectory joint order.
    trajectory = SimpleNamespace(joint_names=names, points=[
        point([.1, .1, .1, .1, .1, .1], 1),
        point([0., 0., 0., 0., 0., 0.], 2),
    ])
    path = dict(start=np.zeros(6), waypoints=[np.ones(6)*.2],
                durations=[3.], total=3.)
    result = planned_entry(path, trajectory, np.ones(6)*.2)
    assert [q.tolist() for q in result['waypoints']] == [
        [.1]*6, [0.]*6, [.2]*6]
    assert result['durations'] == pytest.approx([1., 1., 3.])
    assert result['total'] == pytest.approx(5.)


def test_moveit_entry_must_reach_taught_start():
    trajectory = SimpleNamespace(joint_names=list(
        ['base_joint', 'shoulder_joint', 'elbow_joint',
         'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint']), points=[
            SimpleNamespace(positions=[.02]*6,
                            time_from_start=SimpleNamespace(sec=1, nanosec=0))])
    path = dict(start=np.zeros(6), waypoints=[np.ones(6)*.2],
                durations=[3.], total=3.)
    with pytest.raises(ValueError, match='did not reach'):
        planned_entry(path, trajectory, np.ones(6)*.2)


def test_path_snapshot_accepts_dxl_hunt_but_keeps_rmd_tight():
    names = ['base_joint', 'shoulder_joint', 'elbow_joint',
             'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint']
    node = SimpleNamespace(snapshot=dict.fromkeys(names, 0.),
                           full_state=dict.fromkeys(names, 0.))
    from tool_trajectory_joy import ToolTrajectoryJoy
    node.full_state['wrist_roll_joint'] = .019
    assert ToolTrajectoryJoy.unchanged(node)
    node.full_state['shoulder_joint'] = .0011
    assert not ToolTrajectoryJoy.unchanged(node)


def test_path_validation_allows_only_one_collision_step_of_rmd_drift():
    names = ['base_joint', 'shoulder_joint', 'elbow_joint',
             'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint']
    node = SimpleNamespace(snapshot=dict.fromkeys(names, 0.),
                           full_state=dict.fromkeys(names, 0.),
                           validation_rmd_drift=.005)
    node.full_state['shoulder_joint'] = .0049
    assert ToolTrajectoryJoy.unchanged(node, validation=True)
    node.full_state['shoulder_joint'] = .0051
    assert not ToolTrajectoryJoy.unchanged(node, validation=True)


def test_path_snapshot_accepts_uncommanded_ee_dxl_encoder_hunt():
    names = ['base_joint', 'shoulder_joint', 'elbow_joint',
             'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint']
    node = SimpleNamespace(snapshot=dict.fromkeys(names, 0.),
                           full_state=dict.fromkeys(names, 0.),
                           validation_rmd_drift=.005)
    node.full_state['ee_joint'] = .019
    assert ToolTrajectoryJoy.unchanged(node, validation=True)
    node.full_state['ee_joint'] = .021
    assert not ToolTrajectoryJoy.unchanged(node, validation=True)
