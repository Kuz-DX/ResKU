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
                                 isolate_shoulder_zero_crossings, nearest_path_suffix,
                                 planned_entry, requested_path, ToolTrajectoryJoy)
from tool_trajectory_joy import reverse_path


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


def test_joint_limited_timing_is_independent_of_capture_pauses(tmp_path):
    from arm_mode_core import ModeCore
    model = Mock()
    model.within_limits.return_value = True
    schedules = []
    for times in ((0., 1., 3.), (0., 200., 1000.)):
        path = write_path(tmp_path, times=times)
        data = yaml.safe_load(path.read_text())
        data['timing_mode'] = 'joint_limits'
        path.write_text(yaml.safe_dump(data))
        loaded = load_path(path, model)
        c = ModeCore([(-2., 2.)]*6, [.12]*6,
                     path_speeds=[.08, .04, .06, .06, .08, .08])
        c.ready, c.focus = True, 'arm'
        c.feedback(loaded['start'], [0.]*6, 1.)
        c.mode = c.requested = 'tool_auto'
        assert c.path(c.epoch, 'capture', loaded['waypoints'], 1., loaded['durations'])
        schedules.append(c.path_speeds)
        # All joints travel .01 rad. The .04 rad/s shoulder cap sets .25 s.
        assert c.path_speeds == pytest.approx([.04]*6)
    assert schedules[0] == schedules[1]


def test_unknown_timing_mode_is_rejected(tmp_path):
    path = write_path(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['timing_mode'] = 'joint_limit'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='timing_mode'):
        load_path(path, Mock())


def test_disabled_path_is_not_loadable_for_execution(tmp_path):
    assert load_path(write_path(tmp_path, enabled=False), Mock()) is None


def test_close_and_return_action_is_loaded_and_validated(tmp_path):
    path = write_path(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['after_path'] = dict(ee_close_rad=1.251728, return_to_start=True)
    path.write_text(yaml.safe_dump(data))
    model = Mock()
    model.within_limits.return_value = True
    assert load_path(path, model)['after_path'] == data['after_path']
    data['after_path']['ee_close_rad'] = 20.
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='close-and-return'):
        load_path(path, model)


def test_tool2_corrected_points_preserve_original_fk_xyz_with_joint_margin():
    from tool_trajectory_joy import ArmKinematics
    path = SCRIPTS.parent/'config/tool2_trajectory.yaml'
    data = yaml.safe_load(path.read_text())
    reference = yaml.safe_load((SCRIPTS.parent/'config/tool2_ik_reference.yaml').read_text())
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    assert len(data['points']) == 5
    assert data['execution_enabled'] is True
    assert data['timing_mode'] == 'joint_limits'
    assert reference['frame_id'] == 'base_actuator' and reference['tip_link'] == 'tcp_link'
    invalid_originals = []
    for i, (point, evidence) in enumerate(zip(data['points'], reference['points']), start=1):
        original = np.asarray(evidence['original_positions_rad'])
        corrected = np.asarray(point['positions_rad'])
        assert evidence['point'] == i
        assert len(corrected) == 6
        np.testing.assert_array_equal(corrected, evidence['corrected_positions_rad'])
        if not model.within_limits(original, .02):
            invalid_originals.append(i)
        assert model.within_limits(corrected, .02)
        target = model.fk(original)
        np.testing.assert_allclose(target, evidence['original_tcp_transform'], atol=1e-12)
        error = model.pose_error(corrected, target)
        assert np.linalg.norm(error[:3]) < 1e-6
        assert np.linalg.norm(error[:3]) == pytest.approx(evidence['tcp_position_error_m'], abs=1e-12)
        assert np.linalg.norm(error[3:]) == pytest.approx(evidence['tcp_orientation_change_rad'])
    assert invalid_originals == [2, 3, 4, 5]
    loaded = load_path(path, model)
    assert len(loaded['waypoints']) == 4
    assert loaded['after_path'] == dict(ee_close_rad=1.251728, return_to_start=True)
    returning = reverse_path(loaded)
    np.testing.assert_array_equal(returning['start'], data['points'][4]['positions_rad'])
    np.testing.assert_array_equal(returning['waypoints'],
                                  [p['positions_rad'] for p in data['points'][3::-1]])
    assert returning['durations'] == [.02]*4


def test_tool2_ee_close_is_independent_dynamixel_id4():
    data = yaml.safe_load((SCRIPTS.parent/'config/tool2_trajectory.yaml').read_text())
    hardware = yaml.safe_load((SCRIPTS.parent/'config/hardware.yaml').read_text())
    assert 'ee_joint' not in data['joint_names']
    assert hardware['joints']['ee_joint']['vendor'] == 'dynamixel'
    assert hardware['joints']['ee_joint']['id'] == 4
    assert data['after_path']['ee_close_rad'] == 1.251728


def test_reverse_path_retraces_all_points_with_reversed_segment_times():
    points = [np.ones(6)*i*.1 for i in range(5)]
    path = dict(start=points[0], waypoints=points[1:], durations=[1., 2., 3., 4.], total=10.)
    result = reverse_path(path)
    np.testing.assert_array_equal(result['start'], points[4])
    np.testing.assert_array_equal(result['waypoints'], points[3::-1])
    assert result['durations'] == [4., 3., 2., 1.]


def test_sequence_closes_before_returning_and_finishes_at_original_start():
    points = [np.ones(6)*i*.1 for i in range(5)]
    path = dict(start=points[0], waypoints=points[1:], durations=[1., 2., 3., 4.],
                total=10., after_path=dict(ee_close_rad=1.251728, return_to_start=True))
    from tool_trajectory_joy import ARM_JOINTS
    node = SimpleNamespace(paths={2: path}, path_id=2, sequence_phase='approach',
                           full_state=dict(zip(ARM_JOINTS, points[-1])),
                           start_tolerance=.03, max_start_delta=.1, approach_speed=.05,
                           report=Mock(), begin=Mock(), reset=Mock())
    ToolTrajectoryJoy.path_completed(node)
    assert node.sequence_phase == 'closing'
    closing = node.begin.call_args.args[1]
    assert closing['ee_target_rad'] == 1.251728
    np.testing.assert_array_equal(closing['waypoints'], [points[4]])
    node.reset.assert_not_called()
    # Called again only after the manager reports successful EE completion.
    ToolTrajectoryJoy.path_completed(node)
    assert node.sequence_phase == 'returning'
    returning = node.begin.call_args.args[1]
    np.testing.assert_array_equal(returning['waypoints'], points[3::-1])
    assert 'ee_target_rad' not in returning
    ToolTrajectoryJoy.path_completed(node)
    node.reset.assert_called_once()


def test_failed_ee_execution_does_not_start_return():
    import time
    now = time.monotonic()
    node = SimpleNamespace(stage='executing', status_time=now, state_time=now,
                           status=dict(last_execution_id='close', last_execution_result='aborted'),
                           execution_id='close', path_completed=Mock(), fail=Mock())
    ToolTrajectoryJoy.tick(node)
    node.path_completed.assert_not_called()
    node.fail.assert_called_once()


def test_ee_collision_check_sweeps_gripper_with_arm_held():
    from tool_trajectory_joy import ARM_JOINTS
    measured = np.zeros(6)
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold-epoch'}),
        full_state=dict(zip(ARM_JOINTS, measured)), shoulder_isolation_enabled=False,
        collision=Mock(), mode=Mock(), collision_timeout=10., start_tolerance=.03,
        report=Mock(), begin_failed=Mock())
    node.full_state.update(ee_joint=0., rack_left_joint=.02, rack_right_joint=.02)
    ToolTrajectoryJoy.begin(node, 2, dict(
        start=measured, waypoints=[measured], durations=[.5], total=.5,
        start_error=0., ee_target_rad=.1))
    node.begin_failed.assert_not_called()
    assert node.stage == 'checking'
    values = [dict(zip(req.robot_state.joint_state.name,
                       req.robot_state.joint_state.position)) for req in node.requests]
    assert [value['ee_joint'] for value in values] == pytest.approx(np.linspace(0., .1, 6))
    assert all(all(value[j] == 0. for j in ARM_JOINTS) for value in values)
    assert all('rack_left_joint' not in value and 'rack_right_joint' not in value
               for value in values)


def test_ee_goal_dispatch_uses_manager_and_requires_collision_success():
    import json
    import time
    from arm_control_mode_manager import ControlModeManager
    now = time.monotonic()
    node = SimpleNamespace(
        stage='wait_mode', status_time=now, state_time=now,
        status=dict(mode='tool_auto', epoch='close-epoch'), unchanged=Mock(return_value=True),
        path=dict(ee_target_rad=1.251728), publisher=Mock(), report=Mock(), path_id=2)
    ToolTrajectoryJoy.tick(node)
    message = node.publisher.publish.call_args.args[0]
    command = json.loads(message.data)
    assert command['ee_target_rad'] == 1.251728 and 'waypoints' not in command
    gateway = SimpleNamespace(core=Mock())
    ControlModeManager.on_command(gateway, 'tool_auto', message)
    assert gateway.core.close_ee.call_args.args[:3] == (
        'close-epoch', node.execution_id, 1.251728)
    gateway.core.path.assert_not_called()
    node.stage = 'checking'
    node.healthy_hold = Mock(return_value=dict(epoch='hold-epoch'))
    node.original_epoch = 'hold-epoch'
    node.collision_deadline = now+10.
    node.future = Mock()
    node.future.result.return_value = SimpleNamespace(valid=False, contacts=[])
    node.request_index, node.requests = 0, [object()]
    node.fail, node.mode = Mock(), Mock()
    node.publisher.reset_mock()
    ToolTrajectoryJoy.tick(node)
    node.fail.assert_called_once()
    node.mode.call_async.assert_not_called()
    node.publisher.publish.assert_not_called()


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


@pytest.mark.parametrize('path_id', [0, 1, 2])
def test_each_l2_number_selects_its_path_once(path_id):
    buttons = [0]*13
    buttons[2+path_id] = 1
    request, held = requested_path([0., 0., -1.], buttons, [False]*3)
    assert request == path_id
    assert requested_path([0., 0., -1.], buttons, held)[0] is None


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


def test_docking_command_publishes_requested_state_once():
    node = SimpleNamespace(docking_command=Mock())
    from tool_trajectory_joy import ToolTrajectoryJoy
    ToolTrajectoryJoy.send_docking_pulse(node, True)
    assert [call.args[0].data for call in node.docking_command.publish.call_args_list] == [True]
    node.docking_command.reset_mock()
    ToolTrajectoryJoy.send_docking_pulse(node, False)
    assert [call.args[0].data for call in node.docking_command.publish.call_args_list] == [False]


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
    np.testing.assert_allclose(result['start'], np.ones(6)*.2)
    isolated = isolate_shoulder_zero_crossings(result)
    np.testing.assert_allclose(isolated['start'], np.ones(6)*.2)
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


def test_shoulder_zero_crossing_is_staged_and_other_joints_hold():
    start = np.array([0., -.2, 0., 0., 0., 0.])
    goal = np.array([.4, .2, .3, -.2, .1, -.1])
    result = isolate_shoulder_zero_crossings(dict(
        start=start, waypoints=[goal], durations=[4.], total=4., start_error=0.))
    assert len(result['waypoints']) == 4
    entry, isolated_exit, nominal_exit, final = result['waypoints']
    assert entry[1] == pytest.approx(-.08)
    assert isolated_exit[1] == pytest.approx(.08)
    assert isolated_exit[[0, 2, 3, 4, 5]] == pytest.approx(
        entry[[0, 2, 3, 4, 5]])
    assert nominal_exit[1] == pytest.approx(.08)
    assert final == pytest.approx(goal)
    assert result['stop_at_waypoints'] == [True, True, True, False]
    assert result['durations'][1] >= .16/.015


def test_shoulder_non_crossing_path_is_not_reshaped():
    goal = np.array([.1, .3, 0., 0., 0., 0.])
    result = isolate_shoulder_zero_crossings(dict(
        start=np.zeros(6), waypoints=[goal], durations=[2.], total=2.))
    np.testing.assert_allclose(result['waypoints'], [goal])
    assert result['durations'] == [2.]
    assert result['stop_at_waypoints'] == [False]


def test_shoulder_crossing_inside_window_is_still_isolated():
    start = np.array([0., .038, 0., 0., 0., 0.])
    goal = np.array([.1, -.013, .2, 0., 0., 0.])
    result = isolate_shoulder_zero_crossings(dict(
        start=start, waypoints=[goal], durations=[3.], total=3.))
    entry, isolated_exit, nominal_exit = result['waypoints'][:3]
    assert entry == pytest.approx(start)
    assert isolated_exit[1] == pytest.approx(goal[1])
    assert isolated_exit[[0, 2, 3, 4, 5]] == pytest.approx(
        start[[0, 2, 3, 4, 5]])
    assert nominal_exit == pytest.approx(goal)
    assert result['stop_at_waypoints'][:3] == [True, True, True]


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
