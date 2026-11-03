"""Offline validation of captured path loading and joystick request edges."""
import json
import math
from pathlib import Path
import sys
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from std_msgs.msg import String

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
from tool_trajectory_joy import (collision_check_budget, collision_result_accepted,
                                 execution_plan, load_path,
                                 isolate_shoulder_zero_crossings, nearest_path_suffix,
                                 load_pre_dock_targets, load_route_poses,
                                 nearest_base_equivalent, planned_entry, planned_leg,
                                 simplify_timed_waypoints,
                                 requested_back, requested_path, ARM_JOINTS,
                                 ToolTrajectoryJoy)
from tool_trajectory_joy import reverse_path


def test_scoped_collision_acceptance_allows_only_declared_contact_pair():
    def response(*pairs):
        contacts = [SimpleNamespace(contact_body_1=a, contact_body_2=b)
                    for a, b in pairs]
        return SimpleNamespace(valid=not contacts, contacts=contacts)
    allowed = frozenset((('tool_case_1', 'wrist_link_yaw'),))
    assert collision_result_accepted(response())
    assert collision_result_accepted(
        response(('wrist_link_yaw', 'tool_case_1')), allowed)
    assert not collision_result_accepted(
        response(('wrist_link_yaw', 'tool_case_1')))
    assert not collision_result_accepted(response(
        ('wrist_link_yaw', 'tool_case_1'), ('elbow_link', 'tool_case_1')), allowed)


def test_tool1_final_entry_ignores_only_toolbox2_and_preserves_other_faults():
    from tool_trajectory_joy import final_entry_ignored_links
    def response(*pairs, valid=False):
        return SimpleNamespace(valid=valid, contacts=[SimpleNamespace(
            contact_body_1=a, contact_body_2=b) for a, b in pairs])
    path = {'final_docking_entry': True,
            'final_entry_ignored_collision_links': frozenset({'tool_case_y_plus'})}
    ignored = final_entry_ignored_links(1, path)
    assert collision_result_accepted(response(('wrist_link_yaw', 'tool_case_y_plus')),
                                     ignored_links=ignored)
    assert collision_result_accepted(response(('cam_link', 'tool_case_y_plus')),
                                     ignored_links=ignored)
    for pair in [('wrist_link_yaw', 'tool_case_1'), ('elbow_link', 'circuit_box'),
                 ('shoulder_link', 'wrist_link_yaw')]:
        assert not collision_result_accepted(
            response(('wrist_link_yaw', 'tool_case_y_plus'), pair), ignored_links=ignored)
    assert not collision_result_accepted(response(), ignored_links=ignored)
    assert not collision_result_accepted(None, ignored_links=ignored)
    assert not collision_result_accepted(response(('wrist_link_yaw', 'tool_case_y_plus')))
    assert not final_entry_ignored_links(1, {})  # observation, HOME/BACK and retreat
    for other in [dict(path, final_docking_entry=False),
                  dict(path, final_entry_ignored_collision_links={'circuit_box'})]:
        with pytest.raises(ValueError, match='tool1 final docking entry'):
            final_entry_ignored_links(1, other)
    with pytest.raises(ValueError, match='tool1 final docking entry'):
        final_entry_ignored_links(0, path)


def test_tool1_reverse_exception_is_scoped_to_retreat_and_original_link():
    from tool_trajectory_joy import final_entry_ignored_links
    path = dict(final_docking_retreat=True,
                final_entry_ignored_collision_links={'tool_case_y_plus'})
    ignored = final_entry_ignored_links(1, path)
    contact = SimpleNamespace(contact_body_1='wrist_link_yaw', contact_body_2='tool_case_y_plus')
    assert collision_result_accepted(SimpleNamespace(valid=False, contacts=[contact]),
                                     ignored_links=ignored)
    contact2 = SimpleNamespace(contact_body_1='elbow_link', contact_body_2='circuit_box')
    assert not collision_result_accepted(SimpleNamespace(valid=False, contacts=[contact, contact2]),
                                         ignored_links=ignored)
    for tool in [0, 2]:
        with pytest.raises(ValueError):
            final_entry_ignored_links(tool, path)
    with pytest.raises(ValueError):
        final_entry_ignored_links(1, dict(path, final_docking_retreat=False))
    with pytest.raises(ValueError):
        final_entry_ignored_links(1, dict(path, final_entry_ignored_collision_links={'circuit_box'}))
    assert not final_entry_ignored_links(1, {})


def test_pending_tool1_request_never_stages_or_uses_previous_observation():
    n = SimpleNamespace(stage='idle', path_id=None, report=Mock(), begin=Mock(),
        begin_back=Mock(), pre_dock_targets=load_pre_dock_targets(
            SCRIPTS.parent/'config/cartesian_targets.yaml'))
    n.pre_dock_targets[1] = {'blocked_reason': 'tool1 observation pending'}
    ToolTrajectoryJoy.request_path(n, 1, desired_docked=True)
    n.begin.assert_not_called()
    n.begin_back.assert_not_called()
    assert n.path_id is None
    assert n.report.call_args.args[0] == 'blocked'
    assert 'pending' in n.report.call_args.args[1]


@pytest.mark.parametrize('invalid', ['joint_limit', 'ee_motion'])
def test_final_fixture_exception_cannot_bypass_limits_or_apply_to_ee(invalid):
    n = SimpleNamespace(healthy_hold=Mock(return_value={'epoch': 'hold'}),
        full_state=dict.fromkeys(ARM_JOINTS, 0.), margin=.02, cfg={},
        model=Mock(), collision=Mock(), mode=Mock(), begin_failed=Mock())
    n.model.within_limits.return_value = invalid != 'joint_limit'
    plan = dict(waypoints=[np.zeros(6)], durations=[1.], total=1., start_error=0.,
        preserve_joint_path=True, final_docking_entry=True,
        final_entry_ignored_collision_links={'tool_case_y_plus'})
    if invalid == 'ee_motion':
        plan['ee_target_rad'] = .5
    ToolTrajectoryJoy.begin(n, 1, plan)
    n.begin_failed.assert_called_once()
    assert 'cannot bypass joint limits or apply to EE' in n.begin_failed.call_args.args[0]
    n.collision.call_async.assert_not_called()
    n.mode.call_async.assert_not_called()


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


@pytest.mark.parametrize('module', ['tool_trajectory_joy', 'tool_trajectory_joy_test'])
def test_close_and_return_action_is_loaded_and_validated(tmp_path, module):
    from importlib import import_module
    load_path = import_module(module).load_path
    path = write_path(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['after_path'] = dict(ee_close_rad=1.251728, return_to_start=True)
    path.write_text(yaml.safe_dump(data))
    model = Mock()
    model.within_limits.return_value = True
    assert load_path(path, model)['after_path'] == data['after_path']
    for target in (-20., 20.):
        data['after_path']['ee_close_rad'] = target
        path.write_text(yaml.safe_dump(data))
        assert load_path(path, model)['after_path'] == data['after_path']
    data['after_path']['ee_close_rad'] = float('nan')
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='close-and-return'):
        load_path(path, model)


def test_tool2_approach_runs_from_ready_to_pickup_within_joint_margin():
    from tool_trajectory_joy import ArmKinematics
    path = SCRIPTS.parent/'config/tool2_trajectory.yaml'
    data = yaml.safe_load(path.read_text())
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    assert len(data['points']) == 8
    assert data['execution_enabled'] is True
    assert data['timing_mode'] == 'joint_limits'
    assert 'after_path' not in data
    points = np.asarray([point['positions_rad'] for point in data['points']])
    # Original point 8 is TCP_ready; approach follows original points 8..1.
    np.testing.assert_array_equal(points[:, 0], [
        -2.1475731030401044, -2.1399031991006763, -2.0079808513425115,
        -1.6382914814620722, -0.9863496466106745, -0.3942330624868169,
        -0.012271846303292033, -0.004601942363863909][::-1])
    assert all(model.within_limits(q, .02) for q in points)
    assert points[7, 2] == 1.522
    np.testing.assert_array_equal(points[[7, 5, 1], 3], [-1.572]*3)
    loaded = load_path(path, model)
    np.testing.assert_array_equal([loaded['start'], *loaded['waypoints']], points)
    speeds = np.abs(np.diff(points, axis=0))/np.asarray(loaded['durations'])[:, None]
    assert np.all(speeds <= np.asarray(data['joint_speed_limits_rad_s'])+1e-12)
    assert all(.02 <= d < 120. for d in loaded['durations'])
    assert loaded['after_path'] is None

    # Phase 1 completes at pickup without dispatching EE or return motion.
    node = SimpleNamespace(paths={2: loaded}, path_id=2,
                           requested_docked_state=None, report=Mock(),
                           reset=Mock(), begin=Mock())
    ToolTrajectoryJoy.path_completed(node)
    node.begin.assert_not_called()
    node.reset.assert_called_once()


@pytest.mark.parametrize('retimed', [False, True])
def test_tool2_slow_timing_avoids_feedback_guard_in_tracking_simulation(retimed):
    from arm_mode_core import ModeCore
    from tool_trajectory_joy import ARM_JOINTS, ArmKinematics
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    loaded = load_path(SCRIPTS.parent/'config/tool2_trajectory.yaml', model)
    c = ModeCore([model.limits[j] for j in ARM_JOINTS], [.12]*6,
                 joint_names=ARM_JOINTS, path_speeds=[.08, .04, .06, .06, .08, .08],
                 path_accelerations=[.08, .03, .06, .06, .08, .08],
                 path_jerks=[.30, .04, .15, .15, .30, .30],
                 path_feedback_leads=[.04, .015, .025, .025, .04, .04])
    measured = loaded['start'].copy()
    c.ready, c.focus = True, 'arm'
    c.feedback(measured, [0.]*6, 0.)
    c.mode = c.requested = 'tool_auto'
    durations = loaded['durations'] if retimed else [.02]*7
    assert c.path(c.epoch, 'tool2-test', loaded['waypoints'], 0., durations)
    # A simple 80-ms tracking lag exercises measured-velocity guards, not just
    # YAML validity. This is not a model of hardware stiction or box loading.
    for step in range(1, 30001):
        now = step*.02
        previous = measured.copy()
        measured += (np.asarray(c.target)-measured)*(.02/.08)
        c.feedback(measured, (measured-previous)/.02, now)
        c.tick(now, .02)
        if c.mode == 'fault' or c.last_execution_result is not None:
            break
    if retimed:
        assert c.last_execution_result == 'succeeded', c.reason
        np.testing.assert_allclose(measured, loaded['waypoints'][-1], atol=.01)
    else:
        assert c.mode == 'fault'
        assert 'moving outward near soft limit: wrist_pitch_joint' in c.reason


def test_tool2_current_limit_covers_measured_approach_peak_and_retains_release_gap():
    hardware = yaml.safe_load((SCRIPTS.parent/'config/hardware.yaml').read_text())
    wrist = hardware['joints']['wrist_pitch_joint']
    assert wrist['current_limit_a'] == pytest.approx(2.5)
    assert wrist['current_limit_a'] > 2.33
    assert wrist['current_release_threshold_a'] == pytest.approx(1.8)
    assert wrist['current_release_threshold_a'] < wrist['current_limit_a']


def test_rmd_current_protection_aborts_managed_path_immediately():
    from arm_control_mode_manager import ControlModeManager
    core = Mock(mode='tool_auto', execution_id='tool2-run')
    node = SimpleNamespace(core=core)
    message = String(data=json.dumps(dict(
        joint='wrist_pitch_joint', current_a=2.51, current_protection=True)))
    ControlModeManager.on_rmd_diagnostic(node, message)
    core.abort_path_to_hold.assert_called_once()
    reason = core.abort_path_to_hold.call_args.args[0]
    assert reason == 'RMD current protection active: wrist_pitch_joint at 2.510 A'


@pytest.mark.parametrize('payload', [
    'not-json', '{}', '{"current_protection": false}',
    '{"current_protection": true, "current_a": "bad"}',
])
def test_rmd_diagnostic_non_active_or_malformed_input_does_not_abort(payload):
    from arm_control_mode_manager import ControlModeManager
    core = Mock(mode='tool_auto', execution_id='tool2-run')
    node = SimpleNamespace(core=core)
    ControlModeManager.on_rmd_diagnostic(node, String(data=payload))
    if '"current_protection": true' in payload:
        core.abort_path_to_hold.assert_called_once()
        assert core.abort_path_to_hold.call_args.args[0] == \
            'RMD current protection active: unknown RMD joint'
    else:
        core.abort_path_to_hold.assert_not_called()


@pytest.mark.parametrize('limits', [[.01]*5, [.01]*5+[0.], [.01]*5+[float('nan')],
                                    [.01]*5+[True], [.01]*5+[.13]])
def test_invalid_segment_speed_limits_are_rejected(tmp_path, limits):
    path = write_path(tmp_path)
    data = yaml.safe_load(path.read_text())
    data.update(timing_mode='joint_limits', joint_speed_limits_rad_s=limits)
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='joint_speed_limits_rad_s'):
        load_path(path, Mock())


def test_tool2_commented_close_and_return_connects_to_ready(tmp_path):
    path = SCRIPTS.parent/'config/tool2_trajectory.yaml'
    # Exercise the exact disabled configuration without enabling the live file.
    text = path.read_text()
    for line in ('after_path:', '  ee_close_rad: 1.251728', '  return_to_start: true'):
        assert '# '+line in text
        text = text.replace('# '+line, line)
    candidate = tmp_path/'tool2_full_sequence.yaml'
    candidate.write_text(text)
    model = Mock()
    model.within_limits.return_value = True
    loaded = load_path(candidate, model)
    assert loaded['after_path'] == dict(ee_close_rad=1.251728, return_to_start=True)
    from tool_trajectory_joy import ARM_JOINTS
    node = SimpleNamespace(paths={2: loaded}, path_id=2, sequence_phase='approach',
                           full_state=dict(zip(ARM_JOINTS, loaded['waypoints'][-1])),
                           start_tolerance=.03, max_start_delta=.1, approach_speed=.05,
                           report=Mock(), begin=Mock(), reset=Mock())
    ToolTrajectoryJoy.path_completed(node)
    assert node.sequence_phase == 'closing'
    assert node.begin.call_args.args[1]['ee_target_rad'] == 1.251728
    ToolTrajectoryJoy.path_completed(node)
    assert node.sequence_phase == 'returning'
    returning = node.begin.call_args.args[1]
    np.testing.assert_array_equal(returning['waypoints'],
                                  [*loaded['waypoints'][-2::-1], loaded['start']])
    ToolTrajectoryJoy.path_completed(node)
    node.reset.assert_called_once()


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


def test_preserved_mixed_approach_keeps_geometry_and_still_checks_collisions():
    start = np.array([0., .1, 0., 0., 0., 0.])
    goal = np.array([0., .04, .03, 0., 0., 0.])
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold-epoch'}),
        full_state=dict(zip(ARM_JOINTS, start)), shoulder_isolation_enabled=True,
        collision=Mock(), mode=Mock(), collision_timeout=10., collision_step=.01,
        start_tolerance=.03, report=Mock(), begin_failed=Mock())
    path = dict(start=start, waypoints=[goal], durations=[3.], total=3.,
                start_error=0., cartesian_line=False, preserve_joint_path=True)
    with patch('tool_trajectory_joy.isolate_shoulder_zero_crossings') as isolate:
        ToolTrajectoryJoy.begin(node, 0, path)
    isolate.assert_not_called()
    node.begin_failed.assert_not_called()
    assert node.path is path
    assert node.stage == 'checking'
    assert len(node.requests) >= 7
    node.collision.call_async.assert_called_once_with(node.requests[0])


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


@pytest.mark.parametrize('path_id,button_index', [(0, 2), (1, 1), (2, 0)])
def test_each_l2_face_button_selects_its_path_once(path_id, button_index):
    buttons = [0]*13
    buttons[button_index] = 1
    request, held = requested_path([0., 0., -1.], buttons, [False]*3)
    assert request == path_id
    assert requested_path([0., 0., -1.], buttons, held)[0] is None


def test_simultaneous_path_buttons_are_ignored():
    request, held = requested_path([0., 0., -1.], [0, 1, 1, 0, 0], [False]*3)
    assert request is None and held == [True, True, False]


def test_l2_plus_l1_does_not_select_tool2():
    buttons = [0]*13
    buttons[4] = 1
    request, held = requested_path([0., 0., -1.], buttons, [False]*3)
    assert request is None and held == [False]*3


def test_triangle_circle_x_select_paths_and_square_is_reserved_for_back():
    buttons = [0]*13
    for index, expected in ((2, 0), (1, 1), (0, 2), (3, None)):
        buttons[index] = 1
        assert requested_path([0., 0., -1.], buttons, [False]*3)[0] == expected
        buttons[index] = 0


def test_l2_plus_square_requests_back_once():
    buttons = [0]*13
    buttons[3] = 1
    request, held = requested_back([0., 0., -1.], buttons, False)
    assert request is True and held is True
    assert requested_back([0., 0., -1.], buttons, held)[0] is False
    assert requested_back([0., 0., 1.], buttons, held) == (False, False)


def test_ui_docking_true_is_a_command_and_false_does_not_move():
    node = SimpleNamespace(on_selected_tool=Mock())
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=True), 0, True)
    ToolTrajectoryJoy.on_ui_request(node, SimpleNamespace(data=False), 0, True)
    assert node.on_selected_tool.call_count == 1
    assert node.on_selected_tool.call_args.args[0].data == 0


@pytest.mark.parametrize('tool_id', [0, 1, 2])
def test_ui_selection_stages_home_back_without_latch_short_circuit(tool_id):
    node = SimpleNamespace(stage='idle', paths={0: {}, 1: {}, 2: {}},
                           pre_dock_targets={}, healthy_hold=Mock(),
                           begin_back=Mock(), begin_failed=Mock(), report=Mock(),
                           docked_state=True, pending_tool_request=None)
    ToolTrajectoryJoy.on_selected_tool(node, SimpleNamespace(data=tool_id))
    assert node.pending_tool_request == (tool_id, True, False)
    node.begin_back.assert_called_once_with(force_home=True)
    node.begin_failed.assert_not_called()


@pytest.mark.parametrize('stage,tool_id,fault', [
    ('executing', 0, False), ('idle', 99, False), ('idle', 0, True)])
def test_ui_invalid_busy_or_fault_does_not_start(stage, tool_id, fault):
    node = SimpleNamespace(stage=stage, paths={0: {}}, pre_dock_targets={},
                           healthy_hold=Mock(side_effect=RuntimeError('fault') if fault else None),
                           begin_back=Mock(), begin_failed=Mock(), report=Mock())
    ToolTrajectoryJoy.on_selected_tool(node, SimpleNamespace(data=tool_id))
    node.begin_back.assert_not_called()


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


def test_route_poses_load_home_and_back_while_ignoring_tcp_ready():
    semantic = '''<robot name="arm">
      <group_state name="home" group="arm">%s</group_state>
      <group_state name="back" group="arm">%s</group_state>
      <group_state name="tcp_ready" group="arm">%s</group_state>
    </robot>''' % tuple(
        ''.join(f'<joint name="{name}" value="{value}"/>'
                for name, value in zip(ARM_JOINTS, values))
        for values in ([0.]*6, [1.]*6, [.5]*6))
    poses = load_route_poses(semantic)
    assert set(poses) == {'home', 'back'}
    np.testing.assert_array_equal(poses['home'], np.zeros(6))
    np.testing.assert_array_equal(poses['back'], np.ones(6))


def test_planned_leg_uses_only_moveit_points_and_reaches_requested_goal():
    trajectory = SimpleNamespace(joint_names=list(ARM_JOINTS), points=[
        SimpleNamespace(positions=[.2]*6,
                        time_from_start=SimpleNamespace(sec=1, nanosec=0)),
        SimpleNamespace(positions=[.4]*6,
                        time_from_start=SimpleNamespace(sec=2, nanosec=0)),
    ])
    waypoints, durations = planned_leg(trajectory, np.zeros(6), np.ones(6)*.4)
    np.testing.assert_array_equal(waypoints, [[.2]*6, [.4]*6])
    assert durations == [1., 1.]


def test_dense_linear_moveit_samples_collapse_and_preserve_total_duration():
    start = np.zeros(6)
    waypoints = [np.full(6, index/1200.) for index in range(1, 1201)]
    durations = [.02]*1200
    reduced, timing = simplify_timed_waypoints(start, waypoints, durations)
    assert len(reduced) == 1
    np.testing.assert_allclose(reduced[-1], np.ones(6))
    assert timing == pytest.approx([24.])


def test_moveit_simplification_retains_joint_space_corner():
    start = np.zeros(6)
    waypoints = [np.array([.1, 0, 0, 0, 0, 0]),
                 np.array([.2, 0, 0, 0, 0, 0]),
                 np.array([.2, .1, 0, 0, 0, 0])]
    reduced, timing = simplify_timed_waypoints(
        start, waypoints, [.5, .5, 1.], max_waypoints=2)
    assert len(reduced) == 2
    np.testing.assert_allclose(reduced[0], waypoints[1])
    assert sum(timing) == pytest.approx(2.)


def test_pre_dock_targets_load_reviewed_arm_world_tcp_poses(tmp_path):
    target_file = tmp_path/'targets.yaml'
    target_file.write_text(yaml.safe_dump({'targets': {
        'tool0': dict(frame_id='arm_world', target_link='tcp_link',
                      position_m=[-.4, 0., .2], orientation_xyzw=[0., 1., 0., 0.],
                      executable=True),
        'tool1': dict(frame_id='arm_world', target_link='tcp_link',
                      position_m=[-.3, .09, .16], orientation_xyzw=[0., 0., 0., 1.],
                      executable=False)}}))
    targets = load_pre_dock_targets(target_file)
    assert set(targets) == {0}
    np.testing.assert_allclose(targets[0]['position'], [-.4, 0., .2])
    np.testing.assert_allclose(targets[0]['approach_axis'], [0., 0., -1.])


def test_pre_dock_targets_prefer_reviewed_joint_goal(tmp_path):
    target_file = tmp_path/'targets.yaml'
    goal = [.1, .2, .3, -.4, .5, -.6]
    target_file.write_text(yaml.safe_dump({'targets': {
        'tool0': dict(joint_names=list(ARM_JOINTS), joint_goal_rad=goal,
                      executable=True)}}))
    targets = load_pre_dock_targets(target_file)
    np.testing.assert_allclose(targets[0]['joint_goal'], goal)
    assert set(targets[0]) == {'joint_goal', 'retreat_distance', 'retreat_axis'}
    assert targets[0]['retreat_distance'] == pytest.approx(.030)
    np.testing.assert_allclose(targets[0]['retreat_axis'], [0., 0., -1.])


def test_pre_dock_targets_prefer_explicit_pre_over_final_capture(tmp_path):
    target_file = tmp_path/'targets.yaml'
    pre = [.1, .2, .3, -.4, .5, -.6]
    final = [.6, .5, .4, -.3, .2, -.1]
    target_file.write_text(yaml.safe_dump({'targets': {
        'tool0': dict(joint_names=list(ARM_JOINTS), pre_joint_goal_rad=pre,
                      joint_goal_rad=final, executable=True)}}))
    targets = load_pre_dock_targets(target_file)
    np.testing.assert_allclose(targets[0]['joint_goal'], pre)


def test_source_targets_use_limit_valid_tool1_observation_and_exact_final():
    from tool_trajectory_joy import ArmKinematics
    source = Path(__file__).resolve().parents[1] / 'config/cartesian_targets.yaml'
    targets = load_pre_dock_targets(source)
    assert set(targets) == {0, 1, 2}
    np.testing.assert_allclose(targets[0]['joint_goal'], [
        -math.pi, .6138322659684726, 1.3924236559357874,
        -1.2140509850766783, 0., 0.])
    np.testing.assert_allclose(targets[0]['transit_joint_goal'], [
        3.1001751723166735, 0.6403612785882697, 1.209862232156147,
        -1.427330240975674, -0.0030679615759781065, -0.021475731030191625])
    assert 'blocked_reason' not in targets[1]
    assert targets[1]['retreat_mode'] == 'reverse_docking_path'
    np.testing.assert_allclose(targets[2]['joint_goal'], [
        -2.13180646665643, .8056224119758513, 1.2458380150140123,
        -1.4895040589039743, -.5463633803567056, -.3205989853677441])
    data = yaml.safe_load(source.read_text())['targets']
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    assert model.within_limits(data['tool0']['pre_joint_goal_rad'], .02)
    assert model.within_limits(data['tool1']['final_joint_goal_rad'], .02)
    assert data['tool1']['executable']
    assert model.within_limits(targets[1]['joint_goal'], .02)
    assert model.within_limits(data['tool2']['pre_joint_goal_rad'], .02)
    assert targets[0]['retreat_mode'] == 'reverse_docking_path'
    assert data['tool0']['required_final_motion'] == 'axial_sampled_path'
    assert 'rack_removal_ee_command' not in data['tool0']
    assert data['tool2']['grasp_stage_allowed_contacts'] == [
        ['rack_left', 'supply_box'], ['rack_right', 'supply_box']]


def test_nearest_base_equivalent_avoids_full_turn_from_back():
    limits = {'base_joint': (-3.503, 3.614)}
    goal = np.array([3.123184884, .1, .2, -.3, 0., 0.])
    result = nearest_base_equivalent(
        goal, np.array([-math.pi, .1, .2, -.3, 0., 0.]), limits, .02)
    assert result[0] == pytest.approx(3.123184884-2.*math.pi)
    np.testing.assert_array_equal(result[1:], goal[1:])


def test_face_button_requests_complete_autonomous_docking_sequence():
    node = SimpleNamespace(
        combos=[False]*3, back_combo=False, l2_axis=2, l2_threshold=-.5,
        first_button=2, second_button=1, third_button=0, back_button=3,
        request_path=Mock(), request_back=Mock())
    ToolTrajectoryJoy.on_joy(
        node, SimpleNamespace(axes=[0., 0., -1.], buttons=[0, 0, 1, 0]))
    node.request_path.assert_called_once_with(0, desired_docked=True)
    node.request_back.assert_not_called()


@pytest.mark.parametrize('ready_hold', [True, False])
def test_single_tool_selection_stages_only_from_healthy_hold(ready_hold):
    node = SimpleNamespace(stage='idle', paths={0: {}},
        back_is_ready=Mock(return_value=(False, 'not at BACK')),
        healthy_hold=Mock(), begin_back=Mock(), begin=Mock(),
        begin_failed=Mock(), report=Mock(), pending_tool_request=None)
    if not ready_hold:
        node.healthy_hold.side_effect = RuntimeError('hardware fault')
    ToolTrajectoryJoy.request_path(node, 0, True)
    node.begin.assert_not_called()
    if ready_hold:
        assert node.pending_tool_request == (0, True, False)
        node.begin_back.assert_called_once()
    else:
        assert node.pending_tool_request is None
        node.begin_back.assert_not_called()


def test_verified_staging_continues_selected_tool_without_second_button():
    back = np.ones(6)*.2
    node = SimpleNamespace(route_kind='back', pending_tool_request=(0, True, False),
        full_state=dict(zip(ARM_JOINTS, back)), route_poses={'back': back},
        back_tolerance=.03, status={'epoch': 'back'},
        continue_selected_tool=Mock(), report=Mock(), reset=Mock())
    ToolTrajectoryJoy.path_completed(node)
    node.continue_selected_tool.assert_called_once()
    node.reset.assert_not_called()


def test_selected_tool_is_consumed_before_continuation():
    node = SimpleNamespace(pending_tool_request=(0, True, False),
        report=Mock(), request_path=Mock())
    node.reset = lambda: ToolTrajectoryJoy.reset(node)
    ToolTrajectoryJoy.continue_selected_tool(node)
    assert node.pending_tool_request is None
    assert node.stage == 'idle'
    node.request_path.assert_called_once_with(0, True, nearest_entry=False)


def test_failed_staging_clears_selected_tool():
    node = SimpleNamespace(pending_tool_request=(0, True, False), report=Mock())
    node.reset = lambda: ToolTrajectoryJoy.reset(node)
    ToolTrajectoryJoy.begin_failed(node, 'route collision')
    assert node.pending_tool_request is None
    assert node.stage == 'idle'


def test_ui_staging_visits_home_even_when_already_at_back():
    back = np.ones(6)*.2
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold'}),
        planner=Mock(), model=Mock(), route_poses={
            'home': np.ones(6)*.1, 'back': back},
        back_tolerance=.03, full_state=dict(zip(ARM_JOINTS, back)),
        _request_route_leg=Mock(), continue_selected_tool=Mock())
    ToolTrajectoryJoy.begin_back(node, force_home=True)
    assert node.route_labels == ['home', 'back']
    node.continue_selected_tool.assert_not_called()
    node._request_route_leg.assert_called_once()


def test_automatic_return_plans_directly_to_back_without_home():
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold'}),
        planner=Mock(), model=Mock(), route_poses={
            'home': np.ones(6)*.1, 'back': np.ones(6)*.2},
        back_tolerance=.03,
        full_state=dict(zip(ARM_JOINTS, np.zeros(6))),
        _request_route_leg=Mock())
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True
    ToolTrajectoryJoy.begin_back(node, via_home=False)
    assert node.route_labels == ['back']
    np.testing.assert_allclose(node.route_goals, [np.ones(6)*.2])
    node._request_route_leg.assert_called_once()


def test_home_route_uses_moveit_from_current_pose():
    measured = np.array([-math.pi, 1.35, 1.47, -1.49, 0., 0.])
    home = np.array([0., 1.35, 1.47, -1.49, 0., 0.])
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold'}), planner=Mock(),
        model=Mock(), route_poses={'home': home},
        full_state=dict(zip(ARM_JOINTS, measured)), _request_route_leg=Mock())
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True
    ToolTrajectoryJoy.begin_named_route(node, 'home')
    assert node.route_kind == 'home'
    assert node.route_labels == ['home']
    np.testing.assert_array_equal(node.route_start, measured)
    np.testing.assert_array_equal(node.route_goals, [home])
    node._request_route_leg.assert_called_once()


@pytest.mark.parametrize('route,path_id,expected', [
    ('home', None, 'HOME route'),
    ('back', None, 'BACK staging'),
    ('dock_retreat', None, 'docking retreat'),
    ('tool', 0, 'tool0 (gripper)'),
    ('tool', 1, 'tool1 (drill)'),
    ('tool', 2, 'tool2 (supply_box)'),
])
def test_operation_label_separates_named_routes_from_tool_ids(route, path_id, expected):
    node = SimpleNamespace(route_kind=route, path_id=path_id)
    assert ToolTrajectoryJoy.operation_label(node, path_id) == expected


def test_home_back_route_starts_at_interrupted_measured_pose():
    measured = np.array([.37, 1.31, 1.42, -1.37, .04, -.03])
    home, back = np.ones(6)*.1, np.ones(6)*.2
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold-current'}),
        planner=Mock(), model=Mock(), route_poses={'home': home, 'back': back},
        back_tolerance=.03, full_state=dict(zip(ARM_JOINTS, measured)),
        planning_attempts=3, planning_timeout=8., planning_velocity_scale=.08,
        planning_acceleration_scale=.4, report=Mock())
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True
    node._request_route_leg = lambda: ToolTrajectoryJoy._request_route_leg(node)
    ToolTrajectoryJoy.begin_back(node)
    np.testing.assert_allclose(node.route_start, measured)
    request = node.planner.call_async.call_args.args[0].motion_plan_request
    start = dict(zip(request.start_state.joint_state.name,
                     request.start_state.joint_state.position))
    np.testing.assert_allclose([start[name] for name in ARM_JOINTS], measured)
    assert node.route_labels == ['home', 'back']


@pytest.mark.parametrize('changed_epoch,moved,expected', [
    (True, False, 'ownership changed during route planning'),
    (False, True, 'robot moved during route planning'),
])
def test_stale_route_plan_is_discarded_before_future_is_consumed(
        changed_epoch, moved, expected):
    future = Mock()
    future.done.return_value = True
    node = SimpleNamespace(
        stage='planning_route', original_epoch='original',
        healthy_hold=Mock(return_value={
            'epoch': 'changed' if changed_epoch else 'original'}),
        unchanged=Mock(return_value=not moved), snapshot_change_reason=Mock(
            return_value='shoulder_joint changed'),
        started=0., planning_timeout=8., future=future, fail=Mock())
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr('tool_trajectory_joy.time.monotonic', lambda: 1.)
        ToolTrajectoryJoy.tick(node)
    node.fail.assert_called_once()
    assert expected in node.fail.call_args.args[0]
    future.result.assert_not_called()


def test_fresh_hold_at_back_revalidates_without_historical_latch():
    back = np.array([-.2, .3, .4, -.5, .1, -.1])
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'new-hold'}),
        full_state=dict(zip(ARM_JOINTS, back)),
        route_poses={'back': back}, back_tolerance=.03,
        back_ready=False, back_ready_epoch=None)
    ready, reason = ToolTrajectoryJoy.back_is_ready(node, 10.)
    assert ready and reason == ''
    assert node.back_ready and node.back_ready_epoch == 'new-hold'


def test_back_request_at_back_is_a_no_motion_success():
    back = np.array([-.2, .3, .4, -.5, .1, -.1])
    node = SimpleNamespace(
        healthy_hold=Mock(return_value={'epoch': 'hold'}),
        full_state=dict(zip(ARM_JOINTS, back)), route_poses={'back': back},
        back_tolerance=.03, back_ready=False, back_ready_epoch=None,
        planner=Mock(), report=Mock(), reset=Mock())
    ToolTrajectoryJoy.begin_back(node)
    assert node.back_ready and node.back_ready_epoch == 'hold'
    node.planner.call_async.assert_not_called()
    node.report.assert_called_once_with(
        'completed', 'already at BACK; no motion planned, select tool0/1/2')
    node.reset.assert_called_once_with()


def test_verified_back_tool_request_plans_only_pre_dock_leg():
    home, back, pre_dock = np.ones(6)*.1, np.ones(6)*.2, np.ones(6)*.3
    node = SimpleNamespace(
        planner=Mock(), model=Mock(), margin=.02,
        paths={1: dict(waypoints=[np.ones(6)*.25])},
        pre_dock_targets={1: dict(position=np.array([-.3, .09, .16]),
                                  approach_axis=np.array([1., 0., 0.]))},
        route_poses={'home': home, 'back': back},
        full_state=dict(zip(ARM_JOINTS, np.zeros(6))),
        planning_attempts=3, planning_timeout=8., planning_velocity_scale=.08,
        planning_acceleration_scale=.4, report=Mock())
    node._request_route_leg = lambda: ToolTrajectoryJoy._request_route_leg(node)
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True
    node.model.solve_axis.return_value = pre_dock
    ToolTrajectoryJoy.request_route_plan(
        node, 1, np.zeros(6), {'epoch': 'hold'})
    assert node.stage == 'planning_route'
    assert node.route_index == 0
    np.testing.assert_array_equal(node.route_goals, [pre_dock])
    node.model.solve_axis.assert_called_once()
    request = node.planner.call_async.call_args.args[0].motion_plan_request
    assert [constraint.position for constraint in
            request.goal_constraints[0].joint_constraints] == pytest.approx(pre_dock)
    node.report.assert_called_once()


def test_reviewed_joint_goal_is_moveit_goal_with_nearest_base_equivalent():
    goal = np.array([3.123184884, .1, .2, -.3, 0., 0.])
    limits = {name: (-4., 4.) for name in ARM_JOINTS}
    limits['base_joint'] = (-3.503, 3.614)
    node = SimpleNamespace(
        planner=Mock(), model=Mock(limits=limits), margin=.02,
        pre_dock_targets={0: {'joint_goal': goal}},
        full_state=dict(zip(ARM_JOINTS, np.zeros(6))),
        planning_attempts=3, planning_timeout=8., planning_velocity_scale=.08,
        planning_acceleration_scale=.4, report=Mock())
    node._request_route_leg = lambda: ToolTrajectoryJoy._request_route_leg(node)
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True
    measured = np.array([-math.pi, 0., 0., 0., 0., 0.])
    ToolTrajectoryJoy.request_route_plan(node, 0, measured, {'epoch': 'hold'})
    assert node.route_goals[0][0] == pytest.approx(goal[0]-2.*math.pi)
    node.model.solve_axis.assert_not_called()
    request = node.planner.call_async.call_args.args[0].motion_plan_request
    assert [constraint.position for constraint in
            request.goal_constraints[0].joint_constraints] == pytest.approx(
                node.route_goals[0])


def test_reviewed_joint_goal_can_route_via_safe_transit():
    transit = np.array([3.10, .64, 1.21, -1.43, 0., -.02])
    goal = np.array([-3.143, .40, 1.46, -.94, -.04, -.03])
    limits = {name: (-4., 4.) for name in ARM_JOINTS}
    limits['base_joint'] = (-3.503, 3.614)
    node = SimpleNamespace(
        planner=Mock(), model=Mock(limits=limits), margin=.02,
        pre_dock_targets={0: {'joint_goal': goal, 'transit_joint_goal': transit}},
        full_state=dict(zip(ARM_JOINTS, np.zeros(6))),
        planning_attempts=3, planning_timeout=8., planning_velocity_scale=.08,
        planning_acceleration_scale=.4, report=Mock())
    node._request_route_leg = lambda: ToolTrajectoryJoy._request_route_leg(node)
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True

    ToolTrajectoryJoy.request_route_plan(node, 0, np.zeros(6), {'epoch': 'hold'})

    assert len(node.route_goals) == 2
    np.testing.assert_allclose(node.route_goals[0], transit)
    expected_goal = goal.copy()
    expected_goal[0] += 2.*math.pi
    np.testing.assert_allclose(node.route_goals[1], expected_goal)
    assert node.route_labels == ['tool0 (gripper) safe transit',
                                 'tool0 (gripper) pre-dock']
    request = node.planner.call_async.call_args.args[0].motion_plan_request
    assert [constraint.position for constraint in
            request.goal_constraints[0].joint_constraints] == pytest.approx(transit)


def test_already_at_pre_dock_validates_hold_without_replanning_transit():
    measured = np.array([-3.143127, .331263, 1.488591, -.842994, -.041417, -.032214])
    goal = measured.copy()
    goal[0] = -math.pi
    transit = np.array([3.10, .64, 1.21, -1.43, 0., -.02])
    limits = {name: (-4., 4.) for name in ARM_JOINTS}
    node = SimpleNamespace(
        planner=Mock(), model=Mock(limits=limits), margin=.02,
        pre_dock_targets={0: {'joint_goal': goal, 'transit_joint_goal': transit}},
        full_state=dict(zip(ARM_JOINTS, measured)), start_tolerance=.03,
        report=Mock(), begin=Mock())
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = True

    ToolTrajectoryJoy.request_route_plan(node, 0, measured, {'epoch': 'hold'})

    node.planner.call_async.assert_not_called()
    node.begin.assert_called_once()
    path = node.begin.call_args.args[1]
    np.testing.assert_allclose(path['start'], measured)
    np.testing.assert_allclose(path['waypoints'], [measured])
    assert node.route_labels == ['tool0 (gripper) pre-dock hold validation']


def test_reviewed_joint_goal_outside_margin_is_blocked_before_moveit():
    goal = np.array([2.85, .67, 1.581792, -1.4, 0., 0.])
    limits = {name: (-4., 4.) for name in ARM_JOINTS}
    limits['base_joint'] = (-3.503, 3.614)
    limits['elbow_joint'] = (-1.4960962, 1.5779521)
    node = SimpleNamespace(
        planner=Mock(), model=Mock(limits=limits), margin=.02,
        pre_dock_targets={1: {'joint_goal': goal}}, paths={},
        full_state=dict(zip(ARM_JOINTS, np.zeros(6))))
    node.planner.service_is_ready.return_value = True
    node.model.within_limits.return_value = False
    with pytest.raises(ValueError, match='elbow_joint=.*allowed='):
        ToolTrajectoryJoy.request_route_plan(
            node, 1, np.array([-math.pi, 0., 0., 0., 0., 0.]), {'epoch': 'hold'})
    node.planner.call_async.assert_not_called()


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


@pytest.mark.parametrize('module', ['tool_trajectory_joy', 'tool_trajectory_joy_test'])
@pytest.mark.parametrize('validation', [False, True])
def test_path_snapshot_ignores_ee_position_change(module, validation):
    from importlib import import_module
    ToolTrajectoryJoy = import_module(module).ToolTrajectoryJoy
    names = ['base_joint', 'shoulder_joint', 'elbow_joint',
             'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint']
    node = SimpleNamespace(snapshot=dict.fromkeys(names, 0.),
                           full_state=dict.fromkeys(names, 0.),
                           validation_rmd_drift=.005)
    for position in (.019, .021476, -20., 20.):
        node.full_state['ee_joint'] = position
        assert ToolTrajectoryJoy.unchanged(node, validation=validation)
    node.full_state['shoulder_joint'] = .006
    assert not ToolTrajectoryJoy.unchanged(node, validation=validation)


def test_tool0_retreat_uses_displaced_current_pose_and_orientation():
    from tool_trajectory_joy import ArmKinematics
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    # Real feedback sent after docking; do not require the taught endpoint.
    seed = np.array([-3.143127, .114843, 1.146507, -1.196598, .001534, -.009204])
    n = SimpleNamespace(
        healthy_hold=Mock(), path_id=0, docking_retreat_authorized=True,
        advance_plan=None,  # reverse taught path is intentionally unnecessary
        pre_dock_targets=load_pre_dock_targets(SCRIPTS.parent/'config/cartesian_targets.yaml'),
        full_state=dict(zip(ARM_JOINTS, seed)), model=model, margin=.02,
        report=Mock(), begin=Mock(), return_home_after_back=True)
    # Exercise the optional current-axis mode independently of the taught-path default.
    n.pre_dock_targets[0]['retreat_mode'] = 'current_tcp_axis'
    n.pre_dock_targets[0]['retreat_checkpoints'] = [.0129035032851267, .0266983032300609]
    ToolTrajectoryJoy.begin_docking_retreat(n)
    plan = n.begin.call_args.args[1]
    origin = model.fk(seed)
    np.testing.assert_allclose(plan['start'], seed)
    offsets = sorted([.002*i for i in range(1, 26)] +
                     n.pre_dock_targets[0]['retreat_checkpoints'])
    assert len(plan['waypoints']) == len(offsets) == 27
    previous = seed
    for i, (q, duration) in enumerate(zip(plan['waypoints'], plan['durations']), 1):
        expected = origin.copy()
        expected[:3, 3] -= origin[:3, 2]*offsets[i-1]
        residual = model.pose_error(q, expected)
        assert np.linalg.norm(residual[:3]) < 1e-4
        assert np.linalg.norm(residual[3:]) < 1e-3
        assert model.within_limits(q, .02)
        assert np.max(np.abs(q-previous))/duration <= .025+1e-10
        distance = np.linalg.norm(model.fk(q)[:3, 3]-model.fk(previous)[:3, 3])
        assert distance/duration <= .005+1e-10
        previous = q
    assert plan['cartesian_line']
    assert plan['stop_at_waypoints'] == [False]*26+[True]
    assert plan['allowed_collision_pairs'] == frozenset({('tool_case_1', 'wrist_link_yaw')})
    assert n.return_home_after_back
    assert n.route_kind == 'dock_retreat'
    n.begin_back = Mock()
    n.begin_return_waypoint = Mock()
    ToolTrajectoryJoy.path_completed(n)
    np.testing.assert_array_equal(n.begin_return_waypoint.call_args.args[0],
                                  n.pre_dock_targets[0]['return_joint_goal'])
    n.begin_back.assert_not_called()
    # Failed IK still prevents the path from reaching execution.
    n.begin.reset_mock()
    with patch.object(model, 'solve', side_effect=ValueError('unreachable')):
        with pytest.raises(RuntimeError, match='retreat IK point'):
            ToolTrajectoryJoy.begin_docking_retreat(n)
    n.begin.assert_not_called()


@pytest.mark.parametrize('automatic', [False, True])
def test_back_completion_only_continues_home_for_automatic_return(automatic):
    n = SimpleNamespace(route_kind='back', full_state=dict.fromkeys(ARM_JOINTS, 0.),
                        route_poses={'back': np.zeros(6)}, back_tolerance=.01,
                        status={'epoch': 'test'}, return_home_after_back=automatic,
                        report=Mock(), reset=Mock(), begin_named_route=Mock())
    ToolTrajectoryJoy.path_completed(n)
    if automatic:
        n.begin_named_route.assert_called_once_with('home')
        n.reset.assert_not_called()
    else:
        n.begin_named_route.assert_not_called()
        n.reset.assert_called_once()


@pytest.mark.parametrize('error,accepted', [(0., True), (.1, False)])
def test_return_waypoint_verified_before_back(error, accepted):
    goal = np.array([-3.143127, .747175, 1.300096, -1.363102, .003068, -.004602])
    measured = goal.copy()
    measured[1] += error
    n = SimpleNamespace(route_kind='dock_return_waypoint', route_goals=[goal],
                        full_state=dict(zip(ARM_JOINTS, measured)), back_tolerance=.02,
                        report=Mock(), begin_back=Mock())
    if accepted:
        ToolTrajectoryJoy.path_completed(n)
        n.begin_back.assert_called_once_with(via_home=False)
    else:
        with pytest.raises(RuntimeError, match='waypoint feedback mismatch'):
            ToolTrajectoryJoy.path_completed(n)
        n.begin_back.assert_not_called()


def test_return_waypoint_uses_live_start_and_moveit_planner():
    from tool_trajectory_joy import ArmKinematics
    model = ArmKinematics(
        SCRIPTS.parent.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        SCRIPTS.parent/'config/hardware.yaml')
    goal = load_pre_dock_targets(SCRIPTS.parent/'config/cartesian_targets.yaml')[0]['return_joint_goal']
    expected = [-3.143127, .747175, 1.300096, -1.363102, .003068, -.004602]
    np.testing.assert_array_equal(goal, expected)
    seed = np.array([-3.14, .25, 1.1, -1.2, 0., 0.])
    n = SimpleNamespace(healthy_hold=Mock(return_value={'epoch': 'return'}),
                        full_state=dict(zip(ARM_JOINTS, seed)), model=model, margin=.02,
                        planner=Mock(), _request_route_leg=Mock(), path_id=0)
    ToolTrajectoryJoy.begin_return_waypoint(n, goal)
    n._request_route_leg.assert_called_once()
    assert n.route_kind == 'dock_return_waypoint'
    np.testing.assert_allclose(n.route_goals[0], goal)
    np.testing.assert_array_equal(n.route_start, seed)
    assert n.original_epoch == 'return'
    n.planner.service_is_ready.return_value = False
    n._request_route_leg.reset_mock()
    with pytest.raises(RuntimeError, match='planner unavailable'):
        ToolTrajectoryJoy.begin_return_waypoint(n, goal)
    n._request_route_leg.assert_not_called()


@pytest.mark.parametrize('checkpoints', [[.02, .01], [0.], [.05], [float('nan')], [[.01]], [.01, .01]])
def test_invalid_retreat_checkpoints_rejected(tmp_path, checkpoints):
    path = tmp_path/'invalid_checkpoints.yaml'
    path.write_text(yaml.safe_dump({'targets': {'tool0': {
        'executable': True, 'joint_names': list(ARM_JOINTS),
        'pre_joint_goal_rad': [0.]*6, 'retreat_distance_m': .05,
        'retreat_checkpoints_m': checkpoints}}}))
    with pytest.raises(ValueError, match='invalid retreat checkpoints'):
        load_pre_dock_targets(path)



@pytest.mark.parametrize('path_id', [0, 1])
def test_fixed_joint_reverse_dispatches_without_using_cartesian_distance(path_id):
    targets = load_pre_dock_targets(SCRIPTS.parent/'config/cartesian_targets.yaml')
    # Production tool 1 is pending observation; isolate the common dispatcher.
    targets[1] = dict(targets[0])
    n = SimpleNamespace(healthy_hold=Mock(), path_id=path_id, docking_retreat_authorized=True,
                        pre_dock_targets=targets, begin_reverse_docking_retreat=Mock())
    ToolTrajectoryJoy.begin_docking_retreat(n)
    n.healthy_hold.assert_called_once()
    n.begin_reverse_docking_retreat.assert_called_once()


def test_missing_executed_approach_blocks_reverse_retreat():
    n = SimpleNamespace(advance_plan=None, begin=Mock())
    with pytest.raises(RuntimeError, match='completed docking path unavailable'):
        ToolTrajectoryJoy.begin_reverse_docking_retreat(n)
    n.begin.assert_not_called()
