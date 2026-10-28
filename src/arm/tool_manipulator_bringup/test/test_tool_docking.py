"""Offline contract tests: no ROS graph, controller, or hardware is started."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import sys

import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_docking_core import (load_settings, transform, bounded_target, pose_errors,
                               tag_pose, camera_normal_target, interpolate_direction)
from tool_docking_coordinator import ToolDockingCoordinator
from tool_trajectory_joy import ToolTrajectoryJoy, ARM_JOINTS


def test_tcp_offset_uses_user_axes_and_preserves_tcp_orientation():
    from tool_docking_core import tcp_offset_target
    mapping = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
    tag = transform([.1, .2, .3], [0., 0., 0., 1.])
    rotation = transform([0., 0., 0.], [0., 0., 1., 0.])[:3, :3]
    target = tcp_offset_target(tag, [-.05, 0., .07], rotation, mapping)
    np.testing.assert_allclose(target[:3, 3], [.1, .25, .23])
    np.testing.assert_array_equal(target[:3, :3], rotation)
    with pytest.raises(ValueError):
        tcp_offset_target(tag, [-.05, 0., .07], rotation, np.zeros((3, 3)))


def test_position_mode_requires_measured_size_but_not_actuator_timeout(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    raw = data['tools'][0]
    raw['enabled'] = True
    raw['tag_size_m'] = None
    path = tmp_path/'position.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='tag_size_m'):
        load_settings(path)
    raw['tag_size_m'] = .02
    path.write_text(yaml.safe_dump(data))
    cfg = load_settings(path)[0]
    assert cfg['alignment_mode'] == 'tag_tcp_position'
    assert cfg['stages'][-1]['xyz_m'] == [-.05, 0., .07]
    raw['actuation_enabled'] = True
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='without actuation'):
        load_settings(path)


def settings(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    raw = data['tools'][0]
    raw.update(enabled=True, tag_id=7, tag_size_m=.04, actuation_timeout_s=5.,
               alignment_mode='tag_tcp_stages')
    for stage in raw['stages']:
        stage.update(xyz_m=[0., 0., 0.], xyzw=[0., 0., 0., 1.])
    path = tmp_path/'docking.yaml'
    path.write_text(yaml.safe_dump(data))
    return load_settings(path)[0]


def node(tmp_path):
    cfg = settings(tmp_path)
    result = SimpleNamespace(
        cfg=cfg, active=True, path_id=0, stage='dock_observe', dock_stage=0,
        full_state=dict.fromkeys(ARM_JOINTS, 0.), velocity=np.zeros(6),
        feedback_sequence=1, observed_feedback=-1, observed_image=-1,
        observe_after_ns=0, stable=0, lock_goal=None, docked_state=None,
        docked_feedback_sequence=0, command_feedback_sequence=None,
        expected_docked=None, actuation_deadline=None,
        last_joy_time=1., deadman_timeout=.3, deadman_held=True,
        motion_enabled=True, last_preview_report=-math.inf,
        camera_reason='CameraInfo not received',
        tag_received=0, tag_accepted=0, tag_reason='no tag messages received',
        model=Mock(), report=Mock(), reset=Mock(), begin=Mock(), fail=Mock(),
        docking_command=Mock(), get_clock=Mock(), margin=.02,
        observation=(1, 0., np.eye(4)))
    result.get_clock.return_value.now.return_value.nanoseconds = 0
    result.model.fk.return_value = np.eye(4)
    result.docking_command.get_subscription_count.return_value = 1
    result.fresh_observation = lambda now: (result.observation[0], result.observation[2])
    instance = ToolDockingCoordinator.__new__(ToolDockingCoordinator)
    instance.__dict__.update(vars(result))
    instance.fresh_observation = lambda now: (instance.observation[0], instance.observation[2])
    return instance


def sample(n):
    n.feedback_sequence += 1
    n.observation = (n.observation[0]+1, 0., np.eye(4))
    ToolDockingCoordinator.observe(n, 1.)


def test_shipped_config_only_enables_tool1_camera_alignment_and_relative_advance():
    cfg = load_settings(ROOT/'config/tool_docking.yaml')
    assert set(cfg) == {0, 1}
    assert cfg[0]['tag_size_m'] == .020
    assert cfg[0]['alignment_mode'] == 'tag_tcp_position'
    assert cfg[1]['alignment_mode'] == 'camera_normal_advance'
    assert cfg[1]['advance_distance_m'] == .030
    assert cfg[1]['tag_family'] == 'tag36h11'
    assert cfg[1]['tag_id'] == 1
    assert not cfg[1]['actuation_enabled']
    assert cfg[1]['require_l2_hold'] is True
    assert 'targets' not in cfg[1]


@pytest.mark.parametrize('value', ['false', None, 0])
def test_invalid_l2_hold_policy_rejected(tmp_path, value):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][1]['require_l2_hold'] = value
    path = tmp_path/'invalid_hold.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='require_l2_hold'):
        load_settings(path)


def test_other_docking_configs_keep_continuous_l2_by_default(tmp_path):
    assert settings(tmp_path)['require_l2_hold'] is True


def test_enabling_unmeasured_config_is_rejected(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][0]['enabled'] = True
    data['tools'][0]['tag_size_m'] = None
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_settings(path)


def test_composed_tag_offset_rotates_in_tag_frame():
    base_tag = transform([1, 2, 3], [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    tag_tcp = transform([.1, 0, 0], [0, 0, 0, 1])
    np.testing.assert_allclose((base_tag @ tag_tcp)[:3, 3], [1, 2.1, 3])


def test_translation_and_rotation_are_bounded_independently():
    target = transform([.1, .1, .1], [0, 0, np.sin(.1), np.cos(.1)])
    result = bounded_target(np.eye(4), target, .002, .01)
    assert pose_errors(np.eye(4), result) == pytest.approx((.002, .01))


def test_real_pnp_recovers_synthetic_tag(tmp_path):
    cfg = settings(tmp_path)
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    h = cfg['tag_size_m']/2
    square = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    rotation, translation = np.array([.15, -.1, .03]), np.array([.01, .02, .3])
    pixels = cv2.projectPoints(square, rotation, translation, k, np.zeros(5))[0].reshape(4,2)
    detection = dict(hamming=0, decision_margin=100,
                     corners_px=[dict(u=u,v=v) for u,v in pixels])
    pose = tag_pose(detection, k, np.zeros(5), cfg)
    np.testing.assert_allclose(pose[:3,3], translation, atol=1e-7)
    np.testing.assert_allclose(pose[:3,:3], cv2.Rodrigues(rotation)[0], atol=1e-7)
    detection['hamming'] = 1
    with pytest.raises(ValueError, match='quality'):
        tag_pose(detection, k, np.zeros(5), cfg)


def test_pnp_rejects_tag_that_is_too_small_for_docking(tmp_path):
    cfg = settings(tmp_path)
    detection = dict(
        hamming=0, decision_margin=100,
        corners_px=[dict(u=u, v=v) for u, v in
                    [(319, 241), (321, 241), (321, 239), (319, 239)]])
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    with pytest.raises(ValueError, match='too small'):
        tag_pose(detection, k, np.zeros(5), cfg)


def test_duplicate_images_or_feedback_never_count_as_stability(tmp_path):
    n = node(tmp_path)
    sample(n)
    for _ in range(20):
        ToolDockingCoordinator.observe(n, 1.)
    assert n.stable == 1
    assert n.dock_stage == 0
    n.docking_command.publish.assert_not_called()


def test_three_stages_then_attach_command_and_fresh_docked_response(tmp_path):
    n = node(tmp_path)
    n.cfg['actuation_enabled'] = True
    for _ in range(n.cfg['stable_samples']*3-1):
        sample(n)
        n.docking_command.publish.assert_not_called()
    sample(n)
    assert n.stage == 'dock_actuate'
    assert [call.args[0].data for call in n.docking_command.publish.call_args_list] == [True]
    # A pre-command matching state is not completion; a fresh opposite state is not either.
    ToolDockingCoordinator.observe(n, 1.1)
    n.reset.assert_not_called()
    ToolDockingCoordinator.on_docked(n, SimpleNamespace(data=False))
    ToolDockingCoordinator.observe(n, 1.1)
    n.reset.assert_not_called()
    ToolDockingCoordinator.on_docked(n, SimpleNamespace(data=True))
    ToolDockingCoordinator.observe(n, 1.2)
    n.reset.assert_called_once()
    assert n.docking_command.publish.call_count == 1
    n.begin.assert_not_called()  # no speculative retreat


def test_release_uses_explicit_false_command_and_waits_for_false(tmp_path):
    n = node(tmp_path)
    n.cfg.update(actuation_enabled=True, desired_docked=False)
    for _ in range(n.cfg['stable_samples']*3):
        sample(n)
    assert [call.args[0].data for call in n.docking_command.publish.call_args_list] == [False]
    ToolDockingCoordinator.on_docked(n, SimpleNamespace(data=False))
    ToolDockingCoordinator.observe(n, 1.1)
    n.reset.assert_called_once()


def test_docking_only_never_sends_lock(tmp_path):
    n = node(tmp_path)
    for _ in range(n.cfg['stable_samples']*3):
        sample(n)
    n.reset.assert_called_once()
    n.docking_command.publish.assert_not_called()


@pytest.mark.parametrize('speed', [.011, .1])
def test_moving_feedback_cannot_confirm_docking(tmp_path, speed):
    n = node(tmp_path)
    n.velocity[:] = speed
    for _ in range(20):
        sample(n)
    assert n.stable == 0
    n.docking_command.publish.assert_not_called()


def test_correction_uses_existing_collision_checked_execution(tmp_path):
    n = node(tmp_path)
    n.cfg['targets'][0][:3,3] = [.01, 0., 0.]
    n.model.solve.return_value = np.ones(6)*.005
    n.model.within_limits.return_value = True
    n.model.pose_error.return_value = np.zeros(6)
    sample(n)
    path_id, plan = n.begin.call_args.args
    assert path_id == 0 and len(plan['waypoints']) == 1
    assert plan['durations'][0] >= .5
    n.docking_command.publish.assert_not_called()


def test_preview_reports_error_without_motion_or_lock(tmp_path):
    n = node(tmp_path)
    n.motion_enabled = False
    n.cfg['targets'][0][:3, 3] = [.01, 0., 0.]
    sample(n)
    n.report.assert_called_once()
    assert n.report.call_args.args[0] == 'preview'
    n.begin.assert_not_called()
    n.docking_command.publish.assert_not_called()


def test_excessive_ik_joint_step_is_rejected(tmp_path):
    n = node(tmp_path)
    n.cfg['targets'][0][:3,3] = [.01, 0., 0.]
    n.model.solve.return_value = np.ones(6)*.1
    with pytest.raises(RuntimeError, match='joint step'):
        sample(n)
    n.begin.assert_not_called()


def test_departure_during_lock_is_failure_even_after_ack(tmp_path):
    n = node(tmp_path)
    n.stage = 'dock_actuate'
    n.lock_goal = np.zeros(6)
    n.expected_docked = True
    n.command_feedback_sequence = 0
    n.docked_feedback_sequence = 1
    n.docked_state = True
    n.actuation_deadline = 5.
    n.velocity[:] = .1
    with pytest.raises(RuntimeError, match='departed'):
        ToolDockingCoordinator.observe(n, 1.)
    n.reset.assert_not_called()


def test_tag_timeout_is_checked_using_image_and_arrival_time(tmp_path):
    n = node(tmp_path)
    n.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
    n.observation = (500_000_000, 1., np.eye(4))
    with pytest.raises(RuntimeError, match='stale'):
        ToolDockingCoordinator.fresh_observation(n, 1.1)


def test_failure_cancels_arm_without_sending_an_opposite_actuation(tmp_path):
    n = node(tmp_path)
    n.status = {'mode': 'tool_auto'}
    n.execution_id = None
    n.cancel = Mock()
    n.cancel.service_is_ready.return_value = True
    with patch.object(ToolTrajectoryJoy, 'fail') as parent:
        ToolDockingCoordinator.fail(n, 'cancelled')
        parent.assert_called_once_with('cancelled')
    n.docking_command.publish.assert_not_called()
    n.cancel.call_async.assert_called_once()


def test_failure_after_manager_abort_does_not_overwrite_hold_reason(tmp_path):
    n = node(tmp_path)
    n.status = {'mode': 'hold', 'last_execution_result': 'aborted'}
    n.execution_id = 'already-aborted'
    n.cancel = Mock()
    with patch.object(ToolTrajectoryJoy, 'fail') as parent:
        ToolDockingCoordinator.fail(n, 'waypoint stalled')
        parent.assert_called_once_with('waypoint stalled')
    n.cancel.call_async.assert_not_called()


def test_non_docking_path_completion_keeps_old_behavior(tmp_path):
    n = node(tmp_path)
    n.active = False
    with patch.object(ToolTrajectoryJoy, 'path_completed') as parent:
        ToolDockingCoordinator.path_completed(n)
        parent.assert_called_once()
    n.docking_command.publish.assert_not_called()


def test_missing_tag_cancels_in_flight_correction_before_parent_tick(tmp_path):
    n = node(tmp_path)
    n.docking_started = 0.
    n.stage = 'executing'
    n.guard = Mock()
    n.fresh_observation = Mock(side_effect=RuntimeError('target tag missing'))
    with patch.object(ToolTrajectoryJoy, 'tick') as parent:
        ToolDockingCoordinator.tick(n)
        parent.assert_not_called()
    n.fail.assert_called_once_with('target tag missing')


def test_external_cancel_epoch_blocks_further_alignment(tmp_path):
    n = node(tmp_path)
    n.docking_started = 0.
    n.status_time = n.state_time = 1.
    n.status = dict(ready=True, startup_complete=True, focus='arm', mode='hold',
                    epoch='cancelled-epoch')
    n.hold_epoch = 'previous-epoch'
    with pytest.raises(RuntimeError, match='ownership changed'):
        ToolDockingCoordinator.guard(n, 1.)


def test_released_deadman_blocks_alignment(tmp_path):
    n = node(tmp_path)
    n.docking_started = 0.
    n.deadman_held = False
    with pytest.raises(RuntimeError, match='deadman'):
        ToolDockingCoordinator.guard(n, 1.)


@pytest.mark.parametrize('stale', [False, True])
def test_deadman_abort_during_path_check_reports_distinct_cause_and_progress(tmp_path, stale):
    n = node(tmp_path)
    n.stage = 'checking'
    n.docking_started = None
    n.request_index, n.requests = 12, [None]*861
    n.deadman_held = stale
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.4 if stale else 1.1), \
            patch.object(ToolTrajectoryJoy, 'tick') as parent_tick:
        n.tick()
    parent_tick.assert_not_called()
    reason = n.fail.call_args.args[0]
    assert ('joystick stream stale' if stale else 'deadman L2 released') in reason
    assert 'stage=checking' in reason and 'checked=12/861' in reason
    assert ('joy_age=0.400s' if stale else 'joy_age=0.100s') in reason


def test_actuation_timeout_cannot_report_success(tmp_path):
    n = node(tmp_path)
    n.stage = 'dock_actuate'
    n.lock_goal = np.zeros(6)
    n.expected_docked = True
    n.command_feedback_sequence = 0
    n.docked_feedback_sequence = 0
    n.actuation_deadline = 2.
    with pytest.raises(RuntimeError, match='timeout'):
        ToolDockingCoordinator.observe(n, 3.)
    n.reset.assert_not_called()


def test_legacy_executor_completion_enters_docking_only_after_confirmed_hold(tmp_path):
    n = node(tmp_path)
    n.docking_started = None
    n.stage = 'executing'
    n.execution_id = 'path-0'
    n.path = dict(total=10.)
    n.execution_timeout = 40.
    n.started = 0.
    n.status_time = n.state_time = 1.
    n.status = dict(mode='tool_auto', epoch='path-epoch')
    with patch('tool_trajectory_joy.time.monotonic', return_value=1.):
        ToolTrajectoryJoy.tick(n)
        assert n.stage == 'executing'
        n.status.update(last_execution_id='path-0', last_execution_result='succeeded',
                        mode='hold', transitioning=False, epoch='hold-epoch')
        ToolTrajectoryJoy.tick(n)
    assert n.stage == 'dock_acquire'
    assert n.hold_epoch == 'hold-epoch'
    assert n.execution_id is None
    n.docking_command.publish.assert_not_called()


@pytest.mark.parametrize('enabled,motion,script', [
    (None, None, 'tool_docking_coordinator.py'),
    ('false', None, 'tool_trajectory_joy.py'),
    ('true', 'false', 'tool_docking_coordinator.py')])
def test_launch_selects_exactly_one_path_input_owner(enabled, motion, script):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    spec = importlib.util.spec_from_file_location('ik_test', ROOT/'launch/ik_manual.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    # Exercise actual declared defaults when no docking flags are supplied.
    with patch('launch.logging.get_logger'):
        context = LaunchContext()
        values = {action.name: perform_substitutions(context, action.default_value)
                  for action in launch.generate_launch_description().entities
                  if isinstance(action, DeclareLaunchArgument)}
    values.update(real_hardware='false', hardware_config='hardware.yaml')
    if enabled is not None:
        values['enable_tool_docking'] = enabled
    if motion is not None:
        values['enable_docking_motion'] = motion
    base = SimpleNamespace(hardware_file='hardware.yaml', description={}, spawner=Mock(),
                           moveit_config=Mock(), state_publisher=Mock(), control_node=Mock())
    common = SimpleNamespace(build_arm_base=Mock(return_value=base))
    nodes = []
    def make_node(**kwargs):
        nodes.append(kwargs)
        return Mock()
    # Only the dynamic common module load is replaced; production bringup runs.
    with patch.object(launch.importlib.util, 'spec_from_file_location',
                      return_value=SimpleNamespace(loader=SimpleNamespace(exec_module=lambda module: None))), \
            patch.object(launch.importlib.util, 'module_from_spec', return_value=common), \
            patch.object(launch, 'LaunchConfiguration',
                         side_effect=lambda key: SimpleNamespace(perform=lambda ctx: values[key])), \
            patch.object(launch, 'Node', side_effect=make_node), \
            patch.object(launch, 'RegisterEventHandler', return_value=Mock()), \
            patch.object(launch, 'OnProcessExit', return_value=Mock()):
        launch.bringup(None)
    assert common.build_arm_base.call_args.kwargs['tool_id'] == '99'
    path_nodes = [n for n in nodes if any('tool_trajectory_joy.py' in a or
                  'tool_docking_coordinator.py' in a for a in n.get('arguments', []))]
    assert len(path_nodes) == 1
    assert Path(path_nodes[0]['arguments'][0]).name == script
    if script == 'tool_docking_coordinator.py':
        parameters = path_nodes[0]['parameters'][-1]
        assert parameters['docking_motion_enabled'] is (motion != 'false')
        assert parameters['docking_camera_frame'] == 'arm_camera_color_optical_frame'
        assert set(load_settings(parameters['docking_config'])) == {0, 1}


def camera_node(tmp_path):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[1]
    n.path_id = 1
    n.tcp_camera = np.eye(4)
    n.advance_locked = False
    n.advance_target = None
    n.advance_plan = None
    n.validation_rmd_drift = .005
    n.shoulder_isolation_enabled = True
    n.shoulder_isolation_speed = .015
    n.observation = (1, 0., transform([0, 0, .2], [0, 0, 0, 1]))
    return n


def camera_sample(n):
    n.feedback_sequence += 1
    n.observation = (n.observation[0]+1, 0., n.observation[2])
    n.observe(1.)


@pytest.mark.parametrize('flip', [False, True])
def test_camera_normal_alignment_uses_camera_side_and_centres_optical_axis(flip):
    rotation = cv2.Rodrigues(np.array([0., math.radians(55), 0.]))[0]
    tag = np.eye(4)
    tag[:3, :3] = rotation
    tag[:3, 3] = [.1, .2, .3]
    camera = np.eye(4)
    camera[:3, 3] = tag[:3, 3]-.2*rotation[:, 2]+.03*rotation[:, 0]
    if flip:
        tag[:3, :3] = tag[:3, :3] @ np.diag([1., -1., -1.])
    goal, distance = camera_normal_target(tag, camera, camera[:3, 0])
    assert distance == pytest.approx(.2)
    np.testing.assert_allclose(goal[:3, 2], rotation[:, 2], atol=1e-12)
    np.testing.assert_allclose(tag[:3, 3]-goal[:3, 3], .2*goal[:3, 2])
    np.testing.assert_allclose(goal[:3, :3].T @ goal[:3, :3], np.eye(3), atol=1e-12)
    assert np.linalg.det(goal[:3, :3]) == pytest.approx(1.)


def test_advance_uses_camera_side_of_tag_with_mount_offset(tmp_path):
    n = camera_node(tmp_path)
    n.tcp_camera = transform([.03, .04, -.01], [0, 0, np.sin(.2), np.cos(.2)])
    normal, distance = n.camera_advance_direction(n.observation[2], np.eye(4))
    np.testing.assert_allclose(normal, [0, 0, 1])
    assert distance == pytest.approx(.21)


def test_off_axis_tag_does_not_start_visual_correction(tmp_path):
    n = camera_node(tmp_path)
    n.observation = (1, 0., transform([.03, 0., .2], [0, 0, 0, 1]))
    for _ in range(n.cfg['stable_samples']-1):
        camera_sample(n)
        assert n.stage == 'dock_observe' and not n.advance_locked
        n.begin.assert_not_called()
        n.model.solve.assert_not_called()
    camera_sample(n)
    assert n.stage == 'dock_advance_plan'
    np.testing.assert_allclose(n.advance_target[:3, 3], [0., 0., .030])
    n.model.solve.assert_not_called()


def test_valid_tag_latches_relative_30mm_only_after_distinct_stable_samples(tmp_path):
    n = camera_node(tmp_path)
    for _ in range(n.cfg['stable_samples']-1):
        camera_sample(n)
        assert not n.advance_locked
    camera_sample(n)
    assert n.advance_locked and n.stage == 'dock_advance_plan'
    np.testing.assert_allclose(n.advance_target[:3, 3], [0., 0., .030])
    n.begin.assert_not_called()  # no partial motion before whole-line validation
    n.docking_command.publish.assert_not_called()


def test_preview_never_latches_or_executes_fixed_advance(tmp_path):
    n = camera_node(tmp_path)
    n.motion_enabled = False
    for _ in range(10):
        camera_sample(n)
    assert not n.advance_locked
    n.begin.assert_not_called()


def test_too_close_tag_rejects_advance_instead_of_crossing_plane(tmp_path):
    n = camera_node(tmp_path)
    n.observation[2][:3, 3] = [0., 0., .025]
    with pytest.raises(RuntimeError, match='stand-off too short'):
        camera_sample(n)
    n.begin.assert_not_called()


@pytest.mark.parametrize('tilt', [0., .15])
def test_complete_line_is_solved_before_collision_checked_submission(tmp_path, tilt):
    n = camera_node(tmp_path)
    origin = np.eye(4)
    origin[:3, :3] = cv2.Rodrigues(np.array([0., tilt, 0.]))[0]
    n.model.fk.return_value = origin
    for _ in range(n.cfg['stable_samples']):
        camera_sample(n)
    poses = {tuple(np.zeros(6)): origin}
    def solve_axis(position, direction, local_axis, seed, *a, **k):
        solved = seed+0.0001
        pose = np.eye(4)
        pose[:3, 3] = position
        pose[:3, :3] = cv2.Rodrigues(np.array([0., math.atan2(direction[0], direction[2]), 0.]))[0]
        poses[tuple(solved)] = pose
        return solved
    n.model.solve_axis.side_effect = solve_axis
    n.model.fk.side_effect = lambda q: poses[tuple(q)]
    n.model.within_limits.return_value = True
    n.model.pose_error.return_value = np.zeros(6)
    for _ in range(n.advance_count-1):
        n.plan_advance_step()
        n.begin.assert_not_called()
    n.plan_advance_step()
    path_id, plan = n.begin.call_args.args
    assert path_id == 1 and plan['cartesian_line']
    assert len(plan['waypoints']) == n.advance_count
    assert plan['total'] >= .030/.005-1e-10
    assert plan['stop_at_waypoints'] == [False]*(n.advance_count-1)+[True]
    calls = n.model.solve_axis.call_args_list
    np.testing.assert_allclose(calls[-1].args[0], [0., 0., .030])
    for i, call in enumerate(calls, 1):
        np.testing.assert_allclose(call.args[0], [0, 0, .030*i/n.advance_count])
        np.testing.assert_allclose(call.args[1], interpolate_direction(
            origin[:3, 2], np.array([0, 0, 1]), i/n.advance_count), atol=1e-12)
        np.testing.assert_allclose(call.args[2], [0, 0, 1])
    n.model.solve.assert_not_called()


def test_failed_advance_ik_never_submits_partial_path(tmp_path):
    n = camera_node(tmp_path)
    for _ in range(n.cfg['stable_samples']):
        camera_sample(n)
    n.model.solve_axis.side_effect = ValueError('IK iteration would violate soft limits')
    with pytest.raises(ValueError, match='soft limits'):
        n.plan_advance_step()
    n.begin.assert_not_called()


@pytest.mark.parametrize('locked', [False, True])
def test_tag_loss_only_permitted_after_advance_direction_is_locked(tmp_path, locked):
    n = camera_node(tmp_path)
    n.docking_started = 0.
    n.stage = 'executing'
    n.advance_locked = locked
    n.guard = Mock()
    n.fresh_observation = Mock(side_effect=RuntimeError('target tag missing'))
    with patch.object(ToolTrajectoryJoy, 'tick') as parent:
        n.tick()
        assert parent.called == locked
    assert n.fail.called != locked
    assert n.fresh_observation.called != locked


def test_opt_in_deadman_still_cancels_blind_advance(tmp_path):
    n = camera_node(tmp_path)
    n.cfg['require_l2_hold'] = True
    n.docking_started = 0.
    n.stage = 'executing'
    n.advance_locked = True
    n.deadman_held = False
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1), \
            patch.object(ToolTrajectoryJoy, 'tick') as parent:
        n.tick()
        parent.assert_not_called()
    assert 'deadman' in n.fail.call_args.args[0]


@pytest.mark.parametrize('stage', ['planning_entry', 'checking', 'executing'])
def test_tool1_approach_cancels_on_l2_release(tmp_path, stage):
    n = camera_node(tmp_path)
    n.stage = stage
    n.docking_started = None
    n.deadman_held, n.last_joy_time = False, 1.1
    n.request_index, n.requests = 0, []
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1), \
            patch.object(ToolTrajectoryJoy, 'tick') as parent:
        n.tick()
    parent.assert_not_called()
    assert 'docking deadman L2 released' in n.fail.call_args.args[0]


def test_tool1_completion_automatically_aligns_and_plans_advance_without_joy(tmp_path):
    n = camera_node(tmp_path)
    tag = n.observation[2].copy()
    n.docking_started = None
    n.stage, n.execution_id = 'executing', 'tool1'
    n.path, n.execution_timeout, n.started = dict(total=10.), 40., 0.
    n.deadman_held, n.last_joy_time = True, 1.1
    n.status_time = n.state_time = 1.
    n.status = dict(mode='tool_auto', epoch='path-epoch', ready=True,
                    startup_complete=True, focus='arm')
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1):
        n.tick()
        assert n.stage == 'executing' and n.docking_started is None
        n.status.update(last_execution_id='tool1', last_execution_result='succeeded',
                        mode='hold', transitioning=False, epoch='hold-epoch')
        n.tick()
        assert n.stage == 'dock_acquire'
        assert n.hold_epoch == 'hold-epoch'
        assert n.observation is None  # Approach frames cannot authorize docking.
        n.tick()
        assert n.stage == 'dock_acquire'  # Wait for the tag, not another trigger.
        for i in range(n.cfg['stable_samples']):
            n.feedback_sequence += 1
            n.observation = (i+1, 1.1, tag)
            n.tick()
        assert n.stage == 'dock_advance_plan' and n.advance_locked
        np.testing.assert_allclose(n.advance_target[:3, 3], [0., 0., .030])
    n.fail.assert_not_called()
    n.docking_command.publish.assert_not_called()


def tag_input_node(tmp_path):
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo
    n = camera_node(tmp_path)
    n.reset_tag_acquisition()
    n.camera_frame, n.base_frame = 'arm_camera_color_optical_frame', 'base_actuator'
    n.get_clock.return_value.now.return_value.nanoseconds = 2_000_000_000
    n.tf_buffer = Mock()
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    n.tf_buffer.lookup_transform.return_value = tf
    info = CameraInfo()
    info.header.frame_id = n.camera_frame
    info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    info.d = [0.]*5
    info.distortion_model = 'plumb_bob'
    n.on_camera(info)
    h = n.cfg['tag_size_m']/2
    square = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]])
    pixels = cv2.projectPoints(square, np.array([.15, -.1, .03]),
                              np.array([.01, .02, .3]), np.asarray(info.k).reshape(3, 3),
                              np.zeros(5))[0].reshape(4, 2)
    data = dict(stamp=dict(sec=1, nanosec=950_000_000), frame_id=n.camera_frame,
                detections=[dict(id=1, family='tag36h11', hamming=0, decision_margin=70.62,
                                 corners_px=[dict(u=u, v=v) for u, v in pixels])])
    return n, info, data


def test_existing_detector_message_is_accepted_with_matching_camera_and_image_time_tf(tmp_path):
    n, _, data = tag_input_node(tmp_path)
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    assert n.tag_received == n.tag_accepted == 1
    assert n.observation is not None
    np.testing.assert_allclose(n.observation[2][:3, 3], [.01, .02, .3], atol=1e-7)
    args = n.tf_buffer.lookup_transform.call_args.args
    assert args[:2] == (n.base_frame, n.camera_frame)
    assert args[2].nanoseconds == 1_950_000_000  # Never substitute latest TF.


@pytest.mark.parametrize('failure,reason', [
    ('camera_frame', 'CameraInfo frame mismatch'), ('camera_missing', 'CameraInfo not received'),
    ('tag_frame', 'tag frame mismatch'), ('age', 'tag image age='),
    ('wrong_id', 'expected one tag'), ('quality', 'tag quality rejected'),
    ('tf', 'TF base_actuator <- arm_camera_color_optical_frame'),
    ('jump', 'tag pose discontinuity rejected')])
def test_tag_rejection_retains_exact_reason_and_previous_observation(tmp_path, failure, reason):
    from tf2_ros import TransformException
    n, info, data = tag_input_node(tmp_path)
    n.observation = (1, 1., np.eye(4))
    if failure == 'camera_frame':
        info.header.frame_id = 'camera_color_optical_frame'
        n.on_camera(info)
    if failure == 'camera_missing':
        n.camera, n.camera_reason = None, 'CameraInfo not received'
    if failure == 'tag_frame': data['frame_id'] = 'wrong'
    if failure == 'age': data['stamp']['sec'] = 0
    if failure == 'wrong_id': data['detections'][0]['id'] = 0
    if failure == 'quality': data['detections'][0]['hamming'] = 1
    if failure == 'tf':
        n.tf_buffer.lookup_transform.side_effect = TransformException('image-time transform unavailable')
    if failure == 'jump': n.last_base_tag = np.eye(4)
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    assert n.observation is not None and n.tag_accepted == 0 and n.tag_received == 1
    assert reason in n.tag_diagnostic()
    n.begin.assert_not_called()


def test_one_rejected_frame_keeps_recent_tag_but_stale_tag_stops_motion(tmp_path):
    n, _, data = tag_input_node(tmp_path)
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    accepted = n.observation
    data['detections'] = []
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    assert n.observation is accepted
    assert n.tag_received == 2 and n.tag_accepted == 1
    assert ToolDockingCoordinator.fresh_observation(n, accepted[1] + .1)[0] == accepted[0]
    with pytest.raises(RuntimeError, match='target tag stale: received=2 accepted=1'):
        ToolDockingCoordinator.fresh_observation(n, accepted[1] + .26)


def test_approach_pose_anchor_is_discarded_before_acquiring_stopped_tag(tmp_path):
    n, _, data = tag_input_node(tmp_path)
    n.last_base_tag = np.eye(4)  # An old/noisy approach pose would reject the new tag.
    n.docking_started = None
    n.status = dict(epoch='hold')
    n.path_completed()
    assert n.last_base_tag is None and n.observation is None
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    assert n.tag_accepted == 1 and n.observation is not None


def test_acquisition_wait_and_timeout_report_reason_without_log_flooding(tmp_path):
    n, info, data = tag_input_node(tmp_path)
    n.docking_started = 1.
    n.stage, n.observation = 'dock_acquire', None
    n.guard = Mock()
    info.header.frame_id = 'wrong'
    n.on_camera(info)
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1):
        for _ in range(10):
            n.tick()
    n.report.assert_called_once()
    assert 'CameraInfo frame mismatch' in n.report.call_args.args[1]
    assert 'received=1 accepted=0' in n.report.call_args.args[1]
    with patch('tool_docking_coordinator.time.monotonic', return_value=12.):
        n.tick()
    assert 'target tag acquisition timed out' in n.fail.call_args.args[0]
    assert 'CameraInfo frame mismatch' in n.fail.call_args.args[0]


def test_main_spins_tf_concurrently_and_shuts_executor_down_before_node():
    import tool_docking_coordinator as module
    n = Mock(stage='idle')
    events = []
    n.destroy_node.side_effect = lambda: events.append('node')
    with patch.object(module, 'rclpy') as ros, \
            patch.object(module, 'ToolDockingCoordinator', return_value=n), \
            patch.object(module, 'MultiThreadedExecutor') as executor_type:
        executor = executor_type.return_value
        executor.spin.side_effect = KeyboardInterrupt
        executor.shutdown.side_effect = lambda: events.append('executor')
        module.main()
    executor_type.assert_called_once_with(num_threads=2)
    executor.add_node.assert_called_once_with(n)
    assert events == ['executor', 'node']
    n.fail.assert_not_called()


@pytest.mark.parametrize('entry_offset', [0., .272])
def test_l2_circle_from_tcp_mode_keeps_docking_through_entry_plan_and_execution(tmp_path, entry_offset):
    from sensor_msgs.msg import Joy
    from tool_trajectory_joy import load_path
    n = camera_node(tmp_path)
    del n.begin  # Use both real begin methods, rather than pre-arming the fixture.
    n.paths = {1: load_path(ROOT/'config/tool1_trajectory.yaml', n.model)}
    n.settings = load_settings(ROOT/'config/tool_docking.yaml')
    n.active, n.stage, n.docking_started = False, 'idle', None
    n.combos, n.l2_axis, n.l2_threshold, n.first_button, n.third_button = (
        [False]*3, 2, -.5, 2, 0)
    n.second_button = 1
    n.requested_docked_state, n.use_nearest_entry = None, False
    measured = n.paths[1]['start'].copy()
    measured[0] += entry_offset
    n.full_state = dict(zip(ARM_JOINTS, measured))
    n.status_time = n.state_time = 1.
    n.status = dict(mode='tcp_ik', epoch='tcp', ready=True, startup_complete=True,
                    focus='arm', transitioning=False)
    n.start_tolerance, n.max_start_delta, n.approach_speed = .03, .1, .05
    n.shoulder_isolation_half_width = .08
    n.collision_step, n.collision_timeout = .005, 10.
    n.planning_timeout, n.planning_attempts = 8., 3
    n.planning_velocity_scale, n.planning_acceleration_scale = .08, .4
    n.publisher = Mock()
    n.cancel, n.mode, n.collision, n.planner = Mock(), Mock(), Mock(), Mock()
    for client in (n.cancel, n.mode, n.collision, n.planner):
        client.service_is_ready.return_value = True
        client.call_async.return_value.done.return_value = True
    n.cancel.call_async.return_value.result.return_value = SimpleNamespace(success=True)
    n.mode.call_async.return_value.result.return_value = SimpleNamespace(success=True)
    n.collision.call_async.return_value.result.return_value = SimpleNamespace(valid=True)
    trajectory = SimpleNamespace(joint_names=list(ARM_JOINTS), points=[SimpleNamespace(
        positions=n.paths[1]['start'].tolist(),
        time_from_start=SimpleNamespace(sec=4, nanosec=0))])
    n.planner.call_async.return_value.result.return_value = SimpleNamespace(
        motion_plan_response=SimpleNamespace(error_code=SimpleNamespace(val=1),
                                            trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1):
        n.tick()
        assert n.stage == 'idle' and not n.active
        n.publisher.publish.assert_not_called()
        n.on_joy(Joy(axes=[0., 0., -1.], buttons=[0, 1, 0, 0, 0]))
        assert n.stage == 'request_hold'
        n.tick()
        n.status.update(mode='hold', epoch='hold')
        n.tick()
        assert n.active
        if entry_offset:
            # The current base is already close to the first taught segment,
            # so resume at its projection instead of planning back to point 0.
            assert n.entry_path['start'][0] > n.paths[1]['start'][0]
            n.planner.call_async.assert_not_called()
        assert n.stage == 'checking'
        # Releasing the face button keeps the path active while L2 stays held.
        n.on_joy(Joy(axes=[0., 0., -1.], buttons=[0]*5))
        for _ in n.requests:
            n.tick()
        assert n.stage == 'request_mode'
        n.tick()
        n.status.update(mode='tool_auto', epoch='tool')
        n.tick()
        assert n.stage == 'executing' and n.active
        n.publisher.publish.assert_called_once()
        # A separate manager confirms the path endpoint and returns HOLD.
        n.full_state = dict(zip(ARM_JOINTS, n.paths[1]['waypoints'][-1]))
        n.status.update(mode='hold', epoch='completed',
                        last_execution_id=n.execution_id, last_execution_result='succeeded')
        n.tick()
        assert n.stage == 'dock_acquire' and n.active
        for i in range(n.cfg['stable_samples']):
            n.feedback_sequence += 1
            n.observation = (i+1, 1.1, transform([0, 0, .2], [0, 0, 0, 1]))
            n.tick()
        assert n.stage == 'dock_advance_plan'
    n.fail.assert_not_called()
    n.reset.assert_not_called()


@pytest.mark.parametrize('failure', [None, 'feedback', 'focus', 'fault', 'epoch', 'tag', 'timeout'])
def test_automatic_tool1_keeps_docking_guards(tmp_path, failure):
    n = camera_node(tmp_path)
    n.docking_started = 1.
    n.stage = 'executing'
    n.advance_locked = True
    n.deadman_held, n.last_joy_time = True, 1.1
    n.status_time = n.state_time = 1.
    n.status = dict(mode='tool_auto', ready=True, startup_complete=True, focus='arm', epoch='e')
    if failure == 'feedback': n.state_time = 0.
    if failure == 'focus': n.status['focus'] = 'drive'
    if failure == 'fault': n.status['mode'] = 'fault'
    if failure == 'epoch':
        n.stage = 'dock_observe'
        n.status['mode'], n.hold_epoch = 'hold', 'old'
    if failure == 'tag':
        n.advance_locked = False
        n.fresh_observation = Mock(side_effect=RuntimeError('target tag missing'))
    if failure == 'timeout': n.docking_started = -200.
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1), \
            patch.object(ToolTrajectoryJoy, 'tick') as parent:
        n.tick()
    assert parent.called is (failure is None)
    assert n.fail.called is (failure is not None)


def test_operator_can_cancel_automatic_tool1(tmp_path):
    n = camera_node(tmp_path)
    n.stage = 'dock_observe'
    response = n.on_cancel(None, SimpleNamespace())
    assert response.success
    n.fail.assert_called_once_with('operator cancelled docking/path')


@pytest.mark.parametrize('endpoint_error', [0., .004])
def test_advance_completion_checks_fk_endpoint_without_actuation(tmp_path, endpoint_error):
    n = camera_node(tmp_path)
    n.advance_locked = True
    n.advance_local_axis = n.advance_direction = np.array([0., 0., 1.])
    n.advance_target = transform([0., 0., .030], [0., 0., 0., 1.])
    measured = n.advance_target.copy()
    measured[2, 3] += endpoint_error
    n.model.fk.return_value = measured
    if endpoint_error:
        with pytest.raises(RuntimeError, match='endpoint feedback'):
            n.path_completed()
        n.reset.assert_not_called()
    else:
        n.path_completed()
        n.reset.assert_called_once()
        assert '30.0 mm completed' in n.report.call_args.args[1]
    n.docking_command.publish.assert_not_called()


def test_advance_completion_allows_axial_rotation_but_rejects_wrong_tilt(tmp_path):
    for rotation, passes in (([0., 0., 1.2], True), ([.1, 0., 0.], False)):
        n = camera_node(tmp_path)
        n.advance_locked = True
        n.advance_local_axis = n.advance_direction = np.array([0., 0., 1.])
        n.advance_target = np.eye(4)
        actual = np.eye(4)
        actual[:3, :3] = cv2.Rodrigues(np.array(rotation))[0]
        n.model.fk.return_value = actual
        if passes:
            n.path_completed()
            n.reset.assert_called_once()
        else:
            with pytest.raises(RuntimeError, match='endpoint feedback'):
                n.path_completed()


def test_front_on_tag_accepts_equivalent_ippe_solutions():
    cfg = load_settings(ROOT/'config/tool_docking.yaml')[1]
    k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
    h = cfg['tag_size_m']/2
    square = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    pixels = cv2.projectPoints(square, np.zeros(3), np.array([0., 0., .2]),
                              k, np.zeros(5))[0].reshape(4, 2)
    detection = dict(hamming=0, decision_margin=100,
                     corners_px=[dict(u=u, v=v) for u, v in pixels])
    pose = tag_pose(detection, k, np.zeros(5), cfg)
    np.testing.assert_allclose(pose, transform([0, 0, .2], [0, 0, 0, 1]), atol=1e-10)


def test_distinct_equally_scored_ippe_solutions_remain_rejected():
    cfg = load_settings(ROOT/'config/tool_docking.yaml')[1]
    pixels = np.array([[290., 270.], [350., 270.], [350., 210.], [290., 210.]])
    detection = dict(hamming=0, decision_margin=100,
                     corners_px=[dict(u=u, v=v) for u, v in pixels])
    with patch('tool_docking_core.cv2.solvePnPGeneric', return_value=(
            True, [np.array([.1, 0, 0]), np.array([-.1, 0, 0])],
            [np.array([0, 0, .2]), np.array([0, 0, .2])])), \
            patch('tool_docking_core.cv2.projectPoints', return_value=(pixels, None)):
        with pytest.raises(ValueError, match='ambiguous'):
            tag_pose(detection, np.eye(3), np.zeros(5), cfg)


def test_cartesian_advance_keeps_line_and_still_builds_collision_requests(tmp_path):
    n = camera_node(tmp_path)
    n.healthy_hold = Mock(return_value={'epoch': 'hold'})
    n.collision = Mock()
    n.mode = Mock()
    n.collision_step = .005
    n.collision_timeout = 10.
    n.start_tolerance = .03
    n.begin_failed = Mock()
    plan = dict(waypoints=[np.ones(6)*.01, np.ones(6)*.02], durations=[1., 1.],
                total=2., start_error=0., cartesian_line=True,
                stop_at_waypoints=[False, True])
    with patch('tool_trajectory_joy.isolate_shoulder_zero_crossings') as isolate:
        ToolTrajectoryJoy.begin(n, 1, plan)
        isolate.assert_not_called()
    n.begin_failed.assert_not_called()
    assert n.stage == 'checking'
    assert len(n.requests) == 6  # both segment endpoints are checked
    n.collision.call_async.assert_called_once_with(n.requests[0])
    assert n.path is plan


@pytest.mark.parametrize('distance', [None, 0., -.030, float('nan'), .11])
def test_invalid_fixed_advance_distance_is_rejected(tmp_path, distance):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][1]['advance_distance_m'] = distance
    file = tmp_path/'bad_advance.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='advance_distance_m'):
        load_settings(file)
