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
from tool_docking_core import load_settings, transform, bounded_target, pose_errors, tag_pose
from tool_docking_coordinator import ToolDockingCoordinator
from tool_trajectory_joy import ToolTrajectoryJoy, ARM_JOINTS


def settings(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    raw = data['tools'][0]
    raw.update(enabled=True, tag_id=7, tag_size_m=.04, actuation_timeout_s=5.)
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


def test_shipped_config_disables_every_docking_path():
    assert load_settings(ROOT/'config/tool_docking.yaml') == {}


def test_enabling_unmeasured_config_is_rejected(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][0]['enabled'] = True
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
    n.cancel = Mock()
    n.cancel.service_is_ready.return_value = True
    with patch.object(ToolTrajectoryJoy, 'fail') as parent:
        ToolDockingCoordinator.fail(n, 'cancelled')
        parent.assert_called_once_with('cancelled')
    n.docking_command.publish.assert_not_called()
    n.cancel.call_async.assert_called_once()


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


@pytest.mark.parametrize('enabled,script', [('false','tool_trajectory_joy.py'),
                                         ('true','tool_docking_coordinator.py')])
def test_launch_selects_exactly_one_path_input_owner(enabled, script):
    spec = importlib.util.spec_from_file_location('ik_test', ROOT/'launch/ik_manual.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    values = dict(real_hardware='false', launch_tcp_joy='true', mock_initial_positions_file='',
                  hardware_config='hardware.yaml', launch_joy='false',
                      enable_tool_docking=enabled, enable_docking_motion='false',
                      docking_config='dock.yaml', docking_camera_frame='optical')
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
    path_nodes = [n for n in nodes if any('tool_trajectory_joy.py' in a or
                  'tool_docking_coordinator.py' in a for a in n.get('arguments', []))]
    assert len(path_nodes) == 1
    assert Path(path_nodes[0]['arguments'][0]).name == script
