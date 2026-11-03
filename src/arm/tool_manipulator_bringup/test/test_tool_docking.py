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


def settings(tmp_path):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    raw = data['tools'][0]
    raw.update(enabled=True, require_l2_hold=True,
               alignment_mode='tag_tcp_stages', tag_id=7,
               tag_size_m=.04, actuation_enabled=False, actuation_timeout_s=5.,
               actuation_completion='feedback')
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
        begin_docking_retreat=Mock(),
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


def test_shipped_config_uses_fixed_joint_docking_for_tool0_and_tool1():
    cfg = load_settings(ROOT/'config/tool_docking.yaml')
    assert set(cfg) == {0, 1}
    assert cfg[0]['alignment_mode'] == 'fixed_joint_goal'
    np.testing.assert_allclose(cfg[0]['final_joint_goal'], [
        -3.186078, .047473, 1.091180, -1.234995, .036816, .049087])
    assert cfg[0]['tag_family'] == 'tag36h11'
    assert cfg[0]['tag_id'] == 0
    assert cfg[0]['tag_size_m'] == .020
    assert cfg[0]['actuation_enabled']
    assert cfg[0]['desired_docked'] is True
    assert cfg[0]['require_l2_hold'] is False
    assert cfg[0]['maintain_entry_orientation'] is True
    assert cfg[0]['measured_pre_dock_alignment_enabled'] is False
    assert cfg[0]['max_position_error_m'] == .25
    assert cfg[0]['max_angle_error_rad'] == .35
    assert cfg[1]['alignment_mode'] == 'fixed_joint_goal'
    assert cfg[1]['axial_sampled_path']
    assert cfg[1]['axial_entry_distance_m'] == .040
    np.testing.assert_allclose(cfg[1]['final_joint_goal'], [
        -3.2505052895298805, .5297074013486681, 1.5561354918359558,
        -1.3185961699868833, -.2300971181830529, -1.7855536370986798])
    assert cfg[1]['confirm_tag_before_motion'] is False
    assert cfg[1]['tag_translation_correction']['enabled'] is False
    assert 'circuit_box/wrist_link_yaw' in cfg[1]['blocked_reason']
    assert 'limit_margin_rad' not in cfg[1]
    assert cfg[1]['final_entry_ignored_collision_links'] == ['tool_case_y_plus']
    assert 'advance_distance_m' not in cfg[1]
    assert cfg[1]['tag_family'] == 'tag36h11'
    assert cfg[1]['tag_id'] == 1
    assert cfg[1]['actuation_enabled']
    assert cfg[1]['desired_docked'] is True
    assert cfg[1]['require_l2_hold'] is False
    for tool_id in (0, 1):
        assert cfg[tool_id]['actuation_completion'] == 'delay'
        assert cfg[tool_id]['actuation_delay_s'] == 3.
        assert cfg[tool_id]['return_home_after_back'] is True
    assert cfg[1]['max_angle_error_rad'] == .35
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


def test_pnp_reference_prevents_planar_solution_flip_from_measured_tool0_corners(tmp_path):
    cfg = settings(tmp_path)
    k = np.array([[605.8599853515625, 0., 326.54400634765625],
                  [0., 605.5801391601562, 250.81643676757812], [0., 0., 1.]])
    corners = [(431.3188781738281, 416.9869384765625),
               (481.305419921875, 415.8382263183594),
               (477.633056640625, 365.6019287109375),
               (428.405517578125, 366.53204345703125)]
    detection = dict(hamming=0, decision_margin=67.17005157470703,
                     corners_px=[dict(u=u, v=v) for u, v in corners])
    first = tag_pose(detection, k, np.zeros(5), cfg)
    # The measured frame has two valid IPPE candidates about 0.269 rad apart.
    # A continuity reference close to the alternate branch must select that
    # branch even though its reprojection error is slightly higher.
    alternate = np.eye(4)
    alternate[:3, :3] = cv2.Rodrigues(np.array([-.23930766, .33380690, -.05694369]))[0]
    alternate[:3, 3] = [.05207151, .05714951, .24688093]
    continuous = tag_pose(detection, k, np.zeros(5), cfg, reference=alternate)
    assert pose_errors(first, continuous)[1] > .25
    assert pose_errors(alternate, continuous)[1] < .01


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
    n.begin_docking_retreat.assert_called_once()
    assert n.docking_command.publish.call_count == 1
    n.begin.assert_not_called()


def test_release_uses_explicit_false_command_and_waits_for_false(tmp_path):
    n = node(tmp_path)
    n.cfg.update(actuation_enabled=True, desired_docked=False)
    for _ in range(n.cfg['stable_samples']*3):
        sample(n)
    assert [call.args[0].data for call in n.docking_command.publish.call_args_list] == [False]
    ToolDockingCoordinator.on_docked(n, SimpleNamespace(data=False))
    ToolDockingCoordinator.observe(n, 1.1)
    n.begin_docking_retreat.assert_called_once()


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


def test_production_completion_waits_for_tag_without_another_button(tmp_path):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[0]
    n.docking_started = None
    n.status = {'epoch': 'observation-hold'}
    n.start_final_joint_goal = Mock()
    n.path_completed()
    assert n.stage == 'dock_tag_acquire'
    assert n.tag_translation_samples == []
    n.start_final_joint_goal.assert_not_called()


def translation_node(tmp_path):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[0]
    n.docking_started = 0.
    n.tag_translation_reference = np.array([.1, .2, .3])
    n.tag_translation_samples = []
    n.observe_after_ns = 0
    n.full_state = dict(zip(ARM_JOINTS, n.cfg['tag_translation_correction']['reference_joint_rad']))
    n.stage = 'dock_tag_acquire'
    return n


def feed_translation(n, delta, count=8):
    for i in range(count):
        pose = np.eye(4)
        pose[:3, 3] = n.tag_translation_reference + np.asarray(delta)
        n.observation = (i+1, 1., pose)
        n.feedback_sequence += 1
        n.tick_tag_translation(1.)


@pytest.mark.parametrize('delta', [[0., 0., 0.], [.002, -.001, .001]])
def test_translation_rebuilds_whole_path_and_reverse_from_one_measurement(tmp_path, delta):
    from tool_trajectory_joy import ArmKinematics
    n = translation_node(tmp_path)
    n.model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro', ROOT/'config/hardware.yaml')
    feed_translation(n, delta)
    assert n.stage == 'dock_tag_plan'
    assert n.advance_locked
    original_targets = [np.asarray(q).copy() for q in n.tag_translation_targets]
    n.observation = None  # Tag may disappear once the correction is frozen.
    for _ in original_targets:
        n.plan_tag_translation_step()
    assert n.advance_plan['preserve_joint_path']
    assert len(n.cfg['axial_approach_joint_goals']) == 9
    for original, solved in zip(original_targets, n.tag_translation_solutions):
        expected = n.model.fk(original)
        expected[:3, 3] += delta
        np.testing.assert_allclose(n.model.fk(solved), expected, atol=1e-4)
    n.begin.assert_called_once_with(0, n.advance_plan)
    final_pose = n.model.fk(n.cfg['final_joint_goal'])
    axis = final_pose[:3, 2]
    start_index = n.advance_plan['axial_start_index']
    previous = n.advance_plan['waypoints'][start_index-1]
    np.testing.assert_allclose(n.model.fk(previous)[:3, 3],
                              final_pose[:3, 3] - .070 * axis, atol=2e-5)
    for point in n.advance_plan['waypoints'][start_index:]:
        for q in np.linspace(previous, point, 5):
            assert n.model.within_limits(q, n.margin)
            pose = n.model.fk(q)
            offset = pose[:3, 3]-final_pose[:3, 3]
            assert np.linalg.norm(offset-axis*(axis@offset)) < 2e-5
            assert pose_errors(pose, final_pose)[1] < .001
        previous = point
    n.full_state = dict(zip(ARM_JOINTS, n.cfg['final_joint_goal']))
    n.pre_dock_targets = {}
    ToolTrajectoryJoy.begin_reverse_docking_retreat(n)
    reverse = n.begin.call_args.args[1]
    np.testing.assert_allclose(reverse['waypoints'],
        list(reversed([n.advance_plan['start'], *n.advance_plan['waypoints'][:-1]])))


def test_tag_translation_does_not_move_for_missing_unstable_or_excessive_measurements(tmp_path):
    n = translation_node(tmp_path)
    feed_translation(n, [.001, 0., 0.], count=1)
    for _ in range(10):
        n.tick_tag_translation(1.)  # Repeated image cannot count as new evidence.
    assert len(n.tag_translation_samples) == 1
    assert n.stage == 'dock_tag_acquire'
    n.begin.assert_not_called()
    with pytest.raises(RuntimeError, match='exceeds calibrated envelope'):
        feed_translation(n, [.02, 0., 0.], count=12)
    n.begin.assert_not_called()
    n.observation = None
    with pytest.raises(RuntimeError, match='acquisition timed out'):
        n.tick_tag_translation(11.)


def test_tag_plan_rejects_motion_during_ik(tmp_path):
    n = translation_node(tmp_path)
    feed_translation(n, [0., 0., 0.])
    n.full_state[ARM_JOINTS[1]] += .02
    with pytest.raises(RuntimeError, match='arm moved'):
        n.plan_tag_translation_step()
    n.begin.assert_not_called()


def test_recorded_tag_reference_uses_camera_extrinsics_and_pnp(tmp_path):
    from tool_trajectory_joy import ArmKinematics
    n = translation_node(tmp_path)
    c = n.cfg['tag_translation_correction']
    n.camera = (np.asarray(c['reference_camera_matrix']).reshape(3, 3),
                np.asarray(c['reference_distortion']))
    n.camera_frame = 'arm_camera_color_optical_frame'
    n.model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro', ROOT/'config/hardware.yaml')
    n.tag_translation_reference = None
    n.tf_buffer = Mock()
    n.tf_buffer.lookup_transform.return_value.transform = SimpleNamespace(
        translation=SimpleNamespace(x=.01, y=.02, z=.03),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    n.observation = (1, 1., np.eye(4))
    n.tick_tag_translation(1.)
    detection = dict(hamming=0, decision_margin=68.496788,
        corners_px=[dict(u=u, v=v) for u, v in c['reference_corners_px']])
    camera_tag = tag_pose(detection, *n.camera, dict(n.cfg, min_pnp_solution_gap_px=0.))
    expected = n.model.fk(c['reference_joint_rad']) @ transform([.01, .02, .03], [0., 0., 0., 1.]) @ camera_tag
    np.testing.assert_allclose(n.tag_translation_reference, expected[:3, 3])
    assert .24 < camera_tag[2, 3] < .26
    n.tag_translation_reference = None
    n.camera[0][0, 0] += 10.
    n.observation = (2, 1., np.eye(4))
    n.feedback_sequence += 1
    with pytest.raises(RuntimeError, match='camera calibration differs'):
        n.tick_tag_translation(1.)


def test_unstable_tag_translation_waits_without_motion(tmp_path):
    n = translation_node(tmp_path)
    for i in range(16):
        pose = np.eye(4)
        pose[:3, 3] = n.tag_translation_reference + [(-1)**i * .004, 0., 0.]
        n.observation = (i+1, 1., pose)
        n.feedback_sequence += 1
        n.tick_tag_translation(1.)
    assert n.stage == 'dock_tag_acquire'
    n.begin.assert_not_called()


def test_fixed_joint_completion_starts_checked_goal_without_tag(tmp_path):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[0]
    n.cfg['tag_translation_correction']['enabled'] = False
    n.path_id = 0
    n.docking_started = None
    n.status = {'epoch': 'fixed-joint-hold'}
    n.execution_id = 'tool0-path'
    n.start_final_joint_goal = Mock()

    with patch('tool_docking_coordinator.time.monotonic', return_value=10.):
        n.path_completed()

    n.start_final_joint_goal.assert_called_once()
    q, = n.start_final_joint_goal.call_args.args
    np.testing.assert_allclose(q, np.zeros(6))
    assert n.docking_started == 10.
    assert n.hold_epoch == 'fixed-joint-hold'
    assert n.execution_id is None


@pytest.mark.parametrize('prefix,axial', [([[0.]*5], True),
                                        ([[float('nan')]*6], True),
                                        ([[0.]*6], False)])
def test_axial_approach_prefix_rejects_invalid_configuration(tmp_path, prefix, axial):
    import yaml
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][0]['axial_approach_joint_goals_rad'] = prefix
    data['tools'][0]['axial_sampled_path'] = axial
    path = tmp_path/'invalid_prefix.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='axial'):
        load_settings(path)


def test_fixed_joint_tail_ends_at_measured_docking_pose(tmp_path):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[0]
    n.path_id = 0
    from tool_trajectory_joy import ArmKinematics, load_pre_dock_targets
    n.model = ArmKinematics(
        ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        ROOT/'config/hardware.yaml')
    q = np.asarray(n.cfg['tag_translation_correction']['outer_entry_joint_rad'])

    n.start_final_joint_goal(q)

    expected = n.cfg['final_joint_goal'].copy()
    expected[0] += 2.*math.pi*round((q[0]-expected[0])/(2.*math.pi))
    np.testing.assert_allclose(n.advance_plan['waypoints'][-1], expected)
    assert n.advance_plan['stop_at_waypoints'][-1] is True
    intermediate = n.cfg['intermediate_joint_goals'][0]
    hits = [i for i, point in enumerate(n.advance_plan['waypoints'])
            if np.allclose(point, intermediate)]
    assert len(hits) == 1
    assert n.advance_plan['stop_at_waypoints'][hits[0]] is False
    assert not n.advance_plan['cartesian_line']
    assert n.advance_plan['preserve_joint_path']
    final_capture = np.array([-3.186078, .047473, 1.091180, -1.234995, .036816, .049087])
    np.testing.assert_array_equal(n.cfg['final_joint_goal'], final_capture)
    final_pose = n.model.fk(final_capture)
    axis = final_pose[:3, 2]
    axial_start = n.advance_plan['axial_start_index']
    entry = n.advance_plan['waypoints'][axial_start-1]
    np.testing.assert_allclose(n.model.fk(entry)[:3, 3],
                               final_pose[:3, 3] - .070 * axis, atol=1e-5)
    prefix_hits = []
    for capture in n.cfg['axial_approach_joint_goals']:
        indices = [i for i, point in enumerate(n.advance_plan['waypoints'])
                   if np.allclose(point, capture)]
        assert len(indices) == 1
        assert n.advance_plan['stop_at_waypoints'][indices[0]]
        prefix_hits.append(indices[0])
    assert len(prefix_hits) == 8
    assert prefix_hits == sorted(prefix_hits)
    assert prefix_hits[-1] == axial_start-1
    assert sum(n.advance_plan['stop_at_waypoints'][axial_start:]) == 1
    previous_remaining = .122
    previous = q
    for index, (point, duration) in enumerate(zip(n.advance_plan['waypoints'], n.advance_plan['durations'])):
        for sample in np.linspace(previous, point, 11):
            assert n.model.within_limits(sample, n.margin)
            pose = n.model.fk(sample)
            delta = pose[:3, 3] - final_pose[:3, 3]
            remaining = -float(delta @ axis)
            assert remaining <= previous_remaining + 1e-6
            previous_remaining = remaining
            if index >= axial_start:
                assert np.linalg.norm(delta - axis * (delta @ axis)) < 1e-5
                np.testing.assert_allclose(pose[:3, :3], final_pose[:3, :3], atol=1e-4)
        assert np.max(np.abs(point-previous))/duration <= n.cfg['joint_speed_rad_s']+1e-12
        distance = np.linalg.norm(n.model.fk(point)[:3, 3]-n.model.fk(previous)[:3, 3])
        assert distance/duration <= n.cfg['linear_speed_m_s']+1e-12
        previous = point
    assert n.advance_plan['allowed_collision_pairs'] == frozenset((
        ('tool_case_1', 'wrist_link_yaw'),))
    n.begin.assert_called_once_with(0, n.advance_plan)
    # Reverse the executed samples from live feedback, allowing docking displacement.
    forward = n.advance_plan
    current = final_capture.copy()
    current[1] += .02
    n.full_state = dict(zip(ARM_JOINTS, current))
    n.begin.reset_mock()
    ToolTrajectoryJoy.begin_reverse_docking_retreat(n)
    reverse = n.begin.call_args.args[1]
    assert reverse['preserve_joint_path']
    np.testing.assert_array_equal(reverse['start'], current)
    np.testing.assert_allclose(reverse['waypoints'],
                               list(reversed([q, *forward['waypoints'][:-1]])))
    assert reverse['stop_at_waypoints'] == list(reversed([True, *forward['stop_at_waypoints'][:-1]]))
    assert n.return_home_after_back
    assert n.route_kind == 'dock_retreat'
    # Keep regression coverage for the optional axial-mode entry envelope.
    n.cfg['axial_approach_joint_goals'] = []
    n.cfg['axial_sampled_path'] = True
    with pytest.raises(RuntimeError, match='entry mismatch'):
        n.start_final_joint_goal(expected)
    for position, angle, accepted in [
            (.0079, math.radians(4.99), True),
            (.0081, 0., False), (0., math.radians(5.01), False)]:
        with patch('tool_docking_coordinator.pose_errors', return_value=(position, angle)):
            if accepted:
                n.start_final_joint_goal(entry)
            else:
                with pytest.raises(RuntimeError, match='entry mismatch'):
                    n.start_final_joint_goal(entry)


@pytest.mark.parametrize('joint_error,accepted', [(.004, True), (.006, False)])
@pytest.mark.parametrize('path_id', [0, 1])
def test_fixed_joint_completion_uses_measured_joint_tolerance(
        tmp_path, joint_error, accepted, path_id):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[path_id]
    n.path_id = path_id
    n.status = {'epoch': 'final-hold'}
    goal = n.cfg['final_joint_goal'].copy()
    measured = goal.copy()
    measured[1] += joint_error
    n.full_state = dict(zip(ARM_JOINTS, measured))
    n.velocity = np.zeros(6)
    n.advance_locked = True
    n.advance_plan = {'waypoints': [goal]}
    n.advance_target = np.eye(4)
    n.advance_local_axis = n.advance_direction = np.array([0., 0., 1.])
    # A joint-measured final goal is not rejected by an independently stricter
    # Cartesian tolerance amplified through FK.
    n.model.fk.return_value = transform([.010, 0., 0.], [0., 0., 0., 1.])

    if accepted:
        with patch('tool_docking_coordinator.time.monotonic', return_value=100.):
            n.path_completed()
        assert n.actuation_deadline == 103.
        n.docking_command.publish.assert_called_once()
        assert n.docking_command.publish.call_args.args[0].data is True
        assert n.stage == 'dock_attach_wait'
    else:
        with pytest.raises(RuntimeError, match='joint_error=0.006000'):
            n.path_completed()
        n.docking_command.publish.assert_not_called()


@pytest.mark.parametrize('acknowledged', [False, True])
@pytest.mark.parametrize('path_id', [0, 1])
def test_fixed_joint_three_second_delay_then_retreat_without_ee(tmp_path, acknowledged, path_id):
    n = node(tmp_path)
    n.path_id = path_id
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[path_id]
    n.guard = Mock()
    n.lock_goal = np.zeros(6)
    n.stage = 'dock_attach_wait'
    n.actuation_deadline = 103.
    n.command_feedback_sequence = 0
    n.docked_feedback_sequence = int(acknowledged)
    n.docked_state = acknowledged
    n.docking_status = 'DOCKED' if acknowledged else 'MOVING'
    n.mode = Mock()
    n.publisher = Mock()
    # Movement during the Arduino delay must not compare against lock_goal.
    n.full_state[ARM_JOINTS[0]] = .5
    n.velocity[0] = .1
    for now in (100., 101., 102.999):
        n.tick_timed_finish(now)
        n.begin_docking_retreat.assert_not_called()
    n.velocity[0] = 0.
    n.tick_timed_finish(103.)
    n.begin_docking_retreat.assert_called_once()
    assert n.docking_retreat_authorized
    assert not n.active
    n.docking_command.publish.assert_not_called()  # no second/false pulse
    n.publisher.publish.assert_not_called()  # no EE command
    n.mode.call_async.assert_not_called()
    with patch.object(ToolTrajectoryJoy, 'tick'):
        n.tick()
    n.begin_docking_retreat.assert_called_once()


@pytest.mark.parametrize('failure', ['speed', 'ownership'])
def test_tool0_delay_does_not_bypass_arm_guards(tmp_path, failure):
    n = node(tmp_path)
    n.guard = Mock()
    n.lock_goal = np.zeros(6)
    n.stage = 'dock_attach_wait'
    n.actuation_deadline = 3.
    if failure == 'speed':
        n.velocity[0] = .1
    else:
        n.guard.side_effect = RuntimeError('ownership changed')
    with pytest.raises(RuntimeError) as failure_info:
        n.tick_timed_finish(3.)
    if failure == 'speed':
        assert 'base_joint speed=0.100000rad/s' in str(failure_info.value)
    n.begin_docking_retreat.assert_not_called()


@pytest.mark.parametrize('status', ['ERROR', 'UNKNOWN'])
def test_tool0_arduino_failure_prevents_retreat(tmp_path, status):
    n = node(tmp_path)
    n.guard = Mock()
    n.lock_goal = np.zeros(6)
    n.stage = 'dock_attach_wait'
    n.docking_status = status
    n.mode = Mock()
    with pytest.raises(RuntimeError, match='Arduino unavailable'):
        n.tick_timed_finish(1.)
    n.mode.call_async.assert_not_called()
    n.begin_docking_retreat.assert_not_called()


@pytest.mark.parametrize('path_id', [0, 1])
def test_fixed_joint_disabled_final_motion_stays_in_hold(tmp_path, path_id):
    n = node(tmp_path)
    n.path_id = path_id
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[path_id]
    n.docking_started = None
    n.motion_enabled = False
    n.advance_locked = False
    n.start_final_joint_goal = Mock()
    n.path_completed()
    n.start_final_joint_goal.assert_not_called()
    n.docking_command.publish.assert_not_called()
    n.reset.assert_called_once()


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


@pytest.mark.parametrize('enabled,motion,place_test,script', [
    (None, None, None, 'tool_docking_coordinator.py'),
    ('false', None, None, 'tool_trajectory_joy.py'),
    ('true', 'false', None, 'tool_docking_coordinator.py'),
    ('true', 'false', 'true', 'tool2_pick_place_coordinator.py')])
def test_launch_selects_exactly_one_path_input_owner(enabled, motion, place_test, script):
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
    if place_test is not None:
        values['enable_tool2_place_test'] = place_test
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
    path_nodes = [n for n in nodes if any(
        Path(a).name in ('tool_trajectory_joy.py', 'tool_docking_coordinator.py',
                         'tool2_pick_place_coordinator.py')
        for a in n.get('arguments', []))]
    assert len(path_nodes) == 1
    assert Path(path_nodes[0]['arguments'][0]).name == script
    if script == 'tool_docking_coordinator.py':
        parameters = path_nodes[0]['parameters'][-1]
        assert parameters['docking_motion_enabled'] is (motion != 'false')
        assert parameters['docking_camera_frame'] == 'arm_camera_color_optical_frame'
        assert set(load_settings(parameters['docking_config'])) == {0, 1}
    if script == 'tool2_pick_place_coordinator.py':
        parameters = path_nodes[0]['parameters'][-1]
        assert parameters['tool2_place_test_enabled'] is True


def camera_node(tmp_path):
    n = node(tmp_path)
    # Preserve coverage of the optional legacy camera-normal implementation.
    n.cfg = load_settings(ROOT/'config/test/tool_docking.yaml')[1]
    n.cfg['advance_distance_m'] = .030
    # Fixed-advance unit tests isolate the legacy line planner. Separate tests
    # below cover the measured pre-dock correction added in front of it.
    n.cfg['measured_pre_dock_alignment_enabled'] = False
    n.cfg['maintain_entry_orientation'] = False
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


def tool1_confirmation_node(tmp_path, confirm=True):
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[1]
    n.cfg.pop('blocked_reason', None)  # Inspect candidate geometry/optional tag behavior only.
    if confirm:
        n.cfg['confirm_tag_before_motion'] = True
    n.path_id = 1
    n.advance_locked = False
    n.docking_started = None
    n.status = {'epoch': 'observation-hold'}
    n.start_final_joint_goal = Mock()
    n.reset_tag_acquisition = Mock()
    n.path_completed()
    if confirm:
        assert n.stage == 'dock_tag_confirm'
    assert n.execution_id is None
    return n


def test_tool1_default_starts_final_path_without_tag_acquisition(tmp_path):
    n = tool1_confirmation_node(tmp_path, confirm=False)
    n.start_final_joint_goal.assert_called_once()
    n.reset_tag_acquisition.assert_not_called()
    assert n.stage != 'dock_tag_confirm'


def test_tool1_waits_for_distinct_stopped_tag_feedback_then_checks_joint_path(tmp_path):
    n = tool1_confirmation_node(tmp_path)
    n.observe_after_ns = 10
    n.observation = None
    n.tick_tag_confirmation(n.docking_started)
    n.start_final_joint_goal.assert_not_called()
    for index in range(8):
        n.observation = (11+index, 0., np.eye(4))
        n.feedback_sequence += 1
        n.tick_tag_confirmation(n.docking_started)
        for _ in range(5):
            n.tick_tag_confirmation(n.docking_started)
        assert n.start_final_joint_goal.call_count == int(index == 7)
    n.start_final_joint_goal.assert_called_once()
    n.begin.assert_not_called()
    n.docking_command.publish.assert_not_called()
    assert n.hold_epoch == 'observation-hold'


def test_tool1_tag_confirmation_resets_on_motion_or_stale_feedback_and_times_out(tmp_path):
    n = tool1_confirmation_node(tmp_path)
    n.observe_after_ns = 0
    n.stable = 4
    n.observation = (1, 0., np.eye(4))
    n.velocity[0] = .02
    n.tick_tag_confirmation(n.docking_started)
    assert n.stable == 0
    n.velocity[:] = 0.
    n.stable = 4
    n.fresh_observation = Mock(side_effect=RuntimeError('stale'))
    n.tick_tag_confirmation(n.docking_started)
    assert n.stable == 0 and n.observation is None
    with pytest.raises(RuntimeError, match='tag confirmation timed out'):
        n.tick_tag_confirmation(n.docking_started+11.)
    n.start_final_joint_goal.assert_not_called()


def test_tool1_clear_observation_and_reverse_keep_exact_final_capture(tmp_path):
    from tool_trajectory_joy import ArmKinematics
    n = node(tmp_path)
    n.cfg = load_settings(ROOT/'config/tool_docking.yaml')[1]
    n.cfg.pop('blocked_reason', None)  # Inspect candidate geometry/optional tag behavior only.
    from tool_trajectory_joy import load_pre_dock_targets
    n.path_id = 1
    n.model = ArmKinematics(
        ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        ROOT/'config/hardware.yaml')
    observation = load_pre_dock_targets(ROOT/'config/cartesian_targets.yaml')[1]['joint_goal']
    original = np.array([-3.1446606151657712, .43528509147893596, 1.1995647203271922,
                         -1.5854570285957827, -.33747577333504797, -1.4066603824909265])
    assert n.model.within_limits(observation, .02)
    assert not n.model.within_limits(original, .02)
    n.start_final_joint_goal(observation)
    forward = n.advance_plan
    assert not forward['cartesian_line']
    assert forward['preserve_joint_path']
    assert not forward['allowed_collision_pairs']
    assert forward['final_docking_entry']
    assert forward['final_entry_ignored_collision_links'] == frozenset({'tool_case_y_plus'})
    np.testing.assert_array_equal(forward['waypoints'][-1], n.cfg['final_joint_goal'])
    final_pose = n.model.fk(n.cfg['final_joint_goal'])
    axis = final_pose[:3, 2]
    axial_start = forward['axial_start_index']
    entry = n.model.fk(forward['waypoints'][axial_start-1])
    np.testing.assert_allclose(entry[:3, 3], final_pose[:3, 3]-.040*axis, atol=1e-5)
    assert forward['stop_at_waypoints'][axial_start-1]
    assert sum(forward['stop_at_waypoints'][axial_start:]) == 1
    last_remaining = 1.
    previous = observation
    for index, (goal, duration) in enumerate(zip(forward['waypoints'], forward['durations'])):
        assert n.model.within_limits(goal, .02)
        for sample in np.linspace(previous, goal, 21):
            assert n.model.within_limits(sample, .02)
            pose = n.model.fk(sample)
            delta = pose[:3, 3]-final_pose[:3, 3]
            remaining = -float(delta@axis)
            if index >= axial_start:
                assert remaining <= last_remaining+1e-6
            last_remaining = remaining
            if index >= axial_start:
                assert np.linalg.norm(delta-axis*(delta@axis)) < 1e-5
                assert pose_errors(pose, final_pose)[1] < .001
        assert np.max(np.abs(goal-previous)) <= n.cfg['joint_step_rad']
        assert np.max(np.abs(goal-previous))/duration <= n.cfg['joint_speed_rad_s']+1e-12
        travel = np.linalg.norm(n.model.fk(goal)[:3, 3]-n.model.fk(previous)[:3, 3])
        assert travel/duration <= n.cfg['linear_speed_m_s']+1e-12
        previous = goal
    assert pose_errors(n.model.fk(observation), n.advance_target)[1] > .1
    live = n.cfg['final_joint_goal'].copy()
    live[0] += .001
    n.full_state = dict(zip(ARM_JOINTS, live))
    n.docking_retreat_authorized = False
    with pytest.raises(RuntimeError, match='not authorized'):
        ToolTrajectoryJoy.begin_reverse_docking_retreat(n)
    n.docking_retreat_authorized = True
    n.begin.reset_mock()
    ToolTrajectoryJoy.begin_reverse_docking_retreat(n)
    reverse = n.begin.call_args.args[1]
    np.testing.assert_array_equal(reverse['start'], live)
    np.testing.assert_array_equal(reverse['waypoints'][-1], observation)
    np.testing.assert_allclose(reverse['waypoints'],
                               list(reversed([observation, *forward['waypoints'][:-1]])))
    assert reverse['preserve_joint_path']
    assert not reverse['allowed_collision_pairs']
    assert reverse['final_entry_ignored_collision_links'] == frozenset({'tool_case_y_plus'})
    assert reverse['final_docking_retreat']
    assert 'final_docking_entry' not in reverse
    assert n.return_home_after_back


def test_tool1_reverse_completion_plans_back_then_home_without_extra_retreat(tmp_path):
    from tool_trajectory_joy import load_pre_dock_targets, load_route_poses
    n = node(tmp_path)
    n.path_id = 1
    n.route_kind = 'dock_retreat'
    n.pre_dock_targets = load_pre_dock_targets(ROOT/'config/cartesian_targets.yaml')
    n.begin_back = Mock()
    n.begin_return_waypoint = Mock()
    ToolTrajectoryJoy.path_completed(n)
    n.begin_back.assert_called_once_with(via_home=False)
    n.begin_return_waypoint.assert_not_called()
    n.route_kind = 'back'
    n.route_poses = load_route_poses(
        (ROOT.parent/'tool_manipulator_moveit_config/config/tool_manipulator.srdf').read_text())
    n.full_state = dict(zip(ARM_JOINTS, n.route_poses['back']))
    n.status = {'epoch': 'back-hold'}
    n.back_tolerance = .03
    n.return_home_after_back = True
    n.begin_named_route = Mock()
    ToolTrajectoryJoy.path_completed(n)
    n.begin_named_route.assert_called_once_with('home')


@pytest.mark.parametrize('field,value', [
    ('confirm_tag_before_motion', 'true'), ('return_home_after_back', 1),
    ('actuation_completion', 'unknown'), ('actuation_delay_s', 0.),
    ('actuation_delay_s', float('nan')), ('actuation_delay_s', True)])
def test_fixed_joint_completion_policy_rejects_invalid_values(tmp_path, field, value):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][1][field] = value
    file = tmp_path/'invalid_policy.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match=field):
        load_settings(file)


@pytest.mark.parametrize('links', [['circuit_box'], ['tool_case_1'],
                                  ['tool_case_y_plus', 'cam_link'], 'tool_case_y_plus'])
def test_tool1_final_entry_exception_cannot_expand_to_other_links(tmp_path, links):
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][1]['final_entry_ignored_collision_links'] = links
    file = tmp_path/'invalid_fixture_exception.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='final_entry_ignored_collision_links'):
        load_settings(file)


@pytest.mark.parametrize('margin', [0., .009, .101, True, float('nan')])
def test_tool_margin_cannot_remove_soft_limit_clearance(tmp_path, margin):
    from tool_trajectory_joy import load_pre_dock_targets
    data = yaml.safe_load((ROOT/'config/tool_docking.yaml').read_text())
    data['tools'][1]['limit_margin_rad'] = margin
    file = tmp_path/'margin.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='limit_margin_rad'):
        load_settings(file)
    targets = yaml.safe_load((ROOT/'config/cartesian_targets.yaml').read_text())
    targets['targets']['tool0']['limit_margin_rad'] = margin
    file.write_text(yaml.safe_dump(targets))
    with pytest.raises(ValueError, match='limit_margin_rad'):
        load_pre_dock_targets(file)


def camera_sample(n):
    n.feedback_sequence += 1
    n.observation = (n.observation[0]+1, 0., n.observation[2])
    n.observe(1.)


def test_camera_mode_corrects_measured_pre_dock_position_before_advance(tmp_path):
    n = camera_node(tmp_path)
    n.cfg['measured_pre_dock_alignment_enabled'] = True
    # With the tag at base Z=.2 and a .03 m final advance, this measurement
    # places pre-dock at base [+.01, 0, 0].
    n.cfg['measured_tag_tcp_target'] = transform(
        [.01, 0., -.17], [0., 0., 0., 1.])
    n.model.solve.return_value = np.zeros(6)
    n.model.within_limits.return_value = True
    n.model.pose_error.return_value = np.zeros(6)

    camera_sample(n)

    n.model.solve.assert_called_once()
    increment = n.model.solve.call_args.args[0]
    np.testing.assert_allclose(increment[:3, 3], [.002, 0., 0.])
    assert not n.advance_locked
    n.begin.assert_called_once()


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


def test_maintained_entry_orientation_uses_full_pose_ik_for_straight_advance(tmp_path):
    n = camera_node(tmp_path)
    n.cfg['maintain_entry_orientation'] = True
    q0 = np.zeros(6)
    origin = np.eye(4)
    n.start_advance(q0, origin, np.array([0., 0., 1.]))
    poses = {tuple(q0): origin}

    def solve(target, seed, *args, **kwargs):
        solved = seed+0.0001
        poses[tuple(solved)] = target.copy()
        return solved

    n.model.solve.side_effect = solve
    n.model.fk.side_effect = lambda q: poses[tuple(q)]
    n.model.within_limits.return_value = True
    for _ in range(n.advance_count):
        n.plan_advance_step()

    n.model.solve_axis.assert_not_called()
    for call in n.model.solve.call_args_list:
        np.testing.assert_allclose(call.args[0][:3, :3], origin[:3, :3])
    _, plan = n.begin.call_args.args
    assert len(plan['waypoints']) == n.advance_count


def test_failed_advance_ik_never_submits_partial_path(tmp_path):
    n = camera_node(tmp_path)
    for _ in range(n.cfg['stable_samples']):
        camera_sample(n)
    n.model.solve_axis.side_effect = ValueError('IK iteration would violate soft limits')
    with pytest.raises(ValueError, match='soft limits'):
        n.plan_advance_step()
    n.begin.assert_not_called()


@pytest.mark.parametrize('module', ['tool_docking_coordinator', 'tool_docking_coordinator_test'])
@pytest.mark.parametrize('joint,ignored', [('ee_joint', True), ('shoulder_joint', False)])
def test_advance_planning_ignores_ee_position_only(tmp_path, module, joint, ignored):
    from importlib import import_module
    n = camera_node(tmp_path)
    n.full_state['ee_joint'] = 0.
    for _ in range(n.cfg['stable_samples']):
        camera_sample(n)
    n.full_state[joint] += 20.
    # Stop at the first solve, after the snapshot gate, without submitting motion.
    n.model.solve_axis.side_effect = ValueError('test solve reached')
    error = ValueError if ignored else RuntimeError
    message = 'test solve reached' if ignored else 'robot moved'
    with pytest.raises(error, match=message):
        import_module(module).ToolDockingCoordinator.plan_advance_step(n)
    assert n.model.solve_axis.called is ignored
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
def test_tool1_approach_continues_after_l2_release_and_joy_stops(tmp_path, stage):
    n = camera_node(tmp_path)
    n.stage = stage
    n.docking_started = None
    n.deadman_held, n.last_joy_time = False, -math.inf
    with patch.object(ToolTrajectoryJoy, 'tick') as parent:
        n.tick()
    parent.assert_called_once()
    n.fail.assert_not_called()


def test_tool1_completion_automatically_aligns_and_plans_advance_without_joy(tmp_path):
    n = camera_node(tmp_path)
    tag = n.observation[2].copy()
    n.docking_started = None
    n.stage, n.execution_id = 'executing', 'tool1'
    n.path, n.execution_timeout, n.started = dict(total=10.), 40., 0.
    n.deadman_held, n.last_joy_time = False, -math.inf
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
                detections=[dict(id=n.cfg['tag_id'], family='tag36h11', hamming=0,
                                 decision_margin=70.62,
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
    if failure == 'wrong_id': data['detections'][0]['id'] = n.cfg['tag_id']+1
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
    n.stable = 3
    data['detections'] = []
    n.on_tag(SimpleNamespace(data=json.dumps(data)))
    assert n.observation is accepted
    assert n.stable == 3
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


def test_stale_first_acquisition_sample_is_discarded_while_waiting_for_fresh_tag(tmp_path):
    n = camera_node(tmp_path)
    n.stage = 'dock_acquire'
    n.docking_started = 1.
    n.guard = Mock()
    n.last_base_tag = np.eye(4)
    n.observation = (1, 1., np.eye(4))
    n.fresh_observation = lambda now: (_ for _ in ()).throw(
        RuntimeError('target tag stale: received=1 accepted=1'))
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.3):
        n.tick()
    assert n.observation is None and n.last_base_tag is None
    assert n.stable == 0
    assert 'waiting for fresh tag' in n.tag_reason
    n.fail.assert_not_called()


def test_stale_camera_normal_sample_returns_to_acquisition_before_advance(tmp_path):
    n = camera_node(tmp_path)
    n.stage = 'dock_observe'
    n.docking_started = 1.
    n.guard = Mock()
    n.advance_locked = False
    n.last_base_tag = np.eye(4)
    n.observation = (1, 1., np.eye(4))
    n.fresh_observation = lambda now: (_ for _ in ()).throw(
        RuntimeError('target tag stale: received=2 accepted=1'))
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.3):
        n.tick()
    assert n.stage == 'dock_acquire'
    assert n.observation is None and n.last_base_tag is None
    assert n.stable == 0
    assert 'waiting for fresh tag' in n.tag_reason
    n.fail.assert_not_called()


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


def test_tool_request_from_tcp_mode_is_blocked_until_verified_back(tmp_path):
    from sensor_msgs.msg import Joy
    n = camera_node(tmp_path)
    n.paths = {1: dict(after_path=None)}
    n.settings = load_settings(ROOT/'config/tool_docking.yaml')
    n.active, n.stage, n.docking_started = False, 'idle', None
    n.combos, n.l2_axis, n.l2_threshold, n.first_button, n.third_button = (
        [False]*3, 2, -.5, 2, 0)
    n.second_button = 1
    n.requested_docked_state, n.use_nearest_entry = None, False
    n.full_state = dict(zip(ARM_JOINTS, np.zeros(6)))
    n.status_time = n.state_time = 1.
    n.status = dict(mode='tcp_ik', epoch='tcp', ready=True, startup_complete=True,
                    focus='arm', transitioning=False)
    with patch('tool_docking_coordinator.time.monotonic', return_value=1.1):
        n.on_joy(Joy(axes=[0., 0., -1.], buttons=[0, 1, 0, 0, 0]))
    assert n.stage == 'idle' and not n.active
    assert 'fresh stopped HOLD' in n.report.call_args.args[1]


@pytest.mark.parametrize('failure', [None, 'feedback', 'focus', 'fault', 'epoch', 'tag', 'timeout'])
def test_automatic_tool1_keeps_docking_guards(tmp_path, failure):
    n = camera_node(tmp_path)
    n.docking_started = 1.
    n.stage = 'executing'
    n.advance_locked = True
    n.deadman_held, n.last_joy_time = False, -math.inf
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
def test_advance_completion_checks_fk_endpoint_before_actuation(tmp_path, endpoint_error):
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
        assert n.stage == 'dock_actuate'
        assert [call.args[0].data for call in
                n.docking_command.publish.call_args_list] == [True]
        assert 'advance completed' in n.report.call_args.args[1]


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
            assert n.stage == 'dock_actuate'
            assert [call.args[0].data for call in
                    n.docking_command.publish.call_args_list] == [True]
        else:
            with pytest.raises(RuntimeError, match='endpoint feedback'):
                n.path_completed()


def test_advance_completion_validates_preserved_orientation_not_tag_normal(tmp_path):
    n = camera_node(tmp_path)
    n.cfg['maintain_entry_orientation'] = True
    n.advance_locked = True
    n.advance_local_axis = np.array([0., 0., 1.])
    n.advance_direction = np.array([np.sin(.24), 0., np.cos(.24)])
    n.advance_target = transform([0., 0., .030], [0., 0., 0., 1.])
    n.model.fk.return_value = n.advance_target.copy()
    n.path_completed()
    assert n.stage == 'dock_actuate'


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
    data['tools'][1]['alignment_mode'] = 'camera_normal_advance'
    data['tools'][1]['actuation_completion'] = 'feedback'
    data['tools'][1]['confirm_tag_before_motion'] = False
    data['tools'][1].pop('final_entry_ignored_collision_links', None)
    data['tools'][1]['advance_distance_m'] = distance
    file = tmp_path/'bad_advance.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='advance_distance_m'):
        load_settings(file)
