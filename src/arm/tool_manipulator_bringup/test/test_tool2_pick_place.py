"""No-motor checks for sequence ordering, floor geometry and runtime dispatch."""
from pathlib import Path
import sys
from unittest.mock import Mock
from unittest.mock import patch

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool2_sequence import PickPlaceSequence, checked_config, STAGES
from person_place_geometry import (placement_targets, placement_targets_for_direction,
                                   person_on_floor, person_direction_on_floor,
                                   aligned_base)
from tool2_pick_place_coordinator import Tool2PickPlaceCoordinator
from tool_trajectory_joy import ARM_JOINTS


def config():
    data = yaml.safe_load((ROOT/'config/tool2_pick_place.yaml').read_text())
    data.update(enabled=True, floor_frame='floor', close_rad=1., open_rad=0.,
                ee_speed_rad_s=.05, place_radius_m=.4, placement_mode='radius',
                tcp_to_object_bottom_m=[0., 0., -.2], place_orientation_xyzw=[0., 0., 0., 1.])
    return checked_config(data)


def test_unmeasured_configuration_cannot_enable_sequence():
    data = yaml.safe_load((ROOT/'config/tool2_pick_place.yaml').read_text())
    with pytest.raises(ValueError, match='disabled'):
        checked_config(data)
    data['enabled'] = True
    with pytest.raises(ValueError, match='close_rad'):
        checked_config(data)


def test_place_test_configuration_does_not_invent_gripper_calibration():
    data = yaml.safe_load((ROOT/'config/tool2_pick_place.yaml').read_text())
    result = checked_config(data, place_test=True)
    assert result['enabled'] is False
    assert result['close_rad'] is None and result['open_rad'] is None


def test_sequence_order_rejects_duplicate_and_failed_completions():
    sequence = PickPlaceSequence()
    for current, following in zip(STAGES, STAGES[1:]):
        assert sequence.complete(current) == following
        with pytest.raises(ValueError):
            sequence.complete(current)
    sequence = PickPlaceSequence()
    sequence.fail()
    with pytest.raises(ValueError):
        sequence.complete('approach')


def test_object_bottom_not_tcp_reaches_floor_and_prepose_is_above():
    cfg = config()
    floor_base = np.eye(4)
    floor_base[:3, 3] = [1., 2., .8]
    pre, final = placement_targets([1., 4., 0.], floor_base, cfg)
    bottom = final @ np.r_[cfg['tcp_to_object_bottom_m'], 1.]
    np.testing.assert_allclose(bottom, [1., 2.4, 0., 1.], atol=1e-12)
    assert final[2, 3] == pytest.approx(.2)
    assert pre[2, 3]-final[2, 3] == pytest.approx(.1)
    with pytest.raises(ValueError, match='too close'):
        placement_targets([1.1, 2., 0.], floor_base, cfg)


@pytest.mark.parametrize('bearing', [0., .7, -1.2, 2.8])
def test_measured_half_height_and_vertical_downward_placement(bearing):
    cfg = yaml.safe_load((ROOT/'config/tool2_pick_place.yaml').read_text())
    cfg.update(placement_mode='radius', place_radius_m=.4)
    person = [2*np.cos(bearing), 2*np.sin(bearing), 0.]
    pre, final = placement_targets(person, np.eye(4), cfg)
    np.testing.assert_allclose(final[:3, 2], [0., 0., -1.], atol=1e-12)
    assert final[2, 3] == pytest.approx(.0475)
    assert pre[2, 3] == pytest.approx(.1475)
    bottom = final @ np.r_[cfg['tcp_to_object_bottom_m'], 1.]
    assert bottom[2] == pytest.approx(0.)


def test_foot_ray_uses_full_box_and_calibrated_floor():
    camera = np.diag([1., -1., -1., 1.])
    camera[2, 3] = 1.
    matrix = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    payload = dict(image_size=dict(width=640, height=480), detections=[dict(
        class_name='person', confidence=.9, xyxy=[300., 100., 340., 340.])])
    np.testing.assert_allclose(person_on_floor(payload, matrix, np.zeros(5), camera, .8), [0., -.2, 0.])
    payload['detections'][0]['xyxy'][3] = 480.
    with pytest.raises(ValueError, match='clipped'):
        person_on_floor(payload, matrix, np.zeros(5), camera, .8)
    payload['detections'] *= 2
    with pytest.raises(ValueError, match='exactly one'):
        person_on_floor(payload, matrix, np.zeros(5), camera, .8)


def test_direction_only_accepts_clipped_person_and_uses_bbox_centre_x():
    camera = np.eye(4)
    camera[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    matrix = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    payload = dict(image_size=dict(width=640, height=480), detections=[dict(
        class_name='person', confidence=.9, xyxy=[400., 0., 500., 480.])])
    direction = person_direction_on_floor(payload, matrix, np.zeros(5), camera, .8)
    expected = np.array([1., -.26])
    expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(direction, expected)


def test_direction_only_target_is_chassis_boundary_without_person_distance():
    cfg = config()
    cfg.update(placement_mode='chassis_edge', chassis_half_extents_m=[.39, .25],
               edge_clearance_m=0.)
    base = np.eye(4)
    base[:3, 3] = [.1921, 0., .235]
    _, target = placement_targets_for_direction([1., 0.], base, cfg, np.eye(4))
    bottom = target @ np.r_[cfg['tcp_to_object_bottom_m'], 1.]
    np.testing.assert_allclose(bottom[:3], [.39, 0., 0.], atol=1e-12)


@pytest.mark.parametrize('person,expected', [([2., 0., 0.], [.39, 0., 0.]),
                                            ([.1921, 2., 0.], [.1921, .25, 0.])])
def test_vehicle_boundary_comes_from_chassis_not_arm_reach(person, expected):
    cfg = config()
    cfg.update(placement_mode='chassis_edge', chassis_half_extents_m=[.39, .25], edge_clearance_m=0.)
    base = np.eye(4)
    base[:3, 3] = [.1921, 0., .235]
    _, target = placement_targets(person, base, cfg, np.eye(4))
    bottom = target @ np.r_[cfg['tcp_to_object_bottom_m'], 1.]
    np.testing.assert_allclose(bottom[:3], expected, atol=1e-12)


def test_base_alignment_uses_urdf_axis_and_keeps_other_joints():
    from tool_trajectory_joy import ArmKinematics
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    q = np.array(yaml.safe_load((ROOT/'config/tcp_ready.yaml').read_text())['tcp_ready']['positions_rad'])
    initial = model.fk(q)[:3, 3]
    turn = .1
    person = np.array([[np.cos(turn), -np.sin(turn)], [np.sin(turn), np.cos(turn)]]) @ initial[:2]*3.
    result = aligned_base(model, q, np.r_[person, 0.], .02)
    np.testing.assert_array_equal(result[1:], q[1:])
    actual = model.fk(result)[:2, 3]
    np.testing.assert_allclose(actual/np.linalg.norm(actual), person/np.linalg.norm(person), atol=1e-5)


def coordinator():
    n = Tool2PickPlaceCoordinator.__new__(Tool2PickPlaceCoordinator)
    n.pick_sequence = PickPlaceSequence()
    n.pick_config = config()
    n.full_state = dict(zip(ARM_JOINTS, np.ones(6)*.1))
    n.paths = {2: dict(start=np.zeros(6), waypoints=[np.ones(6)*.1], durations=[2.], total=2.)}
    n.start_tolerance, n.max_start_delta, n.approach_speed = .03, .1, .05
    n.margin = .02
    n.pick_ready = np.ones(6)*.01
    n.place_test_enabled = False
    n.place_test_fallback_extension = .20
    n.begin, n.report, n.reset = Mock(), Mock(), Mock()
    n.status = dict(epoch='stopped')
    return n


def test_place_test_service_requires_home_then_starts_person_stage():
    n = coordinator()
    n.place_test_enabled = True
    n.place_test_home_tolerance = .10
    n.pick_sequence = None
    n.stage = 'idle'
    n.full_state = dict(zip(ARM_JOINTS, [.05, .2, .3, -.4, .02, -.01]))
    n.healthy_hold = Mock(return_value={
        'epoch': 'home-hold',
        'startup_goal_rad': [.8, .2, .3, -.4, .7, -.6],
    })
    response = Mock()
    with patch('tool2_pick_place_coordinator.time.monotonic', return_value=10.):
        n.start_place_test(None, response)
    assert response.success
    assert n.pick_sequence.stage == 'person'
    assert n.stage == 'pick_person'
    assert n.pick_hold_epoch == 'home-hold'


def test_place_test_service_rejects_pose_outside_home_tolerance():
    n = coordinator()
    n.place_test_enabled = True
    n.place_test_home_tolerance = .10
    n.pick_sequence = None
    n.stage = 'idle'
    n.full_state = dict(zip(ARM_JOINTS, [.101, .2, .3, -.4, 0., 0.]))
    n.healthy_hold = Mock(return_value={
        'epoch': 'home-hold',
        'startup_goal_rad': [0., .2, .3, -.4, 0., 0.],
    })
    response = Mock()
    n.start_place_test(None, response)
    assert not response.success
    assert 'exceeds' in response.message
    assert n.pick_sequence is None


def test_place_test_stops_at_preplace_without_descent_or_release():
    n = coordinator()
    n.place_test_enabled = True
    n.pick_sequence.stage = 'preplace'
    n.path_completed()
    n.report.assert_called_once_with(
        'completed', 'tool2 place test reached preplace; remain HOLD')
    n.reset.assert_called_once()
    n.begin.assert_not_called()


def test_place_test_floor_ik_failure_extends_at_home_height():
    n = coordinator()
    n.place_test_enabled = True
    n.pick_sequence.stage = 'align'
    n.pick_preplace = np.eye(4)
    n.pick_direction_base = np.array([1., 0., 0.])
    n.model = Mock()
    home_pose = np.eye(4)
    home_pose[:3, 3] = [.1, .2, .4]
    n.model.fk.return_value = home_pose.copy()
    n.model.solve.side_effect = [ValueError('no floor IK'), np.ones(6)*.2]
    n.current_q = Mock(return_value=np.ones(6)*.1)
    n.request_entry_plan = Mock()
    n.path_completed()
    fallback = n.model.solve.call_args_list[1].args[0]
    np.testing.assert_allclose(fallback[:3, 3], [.3, .2, .4])
    assert n.pick_place_test_fallback
    n.request_entry_plan.assert_called_once()


def test_runtime_grasps_returns_full_route_then_exact_ready_before_person():
    n = coordinator()
    n.path_completed()
    assert n.pick_sequence.stage == 'grasp'
    command = n.begin.call_args.args[1]
    assert command['ee_target_rad'] == 1. and command['ee_speed_rad_s'] == .05
    n.path_completed()
    assert n.pick_sequence.stage == 'return_path'
    np.testing.assert_array_equal(n.begin.call_args.args[1]['waypoints'][-1], np.zeros(6))
    n.path_completed()
    assert n.pick_sequence.stage == 'ready'
    np.testing.assert_array_equal(n.begin.call_args.args[1]['waypoints'][-1], n.pick_ready)
    n.begin.reset_mock()
    n.path_completed()
    assert n.pick_sequence.stage == 'person' and n.stage == 'pick_person'
    n.begin.assert_not_called()


def test_release_is_blocked_until_final_tcp_pose_is_reached():
    n = coordinator()
    n.pick_sequence.stage = 'descend'
    n.pick_final = np.eye(4)
    n.model = Mock()
    n.model.pose_error.return_value = np.array([0., 0., .03, 0., 0., 0.])
    with pytest.raises(ValueError, match='release blocked'):
        n.path_completed()
    n.begin.assert_not_called()


def test_manager_enforces_gripper_speed_bound_and_rejects_invalid_speed():
    from arm_mode_core import ModeCore
    n = ModeCore([(-2., 2.)]*6, [.12]*6)
    n.ready, n.focus = True, 'arm'
    n.feedback([0.]*6, [0.]*6, 1.)
    n.ee_feedback(0., 0., 1.)
    n.mode = n.requested = 'tool_auto'
    with pytest.raises(ValueError, match='ee speed'):
        n.close_ee(n.epoch, 'bad', 1., 1., speed=.5)
    assert n.close_ee(n.epoch, 'good', 1., 1., speed=.04)
    n.run_ee(1.01)
    assert n.ee == pytest.approx(.04)


def test_person_loss_during_descent_stops_without_release():
    n = coordinator()
    n.pick_sequence.stage = 'descend'
    n.pick_started = 1.
    n.stage = 'executing'
    n.person_sample = None
    n.person_last_received = 1.
    n.fail = Mock()
    with patch('tool2_pick_place_coordinator.time.monotonic', return_value=1.1):
        n.tick()
    n.fail.assert_called_once()
    assert 'person observation lost' in n.fail.call_args.args[0]
    n.begin.assert_not_called()


def test_cancelled_hold_does_not_start_alignment():
    n = coordinator()
    n.pick_sequence.stage = 'person'
    n.pick_started = 1.
    n.stage = 'pick_person'
    n.healthy_hold = Mock()
    n.pick_hold_epoch = 'old-owner'
    n.fail = Mock()
    with patch('tool2_pick_place_coordinator.time.monotonic', return_value=1.1):
        n.tick()
    assert 'ownership changed' in n.fail.call_args.args[0]
    n.begin.assert_not_called()
