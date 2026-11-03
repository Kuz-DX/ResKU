"""Hardware-facing alignment contracts, without starting ROS or motor drivers."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import math
import sys
import threading

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_docking import ManualDocking
from manual_docking_core import dxl_reference, plan_alignment
from tool_trajectory_joy import ArmKinematics


@pytest.fixture
def model():
    root = Path(__file__).resolve().parents[1]
    return ArmKinematics(root.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                         root/'config/hardware.yaml')


def test_tool1_yaw_is_retained_and_tool0_still_zeros_it(model):
    q = np.array([-3.1581177718, .4368706785, 1.2252842862,
                  -1.5580610812, -.3278840540, -1.3966743865])
    tool1 = dxl_reference(q, model, .02, wrist_yaw_rad=q[5])
    np.testing.assert_array_equal(tool1[1:4], q[1:4])
    assert tool1[0] == -math.pi and tool1[4] == 0. and tool1[5] == q[5]
    assert dxl_reference(q, model, .02)[5] == 0.
    for invalid in (float('nan'), 9.):
        with pytest.raises(ValueError):
            dxl_reference(q, model, .02, wrist_yaw_rad=invalid)
    cfg = dict(mode='rmd_pitch', wrist_yaw_rad=q[5], angle_tolerance_rad=.0174533,
               max_angle_rad=.6, orientation_step_rad=.02,
               acceleration_rad_s2=.25, joint_sample_rad=.005, position_tolerance_m=.001)
    plan = plan_alignment(model, q, model.fk(tool1), cfg,
                          np.full(6, .06), .02, threading.Event())
    assert all(model.within_limits(sample, .02) for sample in plan['samples'])
    assert all(sample[5] == q[5] for sample in plan['samples'])


@pytest.mark.parametrize('tool,expected', [(0, 0.), (1, -1.4)])
def test_yaw_captured_at_selection_not_later_tag_acquisition(tool, expected):
    d = ManualDocking.__new__(ManualDocking)
    d.core = SimpleNamespace(selected=False, monitoring=False, tool=None,
        settings={'tools': {tool: {'alignment_enabled': True}}}, begin=Mock())
    d.clear_plan = Mock()
    q = np.array([-3., .2, 1., -1.4, -.3, -1.4])
    n = SimpleNamespace(controller_ready=True, focus='arm', neutral_required=False,
        hardware_fault=False, fault=False, startup_recovering=False,
        entry_alignment_complete=True, now_seconds=lambda: 1., fresh_state=lambda _: True,
        valid_measured=lambda _: True, stop=Mock(), last_dt=.02,
        measured=lambda: q.copy(), get_logger=Mock(), get_clock=Mock())
    d.node = n
    d.select(tool)
    assert d.request_wrist_yaw == expected
    q[5] = -.7
    assert d.request_wrist_yaw == expected


def executing_alignment():
    d = ManualDocking.__new__(ManualDocking)
    d.core = SimpleNamespace(monitoring=False, capture=Mock(return_value=True), expires=100.,
        started=0., cfg={'acquire_timeout_s': 5.}, target=np.eye(4),
        settings={'alignment': {'position_tolerance_m': .001, 'angle_tolerance_rad': .0174533},
                  'tools': {1: {'alignment_reference': 'camera_plane'}}},
        tool=1, errors={'angle_rad': 0.}, monitor=Mock())
    reference = np.array([-math.pi, .3, 1.1, -1.4, 0., -1.4])
    goal = reference.copy()
    goal[1] += .01
    start = reference.copy()
    start[0] += .02
    d.plan = dict(start=start, reference=reference, points=[reference.copy(), goal],
                  durations=[1., 1.])
    d.elapsed, d.deadline, d.stage, d.dxl_prepared = 1., 10., 'executing', False
    d.held_ee = 0.
    d.collision_client = Mock()
    d.model = Mock()
    d.model.fk.return_value = np.eye(4)
    d.fail = Mock()
    d.node = SimpleNamespace(measured=Mock(return_value=start), path_l2_held=False,
        joy=SimpleNamespace(buttons=[0]*11), neutral=lambda: True, neutral_required=False,
        positions={'ee_joint': 0.}, state_times={'ee_joint': 1.},
        params={'joint_state_timeout_sec': .3, 'max_target_lead_rad': .05},
        apply_motion=Mock(), get_logger=Mock())
    return d


def test_dxl_settles_before_rmd_time_advances():
    d = executing_alignment()
    d.run(1., .02)
    d.fail.assert_not_called()
    assert d.elapsed == 1. and not d.dxl_prepared
    np.testing.assert_allclose(d.node.apply_motion.call_args.args[0], d.plan['reference'])
    d.node.measured.return_value = d.plan['reference'].copy()
    d.run(1., .02)
    assert d.dxl_prepared and d.elapsed > 1.
    d.fail.assert_not_called()
    d.node.measured.return_value[5] += .01
    d.run(1., .02)
    assert 'DXL reference' in d.fail.call_args.args[0]


def test_dxl_wait_keeps_timeout_and_following_error_guards():
    d = executing_alignment()
    d.node.state_times['ee_joint'] = 11.
    d.run(11., .02)
    assert 'execution timeout' in d.fail.call_args.args[0]
    d.node.apply_motion.assert_not_called()
    d = executing_alignment()
    d.node.measured.return_value[0] += .1
    d.run(1., .02)
    assert 'following error' in d.fail.call_args.args[0]
    d.node.apply_motion.assert_not_called()


def test_async_planner_receives_button_edge_yaw_not_later_measurement():
    from concurrent.futures import Future
    d = executing_alignment()
    d.stage = 'idle'
    d.request_wrist_yaw = -1.4
    d.node.measured.return_value[5] = -1.3
    d.core.cfg.update(max_target_angle_rad=.6, joint_speed_rad_s=.06)
    d.core.settings['alignment']['planning_timeout_s'] = 30.
    d.node.params['limit_margin_rad'] = .02
    d.node.speeds = np.full(6, .06)
    d.node.publish_ee_velocity = Mock()
    d.node.hold = Mock()
    d.executor = Mock()
    d.executor.submit.return_value = Future()
    d.tcp_camera_rotation = np.eye(3)
    d.run(1., .02)
    d.fail.assert_not_called()
    args = d.executor.submit.call_args.args
    assert args[2][5] == -1.3
    assert args[4]['wrist_yaw_rad'] == -1.4
