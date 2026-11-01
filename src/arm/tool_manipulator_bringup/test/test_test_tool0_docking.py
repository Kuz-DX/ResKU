"""Test-only tool0 continuation, without ROS graph or physical actuation."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tool_docking_coordinator_test import ToolDockingCoordinator
from tool_trajectory_joy_test import ARM_JOINTS


def node():
    return SimpleNamespace(
        handle_mid_path_ee=Mock(return_value=False), active=True, advance_locked=True,
        path_id=0, cfg=dict(position_tolerance_m=.002, angle_tolerance_rad=.02,
            stopped_speed_rad_s=.01, actuation_enabled=True, desired_docked=True,
            actuation_timeout_s=10., advance_distance_m=.03,
            alignment_mode='camera_normal_advance', joint_tolerance_rad=.005),
        full_state=dict.fromkeys(ARM_JOINTS, 0.), model=SimpleNamespace(fk=Mock(return_value=np.eye(4))),
        advance_target=np.eye(4), advance_local_axis=np.array([0.,0.,1.]),
        advance_direction=np.array([0.,0.,1.]), velocity=np.zeros(6),
        status={'epoch': 12}, docked_feedback_sequence=3,
        send_docking_pulse=Mock(), report=Mock(), reset=Mock(),
        fresh_observation=Mock(side_effect=AssertionError('locked advance needs no tag')))


def test_tool0_advance_sends_attach_and_requires_new_ack():
    n = node()
    ToolDockingCoordinator.path_completed(n)
    n.send_docking_pulse.assert_called_once_with(True)
    assert n.stage == 'dock_actuate' and n.hold_epoch == 12
    n.docked_state = True
    ToolDockingCoordinator.observe(n, n.actuation_deadline-1.)
    n.reset.assert_not_called()
    n.docked_feedback_sequence += 1
    ToolDockingCoordinator.observe(n, n.actuation_deadline-1.)
    n.reset.assert_called_once()
    n.fresh_observation.assert_not_called()


def test_midpath_opening_does_not_start_attach():
    n = node()
    n.handle_mid_path_ee.return_value = True
    ToolDockingCoordinator.path_completed(n)
    n.send_docking_pulse.assert_not_called()


def test_tool1_keeps_existing_no_actuation_behavior():
    n = node()
    n.path_id = 1
    ToolDockingCoordinator.path_completed(n)
    n.send_docking_pulse.assert_not_called()
    n.reset.assert_called_once()
