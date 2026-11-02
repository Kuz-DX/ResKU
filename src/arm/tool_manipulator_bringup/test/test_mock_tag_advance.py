"""Mock tag assumptions stay isolated; tool0 preserves its complete entry pose."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_docking_coordinator_test import (ToolDockingCoordinator, mock_optical_extrinsic,
                                           advance_endpoint_errors)
from tool_docking_core import load_settings
from test_tool_docking import camera_node


@pytest.mark.parametrize('plugin', ['', 'rmd_hardware_interface/RMDSystem'])
def test_assumed_tag_success_rejects_real_or_unknown_hardware(plugin):
    description = f'<robot><ros2_control><hardware><plugin>{plugin}</plugin></hardware></ros2_control></robot>'
    with pytest.raises(ValueError, match='exclusively mock'):
        mock_optical_extrinsic(description)


def test_nominal_camera_uses_the_mount_and_correct_optical_forward_axis():
    urdf = ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
    robot = xacro.process_file(str(urdf), mappings={'use_mesh': 'true', 'tool_id': '0'}).toxml()
    # Add only the mock control declaration to the geometry-only description.
    robot = robot.replace('</robot>', '<ros2_control><hardware><plugin>mock_components/GenericSystem</plugin></hardware></ros2_control></robot>')
    camera = mock_optical_extrinsic(robot)
    np.testing.assert_allclose(camera[:3, 3], [0., .094768, -.0189], atol=1e-12)
    np.testing.assert_allclose(camera[:3, 2], [0., 0., 1.], atol=1e-12)


def test_mock_confirmation_is_fresh_but_base_anchor_stays_stationary():
    clock = Mock()
    clock.now.return_value.nanoseconds = 100
    n = SimpleNamespace(mock_tag_success=True, advance_locked=False,
        cfg={'alignment_mode': 'camera_normal_advance'}, mock_base_tag=None,
        full_state=dict.fromkeys(('base_joint', 'shoulder_joint', 'elbow_joint',
            'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint'), 0.),
        mock_tcp_camera=np.eye(4), model=SimpleNamespace(fk=Mock(return_value=np.eye(4))),
        get_clock=Mock(return_value=clock), report=Mock(), tag_accepted=0)
    ToolDockingCoordinator.update_mock_tag_observation(n, 1.)
    anchor = n.observation[2].copy()
    np.testing.assert_allclose(anchor[:3, 3], [0., 0., .2])
    clock.now.return_value.nanoseconds = 200
    ToolDockingCoordinator.update_mock_tag_observation(n, 1.02)
    assert n.observation[:2] == (200, 1.02)
    np.testing.assert_array_equal(n.observation[2], anchor)
    n.report.assert_called_once()
    assert n.tag_accepted == 2


@pytest.mark.parametrize('tool', [0, 1])
def test_25mm_line_holds_entry_orientation_even_if_tag_normal_differs(tmp_path, tool):
    n = camera_node(tmp_path)
    n.cfg = load_settings(ROOT/'config/test/tool_docking.yaml')[tool]
    assert n.cfg['advance_distance_m'] == .025 and n.cfg['maintain_entry_orientation']
    n.path_id = tool
    start = np.eye(4)
    start[:3, :3] = cv2.Rodrigues(np.array([0., .15, 0.]))[0]
    q = np.zeros(6)
    ToolDockingCoordinator.start_advance(n, q, start, np.array([0., 0., 1.]))
    poses = {tuple(q): start}
    def solve(target, seed, *args, **kwargs):
        next_q = seed+.0001
        poses[tuple(next_q)] = target.copy()
        return next_q
    n.model.solve.side_effect = solve
    n.model.fk.side_effect = lambda values: poses[tuple(values)]
    n.model.within_limits.return_value = True
    for _ in range(n.advance_count):
        ToolDockingCoordinator.plan_advance_step(n)
    n.model.solve_axis.assert_not_called()
    for call in n.model.solve.call_args_list:
        np.testing.assert_array_equal(call.args[0][:3, :3], start[:3, :3])
    np.testing.assert_allclose(n.model.solve.call_args.args[0][:3, 3], [0., 0., .025])
    assert len(n.begin.call_args.args[1]['waypoints']) == n.advance_count
    end = n.advance_target.copy()
    distance, angle = advance_endpoint_errors(end, end, np.array([0., 0., 1.]),
                                               np.array([0., 0., 1.]), True)
    assert distance == 0. and angle == 0.
    # Matching only the approach axis cannot accept an unintended axial turn.
    turned = end.copy()
    turned[:3, :3] = end[:3, :3] @ cv2.Rodrigues(np.array([0., 0., .1]))[0]
    assert advance_endpoint_errors(turned, end, np.array([0., 0., 1.]),
                                   np.array([0., 0., 1.]), True)[1] > .09
