"""Tool1 observation approach, tag continuation, and legacy mating policy guards."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
import xacro

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from resample_tool_paths import resample
from tool_trajectory_joy_test import (ARM_JOINTS, ArmKinematics, ToolTrajectoryJoy,
    collision_result_accepted, docking_contact_pairs, execution_plan, load_path,
    nearest_path_suffix, isolate_shoulder_zero_crossings)

POLICY = dict(start_point=15,
              allowed_pairs=[['ee_actuator', 'tool_case_y_plus'],
                             ['wrist_link_yaw', 'tool_case_y_plus']],
              endpoint_hold=True)


def fixture_path(tmp_path, sampled=False):
    data = dict(execution_enabled=True, joint_names=list(ARM_JOINTS),
                terminal_docking_contact=POLICY,
                points=[dict(time_from_start_sec=float(i), positions_rad=[i*.01]*6)
                        for i in range(17)])
    if sampled:
        data = resample(data)
    path = tmp_path/'tool1_trajectory.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize('sampled', [False, True])
def test_contact_policy_is_restricted_to_the_taught_terminal_lines(tmp_path, sampled):
    model = Mock()
    model.within_limits.return_value = True
    path = load_path(fixture_path(tmp_path, sampled), model)
    allowed = path['terminal_docking_contact']['pairs']
    assert docking_contact_pairs(path, [.145]*6) == allowed
    assert docking_contact_pairs(path, [.155]*6) == allowed
    assert not docking_contact_pairs(path, [.135]*6)
    assert not docking_contact_pairs(path, [.161]*6)
    assert not docking_contact_pairs(path, [.145, .146, .145, .145, .145, .145])
    for converted in [execution_plan(path, path['start']),
                      nearest_path_suffix(path, [.13]*6),
                      isolate_shoulder_zero_crossings(path)]:
        assert docking_contact_pairs(converted, [.155]*6) == allowed


def response(*pairs, valid=False):
    return SimpleNamespace(valid=valid, contacts=[SimpleNamespace(
        contact_body_1=a, contact_body_2=b) for a, b in pairs])


def test_mating_contacts_never_hide_camera_or_other_collisions():
    pairs = frozenset(tuple(sorted(pair)) for pair in POLICY['allowed_pairs'])
    mating = ('tool_case_y_plus', 'ee_actuator')
    camera = ('cam_link', 'tool_case_y_plus')
    assert collision_result_accepted(response(mating), pairs)
    assert not collision_result_accepted(response(mating))
    assert not collision_result_accepted(response(mating, camera), pairs)
    assert not collision_result_accepted(response(), pairs)
    assert not collision_result_accepted(None, pairs)
    assert collision_result_accepted(response(valid=True))


def test_policy_cannot_be_expanded_or_attached_to_another_tool(tmp_path):
    path = fixture_path(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['terminal_docking_contact']['allowed_pairs'].append(['cam_link', 'tool_case_y_plus'])
    path.write_text(yaml.safe_dump(data))
    model = Mock()
    model.within_limits.return_value = True
    with pytest.raises(ValueError, match='contact policy'):
        load_path(path, model)
    path = fixture_path(tmp_path)
    other = tmp_path/'tool0_trajectory.yaml'
    other.write_bytes(path.read_bytes())
    with pytest.raises(ValueError, match='contact policy'):
        load_path(other, model)


def test_taught_docked_endpoint_does_not_arm_an_extra_tag_normal_advance():
    from tool_docking_coordinator_test import ToolDockingCoordinator
    node = object.__new__(ToolDockingCoordinator)
    node.paths = {1: {'terminal_docking_contact': {'endpoint_hold': True}}}
    node.settings = {1: {}}
    node.requested_docked_state = None
    node.active = False
    with patch.object(ToolTrajectoryJoy, 'begin') as begin:
        node.begin(1)
    begin.assert_called_once_with(1, None)
    assert not node.active


def test_test_tool1_fixed_vertices_margins_and_resampling():
    source = yaml.safe_load((ROOT/'config/test/tool1_trajectory.yaml').read_text())
    generated = yaml.safe_load((ROOT/'config/test/resampled/tool1_trajectory.yaml').read_text())
    assert generated == resample(source)
    points = np.array([p['positions_rad'] for p in source['points']])
    assert len(points) == 17
    np.testing.assert_array_equal(points[:4], [
        [-.026077673394262924, 1.1910126442766753, 1.1822859980167035, -1.45, 0., 0.],
        [-.026077673394262924, .7539822501773558, 1.1611675300465387, -1.3580406483662104, 0., 0.],
        [.0628932123031043, .7325146417882816, 1.3103932343552758, -1.2552407660763887, 0., 0.],
        [.0628932123031043, .7325146417882816, 1.3103932343552758, -1.2552407660763887, .015339807878649747, .03067961575792033]])
    np.testing.assert_array_equal(points[-1], [3.181114399394, .771358751933,
        1.436267041139, -1.5759526, -.393483321428, -1.593141874779])
    assert np.all(np.diff(points[3:, 5]) < 0.)
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    assert all(model.within_limits(q, .02) for q in points)
    loaded = load_path(ROOT/'config/test/resampled/tool1_trajectory.yaml', model)
    assert 'terminal_docking_contact' not in loaded


def test_tool1_observation_is_80mm_away_and_retains_the_requested_wrist_angle():
    from tool_change_min.kinematics import _origin
    urdf = ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
    description = xacro.process_file(str(urdf), mappings={'use_mesh': 'false', 'tool_id': '99'}).toxml()
    model = ArmKinematics(urdf, ROOT/'config/hardware.yaml', description=description)
    joints = {j.find('child').get('link'): j for j in ET.fromstring(description).findall('joint')}
    def world(link):
        if link == 'arm_world':
            return np.eye(4)
        joint = joints[link]
        assert joint.get('type') == 'fixed'
        return world(joint.find('parent').get('link'))@_origin(joint.find('origin'))
    target = np.linalg.inv(world('base_actuator'))@world('tool_case_y_plus_docking')
    source = yaml.safe_load((ROOT/'config/test/tool1_trajectory.yaml').read_text())
    q = np.array(source['points'][-1]['positions_rad'])
    actual = model.fk(q)
    assert np.linalg.norm(actual[:3, 3]-target[:3, 3]) == pytest.approx(.080, abs=1e-9)
    # Distance alone accepted the former 33-mm lateral miss. Enforce both
    # the actual socket normal and the line from the TCP to its green centre.
    np.testing.assert_allclose(target[:3, 3]-actual[:3, 3],
                               .080*actual[:3, 2], atol=1e-9, rtol=0.)
    mesh = ET.fromstring(description).find("link[@name='tool_case_y_plus']/visual/origin")
    mesh_pose = np.linalg.inv(world('base_actuator'))@world('tool_case_y_plus')@_origin(mesh)
    np.testing.assert_allclose(actual[:3, 2], -mesh_pose[:3, 2], atol=1e-9, rtol=0.)
    assert q[5] == -1.593141874779
    assert source['pre_dock_reference'] == dict(center_frame='tool_case_y_plus_docking',
        center_distance_m=.080, wrist_yaw_rad=-1.593141874779,
        tcp_axis_points_to_center=True, tcp_plane_tilt_deg=55.)
    # Exercise the real fixed-orientation solver along the mock optical normal.
    start = actual.copy()
    for index in range(1, 14):
        next_pose = start.copy()
        next_pose[:3, 3] += start[:3, 2]*.025*index/13
        previous = q.copy()
        q = model.solve(next_pose, q, .02, max_iterations=100)
        assert np.max(np.abs(q-previous)) < .03
        np.testing.assert_allclose(model.fk(q)[:3, :3], start[:3, :3], atol=1e-4, rtol=0.)
        remaining = target[:3, 3]-model.fk(q)[:3, 3]
        lateral = remaining-(remaining@start[:3, 2])*start[:3, 2]
        assert np.linalg.norm(lateral) < 1e-5
        assert model.within_limits(q, .02)
    np.testing.assert_allclose(model.fk(q)[:3, 3]-start[:3, 3],
                               start[:3, 2]*.025, atol=1e-5, rtol=0.)
    assert np.linalg.norm(model.fk(q)[:3, 3]-target[:3, 3]) == pytest.approx(.055, abs=1e-5)


def test_tool1_observation_endpoint_arms_tag1_confirmation_and_25mm_advance():
    from tool_docking_coordinator_test import ToolDockingCoordinator
    from tool_docking_core import load_settings
    node = object.__new__(ToolDockingCoordinator)
    model = Mock()
    model.within_limits.return_value = True
    loaded = load_path(ROOT/'config/test/resampled/tool1_trajectory.yaml', model)
    node.paths = {1: loaded}
    node.settings = load_settings(ROOT/'config/test/tool_docking.yaml')
    node.requested_docked_state = None
    node.active = False
    node.motion_enabled = True
    node.report = Mock()
    node.docking_command = Mock()
    node.docking_command.get_subscription_count.return_value = 1
    node.count_publishers = Mock(return_value=1)
    with patch.object(ToolTrajectoryJoy, 'begin') as begin:
        node.begin(1)
    begin.assert_called_once_with(1, None)
    assert node.active and node.cfg['tag_id'] == 1
    assert node.cfg['advance_distance_m'] == .025
    assert node.cfg['maintain_entry_orientation'] is True
    assert node.cfg['joint_speed_rad_s'] == .015
