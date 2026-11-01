"""Test-only tool2 endpoint geometry relative to the physical supply box."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT.parent/'tool_change_min'))
from resample_tool_paths import resample
from tool_change_min.kinematics import ArmKinematics, _origin
from tool_trajectory_joy_test import load_path


@pytest.fixture
def endpoint():
    urdf = ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
    description = xacro.process_file(
        str(urdf), mappings={'use_mesh': 'true', 'tool_id': '0'}).toxml()
    model = ArmKinematics(urdf, ROOT/'config/hardware.yaml', description=description)
    joints = {j.find('child').get('link'): j
              for j in ET.fromstring(description).findall('joint')}

    def fixed_pose(link):
        if link == 'ground':
            return np.eye(4)
        joint = joints[link]
        assert joint.get('type') == 'fixed'
        return fixed_pose(joint.find('parent').get('link')) @ _origin(joint.find('origin'))

    base_inverse = np.linalg.inv(fixed_pose('base_actuator'))
    box = base_inverse @ fixed_pose('supply_box')
    top_normal = (base_inverse @ fixed_pose('circuit_box'))[:3, 2]
    source = yaml.safe_load((ROOT/'config/test/tool2_trajectory.yaml').read_text())
    q = np.array(source['points'][-1]['positions_rad'])
    return model, source, model.fk(q), box[:3, 3], top_normal


def test_tool2_endpoint_has_requested_center_distance_and_elevation(endpoint):
    model, _, pose, center, normal = endpoint
    offset = pose[:3, 3] - center
    height = offset @ normal
    horizontal = offset - height*normal
    assert np.linalg.norm(offset) == pytest.approx(.095*np.sqrt(2)/2, abs=1e-10)
    assert height == pytest.approx(.0475, abs=1e-10)
    assert np.linalg.norm(horizontal) == pytest.approx(.0475, abs=1e-10)
    assert np.arctan2(height, np.linalg.norm(horizontal)) == pytest.approx(np.pi/4, abs=1e-9)
    previous_q = np.array([-2.5280003384357435, .8353145906571304,
                           1.5156438585160097, -1.5091862255047854,
                           -.2316310989709387, -.0828349625456184])
    previous_offset = model.fk(previous_q)[:3, 3] - center
    previous_horizontal = previous_offset - (previous_offset @ normal)*normal
    np.testing.assert_allclose(horizontal/np.linalg.norm(horizontal),
                               previous_horizontal/np.linalg.norm(previous_horizontal),
                               atol=1e-9, rtol=0.)


def test_tool2_tcp_approach_axis_points_down_toward_box_center(endpoint):
    _, _, pose, center, normal = endpoint
    direction = center - pose[:3, 3]
    direction /= np.linalg.norm(direction)
    np.testing.assert_allclose(pose[:3, 2], direction, atol=1e-9, rtol=0.)
    assert pose[:3, 2] @ normal == pytest.approx(-np.sqrt(.5), abs=1e-9)


def test_tool2_selected_target_reduces_travel_and_preserves_previous_wrist_yaw(endpoint):
    model, source, pose, center, normal = endpoint
    previous = np.array(source['points'][-2]['positions_rad'])
    previous_position = model.fk(previous)[:3, 3]
    overhead = center + (.095*np.sqrt(2)/2)*normal
    assert np.linalg.norm(pose[:3, 3]-previous_position) < np.linalg.norm(overhead-previous_position)
    assert source['points'][-1]['positions_rad'][-1] == previous[-1]


def test_tool2_test_path_resampling_preserves_vertices_limits_and_speed_caps(endpoint):
    model, source, _, _, _ = endpoint
    generated_path = ROOT/'config/test/resampled/tool2_trajectory.yaml'
    generated = yaml.safe_load(generated_path.read_text())
    assert generated == resample(source)
    assert len(source['points']) == 17
    points = np.array([p['positions_rad'] for p in generated['points']])
    vertices = np.array([p['positions_rad'] for p in source['points']])
    np.testing.assert_array_equal(points[generated['source_point_indices']], vertices)
    assert all(model.within_limits(q, .02) for q in points)
    times = np.array([p['time_from_start_sec'] for p in generated['points']])
    speeds = np.abs(np.diff(points, axis=0))/np.diff(times)[:, None]
    assert np.all(speeds <= np.array(source['joint_speed_limits_rad_s'])+1e-12)
    assert 'after_path' not in source
    assert load_path(generated_path, model) is not None
