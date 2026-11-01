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


def test_tool2_endpoint_centers_the_box_inside_the_opposing_fingers(endpoint):
    _, source, pose, center, _ = endpoint
    local_center = pose[:3, :3].T @ (center-pose[:3, 3])
    assert local_center[2] == pytest.approx(.110, abs=1e-8)
    np.testing.assert_allclose(local_center,
        source['grasp_reference']['object_center_in_tcp_m'], atol=1e-8, rtol=0.)
    assert abs(local_center[0]) < .005 and abs(local_center[1]) < .005


def test_tool2_first_three_points_are_preserved_with_matching_bases(endpoint):
    _, source, _, _, _ = endpoint
    points = np.array([p['positions_rad'] for p in source['points'][:3]])
    np.testing.assert_array_equal(points, [
        [-.026077673394262924, 1.1910126442766753, 1.1822859980167035, -1.45, 0., 0.],
        [-.026077673394262924, .7539822501773558, 1.1611675300465387,
         -1.3580406483662104, 0., 0.],
        [-.026077673394262924, .7325146417882816, 1.3103932343552758,
         -1.2552407660763887, 0., 0.]])


def test_tool2_replanned_tail_has_no_joint_reversals_or_upward_tcp_motion(endpoint):
    model, source, _, _, normal = endpoint
    points = np.array([p['positions_rad'] for p in source['points'][2:]])
    delta = np.diff(points, axis=0)
    assert np.all(delta*(points[-1]-points[0]) >= -1e-12)
    assert all(model.within_limits(q, .02) for q in points)
    heights = np.array([model.fk(q)[:3, 3] @ normal for q in points])
    assert np.all(np.diff(heights) < 0.)


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
