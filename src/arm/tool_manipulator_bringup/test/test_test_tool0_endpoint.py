"""Tool0 waits clear of the tag mount before live visual docking."""
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT.parent/'tool_change_min'))
from resample_tool_paths import resample
from tool_change_min.kinematics import ArmKinematics, _origin
from tool_trajectory_joy_test import load_path
from test_test_tool2_housing_path import collision_probe


def test_point17_is_80mm_from_blue_center_along_the_physical_docking_normal():
    description_dir = ROOT.parent/'tool_manipulator_description'
    urdf = description_dir/'urdf/tool_manipulator.urdf.xacro'
    description = xacro.process_file(str(urdf), mappings={
        'use_mesh': 'true', 'tool_id': '0'}).toxml()
    robot = ET.fromstring(description)
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}

    def fixed_pose(link):
        if link == 'ground':
            return np.eye(4)
        joint = joints[link]
        assert joint.get('type') == 'fixed'
        return fixed_pose(joint.find('parent').get('link')) @ _origin(joint.find('origin'))

    base_inverse = np.linalg.inv(fixed_pose('base_actuator'))
    center = (base_inverse @ fixed_pose('tool_case_1_center'))[:3, 3]
    circuit_normal = (base_inverse @ fixed_pose('circuit_box'))[:3, 2]
    # Use the actual circular face normals, rather than the case frame axes:
    # the Drawing1 mesh has its own rotation relative to tool_case_1.
    data = (description_dir/'meshes/visual/Drawing1.stl').read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    facets = np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
    vertices = facets['vertices'].astype(float)
    circle = np.array([113.500001, 27.701940, 143.046537])
    nearby = np.linalg.norm(vertices.mean(axis=1)-circle, axis=1) < 12.
    areas = np.linalg.norm(np.cross(vertices[:, 1]-vertices[:, 0],
                                   vertices[:, 2]-vertices[:, 0]), axis=1)/2
    normals, indices = np.unique(facets['normal'][nearby], axis=0, return_inverse=True)
    cad_normal = normals[np.argmax(np.bincount(indices, weights=areas[nearby]))]
    mesh_origin = robot.find("link[@name='tool_case_1']/visual/origin")
    mesh_pose = base_inverse @ fixed_pose('tool_case_1') @ _origin(mesh_origin)
    outward = mesh_pose[:3, :3] @ cad_normal
    outward /= np.linalg.norm(outward)
    model = ArmKinematics(urdf, ROOT/'config/hardware.yaml', description=description)
    source = yaml.safe_load((ROOT/'config/test/tool0_trajectory.yaml').read_text())
    assert len(source['points']) == 17
    tcp = model.fk(np.array(source['points'][16]['positions_rad']))
    assert source['pre_dock_reference'] == dict(center_frame='tool_case_1_center',
        outward_standoff_m=.080, tcp_plane_tilt_deg=55.)
    np.testing.assert_allclose(tcp[:3, 3]-center, .080*outward, atol=1e-8, rtol=0.)
    np.testing.assert_allclose(tcp[:3, 2], -outward, atol=1e-7, rtol=0.)
    plane_tilt = np.degrees(np.arccos(abs(tcp[:3, 2] @ circuit_normal)))
    np.testing.assert_allclose(plane_tilt, 55., atol=1e-7, rtol=0.)


def test_updated_tool0_samples_preserve_vertices_joint_margins_and_speed_limits():
    source = yaml.safe_load((ROOT/'config/test/tool0_trajectory.yaml').read_text())
    generated_path = ROOT/'config/test/resampled/tool0_trajectory.yaml'
    generated = yaml.safe_load(generated_path.read_text())
    assert generated == resample(source)
    points = np.array([p['positions_rad'] for p in generated['points']])
    vertices = np.array([p['positions_rad'] for p in source['points']])
    np.testing.assert_array_equal(points[generated['source_point_indices']], vertices)
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    assert all(model.within_limits(q, .02) for q in points)
    times = np.array([p['time_from_start_sec'] for p in generated['points']])
    speeds = np.abs(np.diff(points, axis=0))/np.diff(times)[:, None]
    assert np.all(speeds <= np.array(generated['nominal_joint_speed_limits_rad_s'])+1e-12)
    assert load_path(generated_path, model) is not None


def test_post_opening_tail_and_fixed_orientation_25mm_advance_are_collision_free(collision_probe):
    source = yaml.safe_load((ROOT/'config/test/tool0_trajectory.yaml').read_text())
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    vertices = np.array([p['positions_rad'] for p in source['points'][14:]])
    points = []
    for start, goal in zip(vertices, vertices[1:]):
        count = int(np.ceil(np.max(np.abs(goal-start))/.002))
        points.extend(np.linspace(start, goal, count+1))
    origin = model.fk(vertices[-1])
    seed = vertices[-1].copy()
    for distance in np.linspace(0., .025, 51):
        target = origin.copy()
        target[:3, 3] += distance*origin[:3, 2]
        seed = model.solve(target, seed, .02)
        np.testing.assert_allclose(model.fk(seed)[:3, :3], origin[:3, :3], atol=1e-4, rtol=0.)
        points.append(seed.copy())
    # Use both observed mock opening results; no collision pairs are ignored.
    for ee in [-3.2, -3.19]:
        for index, result in enumerate(collision_probe(points, ee)):
            assert result.valid and not result.contacts, (ee, index, result.contacts)
