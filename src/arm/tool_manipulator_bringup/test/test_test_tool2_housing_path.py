"""Real MoveIt/FCL checks of the test tool2 tail with its housing enabled."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_trajectory_joy_test import (TEST_TOOL2_GRIPPER_LINKS,
                                      collision_result_accepted)


@pytest.fixture(scope='module')
def collision_probe(tmp_path_factory):
    work = tmp_path_factory.mktemp('tool2_housing_collision')
    (work/'CMakeLists.txt').write_text(f'''
cmake_minimum_required(VERSION 3.16)
project(tool2_housing_collision_test)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
add_executable(probe "{ROOT/'test/tool2_housing_collision_probe.cpp'}")
ament_target_dependencies(probe moveit_core urdf srdfdom)
''')
    for command in [['cmake', '-S', str(work), '-B', str(work/'build')],
                    ['cmake', '--build', str(work/'build'), '-j2']]:
        result = subprocess.run(command, capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stdout+result.stderr
    description_dir = ROOT.parent/'tool_manipulator_description'
    description = xacro.process_file(
        str(description_dir/'urdf/tool_manipulator.urdf.xacro'),
        mappings={'use_mesh': 'true', 'tool_id': '0'}).toxml()
    robot = ET.fromstring(description)
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename')
        assert filename.startswith('package://tool_manipulator_description/')
        path = description_dir/filename.split('/', 3)[3]
        assert path.is_file()
        mesh.set('filename', path.resolve().as_uri())
    urdf = work/'tool2.urdf'
    urdf.write_text(ET.tostring(robot, encoding='unicode'))
    srdf = ROOT.parent/'tool_manipulator_moveit_config/config/tool_manipulator.srdf'

    def check(points):
        inputs = ''.join(' '.join(map(str, [*q, 0.]))+'\n' for q in points)
        result = subprocess.run([str(work/'build/probe'), str(urdf), str(srdf)],
                                input=inputs, capture_output=True, text=True, timeout=90)
        assert result.returncode == 0, result.stderr
        responses = []
        for line in result.stdout.splitlines():
            valid, _, pairs = line.partition(' ')
            assert valid in ('0', '1'), result.stdout
            responses.append(SimpleNamespace(valid=valid == '1', contacts=[SimpleNamespace(
                contact_body_1=pair.split(':')[0], contact_body_2=pair.split(':')[1])
                for pair in pairs.strip(',').split(',') if pair]))
        assert len(responses) == len(points)
        return responses

    return check


def test_previous_endpoint_is_rejected_for_its_housing_collision(collision_probe):
    old_goal = [-2.827621855553809, .528253512452711, 1.3501982949941216,
                -1.5523236274122452, .18786343035738898, -.12041749184881656]
    result, = collision_probe([old_goal])
    assert any({c.contact_body_1, c.contact_body_2} == {'wrist_link_yaw', 'supply_box'}
               for c in result.contacts)
    assert not collision_result_accepted(result, ignored_links=TEST_TOOL2_GRIPPER_LINKS)


def test_entire_stored_tool2_path_passes_with_housing_collision_checks(collision_probe):
    data = yaml.safe_load((ROOT/'config/test/resampled/tool2_trajectory.yaml').read_text())
    vertices = np.array([p['positions_rad'] for p in data['points']])
    points = []
    for start, goal in zip(vertices, vertices[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(goal-start))/.005)))
        points.extend(np.linspace(start, goal, count+1))
    assert 'wrist_link_yaw' not in TEST_TOOL2_GRIPPER_LINKS
    for i, result in enumerate(collision_probe(points)):
        assert collision_result_accepted(result, ignored_links=TEST_TOOL2_GRIPPER_LINKS), (
            i, [(c.contact_body_1, c.contact_body_2) for c in result.contacts])
