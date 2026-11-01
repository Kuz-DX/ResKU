"""ID 3's physical axis changes the model, never the calibrated motor mapping."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

ARM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ARM/'tool_change_min'))
sys.path.insert(0, str(ARM/'tool_manipulator_bringup/scripts'))
from tool_change_min.kinematics import ArmKinematics, _rotvec
from tcp_joy_core import TcpStepper

URDF = ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
HARDWARE = ARM/'tool_manipulator_bringup/config/hardware.yaml'


def description(tool='99'):
    return xacro.process_file(str(URDF), mappings={'use_mesh': 'false', 'tool_id': tool}).toxml()


@pytest.fixture
def model():
    return ArmKinematics(URDF, HARDWARE, description=description())


@pytest.mark.parametrize('tool', ['0', '1', '2', '99'])
def test_every_display_tool_uses_the_actual_negative_wrist_axis(tool):
    root = ET.fromstring(description(tool))
    joint = root.find("joint[@name='wrist_yaw_joint']")
    assert joint.find('axis').get('xyz') == '0 0 -1'
    assert joint.find('limit').get('lower') == '-5.76'
    assert joint.find('limit').get('upper') == '6.5'


def test_fk_and_jacobian_follow_the_axis_without_reinterpreting_joint_states(model):
    old = ET.fromstring(description())
    old.find("joint[@name='wrist_yaw_joint']/axis").set('xyz', '0 0 1')
    legacy = ArmKinematics(URDF, HARDWARE, description=ET.tostring(old, encoding='unicode'))
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = np.array([rng.uniform(lo+.03, hi-.03) for lo, hi in model.limits.values()])
        equivalent = q.copy()
        equivalent[5] *= -1.
        pose = model.fk(q)
        np.testing.assert_allclose(pose, legacy.fk(equivalent), atol=1e-12)
        np.testing.assert_allclose(model.jacobian(q)[3:, 5], -pose[:3, 2], atol=1e-12)
        np.testing.assert_allclose(model.jacobian(q)[:3, 5], 0., atol=1e-12)


@pytest.mark.parametrize('q', [-1.2, -.3, 0., .3, 1.2])
def test_real_driver_feedback_and_raw_commands_keep_the_existing_sign(q):
    wrapper = ARM/'tool_manipulator_moveit_config/config/tool_manipulator.urdf.xacro'
    xml = xacro.process_file(str(wrapper), mappings={'use_mesh': 'false',
        'use_mock_hardware': 'false', 'hardware_config_file': str(HARDWARE)}).toxml()
    root = ET.fromstring(xml)
    joint = root.find("ros2_control/joint[@name='wrist_yaw_joint']")
    direction = float(joint.find("param[@name='position_direction']").text)
    offset = float(joint.find("param[@name='position_zero_offset']").text)
    spec = yaml.safe_load(HARDWARE.read_text())['joints']['wrist_yaw_joint']
    assert spec['id'] == 3 and spec['raw_increases_ccw'] is False
    assert direction == -1.
    raw_model_position = offset+direction*q
    raw_tick = (raw_model_position+np.pi)*4096/(2*np.pi)
    assert raw_tick == pytest.approx(spec['zero_raw']-q*4096/(2*np.pi))
    assert direction*(raw_model_position-offset) == pytest.approx(q)


def ready():
    data = yaml.safe_load((ARM/'tool_manipulator_bringup/config/tcp_ready.yaml').read_text())
    return np.array(data['tcp_ready']['positions_rad'])


@pytest.mark.parametrize('sign', [-1., 1.])
def test_direct_wrist_jog_keeps_motor_direction_and_returns_correct_fk(model, sign):
    q = ready()
    step = TcpStepper(model, [.12]*6, budget=1.)
    step.reset(q)
    candidate, pose = step.wrist_candidate(q, [0., sign], .02)
    expected = q.copy()
    expected[5] += sign*.12*.02
    np.testing.assert_allclose(candidate, expected, atol=1e-12)
    np.testing.assert_allclose(pose, model.fk(expected), atol=1e-12)
    # The same positive motor-angle command now renders the real -Z motion.
    local = _rotvec(model.fk(q)[:3, :3].T@pose[:3, :3])
    assert sign*local[2] < 0.


@pytest.mark.parametrize('axis', [0, 1, 2, 5])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_cartesian_ik_preserves_requested_tcp_directions(model, axis, sign):
    q = ready()
    step = TcpStepper(model, [.12]*6, budget=1., iterations=120)
    step.reset(q)
    before = model.fk(q)
    twist = np.zeros(6)
    twist[axis] = sign
    candidate, target = step.candidate(q, twist, .02)
    actual = model.fk(candidate)
    error = model.pose_error(candidate, target)
    assert np.linalg.norm(error[:3]) < 1e-5
    assert np.linalg.norm(error[3:]) < 1e-4
    assert model.within_limits(candidate, .02)
    if axis < 3:
        assert sign*(target[axis, 3]-before[axis, 3]) > 0.
        assert sign*(actual[axis, 3]-before[axis, 3]) > 0.
    else:
        local = _rotvec(before[:3, :3].T@actual[:3, :3])
        assert sign*local[2] > 0.
        assert sign*(candidate[5]-q[5]) < 0.
