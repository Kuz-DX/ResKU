"""R2 nearest TCP entry geometry without motors or ROS services."""
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml
import xacro

ARM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ARM/'tool_change_min'))
sys.path.insert(0, str(ARM/'tool_manipulator_bringup/scripts'))
from tcp_joy_core import nearest_tcp_entry
from tool_change_min.kinematics import ArmKinematics


@pytest.fixture(scope='module')
def model():
    return ArmKinematics(ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                         ARM/'tool_manipulator_bringup/config/hardware.yaml')


def condition(model, q):
    jacobian = model.jacobian(q).copy()
    jacobian[:3] /= .2
    return np.linalg.cond(jacobian)


def test_valid_current_pose_is_returned_without_motion(model):
    ready = np.array(yaml.safe_load((ARM/'tool_manipulator_bringup/config/tcp_ready.yaml').read_text())
                     ['tcp_ready']['positions_rad'])
    result = nearest_tcp_entry(model, ready, ready)
    np.testing.assert_array_equal(result, ready)
    assert condition(model, result) <= 20.


def test_nearest_pose_projects_limit_margin_and_has_valid_condition(model):
    ready = np.array(yaml.safe_load((ARM/'tool_manipulator_bringup/config/tcp_ready.yaml').read_text())
                     ['tcp_ready']['positions_rad'])
    measured = ready.copy()
    measured[3] = model.limits['wrist_pitch_joint'][0]+.01
    result = nearest_tcp_entry(model, measured, ready)
    assert model.within_limits(result, .101)
    assert model.within_limits(result, .121)
    assert condition(model, result) <= 20.
    assert np.linalg.norm(result-measured) <= np.linalg.norm(ready-measured)+1e-12


def test_invalid_inputs_and_unusable_reference_are_rejected(model):
    ready = np.zeros(6)
    with pytest.raises(ValueError, match='finite six-joint'):
        nearest_tcp_entry(model, [0.]*5, ready)
    with pytest.raises(ValueError, match='no TCP entry pose'):
        nearest_tcp_entry(model, ready, ready)
