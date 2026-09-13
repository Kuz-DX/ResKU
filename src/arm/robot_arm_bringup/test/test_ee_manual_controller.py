"""Regression tests for MANUAL_EE home/readiness synchronization."""

import importlib.util
from pathlib import Path

import numpy as np
from std_msgs.msg import Bool


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'ee_manual_controller.py'
SPEC = importlib.util.spec_from_file_location('ee_manual_controller', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_controller():
    controller = object.__new__(MODULE.ManualEeController)
    controller.enabled = True
    controller.ready = False
    controller.target = np.asarray([9.0, 9.0, 9.0, 9.0])
    controller.positions = {
        name: value for name, value in zip(controller.JOINTS, [0.1, -0.5, -1.5, -1.4])
    }
    return controller


def test_not_ready_discards_pre_home_target():
    controller = make_controller()

    controller.on_ready(Bool(data=False))

    assert not controller.ready
    assert controller.target is None


def test_ready_reseeds_target_from_measured_home_position():
    controller = make_controller()

    controller.on_ready(Bool(data=True))

    assert controller.ready
    assert np.allclose(controller.target, [0.1, -0.5, -1.5, -1.4])


def test_translation_ik_does_not_add_an_orientation_hold_constraint():
    # Joint 1 can produce the requested downward TCP motion but also rotates
    # the tool. A 6-D solve would fight that rotation and weaken the descent.
    jacobian = np.asarray([
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 0.0],
        [1.0, 1.0],
        [0.0, 0.0],
    ])

    velocity = MODULE.translation_joint_velocity(
        jacobian, [0.0, 0.0, -0.03], damping=0.01,
        max_joint_speed=1.0)

    assert velocity[0] < -0.029
    assert abs(velocity[1]) < 1e-12


def test_joint_speed_limit_scales_instead_of_clipping_direction():
    jacobian = np.eye(3)

    velocity = MODULE.translation_joint_velocity(
        jacobian, [0.2, 0.1, 0.0], damping=0.0,
        max_joint_speed=0.05)

    assert np.allclose(velocity, [0.05, 0.025, 0.0])
