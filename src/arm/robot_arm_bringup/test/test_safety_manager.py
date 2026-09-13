"""Mode-transition regression tests for safety_manager."""

import importlib.util
from pathlib import Path

from std_msgs.msg import String


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'safety_manager.py'
SPEC = importlib.util.spec_from_file_location('safety_manager', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_drive_to_arm_temporarily_starts_in_manual_100_for_calibration():
    assert MODULE.next_mode('OFF', control_toggle=True) == 'MANUAL_100'


def test_button_8_toggles_manual_ee_and_auto():
    assert MODULE.next_mode('MANUAL_EE', auto_toggle=True) == 'AUTO'
    assert MODULE.next_mode('AUTO', auto_toggle=True) == 'MANUAL_EE'


def test_share_toggles_manual_ee_and_manual_100():
    assert MODULE.next_mode(
        'MANUAL_EE', manual_100_toggle=True) == 'MANUAL_100'
    assert MODULE.next_mode(
        'MANUAL_100', manual_100_toggle=True) == 'MANUAL_EE'


def test_square_toggles_manual_ee_pause():
    assert MODULE.next_mode(
        'MANUAL_EE', manual_ee_pause_toggle=True
    ) == 'MANUAL_EE_PAUSED'
    assert MODULE.next_mode(
        'MANUAL_EE_PAUSED', manual_ee_pause_toggle=True
    ) == 'MANUAL_EE'


def test_share_does_not_leave_manual_ee_pause():
    assert MODULE.next_mode(
        'MANUAL_EE_PAUSED', manual_100_toggle=True
    ) == 'MANUAL_EE_PAUSED'


def test_button_8_does_nothing_in_drive_mode():
    assert MODULE.next_mode('OFF', auto_toggle=True) == 'OFF'


def test_control_button_returns_each_arm_mode_to_drive():
    assert MODULE.next_mode('MANUAL_EE', control_toggle=True) == 'OFF'
    assert MODULE.next_mode('AUTO', control_toggle=True) == 'AUTO'
    assert MODULE.next_mode('MANUAL_100', control_toggle=True) == 'OFF'
    assert MODULE.next_mode(
        'MANUAL_EE_PAUSED', control_toggle=True) == 'OFF'


def test_manual_100_and_auto_can_switch_directly():
    assert MODULE.next_mode('MANUAL_100', auto_toggle=True) == 'AUTO'
    assert MODULE.next_mode(
        'AUTO', auto_toggle=True, auto_return='MANUAL_100'
    ) == 'MANUAL_100'
    assert MODULE.next_mode(
        'AUTO', manual_100_toggle=True) == 'MANUAL_100'
    assert MODULE.next_mode(
        'MANUAL_100', manual_100_toggle=True,
        manual_100_return='AUTO'
    ) == 'AUTO'


def test_estop_cannot_be_cleared_by_mode_buttons():
    assert MODULE.next_mode(
        'ESTOP_LATCHED', control_toggle=True, auto_toggle=True
    ) == 'ESTOP_LATCHED'


def test_control_button_has_priority_if_both_edges_arrive_together():
    assert MODULE.next_mode(
        'MANUAL_EE', control_toggle=True, auto_toggle=True
    ) == 'OFF'


def test_auto_uses_moveit_controllers_exclusively():
    activate, deactivate = MODULE.controller_switch_for_mode('AUTO')
    assert activate == ['arm_controller', 'gripper_controller']
    assert deactivate == ['position_controller']


def test_manual_100_uses_position_controller_exclusively():
    activate, deactivate = MODULE.controller_switch_for_mode('MANUAL_100')
    assert activate == ['position_controller']
    assert deactivate == ['arm_controller', 'gripper_controller']


def test_manual_ee_uses_position_controller_exclusively():
    activate, deactivate = MODULE.controller_switch_for_mode('MANUAL_EE')
    assert activate == ['position_controller']
    assert deactivate == ['arm_controller', 'gripper_controller']


def test_hardware_fault_requests_estop_with_reason():
    manager = object.__new__(MODULE.SafetyManager)
    reasons = []
    manager.trigger_estop = reasons.append

    manager.on_hardware_fault(String(data='wrist_joint: feedback timeout'))

    assert reasons == ['wrist_joint: feedback timeout']
