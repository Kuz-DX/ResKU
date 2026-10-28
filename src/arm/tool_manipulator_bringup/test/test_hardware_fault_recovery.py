"""Per-component recovery must not hide another actuator fault."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_control_mode_manager import update_hardware_faults


def test_joint_ok_only_clears_matching_fault():
    faults = update_hardware_faults({}, 'shoulder_joint: CAN timeout')
    faults = update_hardware_faults(faults, 'elbow_joint: position jump')
    assert set(faults) == {'shoulder_joint', 'elbow_joint'}
    faults = update_hardware_faults(faults, 'shoulder_joint: ok')
    assert faults == {'elbow_joint': 'position jump'}
    assert update_hardware_faults(faults, 'elbow_joint: ok') == {}


def test_legacy_unscoped_fault_stays_latched_until_explicit_unknown_ok():
    faults = update_hardware_faults({}, 'CAN disconnected')
    assert faults == {'unknown': 'CAN disconnected'}
    assert update_hardware_faults(faults, 'shoulder_joint: ok') == faults
    assert update_hardware_faults(faults, 'unknown: ok') == {}
