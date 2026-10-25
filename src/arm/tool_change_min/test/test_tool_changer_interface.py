"""Removed arm-lock operation must fail closed until Arduino integration exists."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from tool_change_min.config import load_poses, validate_request_config


@pytest.mark.parametrize('stage', ['lock', 'return_docking_wait', 'return_home', 'done'])
def test_unimplemented_handshake_rejected_before_any_arm_motion(stage):
    config = load_poses(PACKAGE/'config/poses.yaml')
    config['development']['stop_after_state'] = stage
    with pytest.raises(ValueError, match='Arduino tool changer interface is disabled'):
        validate_request_config(config)


def fsm_class(validate):
    path = PACKAGE/'scripts/tool_change_fsm.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in ('_run', '_arduino_lock')]
    namespace = dict(Node=object, validate_request_config=validate,
                     AlignEe=SimpleNamespace(Request=SimpleNamespace))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(path), 'exec'), namespace)
    return namespace['ToolChangeFsm']


def harness(validate):
    n = fsm_class(validate)()
    n.config = load_poses(PACKAGE/'config/poses.yaml')
    n.config['timeouts_s']['ee_align'] = 1.
    n.state = 'IDLE'
    n.hardware_error = ''
    n._named = Mock()
    n._cartesian_target = Mock()
    n._pause_after = Mock(return_value=False)
    n._call = Mock()
    n.align = Mock()
    n.get_logger = Mock()
    n._publish = Mock(side_effect=lambda state: setattr(n, 'state', state))
    return n


def test_request_validation_prevents_all_motion_for_full_sequence():
    n = harness(validate_request_config)
    n.config['development']['stop_after_state'] = 'done'
    n._run(1)
    n._named.assert_not_called()
    n._cartesian_target.assert_not_called()
    assert n.state == 'HOLD'


def test_lock_guard_prevents_retreat_even_if_configuration_gate_is_bypassed():
    n = harness(lambda *args: None)
    n._run(1)
    assert [call.args[0] for call in n._named.call_args_list] == ['HOME', 'DOCKING_WAIT', 'TOOL1_PRE']
    n._cartesian_target.assert_called_once()
    assert n.state == 'HOLD'
    assert 'RETURN_HOME' not in [call.args[0] for call in n._named.call_args_list]


def test_old_arm_lock_endpoint_and_runtime_configuration_removed():
    tree = ast.parse((PACKAGE/'scripts/motion_executor.py').read_text())
    assert not any(isinstance(n, ast.FunctionDef) and n.name == '_rotate_yaw'
                   for n in ast.walk(tree))
    assert not (PACKAGE/'srv/RotateWristYaw.srv').exists()
    config = load_poses(PACKAGE/'config/poses.yaml')
    assert 'yaw_delta_rad' not in config['motion']
    assert 'lock_duration_s' not in config['motion']
    assert not {'tool_lock', 'tool_lock1'} & config['named_poses'].keys()
    assert config['development']['stop_after_state'] == 'home'


@pytest.mark.parametrize('success', [False, True])
def test_fsm_retreat_only_after_handshake_success(success):
    n = harness(lambda *args: None)
    events = []
    n._named.side_effect = lambda state, *args: events.append(state)
    def handshake():
        events.append('ARDUINO_WAIT')
        if not success:
            raise RuntimeError('Arduino timeout')
        events.append('ARDUINO_COMPLETE')
    n._arduino_lock = handshake
    n._run(1)
    if success:
        assert events[-3:] == ['ARDUINO_COMPLETE', 'RETURN_DOCKING_WAIT', 'RETURN_HOME']
        assert n.state == 'DONE'
    else:
        assert events[-1] == 'ARDUINO_WAIT' and n.state == 'HOLD'
        assert 'RETURN_DOCKING_WAIT' not in events
