"""Connection-only console output checks; no launch/process/device execution."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


source = Path(__file__).resolve().parents[1]/'launch/manual.launch.py'
spec = importlib.util.spec_from_file_location('manual_launch_joy_log', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def consume_chunks(chunks):
    monitor = module.JoystickConnectionLog()
    messages = []
    with patch.object(module, 'LogInfo', side_effect=lambda **kwargs: kwargs['msg']):
        for stream, chunk in chunks:
            messages.extend(getattr(monitor, stream)(SimpleNamespace(text=chunk)))
    return monitor, messages


def test_missing_disconnected_or_idle_joystick_prints_no_connection_status():
    _, messages = consume_chunks([
        ('stderr', b'No joystick found\n'),
        ('stderr', b'[WARN] joystick disconnected\n'),
        ('stdout', b'Waiting for joystick\n')])
    assert messages == []


def test_real_driver_open_event_prints_one_connection_line():
    _, messages = consume_chunks([('stderr',
        b'[INFO] [joy_node]: Opened joystick: Wireless Controller. deadzone: 0.050000\n')])
    assert messages == ['조이스틱 연결 완료']


def test_split_driver_output_does_not_duplicate_connection_notice():
    _, messages = consume_chunks([
        ('stderr', b'[INFO] [joy_node]: Opened joy'),
        ('stderr', b'stick: Wireless Controller.'),
        ('stderr', b' deadzone: 0.050000\nadditional log\n')])
    assert messages == ['조이스틱 연결 완료']


def test_reconnection_prints_one_additional_line_without_disconnect_notice():
    _, messages = consume_chunks([
        ('stderr', b'Opened joystick: Wireless Controller\nDisconnected joystick\n'),
        ('stderr', b'Opened joystick: Wireless Controller\n')])
    assert messages == ['조이스틱 연결 완료', '조이스틱 연결 완료']


def test_stdout_and_stderr_fragments_are_independent_and_bounded():
    monitor, messages = consume_chunks([
        ('stderr', b'Opened joy'), ('stdout', b'stick: unrelated output\n'),
        ('stderr', b'stick: Wireless Controller\n'), ('stdout', b'x'*20000)])
    assert messages == ['조이스틱 연결 완료']
    assert len(monitor.pending['stdout']) == 8192
