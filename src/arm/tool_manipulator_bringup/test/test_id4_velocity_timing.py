"""Checks for ID4 command timing."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import manual_control_common as common
from managed_manual import ManagedManual


def test_direct_velocity_logs_once_after_release(monkeypatch):
    ticks = iter((10., 12.5))
    monkeypatch.setattr(common.time, 'monotonic', lambda: next(ticks))
    node = common.ManualController.__new__(common.ManualController)
    node._ee_drive_started_at = None
    node._ee_drive_velocity = 0.
    node.ee_velocity_publisher = SimpleNamespace(publish=Mock())
    logger = Mock()
    node.get_logger = lambda: logger

    for velocity in (0., -.36, -.36, -.36, 0., 0.):
        node.publish_ee_velocity(velocity)

    assert node.ee_velocity_publisher.publish.call_count == 6
    assert [call.args[0].data[0] for call in node.ee_velocity_publisher.publish.call_args_list] == [
        0., -.36, -.36, -.36, 0., 0.]
    logger.info.assert_called_once_with('ID4 velocity=-0.360000 rad/s duration=2.500 s')


def test_managed_mode_tracks_only_published_commands(monkeypatch):
    ticks = iter((10., 20., 10., 10., 21.25))
    monkeypatch.setattr(common.time, 'monotonic', lambda: next(ticks))
    node = ManagedManual.__new__(ManagedManual)
    node._ee_drive_started_at = None
    node._ee_drive_velocity = 0.
    node.pending_ee = .36
    node.controller_ready = False
    node.target = SimpleNamespace(tolist=lambda: [0.] * 6)
    node.manager_epoch = 'epoch'
    node.manager_time = 10.
    node.fault = node.hardware_fault = False
    node.input_pub = SimpleNamespace(publish=Mock())
    logger = Mock()
    node.get_logger = lambda: logger

    node.hold()
    assert node._ee_drive_started_at is None
    node.controller_ready = True
    node.hold()
    node.hold()
    node.pending_ee = 0.
    node.hold()
    assert node.input_pub.publish.call_count == 3
    logger.info.assert_called_once_with('ID4 velocity=+0.360000 rad/s duration=1.250 s')
