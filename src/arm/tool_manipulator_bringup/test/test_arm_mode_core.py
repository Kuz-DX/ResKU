"""Ownership, cancellation and watchdog tests without ROS or hardware."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore


def ready(mode='manual'):
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.focus, c.ready = 'arm', True
    c.feedback([0.]*6, [0.]*6, 0.)
    c.request(mode, 0.)
    for i in range(1, 6):
        c.feedback([0.]*6, [0.]*6, i*.02)
        c.tick(i*.02, .02)
    assert c.mode == mode and not c.transitioning
    return c


def test_only_owner_and_current_epoch_can_command():
    c = ready()
    assert not c.command('tcp_ik', c.epoch, [.05]*6, 0., .1)
    assert not c.command('manual', 'old', [.05]*6, 0., .1)
    assert c.command('manual', c.epoch, [.05]*6, .1, .1)
    assert max(c.tick(.12, .02)) <= .12*.02


def test_transition_blocks_commands_and_invalidates_pending_messages():
    c = ready()
    old = c.epoch
    c.request('tcp_ik', .1)
    assert c.mode == 'hold' and c.transitioning and c.epoch != old
    assert not c.command('manual', old, [.05]*6, 0., .1)


def test_cancel_discards_remaining_waypoints_and_late_paths():
    c = ready('tool_auto')
    old = c.epoch
    assert c.path(old, 'run1', [[.01]*6, [.2]*6], .1)
    c.request('hold', .12)
    assert c.queue == [] and c.execution_id is None
    assert not c.path(old, 'late', [[1.]*6], .12)
    assert not c.command('manual', c.epoch, [.05]*6, .1, .12)


def test_fault_latches_and_reset_requires_healthy_feedback():
    c = ready()
    c.fault('motor error')
    with pytest.raises(ValueError):
        c.request('manual', .1)
    c.ready = False
    with pytest.raises(ValueError):
        c.reset(.1)
    c.ready = True
    c.reset(.1)
    assert c.mode == 'hold'


def test_stale_feedback_blocks_output_and_zeroes_ee():
    c = ready()
    c.command('manual', c.epoch, [.05]*6, .1, .1)
    assert c.tick(1., .02) is None
    assert c.mode == 'fault' and c.ee == 0.


def test_lost_stream_goes_to_hold():
    c = ready()
    c.feedback([0.]*6, [0.]*6, .5)
    c.tick(.5, .02)
    assert c.mode == 'hold' and c.ee == 0.


def test_limits_and_feedback_lead_are_enforced():
    c = ready()
    for target in ([float('nan')]*6, [3.]*6, [.11]*6, [0.]):
        with pytest.raises(ValueError):
            c.command('manual', c.epoch, target, 0., .1)


def test_path_waits_for_measured_settlement_before_next_waypoint():
    c = ready('tool_auto')
    c.path(c.epoch, 'run1', [[.01]*6, [.02]*6], .1)
    for i in range(1, 11):
        now = .1+i*.02
        c.feedback([0.]*6, [.03]*6, now)
        c.tick(now, .02)
    assert len(c.queue) == 1
    for i in range(1, 6):
        now = .3+i*.02
        c.feedback([.01]*6, [0.]*6, now)
        c.tick(now, .02)
    assert c.queue == [] and c.goal == [.02]*6


def test_focus_and_controller_gates():
    c = ready()
    c.focus = 'drive'
    assert not c.command('manual', c.epoch, [0.]*6, 0., .1)
    with pytest.raises(ValueError):
        c.request('tool_auto', .1)
