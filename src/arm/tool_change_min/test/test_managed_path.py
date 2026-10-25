"""Managed adapter against the real ModeCore, with a deterministic feedback clock."""
from concurrent.futures import Future
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
import pytest

ARM = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ARM/'tool_change_min'), str(ARM/'tool_manipulator_bringup/scripts')]
from tool_change_min.managed_path import ManagedPath
from tool_change_min.config import validate_request_config, load_poses
from arm_mode_core import ModeCore


def done(value):
    f = Future(); f.set_result(value); return f


def fixture():
    now = [10.]
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.feedback([0.]*6, [0.]*6, now[0])
    c.ready, c.focus = True, 'arm'
    names = list('abcdef')
    model = Mock()
    model.within_limits.side_effect = lambda q, margin: (len(q) == 6 and
        np.all(np.isfinite(q)) and np.max(np.abs(q)) <= 2-margin)
    n = Mock()
    mode, collision, publisher = Mock(), Mock(), Mock()
    n.create_client.side_effect = [mode, collision]
    n.create_publisher.return_value = publisher
    mode.service_is_ready.return_value = collision.service_is_ready.return_value = True
    collision.call_async.return_value = done(NS(valid=True))
    adapter = ManagedPath(n, model, names, .01, Mock())
    def sync():
        adapter.status = (dict(mode=c.mode, requested=c.requested, epoch=c.epoch,
            ready=c.ready, focus=c.focus, transitioning=c.transitioning, startup_complete=True,
            last_execution_id=c.last_execution_id, last_execution_result=c.last_execution_result), now[0])
        adapter.state = (dict(zip(names, c.position)), now[0])
    def pump(dt):
        now[0] += .02
        c.feedback(list(c.target), [0.]*6, now[0])
        c.tick(now[0], .02)
        sync()
    def enter(req):
        c.request('tool_auto', now[0]); sync(); return done(NS(success=True))
    mode.call_async.side_effect = enter
    def publish(msg):
        d = json.loads(msg.data)
        if d.get('cancel'):
            c.cancel_path(d['epoch'], d['execution_id'], now[0])
        else:
            c.path(d['epoch'], d['execution_id'], d['waypoints'], now[0], d['durations'])
        sync()
    publisher.publish.side_effect = publish
    sync()
    trajectory = NS(joint_names=names, points=[NS(positions=[.01]*6,
        velocities=[], accelerations=[], effort=[], time_from_start=NS(sec=0, nanosec=200000000))])
    return adapter, c, n, trajectory, now, pump


def test_adapter_completes_through_manager_only_and_honors_duration():
    a, c, n, trajectory, now, pump = fixture()
    with patch('time.monotonic', side_effect=lambda: now[0]), patch('time.sleep', side_effect=pump):
        assert 'completed' in a.send(trajectory, 2.)
    assert now[0]-10 >= .2
    assert c.last_execution_result == 'succeeded' and c.mode == 'hold'
    np.testing.assert_allclose(c.position, .01)
    assert n.create_publisher.call_args.args[1] == '/arm/input/tool_auto'
    assert a.publisher.publish.call_count == 1


def test_collision_rejection_sends_no_motion_or_mode_request():
    a, c, n, trajectory, now, pump = fixture()
    a.collision.call_async.return_value = done(NS(valid=False))
    with patch('time.monotonic', return_value=10.):
        with pytest.raises(ValueError, match='collision'): a.send(trajectory, 2.)
    a.mode.call_async.assert_not_called()
    a.publisher.publish.assert_not_called()


@pytest.mark.parametrize('bad', ['time', 'limit', 'velocity', 'order', 'scheduled', 'nanoseconds'])
def test_bad_trajectory_never_sent(bad):
    a, c, n, t, now, pump = fixture()
    if bad == 'time': t.points[0].time_from_start.nanosec = 0
    if bad == 'limit': t.points[0].positions = [3.]*6
    if bad == 'velocity': t.points[0].velocities = [.01]*6
    if bad == 'order': t.joint_names = list(reversed(t.joint_names))
    if bad == 'scheduled': t.header = NS(stamp=NS(sec=100, nanosec=0))
    if bad == 'nanoseconds': t.points[0].time_from_start.nanosec = 1000000000
    with patch('time.monotonic', return_value=10.):
        with pytest.raises(ValueError): a.send(t, 2.)
    a.publisher.publish.assert_not_called()


def test_execution_timeout_cancels_only_its_epoch_and_id():
    a, c, n, t, now, pump = fixture()
    original = a.publisher.publish.side_effect
    def send(msg):
        original(msg)
        # Accepted path, but actuator cannot track the command.
        if not json.loads(msg.data).get('cancel'):
            c.speeds = [1e-9]*6
            c.path_speeds = [1e-9]*6
    a.publisher.publish.side_effect = send
    with patch('time.monotonic', side_effect=lambda: now[0]), patch('time.sleep', side_effect=pump):
        with pytest.raises(TimeoutError): a.send(t, .8)
    cmds = [json.loads(x.args[0].data) for x in a.publisher.publish.call_args_list]
    assert len(cmds) == 2 and cmds[-1]['cancel']
    assert cmds[0]['epoch'] == cmds[1]['epoch']
    assert cmds[0]['execution_id'] == cmds[1]['execution_id']
    assert c.last_execution_result == 'aborted' and c.mode == 'hold' and a.failed


def test_timed_core_segment_preserves_direction_with_joint_speed_caps():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.feedback([0.]*6, [0.]*6, 0.)
    c.ready, c.focus, c.mode = True, 'arm', 'tool_auto'
    c.path(c.epoch, 'job', [[.6, .3, 0, 0, 0, 0]], 0., durations=[1.])
    result = c.tick(.02, .02)
    assert result[0] == pytest.approx(.0024)
    assert result[1] == pytest.approx(.0012)
    old = c.epoch
    c.request('manual', .02)
    assert c.last_execution_result == 'aborted'
    assert not c.cancel_path(old, 'job', .03)
    assert c.requested == 'manual'


@pytest.mark.parametrize('stage', ['ee_align', 'tool1_pre', 'lock', 'done'])
def test_required_ee_alignment_blocks_managed_sequence_before_motion(stage):
    data = load_poses(ARM/'tool_change_min/config/poses.yaml')
    data['control_backend'] = 'managed'
    data['development']['stop_after_state'] = stage
    with pytest.raises(ValueError, match='EE alignment is required'):
        validate_request_config(data)
