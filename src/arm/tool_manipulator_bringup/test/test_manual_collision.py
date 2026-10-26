"""Collision gate behavior with delayed replies; no ROS graph or hardware."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_collision import CHECK_GROUP, CHECK_JOINTS, CollisionWindow, CollisionManual, path_samples
from test_manual import harness as manual_harness


def ready(valid=True, contacts=()):
    f = Future()
    f.set_result(SimpleNamespace(valid=valid, contacts=contacts))
    return f


def window(result=True):
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda req: ready(result)
    return CollisionWindow(client)


def step(w, now, command=None, measured=None, velocity=None, context=('operator',), goal=None):
    command = np.zeros(6) if command is None else np.asarray(command)
    measured = np.r_[command, 0.] if measured is None else np.asarray(measured)
    velocity = np.array([.12, 0., 0., 0., 0., 0., 0.]) if velocity is None else np.asarray(velocity)
    return w.step(now, .02, measured, command, velocity, context,
                  (np.full(6, -2.), np.full(6, 2.)), .1, goal)


def test_requests_are_rate_limited_while_position_steps_stay_bounded_at_50hz():
    w, q, request_times = window(), np.zeros(6), []
    for index in range(101):
        now = index*.02
        batches = w.batches
        before = q.copy()
        q, ee = step(w, now, command=q)
        if batches != w.batches:
            request_times.append(now)
        assert np.max(np.abs(q-before)) <= .12*.02+1e-10
        assert ee == 0.
    assert q[0] > .12  # sustained motion rather than one tiny step per check
    assert len(request_times) <= 21
    assert min(np.diff(request_times)) >= .1-1e-10
    assert w.accepted > 10
    requests = [c.args[0] for c in w.client.call_async.call_args_list]
    assert all(r.group_name == CHECK_GROUP for r in requests)
    assert all(r.robot_state.joint_state.name == list(CHECK_JOINTS) for r in requests)


def test_pending_check_never_publishes_unchecked_movement_or_ee_velocity():
    w = window()
    futures = []
    def delayed(req):
        f = Future()
        futures.append(f)
        return f
    w.client.call_async.side_effect = delayed
    v = [.12, 0, 0, 0, 0, 0, .36]
    for now in (0., .02, .04, .08):
        q, ee = step(w, now, velocity=v)
        np.testing.assert_array_equal(q, np.zeros(6))
        assert ee == 0.
    assert w.batches == 1
    for f in futures:
        f.set_result(SimpleNamespace(valid=True, contacts=[]))
    q, ee = step(w, .10, velocity=v)
    assert q[0] > 0 and ee > 0


def test_collision_stops_at_measured_position_not_last_command():
    w = window(False)
    command = np.array([.04, 0, 0, 0, 0, 0])
    measured = np.zeros(7)
    step(w, 0., command, measured)
    measured[0] = .002
    q, ee = step(w, .02, command, measured)
    np.testing.assert_array_equal(q, measured[:6])
    assert ee == 0. and w.plan is None and w.rejected == 1


def test_collision_at_intermediate_sample_blocks_safe_endpoint():
    w = window()
    w.client.call_async.side_effect = lambda req: ready(
        not .015 < req.robot_state.joint_state.position[0] < .025)
    step(w, 0.)
    q, ee = step(w, .02)
    assert w.rejected == 1
    np.testing.assert_array_equal(q, np.zeros(6))


@pytest.mark.parametrize('failure', ['timeout', 'exception', 'unavailable', 'feedback'])
def test_failed_or_stale_response_cannot_commit(failure):
    w = window()
    if failure == 'timeout':
        w.client.call_async.side_effect = lambda req: Future()
    elif failure == 'exception':
        def failed(req):
            f = Future()
            f.set_exception(RuntimeError('transport lost'))
            return f
        w.client.call_async.side_effect = failed
    step(w, 0.)
    if failure == 'unavailable':
        w.client.service_is_ready.return_value = False
    measured = np.zeros(7)
    if failure == 'feedback':
        measured[1] = .02
    q, ee = step(w, .16 if failure == 'timeout' else .02, measured=measured)
    np.testing.assert_array_equal(q, measured[:6])
    assert ee == 0. and w.pending is None and w.plan is None


def test_release_and_reverse_discard_old_approval():
    w = window()
    step(w, 0., context=('operator', 1))
    q, ee = step(w, .02, context=('operator', -1), velocity=[-.12, 0, 0, 0, 0, 0, 0])
    assert q[0] == 0. and w.pending is None and w.plan is None
    step(w, .04, context=('operator', -1), velocity=[-.12, 0, 0, 0, 0, 0, 0])
    assert w.batches == 1  # direction changes cannot flood the service
    q, ee = step(w, .06, velocity=np.zeros(7), context=('operator', 0))
    assert q[0] == 0. and ee == 0.


def test_endpoint_is_never_exceeded_and_existing_recovery_goal_is_preserved():
    w, q = window(), np.array([0., 2.03, 0., 0., 0., 0.])
    goal = q.copy()
    goal[1] = 1.99
    for index in range(110):
        v = np.zeros(7)
        v[1] = -min(.03, max(0., (q[1]-goal[1])/.02))
        q, _ = step(w, index*.02, command=q, velocity=v, context=('startup',), goal=goal)
        assert q[1] >= goal[1]-1e-10
    assert q[1] == pytest.approx(goal[1])


def test_sampling_covers_feedback_bridge_and_has_finite_budget():
    a, b, c = np.zeros(7), np.zeros(7), np.zeros(7)
    a[0], b[1], c[2] = .02, .02, .02
    samples = path_samples(a, b, c, np.full(7, .005))
    for endpoint in (a,b,c):
        assert tuple(endpoint) in samples
    assert tuple((a+b)/2) in samples
    with pytest.raises(ValueError, match='budget'):
        path_samples(a, b, np.full(7, 100.), np.full(7, .005))


def test_main_manual_stop_cancels_pending_and_captures_fresh_measurement():
    node = CollisionManual.__new__(CollisionManual)
    node.__dict__.update(manual_harness().__dict__)
    node.window = window()
    node.target = node.measured()+.05
    node.stop('collision', .02)
    np.testing.assert_array_equal(node.target, node.measured())
    assert node.window.pending is None and node.neutral_required
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]
