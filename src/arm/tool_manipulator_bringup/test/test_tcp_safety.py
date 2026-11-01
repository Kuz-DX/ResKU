"""Immediate stop, transactional seed and bounded diagnostics regressions."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tcp_joy import TcpJoy, ARM_JOINTS
from tcp_joy_core import TcpStepper, TcpMetrics, SafetyError
from arm_mode_core import ModeCore
from arm_control_mode_manager import ControlModeManager


def stepper():
    model = Mock()
    model.within_limits.return_value = True
    model.fk.return_value = np.eye(4)
    model.jacobian.return_value = np.diag([.2]*3+[1.]*3)
    model.solve.return_value = np.full(6, .001)
    model.pose_error.return_value = np.zeros(6)
    s = TcpStepper(model, [.12]*6)
    s.reset(np.zeros(6))
    return s


def node():
    n = SimpleNamespace(stepper=stepper(), metrics=TcpMetrics(), epoch='e',
        status=dict(mode='tcp_ik', focus='arm', ready=True, transitioning=False, epoch='e'),
        status_time=10., state_time=10., joy_time=10., last_time=10.,
        measured=np.zeros(6), full_state=dict(zip(ARM_JOINTS, [0.]*6)),
        deadman=True, deadman_engaged=False, rearmed=True, pending=None, entry_pending=None, failures=0,
        blocked=False, reason='', last_failure_reason='', last_failure_time=-float('inf'),
        last_ik_duration_ms=None, last_logged_state=None, twist=np.array([1., 0, 0, 0, 0, 0]),
        pub=Mock(), diag=Mock(), check=Mock(), cancel=Mock(), get_logger=Mock())
    n.model = n.stepper.model
    n.check.service_is_ready.return_value = True
    n.cancel.service_is_ready.return_value = True
    for name in ('discard', 'assess_entry', 'fail', 'publish', 'safety_stop', 'submit_collision', 'deadman_stop'):
        setattr(n, name, getattr(TcpJoy, name).__get__(n))
    return n


def tick(n, now=10.02):
    with patch('time.monotonic', return_value=now):
        TcpJoy.tick(n)


def messages(n):
    return [json.loads(c.args[0].data) for c in n.pub.publish.call_args_list]


def test_collision_first_failure_faults_without_committing_or_heartbeat():
    n = node()
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(valid=False, contacts=[])
    n.pending = dict(q=np.full(6, .001), pose=np.eye(4), measured=np.zeros(6),
                     started=10., epoch='e', futures=[future])
    tick(n)
    assert n.blocked and n.failures == 0
    assert messages(n) == [dict(epoch='e', fault=True)]
    np.testing.assert_array_equal(n.stepper.q, np.zeros(6))
    tick(n, 10.04)
    assert len(messages(n)) == 1
    assert n.metrics.counts['safety_stops'] == 1


def test_feedback_loss_stops_on_first_tick():
    n = node()
    n.state_time = 9.
    tick(n)
    assert n.blocked and n.failures == 0
    assert messages(n)[0]['fault'] is True


@pytest.mark.parametrize('module', ['tcp_joy', 'tcp_joy_test'])
@pytest.mark.parametrize('joint,accepted', [('ee_joint', True), ('other_joint', False)])
def test_pending_collision_ignores_only_ee_position(module, joint, accepted):
    from importlib import import_module
    cls = import_module(module).TcpJoy
    n = node()
    n.full_state[joint] = 0.
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(valid=True)
    n.pending = dict(q=np.full(6, .001), pose=np.eye(4), measured=np.zeros(6),
                     full_state=dict(n.full_state), started=10., epoch='e', futures=[future])
    n.full_state[joint] = 20.
    with patch('time.monotonic', return_value=10.02):
        cls.tick(n)
    assert (n.metrics.counts['accepted_commands'] == 1) is accepted
    assert n.failures == (0 if accepted else 1)


def test_lead_stop_requests_hold_once_without_old_target_publication():
    n = node()
    n.measured[:] = -.09
    tick(n)
    assert n.blocked
    n.cancel.call_async.assert_called_once()
    n.pub.publish.assert_not_called()


@pytest.mark.parametrize('kind', ['missing', 'nan', 'duplicate'])
def test_bad_joint_packet_invalidates_freshness_immediately(kind):
    n = node()
    msg = JointState(name=list(ARM_JOINTS), position=[0.]*6)
    if kind == 'missing': msg.name.pop(); msg.position.pop()
    if kind == 'nan': msg.position[0] = float('nan')
    if kind == 'duplicate': msg.name[0] = msg.name[1]
    TcpJoy.on_state(n, msg)
    assert n.state_time == -float('inf') and n.blocked
    assert messages(n)[0]['fault']


def test_limit_block_does_not_accumulate_failures_and_reverse_recovers():
    n = node()
    n.failures = 2
    n.model.solve.side_effect = ValueError('IK iteration would violate soft limits')
    for i in range(6): tick(n, 10.02+i*.02)
    assert n.failures == 2 and not n.blocked
    assert n.metrics.counts['limit_blocked'] == 6
    assert all('fault' not in m for m in messages(n))
    n.model.solve.side_effect = None
    n.twist *= -1
    tick(n, 10.14)
    assert n.pending['pose'][0, 3] < 0
    np.testing.assert_array_equal(n.stepper.q, np.zeros(6))


def test_general_failures_hold_after_five_and_preserve_seed():
    n = node()
    n.model.solve.side_effect = ValueError('IK did not converge')
    for i in range(5): tick(n, 10.02+i*.02)
    assert n.failures == 5 and n.blocked
    n.cancel.call_async.assert_called_once()
    assert len(messages(n)) == 4
    np.testing.assert_array_equal(n.stepper.q, np.zeros(6))


def test_success_after_general_failure_resets_counter_after_collision_only():
    n = node()
    n.model.solve.side_effect = ValueError('IK did not converge')
    tick(n)
    assert n.failures == 1
    n.model.solve.side_effect = None
    tick(n, 10.04)
    assert n.failures == 1
    for f in n.pending['futures']:
        f.done.return_value = True
        f.result.return_value = SimpleNamespace(valid=True)
    tick(n, 10.06)
    assert n.failures == 0
    np.testing.assert_allclose(n.stepper.q, .001)


def test_collision_wait_does_not_integrate_catch_up_time():
    n = node()
    original_candidate = n.stepper.candidate
    n.stepper.candidate = Mock(wraps=original_candidate)
    tick(n, 10.02)
    assert n.pending is not None
    for future in n.pending['futures']:
        future.done.return_value = False
    assert n.stepper.candidate.call_count == 1
    for now in (10.04, 10.08, 10.12):
        tick(n, now)
    assert n.stepper.candidate.call_count == 1
    for future in n.pending['futures']:
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=True)
    tick(n, 10.14)
    assert n.stepper.candidate.call_count == 1
    tick(n, 10.16)
    assert n.stepper.candidate.call_count == 2
    # Candidate integration remains one 20-ms control quantum after the wait.
    assert n.stepper.candidate.call_args.args[2] == pytest.approx(.02)


def test_cancelled_old_collision_response_cannot_commit_after_repress():
    n = node()
    tick(n, 10.02)
    old = n.pending
    TcpJoy.on_joy(n, SimpleNamespace(axes=[0.]*8, buttons=[0]*13))
    assert n.pending is None
    old_q = old['q'].copy()
    for future in old['futures']:
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=True)
    # Re-arm/repress creates a distinct transaction from measured feedback.
    release = SimpleNamespace(axes=[0.]*8, buttons=[0]*13)
    press = SimpleNamespace(axes=[0., 1., 0., 0., 0., 0., 0., 0.],
                            buttons=[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    with patch('time.monotonic', return_value=10.03):
        TcpJoy.on_joy(n, release)
    with patch('time.monotonic', return_value=10.04):
        TcpJoy.on_joy(n, press)
    tick(n, 10.06)
    assert n.pending is not old
    np.testing.assert_array_equal(n.stepper.q, n.measured)
    np.testing.assert_array_equal(old['q'], old_q)


def test_mutating_solver_cannot_modify_accepted_seed_or_pose():
    s = stepper()
    before_q, before_pose = s.q.copy(), s.pose.copy()
    def bad_solver(target, seed, *args, **kwargs):
        target[:] = 9.
        seed[:] = 9.
        raise ValueError('solver failed')
    s.model.solve.side_effect = bad_solver
    with patch('time.monotonic', return_value=10.):
        with pytest.raises(ValueError): s.candidate(np.zeros(6), np.ones(6), .02)
    np.testing.assert_array_equal(s.q, before_q)
    np.testing.assert_array_equal(s.pose, before_pose)


@pytest.mark.parametrize('bad', [np.full(6, float('nan')), np.zeros(5)])
def test_invalid_solution_is_immediate_safety_error(bad):
    s = stepper()
    s.model.solve.return_value = bad
    with pytest.raises(SafetyError): s.candidate(np.zeros(6), np.ones(6), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))


def test_fk_residual_rejects_false_solver_success():
    s = stepper()
    s.model.pose_error.side_effect = [np.zeros(6), np.array([.001, 0, 0, 0, 0, 0])]
    with pytest.raises(ValueError, match='residual'): s.candidate(np.zeros(6), np.ones(6), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))


def test_manager_malformed_feedback_faults_using_last_valid_position():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.feedback([.1]*6, [0.]*6, 1.)
    n = SimpleNamespace(core=c)
    msg = JointState(name=list(ARM_JOINTS), position=[.2]*6, velocity=[])
    ControlModeManager.on_state(n, msg)
    assert c.mode == 'fault' and c.target == [.1]*6


def test_metrics_window_bounded_and_overruns_counted():
    m = TcpMetrics(window=3)
    for x in [1., 10., 21., 30.]: m.observe('ik', x)
    report = m.snapshot()
    assert report['counts']['ik_samples'] == 4
    assert report['counts']['ik_over_20ms'] == 2
    assert report['ik_ms']['window_samples'] == 3
    assert report['ik_ms']['mean'] == pytest.approx(61/3)
