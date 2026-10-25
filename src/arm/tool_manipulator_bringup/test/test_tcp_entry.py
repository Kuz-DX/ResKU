"""TCP entry must stay in HOLD without a fresh, stationary, collision-checked pose."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore
from arm_control_mode_manager import ControlModeManager
from tcp_joy import TcpJoy


def core():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.tcp_entry_required = True
    c.ready, c.focus = True, 'arm'
    c.feedback([0.]*6, [0.]*6, 1.)
    c.tcp_entry = dict(epoch=c.epoch, safe=True, reason='checked', positions=[0.]*6, time=1.)
    return c


@pytest.mark.parametrize('reason', ['missing', 'stale', 'collision', 'changed', 'margin', 'moving', 'epoch'])
def test_unsafe_entry_rejected_in_hold(reason):
    c = core()
    if reason == 'missing': c.tcp_entry = None
    if reason == 'stale': c.tcp_entry['time'] = 0.
    if reason == 'collision': c.tcp_entry.update(safe=False, reason='collision')
    if reason == 'changed': c.position[0] = .01
    if reason == 'margin': c.position[0] = 1.95
    if reason == 'moving': c.velocity[0] = .03
    if reason == 'epoch': c.tcp_entry['epoch'] = 'old'
    with pytest.raises(ValueError): c.request('tcp_ik', 1.01)
    assert c.mode == c.requested == 'hold'
    assert c.reason
    assert not c.command('tcp_ik', c.epoch, [.01]*6, 0., 1.01)


def test_safe_current_pose_enters_without_ready_motion():
    c = core()
    c.request('tcp_ik', 1.01)
    for i in range(1, 6):
        now = 1.01+i*.02
        c.feedback([0.]*6, [0.]*6, now)
        assert c.tick(now, .02) == [0.]*6
    assert c.mode == 'tcp_ik' and not c.transitioning


def test_expired_assessment_cannot_complete_transition():
    c = core()
    c.request('tcp_ik', 1.01)
    for i in range(1, 6):
        now = 1.5+i*.02
        c.feedback([0.]*6, [0.]*6, now)
        c.tick(now, .02)
    assert c.mode == c.requested == 'hold'


def test_manual_does_not_require_tcp_assessment():
    c = core()
    c.tcp_entry = None
    c.request('manual', 1.01)
    assert c.requested == 'manual'


def test_manager_rejects_old_or_malformed_proof():
    node = SimpleNamespace(core=core())
    old = node.core.tcp_entry
    ControlModeManager.on_tcp_entry(node, String(data=json.dumps(dict(epoch='old'))))
    assert node.core.tcp_entry is old
    ControlModeManager.on_tcp_entry(node, String(data=json.dumps(dict(
        epoch=node.core.epoch, safe='yes', positions=[0.]*6, reason='bad'))))
    assert node.core.tcp_entry is None


def assessment_node():
    n = Mock()
    n.status = dict(mode='hold', ready=True, startup_complete=True, epoch='e')
    n.status_time = n.state_time = 1.
    n.measured = np.zeros(6)
    n.full_state = dict(zip(('a', 'b', 'c', 'd', 'e', 'f'), [0.]*6))
    n.entry_pending = None
    n.model.within_limits.return_value = True
    n.model.jacobian.return_value = np.diag([.2]*3+[1.]*3)
    n.check.service_is_ready.return_value = True
    return n


def test_repeated_safe_proofs_do_not_restart_auto_entry_delay():
    n = assessment_node()
    n.auto_entry_armed = True
    n.ready_state = 'await_entry'
    n.ready_safe_at = None
    n.check.call_async.return_value.done.return_value = True
    n.check.call_async.return_value.result.return_value = SimpleNamespace(valid=True)
    TcpJoy.assess_entry(n, 1.)
    TcpJoy.assess_entry(n, 1.02)
    assert n.ready_safe_at == 1.02
    TcpJoy.assess_entry(n, 1.04)
    TcpJoy.assess_entry(n, 1.06)
    assert n.ready_safe_at == 1.02
    n.model.within_limits.return_value = False
    TcpJoy.assess_entry(n, 1.08)
    assert n.ready_safe_at is None
    assert 'entry blocked' in n.reason


def test_entry_requires_collision_result_before_approval():
    n = assessment_node()
    future = n.check.call_async.return_value
    future.done.return_value = False
    TcpJoy.assess_entry(n, 1.)
    n.entry_pub.publish.assert_not_called()
    TcpJoy.assess_entry(n, 1.02)
    n.entry_pub.publish.assert_not_called()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(valid=True)
    TcpJoy.assess_entry(n, 1.04)
    assert json.loads(n.entry_pub.publish.call_args.args[0].data)['safe']
    n.pub.publish.assert_not_called()


@pytest.mark.parametrize('failure', ['margin', 'singular', 'unavailable', 'collision', 'timeout', 'changed'])
def test_entry_assessment_fails_closed(failure):
    n = assessment_node()
    if failure == 'margin': n.model.within_limits.return_value = False
    if failure == 'singular': n.model.jacobian.return_value = np.zeros((6, 6))
    if failure == 'unavailable': n.check.service_is_ready.return_value = False
    if failure in ('collision', 'timeout', 'changed'):
        TcpJoy.assess_entry(n, 1.)
        n.entry_pending['future'].done.return_value = True
        n.entry_pending['future'].result.return_value = SimpleNamespace(valid=False)
    if failure == 'changed': n.measured[0] = .01
    TcpJoy.assess_entry(n, 1.2 if failure == 'timeout' else 1.02)
    data = json.loads(n.entry_pub.publish.call_args.args[0].data)
    assert data['safe'] is False and data['reason']


def test_candidate_a_passes_all_directions_without_claiming_collision_safety():
    from evaluate_tcp_ready import ArmKinematics, CANDIDATES, evaluate
    arm = Path(__file__).resolve().parents[2]
    model = ArmKinematics(arm/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          arm/'tool_manipulator_bringup/config/hardware.yaml')
    result = evaluate(model, CANDIDATES['candidate_a'], CANDIDATES['recovery_reference'])
    assert min(result['limit_margins_rad']) > .3
    assert result['condition'] < 20
    assert all(d['ik'] and d['max_condition'] < 20 for d in result['directions'].values())
    assert result['collision_free'] is None
    assert result['approach']['collision_free'] is None
    assert not result['acceleration_verified']
    assert max(result['approach']['peak_velocity_rad_s']) < .12


def test_focus_request_without_proof_is_rejected_without_auto_retry():
    node = SimpleNamespace(core=core(), focus_activation_pending=True, focus_arm_mode='tcp_ik')
    node.core.tcp_entry = None
    ControlModeManager.activate_pending_focus(node, 1.01)
    assert not node.focus_activation_pending
    assert node.core.mode == node.core.requested == 'hold'
    assert node.core.reason
    for i in range(1, 6):
        now = 1.01+i*.02
        node.core.feedback([0.]*6, [0.]*6, now)
        node.core.tick(now, .02)
    assert not node.core.transitioning
    assert node.core.reason.startswith('TCP entry')


def ready_node():
    n = SimpleNamespace(
        ready_enabled=True, auto_entry_armed=True, ready_state='idle',
        ready_goal=np.array([0., 1.2, 1.2, -1.2, 0., 0.]), ready_duration=10.,
        ready_step=.005, ready_timeout=3., ready_checks=[], ready_snapshot=None,
        ready_started=None, ready_future=None, ready_execution_id=None,
        ready_safe_at=None, measured=np.array([0., 1.48, 1.50, -1.55, 0., 0.]),
        full_state=dict(zip(('base_joint', 'shoulder_joint', 'elbow_joint',
                             'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint'),
                            [0., 1.48, 1.50, -1.55, 0., 0.])),
        status=dict(mode='hold', ready=True, startup_complete=True, focus='arm',
                    transitioning=False, epoch='hold-e'), status_time=1., state_time=1.,
        check=Mock(), tool_mode=Mock(), tcp_mode=Mock(), tool_pub=Mock(), reason='',
        deadman=True, joy_time=1.3, cancel=Mock())
    n.check.service_is_ready.return_value = True
    n.tool_mode.service_is_ready.return_value = True
    n.tcp_mode.service_is_ready.return_value = True
    n._reset_ready = lambda: TcpJoy._reset_ready(n)
    n._fail_ready = lambda reason: TcpJoy._fail_ready(n, reason)
    return n


def test_tcp_ready_path_is_checked_before_managed_execution():
    n = ready_node()
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'checking' and len(n.ready_checks) == 1
    assert len(n.ready_requests) > 1
    n.tool_mode.call_async.assert_not_called()
    for future in n.ready_checks:
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=True)
    mode_future = Mock()
    n.tool_mode.call_async.return_value = mode_future
    TcpJoy.drive_ready(n, 1.1)
    while n.ready_state == 'checking':
        TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'request_tool'
    mode_future.done.return_value = True
    mode_future.result.return_value = SimpleNamespace(success=True)
    TcpJoy.drive_ready(n, 1.2)
    n.status.update(mode='tool_auto', epoch='tool-e')
    n.status_time = n.state_time = 1.3
    TcpJoy.drive_ready(n, 1.3)
    payload = json.loads(n.tool_pub.publish.call_args.args[0].data)
    assert payload['epoch'] == 'tool-e'
    assert payload['waypoints'] == [n.ready_goal.tolist()]
    assert payload['durations'] == [10.]
    assert n.ready_state == 'executing'


def test_tcp_ready_collision_failure_never_requests_motion():
    n = ready_node()
    TcpJoy.drive_ready(n, 1.)
    for future in n.ready_checks:
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=True)
    n.ready_checks[-1].result.return_value = SimpleNamespace(valid=False)
    TcpJoy.drive_ready(n, 1.1)
    assert not n.auto_entry_armed and n.ready_state == 'failed'
    n.tool_mode.call_async.assert_not_called()
    n.tool_pub.publish.assert_not_called()


def test_tcp_ready_waits_for_one_response_and_times_out_without_motion():
    n = ready_node()
    n.check.call_async.return_value.done.return_value = False
    TcpJoy.drive_ready(n, 1.)
    TcpJoy.drive_ready(n, 1.1)
    assert n.check.call_async.call_count == 1
    n.status_time = n.state_time = 5.
    TcpJoy.drive_ready(n, 5.)
    assert n.ready_state == 'failed'
    assert '0/' in n.reason
    n.tool_mode.call_async.assert_not_called()
    n.tool_pub.publish.assert_not_called()


def test_tcp_ready_verified_path_requires_l1():
    n = ready_node()
    n.deadman = False
    n.check.call_async.return_value.done.return_value = True
    n.check.call_async.return_value.result.return_value = SimpleNamespace(valid=True)
    TcpJoy.drive_ready(n, 1.)
    for _ in range(len(n.ready_requests)+1):
        TcpJoy.drive_ready(n, 1.1)
    assert 'hold L1' in n.reason
    n.tool_mode.call_async.assert_not_called()


def test_tcp_ready_execution_cancels_on_stale_joystick():
    n = ready_node()
    n.ready_state = 'executing'
    n.ready_execution_id = 'ready-test'
    n.joy_time = 0.
    n.status.update(mode='tool_auto')
    TcpJoy.drive_ready(n, 1.)
    payload = json.loads(n.tool_pub.publish.call_args.args[0].data)
    assert payload['cancel'] is True
    assert payload['execution_id'] == 'ready-test'
    assert n.ready_state == 'failed'
