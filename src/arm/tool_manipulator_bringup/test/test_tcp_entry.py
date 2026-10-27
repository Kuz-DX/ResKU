"""TCP entry must stay in HOLD without a fresh, stationary, collision-checked pose."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import ModeCore
from arm_control_mode_manager import ControlModeManager
from tcp_joy import TcpJoy


def test_options_tcp_ready_alignment_zeros_dxl_axes():
    config_path = Path(__file__).resolve().parents[1]/'config/tcp_ready.yaml'
    goal = yaml.safe_load(config_path.read_text())['tcp_ready']['positions_rad']
    assert len(goal) == 6
    assert [goal[i] for i in (0, 4, 5)] == [0.0, 0.0, 0.0]


def core():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.tcp_entry_required = True
    c.ready, c.focus = True, 'arm'
    c.feedback([0.]*6, [0.]*6, 1.)
    c.entry_stable_since = .7
    c.tcp_entry = dict(epoch=c.epoch, safe=True, reason='checked', positions=[0.]*6, time=1.)
    return c


@pytest.mark.parametrize('reason', ['missing', 'stale', 'collision', 'changed', 'margin', 'moving', 'epoch'])
def test_unsafe_entry_rejected_in_hold(reason):
    c = core()
    if reason == 'missing': c.tcp_entry = None
    if reason == 'stale': c.tcp_entry['time'] = 0.
    if reason == 'collision': c.tcp_entry.update(safe=False, reason='collision')
    if reason == 'changed': c.position[0] = .021
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


def test_entry_accepts_stopped_dxl_hunt_but_rejects_larger_change():
    c = core()
    c.position[0] = .019
    assert c.tcp_entry_blocker(1.01) == ''
    c.position[0] = .021
    assert 'feedback changed' in c.tcp_entry_blocker(1.01)


def test_rmd_entry_tolerance_remains_one_milliradian():
    c = core()
    c.position[1] = .0011
    assert 'feedback changed' in c.tcp_entry_blocker(1.01)


def test_entry_requires_stability_and_resets_on_drift():
    c = core()
    c.entry_stable_since = 1.
    assert 'stable feedback' in c.tcp_entry_blocker(1.1)
    c.feedback([0.]*6, [0.]*6, 1.21)
    assert c.tcp_entry_blocker(1.21) == ''
    c.feedback([.021]+[0.]*5, [0.]*6, 1.22)
    assert c.entry_stable_since == 1.22
    c.feedback([.021]+[0.]*5, [.03]+[0.]*5, 1.23)
    assert c.entry_stable_since is None


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
        ready_step=.005, ready_timeout=3., ready_max_delta=.5, home_max_delta=1.,
        entry_retry_count=0,
        ready_checks=[], ready_snapshot=None,
        ready_started=1., ready_future=None, ready_execution_id=None,
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
    assert 'sample 1/' in n.reason
    assert 'no contact pairs reported' in n.reason
    n.tool_mode.call_async.assert_not_called()
    n.tool_pub.publish.assert_not_called()


def test_tcp_ready_collision_failure_reports_contact_pair():
    n = ready_node()
    TcpJoy.drive_ready(n, 1.)
    n.ready_checks[-1].done.return_value = True
    n.ready_checks[-1].result.return_value = SimpleNamespace(
        valid=False, contacts=[SimpleNamespace(
            contact_body_1='wrist_link', contact_body_2='chassis_link')])
    TcpJoy.drive_ready(n, 1.1)
    assert 'chassis_link <-> wrist_link' in n.reason
    assert n.ready_state == 'failed'
    n.tool_mode.call_async.assert_not_called()


def test_tcp_ready_delta_failure_names_limiting_joint():
    n = ready_node()
    n.measured[1] = 1.626
    n.full_state['shoulder_joint'] = 1.626
    n.ready_max_delta = .35
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'failed'
    assert 'shoulder_joint 0.426 rad exceeds 0.350 rad' in n.reason
    n.check.call_async.assert_not_called()
    n.tool_mode.call_async.assert_not_called()


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


def test_tcp_ready_execution_is_one_shot_after_l1_authorizes_start():
    n = ready_node()
    n.ready_state = 'executing'
    n.ready_execution_id = 'ready-test'
    n.deadman = False
    n.joy_time = 0.
    n.status.update(mode='tool_auto')
    TcpJoy.drive_ready(n, 1.)
    n.tool_pub.publish.assert_not_called()
    assert n.ready_state == 'executing'


def test_entry_feedback_race_rechecks_without_relaxing_guard():
    n = ready_node()
    n.ready_state = 'request_tcp'
    n.ready_future = Mock()
    n.ready_future.result.return_value = SimpleNamespace(
        success=False, message='TCP entry feedback changed; recheck required')
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'await_entry'
    assert n.auto_entry_armed and n.ready_safe_at is None
    n.tool_pub.publish.assert_not_called()


def test_entry_feedback_race_stops_after_three_attempts():
    n = ready_node()
    n.ready_state = 'request_tcp'
    n.entry_retry_count = 2
    n.ready_future = Mock()
    n.ready_future.result.return_value = SimpleNamespace(
        success=False, message='TCP entry feedback changed; recheck required')
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'failed' and not n.auto_entry_armed
    assert 'three checks' in n.reason


def test_home_completion_stays_hold_and_restores_tcp_ready_target():
    n = ready_node()
    n.default_ready_goal = n.ready_goal.copy()
    n.ready_goal = np.radians([0., 85., 87., -89., 0., 0.])
    n.returning_home = True
    n.ready_state = 'executing'
    n.ready_execution_id = 'home'
    n.status.update(last_execution_id='home', last_execution_result='succeeded')
    TcpJoy.drive_ready(n, 1.)
    assert not n.returning_home and n.auto_entry_armed
    assert n.ready_state == 'idle'
    np.testing.assert_array_equal(n.ready_goal, n.default_ready_goal)
    n.tcp_mode.call_async.assert_not_called()


def test_home_executes_without_l1_and_does_not_cancel_on_release():
    n = ready_node()
    n.returning_home = True
    n.default_ready_goal = n.ready_goal.copy()
    n.ready_goal = np.radians([0., 85., 87., -89., 0., 0.])
    n.deadman = False
    n.joy_time = 0.
    n.check.call_async.return_value.done.return_value = True
    n.check.call_async.return_value.result.return_value = SimpleNamespace(valid=True)
    TcpJoy.drive_ready(n, 1.)
    for _ in range(len(n.ready_requests)+1):
        if n.ready_state == 'checking':
            TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'request_tool'
    n.ready_future.done.return_value = True
    n.ready_future.result.return_value = SimpleNamespace(success=True)
    TcpJoy.drive_ready(n, 1.1)
    n.status.update(mode='tool_auto')
    TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'executing'
    n.tool_pub.publish.reset_mock()
    TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'executing'
    n.tool_pub.publish.assert_not_called()


def test_explicit_home_uses_separate_larger_distance_limit():
    n = ready_node()
    n.returning_home = True
    n.ready_goal = n.measured.copy()
    n.ready_goal[1] -= .6
    n.check.call_async.return_value.done.return_value = False
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'checking'
    assert n.auto_entry_armed


def test_accepted_tcp_request_waits_for_actual_mode_and_recovers_on_rejection():
    n = ready_node()
    n.ready_state = 'request_tcp'
    n.ready_request_epoch = 'hold-e'
    n.ready_future = Mock()
    n.ready_future.result.return_value = SimpleNamespace(success=True)
    TcpJoy.drive_ready(n, 1.)
    assert n.ready_state == 'wait_tcp' and n.auto_entry_armed
    n.status.update(epoch='new', mode='hold', transitioning=False,
                    reason='TCP entry waiting for 0.20 sec stable feedback')
    TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'failed' and not n.auto_entry_armed


def test_tcp_transition_only_finishes_when_mode_is_confirmed():
    n = ready_node()
    n.ready_state = 'wait_tcp'
    n.ready_request_epoch = 'hold-e'
    n.status.update(mode='tcp_ik', transitioning=False)
    TcpJoy.drive_ready(n, 1.1)
    assert n.ready_state == 'idle' and not n.auto_entry_armed


def test_tcp_transition_timeout_is_retryable():
    n = ready_node()
    n.ready_state = 'request_tcp'
    n.ready_future = Mock()
    n.ready_future.done.return_value = False
    TcpJoy.drive_ready(n, 4.1)
    assert n.ready_state == 'failed'


@pytest.mark.parametrize('mode,pressed,twist,retry', [
    ('hold', False, 0., True), ('hold', True, 0., False),
    ('hold', False, 1., False), ('fault', False, 0., False),
    ('tool_auto', False, 0., False)])
def test_failed_ready_retries_only_on_fresh_neutral_l1_in_hold(mode, pressed, twist, retry):
    n = ready_node()
    n.ready_state = 'failed'
    n.auto_entry_armed = False
    n.default_ready_goal = n.ready_goal.copy()
    n.status['mode'] = mode
    n.twist = np.ones(6)*twist
    TcpJoy.retry_ready(n, True, pressed, False, 1.1)
    assert n.auto_entry_armed is retry
    assert n.ready_state == ('idle' if retry else 'failed')
