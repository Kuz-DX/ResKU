"""Production protocol and ROS adapter methods, no hardware or ROS graph."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tool_change_min.changer_protocol import ChangerTransaction


@pytest.mark.parametrize('command,done', [('LOCK', 'LOCKED'), ('UNLOCK', 'UNLOCKED')])
def test_only_matching_terminal_response_completes(command, done):
    t = ChangerTransaction(command, 1., 10.)
    for msg in ['bad JSON', '[]', json.dumps(dict(id='old', state=done)),
                json.dumps(dict(id=t.id, state='MOVING')),
                json.dumps(dict(id=t.id, state='UNLOCKED' if done == 'LOCKED' else 'LOCKED'))]:
        t.receive(msg, 10.1)
        assert not t.check(10.1)
    t.receive(json.dumps(dict(id=t.id, state=done)), 10.2)
    assert t.check(10.3)
    t.receive(json.dumps(dict(id=t.id, state='ERROR')), 10.4)
    assert t.check(10.4)


def test_cancel_wins_over_queued_completion_and_stop_once():
    t = ChangerTransaction('LOCK', 1., 10.)
    t.receive(json.dumps(dict(id=t.id, state='LOCKED')), 10.1)
    assert json.loads(t.abort('cancelled')) == dict(id=t.id, command='STOP')
    assert t.abort('cancelled') is None
    t.receive(json.dumps(dict(id=t.id, state='LOCKED')), 10.2)
    with pytest.raises(RuntimeError, match='cancelled'): t.check(10.2)


def test_timeout_and_error_latch_and_ids_are_unique():
    t = ChangerTransaction('LOCK', 1., 10.)
    assert t.id != ChangerTransaction('LOCK', 1., 10.).id
    with pytest.raises(RuntimeError, match='timeout'):
        t.receive(json.dumps(dict(id=t.id, state='LOCKED')), 11.)
    assert not t.complete
    t = ChangerTransaction('LOCK', 1., 10.)
    t.receive(json.dumps(dict(id=t.id, state='ERROR', detail='jam')), 10.1)
    with pytest.raises(RuntimeError, match='jam'): t.check(10.1)


def adapter():
    path = ROOT/'scripts/tool_change_fsm.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = {'_arduino_lock', '_check_changer_safety', '_docked', '_changer_status',
               '_abort_changer', '_feedback', '_cancel_changer', '_hardware_fault'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    clock = NS(now=10.)
    clock.monotonic = lambda: clock.now
    clock.sleep = lambda dt: setattr(clock, 'now', clock.now+dt)
    ns = dict(Node=object, time=clock, math=math, ChangerTransaction=ChangerTransaction,
              String=NS, rclpy=NS(ok=lambda: True))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])),
                 str(path), 'exec'), ns)
    n = ns['ToolChangeFsm']()
    n.changer_guard = threading.RLock()
    n.transaction = None
    n.docking_goal = [0.]*6
    n.feedback_sequence = 0
    n.feedback_snapshot = (10., [0.]*6, [0.]*6)
    n.hardware_error = ''
    n.cancelled = False
    n.controller_active, n.controller_checked = True, 10.
    n.state = 'LOCK'
    n.config = dict(joint_names=list('abcdef'), feedback=dict(joint_state_max_age_s=.5),
        timeouts_s=dict(lock=.1), tool_changer=dict(enabled=True, command='LOCK',
        docking_timeout_s=.2, settled_samples=3, position_tolerance_rad=.005,
        velocity_tolerance_rad_s=.02))
    n.changer_pub = Mock()
    n.changer_pub.get_subscription_count.return_value = 1
    def advance(dt):
        clock.now += dt
        n.feedback_sequence += 1
        n.feedback_snapshot = (clock.now, [0.]*6, [0.]*6)
    clock.sleep = advance
    return n, clock


def test_adapter_waits_for_new_samples_then_sends_once_and_completes():
    n, clock = adapter()
    def complete(msg):
        d = json.loads(msg.data)
        assert n.feedback_sequence >= 3
        n._changer_status(NS(data=json.dumps(dict(id=d['id'], state='LOCKED'))))
    n.changer_pub.publish.side_effect = complete
    n._arduino_lock()
    n.changer_pub.publish.assert_called_once()
    assert n.transaction is None


@pytest.mark.parametrize('outcome', ['timeout', 'ERROR', 'cancel', 'drift', 'stale', 'controller'])
def test_adapter_failure_sends_stop_never_success(outcome):
    n, clock = adapter()
    def sent(msg):
        d = json.loads(msg.data)
        if d['command'] != 'LOCK': return
        if outcome == 'ERROR': n._changer_status(NS(data=json.dumps(dict(id=d['id'], state='ERROR'))))
        if outcome == 'cancel': n._cancel_changer(None, NS())
        if outcome == 'drift': n.feedback_snapshot = (clock.now, [.1]*6, [0.]*6)
        if outcome == 'stale': n.feedback_snapshot = None
        if outcome == 'controller': n.controller_active = False
    n.changer_pub.publish.side_effect = sent
    with pytest.raises(RuntimeError): n._arduino_lock()
    commands = [json.loads(c.args[0].data) for c in n.changer_pub.publish.call_args_list]
    assert [c['command'] for c in commands] == ['LOCK', 'STOP']
    assert commands[0]['id'] == commands[1]['id']
    assert n.transaction is None


def test_no_command_when_feedback_does_not_settle_or_receiver_absent():
    n, clock = adapter()
    clock.sleep = lambda dt: setattr(clock, 'now', clock.now+dt)
    with pytest.raises(TimeoutError): n._arduino_lock()
    n.changer_pub.publish.assert_not_called()
    n, clock = adapter()
    n.changer_pub.get_subscription_count.return_value = 0
    with pytest.raises(RuntimeError, match='subscriber'): n._arduino_lock()
    n.changer_pub.publish.assert_not_called()
