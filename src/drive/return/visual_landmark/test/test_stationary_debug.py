"""Temporary manual VPR: gates, same pipeline, bounded lifetime, no motion authority."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
pytest.importorskip('rclpy')
from std_msgs.msg import Bool, String
from test_cross_camera import live_fixture
from visual_landmark.visual_localization_node import VisualLocalizationNode as Localizer


def debug_fixture(monkeypatch):
    n, out, _, pair = live_fixture(monkeypatch)
    n.stationary_debug_enabled = True
    n.state = 'RETURN_MANUAL'
    n.check = None
    n.last_auto_progress = (0., 1)  # Manual intentionally uses a frozen Auto arc.
    messages = []
    n.status = NS(publish=lambda msg: messages.append(json.loads(msg.data)))
    n.event = lambda kind, **kw: Localizer.event(n, kind, **kw)
    n.stationary_debug_reason = lambda: Localizer.stationary_debug_reason(n)
    def fresh(vx=0., wz=0.):
        now = n.get_clock().now().nanoseconds*1e-9
        n.debug_odom = (now, now, vx, wz)
    fresh()
    n.raw_pair = pair
    def frame():
        fresh()
        pair()
    return n, out, messages, frame, fresh


def test_debug_off_and_false_trigger_do_nothing(monkeypatch):
    n, out, events, _, _ = debug_fixture(monkeypatch)
    n.stationary_debug_enabled = False
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check is None and not events and not out
    n.stationary_debug_enabled = True
    Localizer.on_debug_check(n, Bool(data=False))
    assert n.check is None and not events


@pytest.mark.parametrize('vx,wz', [(.031,0),(-.031,0),(0,.031),(0,-.031)])
def test_moving_trigger_rejected(monkeypatch, vx, wz):
    n, out, events, _, fresh = debug_fixture(monkeypatch)
    fresh(vx,wz)
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check is None and not out and n.state == 'RETURN_MANUAL'
    assert events[-1]['reason'] == 'stationary_debug_robot_moving'


@pytest.mark.parametrize('sample', [None, (0.,0.,0.,0.), (100.,100.,float('nan'),0.),
                                  (101.,101.,0.,0.)])
def test_missing_stale_nan_future_odom_rejected(monkeypatch, sample):
    n, out, events, _, _ = debug_fixture(monkeypatch)
    n.debug_odom = sample
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check is None and not out and events[-1]['event'] == 'debug_check_rejected'


def test_manual_two_frames_observation_and_retrigger(monkeypatch):
    n, out, events, frame, fresh = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    token = n.check['request']
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check['request'] == token
    assert events[-1]['reason'] == 'stationary_debug_busy'
    frame()
    assert n.pending is not None and n.check is not None
    frame()
    assert events[-1]['event'] == 'validated'
    assert events[-1]['trigger'] == 'stationary_debug'
    assert events[-1]['check_complete'] and not events[-1]['correction_applied']
    assert events[-1]['planar_correction']
    assert n.check is None and n.state == 'RETURN_MANUAL' and not out
    assert n.correction is None and n.last_reference_arc is None
    fresh()
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check['request'] != token


def test_motion_during_check_aborts_and_stale_odom_aborts(monkeypatch):
    n, out, events, _, fresh = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    fresh(.04)
    Localizer.check_timeout(n)
    assert n.check is None and events[-1]['reason'] == 'stationary_debug_robot_moving'
    fresh()
    Localizer.on_debug_check(n, Bool(data=True))
    n.debug_odom = (0.,0.,0.,0.)
    Localizer.check_timeout(n)
    assert n.check is None and not out


def test_timeout_and_max_frames_reused(monkeypatch):
    import visual_landmark.visual_localization_node as module
    n, out, events, frame, fresh = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    n.check['deadline'] = 99.
    Localizer.check_timeout(n)
    assert n.check is None and events[-1]['reason'] == 'check timeout'
    fresh()
    Localizer.on_debug_check(n, Bool(data=True))
    monkeypatch.setattr(module, 'match', lambda *a, **kw: None)
    for _ in range(4):
        frame()
    assert n.check is None and events[-1]['event'] == 'no_match'
    assert events[-1]['check_complete'] and not out


@pytest.mark.parametrize('state', ['FOLLOW_RETURN_PATH','IDLE','RETURN_RESUME_STOP'])
def test_wrong_state_rejected(monkeypatch, state):
    n, out, events, _, _ = debug_fixture(monkeypatch)
    n.state = state
    Localizer.on_debug_check(n, Bool(data=True))
    assert n.check is None and events[-1]['reason'] == 'stationary_debug_requires_RETURN_MANUAL'
    assert n.state == state and not out


def test_enabled_correction_rejected_and_missing_route_rejected(monkeypatch):
    n, out, events, _, _ = debug_fixture(monkeypatch)
    original = n.p
    n.p = lambda key: True if key == 'apply_correction' else original(key)
    Localizer.on_debug_check(n, Bool(data=True))
    assert events[-1]['reason'] == 'stationary_debug_requires_observation_only'
    n.p = original
    n.last_auto_progress = None
    Localizer.on_debug_check(n, Bool(data=True))
    assert events[-1]['reason'] == 'stationary_debug_missing_auto_route_progress'
    assert not out


def test_state_change_cancels_without_requesting_auto(monkeypatch):
    n, out, events, _, _ = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    Localizer.on_state(n, String(data='RETURN_RESUME_STOP'))
    assert n.check is None and n.state == 'RETURN_RESUME_STOP' and not out


def test_debug_pipeline_has_no_motion_or_state_publishers():
    import visual_landmark.visual_localization_node as module
    tree = ast.parse(Path(module.__file__).read_text())
    topics = [call.args[1].value for call in ast.walk(tree)
              if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
              and call.func.attr == 'create_publisher']
    assert set(topics) == {'/visual_landmark/correction', '/visual_landmark/localization_event',
                           '/visual_landmark/debug/matches/compressed',
                           '/visual_landmark/debug/reference/compressed'}


def test_motion_at_commit_suppresses_validation(monkeypatch):
    import visual_landmark.visual_localization_node as module
    import numpy as np
    n, out, events, frame, fresh = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    frame()
    def moving_match(*args, **kwargs):
        fresh(.04)
        return np.eye(4), 30, .2
    monkeypatch.setattr(module, 'match', moving_match)
    frame()
    assert n.check is None and not out
    assert events[-1]['reason'] == 'stationary_debug_robot_moving'
    assert n.state == 'RETURN_MANUAL'


def test_motion_before_frame_does_not_run_sift(monkeypatch):
    import visual_landmark.visual_localization_node as module
    n, out, events, _, fresh = debug_fixture(monkeypatch)
    Localizer.on_debug_check(n, Bool(data=True))
    fresh(.04)
    monkeypatch.setattr(module, 'features', lambda *args: pytest.fail('SIFT ran while moving'))
    n.raw_pair()
    assert n.check is None and not out
