"""Dynamixel ID4 jog mapping, speed edges, limits and deadman gating."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tcp_joy_core import EeDpad
from tcp_joy import TcpJoy


def axes(horizontal=0., vertical=0.):
    return [0., 0., 1., 0., 0., 0., horizontal, vertical]


def test_speed_changes_once_per_press_and_stays_within_manager_limit():
    dpad = EeDpad()
    dpad.update(axes(vertical=1.))
    assert dpad.speed == .08
    for _ in range(100):
        dpad.update(axes(vertical=1.))
    assert dpad.speed == .08
    for direction, limit in [(1., .12), (-1., .02)]:
        for _ in range(20):
            dpad.update(axes())
            dpad.update(axes(vertical=direction))
        assert dpad.speed == limit


def test_direction_release_diagonal_and_disabled_input():
    dpad = EeDpad()
    for h, expected in [(1., 1), (-1., -1), (0., 0)]:
        dpad.update(axes(horizontal=h))
        assert dpad.direction == expected
    dpad.update(axes(1., 1.))
    assert dpad.direction == 0
    dpad.update(axes(1.), enabled=False)
    assert dpad.direction == 0


@pytest.mark.parametrize('deadman,rearmed,age,position,direction,expected', [
    (True, True, 0., 0., 1, .06), (True, True, 0., 0., -1, -.06),
    (False, True, 0., 0., 1, 0.), (True, False, 0., 0., 1, 0.),
    (True, True, 1., 0., 1, 0.), (True, True, 0., None, 1, 0.),
    (True, True, 0., 2.839, 1, 0.), (True, True, 0., -2.351, -1, 0.),
])
def test_publish_ee_only_when_authorized(deadman, rearmed, age, position, direction, expected):
    dpad = EeDpad()
    dpad.direction = direction
    node = SimpleNamespace(blocked=False, epoch='test', deadman=deadman, rearmed=rearmed,
                           ee_dpad=dpad, ee_limits=(-2.351592547829, 2.839398438376),
                           joy_time=10.-age, state_time=10., full_state={'ee_joint': position},
                           stepper=SimpleNamespace(q=np.zeros(6)), pub=Mock())
    with patch('tcp_joy.time.monotonic', return_value=10.):
        TcpJoy.publish(node)
    data = json.loads(node.pub.publish.call_args.args[0].data)
    if deadman:
        assert data['ee_velocity'] == pytest.approx(expected)
    else:
        assert data['pause'] is True
