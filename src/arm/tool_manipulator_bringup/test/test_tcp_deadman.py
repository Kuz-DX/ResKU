"""Deadman release/reconnect and epoch-bound measured HOLD, without ROS execution."""
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from test_tcp_safety import node, tick, messages
from tcp_joy import TcpJoy
from tcp_joy_core import EeDpad
from arm_mode_core import ModeCore
from arm_control_mode_manager import ControlModeManager


def joy(held, axis=0.):
    msg = Joy(axes=[0.]*8, buttons=[0]*13)
    msg.buttons[4] = int(held)
    msg.axes[1] = axis
    return msg


def send(n, msg, now=10.01):
    with patch('time.monotonic', return_value=now):
        TcpJoy.on_joy(n, msg)


def active_core():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.focus, c.ready = 'arm', True
    c.feedback([.2]*6, [0.]*6, 10.)
    c.mode = c.requested = 'tcp_ik'
    c.target, c.goal = [.25]*6, [.28]*6
    c.command_time = 10.
    return c


@pytest.mark.parametrize('button', [2, 3, 4])
def test_l2_path_buttons_never_request_home_or_tcp_ready(button):
    n = node()
    n.ready_enabled = True
    n.ready_state = 'idle'
    n.ready_checks = []
    n.default_ready_goal = np.zeros(6)
    n.auto_entry_armed = True
    n._reset_ready = lambda: TcpJoy._reset_ready(n)
    msg = joy(False)
    msg.axes[2] = -1.
    msg.buttons[button] = 1
    send(n, msg)
    assert not n.auto_entry_armed and not n.returning_home
    assert not n.deadman and not n.rearmed
    assert n.ready_state == 'idle'
    n.cancel.call_async.assert_not_called()
    assert messages(n) == [dict(epoch='e', pause=True)]


def test_l2_with_home_and_l1_cannot_start_tcp_or_home_workflow():
    n = node()
    n.ready_enabled = True
    n.ready_state = 'idle'
    n.ready_checks = []
    n.default_ready_goal = np.zeros(6)
    n.auto_entry_armed = True
    n._reset_ready = lambda: TcpJoy._reset_ready(n)
    n.request_named_pose = Mock()
    msg = joy(True, 1.)
    msg.axes[2] = -1.
    msg.buttons[3] = 1
    send(n, msg)
    n.request_named_pose.assert_not_called()
    n.model.solve.assert_not_called()
    assert not n.deadman and not n.rearmed
    assert messages(n) == [dict(epoch='e', pause=True)]


def test_r2_only_assesses_current_stopped_pose_without_motion_request():
    n = node()
    n.status.update(mode='hold', startup_complete=True,
                    velocities_rad_s=[0.]*6)
    n.entry_pub = Mock()
    n.auto_entry_armed = True
    n.ready_state = 'idle'
    n.ready_safe_at = None
    n.r2_assessment_requested = False
    n.previous_nearest = False
    n.request_named_pose = Mock()
    msg = joy(False)
    msg.axes[5] = -1.
    send(n, msg)
    assert n.entry_pending is not None
    assert not n.auto_entry_armed
    n.request_named_pose.assert_not_called()
    n.model.solve.assert_not_called()
    n.entry_pending['future'].done.return_value = True
    n.entry_pending['future'].result.return_value = SimpleNamespace(valid=True)
    TcpJoy.assess_entry(n, 10.02)
    result = json.loads(n.entry_pub.publish.call_args.args[0].data)
    assert result['safe'] is True
    assert 'no motion' in n.reason
    assert n.ready_state == 'idle'


def test_r2_rejects_moving_feedback_without_collision_or_motion_request():
    n = node()
    n.deadman = n.deadman_engaged = False
    n.status.update(mode='hold', startup_complete=True,
                    velocities_rad_s=[0., 0., .021, 0., 0., 0.])
    n.entry_pub = Mock()
    n.auto_entry_armed = True
    n.r2_assessment_requested = False
    n.previous_nearest = False
    n.request_named_pose = Mock()
    msg = joy(False)
    msg.axes[5] = -1.
    send(n, msg)
    assert 'stopped HOLD' in n.reason
    n.check.call_async.assert_not_called()
    n.request_named_pose.assert_not_called()


def test_wrist_modifier_moves_only_wrist_and_skips_ik():
    n = node()
    msg = joy(True)
    msg.buttons[5] = 1
    msg.axes[0], msg.axes[3] = 1., -1.
    send(n, msg)
    tick(n)
    n.model.solve.assert_not_called()
    assert n.pending is not None
    np.testing.assert_array_equal(n.pending['q'][:4], np.zeros(4))
    assert n.pending['q'][4] > 0 and n.pending['q'][5] < 0
    # Proposed command remains uncommitted until collision results arrive.
    np.testing.assert_array_equal(n.stepper.q, n.measured)
    send(n, joy(True), 10.03)
    assert n.pending is None and n.stepper.q is None


def test_translation_keeps_simultaneous_x_and_z_input():
    n = node()
    msg = joy(True, 1.)
    msg.axes[4] = 1.
    send(n, msg)
    assert n.twist[0] > 0 and n.twist[2] > 0
    np.testing.assert_array_equal(n.twist[3:], np.zeros(3))
    assert np.linalg.norm(n.twist[:3]) <= 1.000001


def test_release_cancels_pending_and_repress_solves_from_measured_pose():
    n = node()
    future = Mock()
    future.done.return_value = False
    n.pending = dict(futures=[future])
    send(n, joy(False))
    future.cancel.assert_called_once()
    assert n.pending is None and not n.blocked and n.rearmed
    assert messages(n) == [dict(epoch='e', pause=True)]
    send(n, joy(True, 1.), 10.015)
    tick(n)
    assert messages(n)[0] == dict(epoch='e', pause=True)
    n.model.solve.assert_called()
    assert n.pending is not None
    assert n.failures == 0


def test_timeout_stops_even_when_collision_service_is_unavailable():
    n = node()
    n.joy_time = 9.7
    n.check.service_is_ready.return_value = False
    tick(n)
    assert n.blocked and not n.rearmed and not n.deadman
    assert messages(n) == [dict(epoch='e', stop=True)]
    send(n, joy(True, 1.))
    tick(n, 10.04)
    n.model.solve.assert_not_called()


@pytest.mark.parametrize('kind', ['short_axes', 'short_buttons', 'nan', 'bad_button'])
def test_bad_joy_packet_cannot_rearm(kind):
    n = node()
    msg = joy(False)
    if kind == 'short_axes': msg.axes = [0.]*2
    if kind == 'short_buttons': msg.buttons = []
    if kind == 'nan': msg.axes[0] = float('nan')
    if kind == 'bad_button': msg.buttons[4] = 2
    send(n, msg)
    assert n.blocked and not n.rearmed
    assert messages(n)[0]['stop']


def test_new_epoch_requires_release_before_press():
    n = node()
    n.epoch = 'old'
    tick(n)
    assert not n.rearmed
    n.model.solve.assert_not_called()
    send(n, joy(False), 10.03)
    assert n.rearmed and not n.blocked
    send(n, joy(True, 1.), 10.04)
    tick(n, 10.06)
    n.model.solve.assert_called()
    assert n.pending is not None


def test_held_neutral_preserves_target_without_ik():
    n = node()
    n.stepper.q[:] = .01
    send(n, joy(True))
    tick(n)
    n.model.solve.assert_not_called()
    assert messages(n)[0]['positions'] == [.01]*6
    assert not n.blocked


def test_hold_never_produces_new_joint_command():
    n = node()
    n.status['mode'] = 'hold'
    tick(n)
    send(n, joy(True, 1.))
    tick(n, 10.04)
    n.pub.publish.assert_not_called()
    n.model.solve.assert_not_called()


@pytest.mark.parametrize('mode', ['manual', 'tool_auto', 'hold', 'fault'])
def test_non_tcp_modes_publish_only_explicit_l1_r1_dpad_ee_jog(mode):
    n = node()
    n.ee_dpad = EeDpad()
    n.full_state['ee_joint'] = 0.
    n.status['mode'] = mode
    tick(n)
    n.pub.publish.assert_not_called()
    n.pub.reset_mock()
    msg = joy(True)
    msg.buttons[5] = 1
    msg.axes[6] = 1.
    send(n, msg, 10.02)
    tick(n, 10.04)
    assert messages(n)[-1] == dict(independent_ee_velocity=5.)
    n.model.solve.assert_not_called()


@pytest.mark.parametrize('mode', ['manual', 'tcp_ik', 'tool_auto', 'hold', 'fault'])
def test_gateway_accepts_independent_ee_in_every_mode_with_fresh_ready_ee(mode):
    core = SimpleNamespace(mode=mode, focus='arm', ready=True, ee_state_time=10.)
    gateway = SimpleNamespace(
        core=core, controllers={'ee_velocity_controller': 'active'}, last_error='',
        independent_ee_velocity=0., independent_ee_command_time=-float('inf'))
    with patch('arm_control_mode_manager.time.monotonic', return_value=10.1):
        ControlModeManager.on_command(gateway, 'tcp_ik', String(data=json.dumps(
            dict(independent_ee_velocity=5.))))
    assert gateway.independent_ee_velocity == 5.
    assert gateway.independent_ee_command_time == pytest.approx(10.1)
    core.ready = False
    with patch('arm_control_mode_manager.time.monotonic', return_value=10.2):
        ControlModeManager.on_command(gateway, 'tcp_ik', String(data=json.dumps(
            dict(independent_ee_velocity=6.))))
    assert gateway.independent_ee_velocity == 5.
    assert 'controller or feedback unavailable' in gateway.last_error


def test_gateway_stop_captures_measurement_and_invalidates_queued_targets():
    c = active_core()
    epoch = c.epoch
    gateway = SimpleNamespace(core=c, last_error='')
    with patch('time.monotonic', return_value=10.01):
        ControlModeManager.on_command(gateway, 'tcp_ik', String(data=json.dumps(
            dict(epoch=epoch, stop=True))))
    assert c.mode == c.requested == 'hold' and c.epoch != epoch
    assert c.target == c.goal == [.2]*6 and c.ee == 0.
    assert c.tick(10.02, .02) == [.2]*6
    assert not c.command('tcp_ik', epoch, [.28]*6, 0., 10.03)
    assert not c.stop_tcp(epoch, 10.03)


@pytest.mark.parametrize('mode', ['manual', 'hold', 'fault'])
def test_delayed_stop_cannot_interfere_with_other_modes(mode):
    c = active_core()
    c.mode = c.requested = mode
    old_target = c.target.copy()
    assert not c.stop_tcp(c.epoch, 10.01)
    assert c.mode == mode and c.target == old_target


def test_stale_stop_cannot_cancel_new_tcp_epoch():
    c = active_core()
    assert not c.stop_tcp('old', 10.01)
    assert c.mode == 'tcp_ik' and c.target == [.25]*6


def test_stop_with_lost_feedback_faults_instead_of_using_old_goal():
    c = active_core()
    assert c.stop_tcp(c.epoch, 11.)
    assert c.mode == 'fault' and c.target == [.2]*6


def test_release_stops_even_after_collision_loss_clears_rearm_flag():
    n = node()
    n.check.service_is_ready.return_value = False
    tick(n)
    assert not n.rearmed and n.deadman_engaged
    n.pub.publish.reset_mock()
    send(n, joy(False), 10.03)
    assert not n.blocked
    assert messages(n) == [dict(epoch='e', pause=True)]


def test_pause_retains_mode_and_epoch_but_discards_old_lead():
    c = active_core()
    epoch = c.epoch
    assert c.pause_tcp(epoch, 10.01)
    assert c.mode == 'tcp_ik' and c.epoch == epoch
    assert c.target == c.goal == [.2]*6
    assert not c.pause_tcp('old', 10.02)
