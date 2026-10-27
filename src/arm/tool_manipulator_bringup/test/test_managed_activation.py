"""Exercise manager activation ordering without services or hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_control_mode_manager import ControlModeManager
from arm_mode_core import ModeCore


def harness(subscribers):
    node = SimpleNamespace()
    node.core = ModeCore([(-2., 2.)]*6, [.12]*6, startup_recovery_enabled=True)
    node.core.feedback([0., 2.03, 0., 0., 0., 0.], [0.]*6, 1.)
    node.list_future = node.switch_future = None
    node.controllers = {'manual_position_controller': 'inactive', 'ee_velocity_controller': 'active'}
    node.controller_time = node.last_poll = 1.
    node.activation_started = None
    node.activated = False
    node.list_client = Mock()
    node.switch_client = Mock()
    node.switch_client.service_is_ready.return_value = True
    node.position = Mock()
    node.position.get_subscription_count.return_value = subscribers
    return node


def test_no_activation_until_command_subscriber_discovered():
    node = harness(0)
    ControlModeManager.poll(node, 1.)
    node.switch_client.call_async.assert_not_called()
    assert 'position command subscriber not discovered' in node.activation_blockers


def test_measured_seed_is_published_before_strict_switch():
    node = harness(1)
    events = []
    node.position.publish.side_effect = lambda msg: events.append(('seed', list(msg.data)))
    node.switch_client.call_async.side_effect = lambda req: events.append(('switch', req)) or Mock()
    ControlModeManager.poll(node, 1.)
    assert [e[0] for e in events] == ['seed', 'switch']
    assert events[0][1] == node.core.position
    assert node.core.startup_recovering
    assert not node.core.ready


def test_drive_to_arm_edge_requests_configured_tcp_mode_once():
    node = SimpleNamespace()
    node.core = ModeCore([(-2., 2.)]*6, [.12]*6)
    node.core.feedback([0.]*6, [0.]*6, 1.)
    node.core.ready = True
    node.focus_arm_mode = 'tcp_ik'
    node.focus_activation_pending = False
    ControlModeManager.on_focus(node, String(data='arm'))
    assert node.focus_activation_pending
    ControlModeManager.activate_pending_focus(node, 1.)
    assert not node.focus_activation_pending
    assert node.core.requested == 'tcp_ik' and node.core.transitioning

    # HOLD/cancel while focus remains arm must not silently restart motion.
    node.core.mode = node.core.requested = 'hold'
    node.core.transitioning = False
    ControlModeManager.on_focus(node, String(data='arm'))
    assert not node.focus_activation_pending


def test_focus_activation_waits_for_controller_readiness():
    node = SimpleNamespace()
    node.core = ModeCore([(-2., 2.)]*6, [.12]*6)
    node.core.feedback([0.]*6, [0.]*6, 1.)
    node.core.focus = 'arm'
    node.core.ready = False
    node.focus_arm_mode = 'tcp_ik'
    node.focus_activation_pending = True
    ControlModeManager.activate_pending_focus(node, 1.)
    assert node.focus_activation_pending and node.core.mode == 'hold'
    node.core.ready = True
    ControlModeManager.activate_pending_focus(node, 1.)
    assert not node.focus_activation_pending and node.core.requested == 'tcp_ik'


def test_focus_activation_waits_for_first_dxl_alignment():
    node = SimpleNamespace()
    node.core = ModeCore([(-2., 2.)]*6, [.12]*6)
    node.core.feedback([0.]*6, [0.]*6, 1.)
    node.core.focus, node.core.ready = 'arm', True
    node.core.entry_alignment_enabled = True
    node.core.entry_alignment_complete = False
    node.focus_arm_mode = 'tcp_ik'
    node.focus_activation_pending = True
    ControlModeManager.activate_pending_focus(node, 1.)
    assert node.focus_activation_pending and node.core.mode == 'hold'
    node.core.tick(1.02, .02)
    assert node.core.entry_alignment_complete
    ControlModeManager.activate_pending_focus(node, 1.02)
    assert not node.focus_activation_pending
