"""Exercise manager activation ordering without services or hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

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
