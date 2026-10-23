"""Check restored clone Servo teleop mappings and stop behavior without hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import joint_joy_teleop as legacy


class Stamp:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds*1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds-other.nanoseconds)

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


def harness(module):
    node = module.JointJoyTeleop.__new__(module.JointJoyTeleop)
    parameters = {}
    node.declare_parameter = lambda name, value: parameters.setdefault(name, value)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    clock = SimpleNamespace(seconds=0.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Stamp(clock.seconds))
    node.get_logger = lambda: Mock()
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription, node.create_timer = Mock(), Mock()
    with patch.object(module.Node, '__init__', return_value=None):
        module.JointJoyTeleop.__init__(node)
    return node, clock


def joy(axes=None, buttons=()):
    message = legacy.Joy(axes=[0.]*8, buttons=[0]*10)
    for index, value in (axes or {}).items():
        message.axes[index] = value
    for index in buttons:
        message.buttons[index] = 1
    return message


class JointJoyTeleopTest(unittest.TestCase):
    def test_clone_original_button_and_axis_mappings(self):
        node, _ = harness(legacy)
        self.assertEqual(node._velocities(joy({7: 1., 6: -1.}, (4, 3, 5, 8))),
                         [.12, .12, .12, -.12, .12, .12])
        self.assertEqual(node._velocities(joy({7: -1., 6: 1.}, (2, 1, 6, 7))),
                         [-.12, -.12, -.12, .12, -.12, -.12])

    def test_neutral_missing_deadzone_and_nonfinite_inputs_stop(self):
        for module in (legacy,):
            node, _ = harness(module)
            for message in (joy(), module.Joy(), joy({4: .05}),
                            joy({4: float('nan'), 1: float('inf')})):
                self.assertEqual(node._velocities(message), [0.]*6)

    def test_focus_release_and_timeout_stop(self):
        for module in (legacy,):
            node, clock = harness(module)
            # L1 drives base in both versions; Options only changes focus.
            node._joy_cb(joy(buttons=(4,)))
            node._publish_command()
            node._publisher.publish.assert_not_called()
            node._joy_cb(joy(buttons=(4, 9)))
            node._publish_command()
            self.assertTrue(any(node._publisher.publish.call_args.args[0].velocities))
            node._joy_cb(joy())
            node._publish_command()
            self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities), [0.]*6)
            node._joy_cb(joy(buttons=(4,)))
            node._publish_command()
            clock.seconds = .3
            node._publish_command()
            self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities), [0.]*6)
            node._joy_cb(joy(buttons=(4,)))
            node._publish_command()
            node._focus_cb(module.String(data='drive'))
            self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities), [0.]*6)


if __name__ == '__main__':
    unittest.main()
