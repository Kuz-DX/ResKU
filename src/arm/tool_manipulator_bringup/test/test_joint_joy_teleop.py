"""Check joystick mappings and stop behavior without ROS nodes or motor commands."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
try:
    import joint_joy_teleop as grouped
    import joint_joy_teleop_sep as separate
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f"ROS Python environment required: {exc}")


class Stamp:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time()


def harness(module, **overrides):
    """Run the real constructor with only ROS infrastructure replaced."""
    node = module.JointJoyTeleop.__new__(module.JointJoyTeleop)
    parameters = dict(overrides)
    node.declare_parameter = lambda name, value: parameters.setdefault(name, value)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    clock = SimpleNamespace(seconds=0.0)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Stamp(clock.seconds))
    node.get_logger = lambda: Mock()
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription = Mock()
    node.create_timer = Mock()
    with patch.object(module.Node, "__init__", return_value=None):
        module.JointJoyTeleop.__init__(node)
    if module is grouped:
        node._state_cb(grouped.JointState(name=node._joints, position=[0.0]*len(node._joints)))
    return node, clock


def joy(axes=None, buttons=()):
    message = grouped.Joy()
    message.axes = [0.0] * 8
    message.buttons = [0] * 10
    for index, value in (axes or {}).items():
        message.axes[index] = value
    for index in buttons:
        message.buttons[index] = 1
    return message


class JointJoyTeleopTest(unittest.TestCase):
    def test_saturated_shoulder_does_not_stop_other_joints(self):
        names = ["shoulder_joint", "elbow_joint", "wrist_pitch_joint"]
        limits = dict.fromkeys(names, (-1.0, 1.0))
        positions = dict(zip(names, [0.95, 0.0, 0.0]))
        v = grouped.coordinated_velocities(names, [0.12, -0.12, 0.12], positions, limits, 0.05)
        self.assertEqual(v, [0.0, -0.12, 0.12])
        reverse = grouped.coordinated_velocities(names, [-0.12, 0.12, -0.12], positions, limits, 0.05)
        self.assertEqual(reverse, [-0.12, 0.12, -0.12])

    def test_limit_transition_is_smooth_monotonic_and_bounded(self):
        values = [grouped.coordinated_velocities(["j"], [0.12], {"j": q},
                  {"j": (-1, 1)}, 0.05)[0] for q in (0.7, 0.8, 0.85, 0.9, 0.94, 0.95)]
        self.assertTrue(all(0 <= v <= 0.12 for v in values))
        self.assertTrue(all(a >= b for a, b in zip(values, values[1:])))
        self.assertEqual(values[-1], 0.0)

    def test_lower_limit_and_missing_state(self):
        self.assertEqual(grouped.coordinated_velocities(["j"], [-0.12],
                         {"j": -0.96}, {"j": (-1, 1)}, 0.05), [0.0])
        self.assertEqual(grouped.coordinated_velocities(["j"], [0.12],
                         {"j": -0.96}, {"j": (-1, 1)}, 0.05), [0.12])
        self.assertEqual(grouped.coordinated_velocities(["j"], [0.12],
                         {}, {"j": (-1, 1)}, 0.05), [0.0])

    def test_stale_joint_state_stops_with_fresh_joystick(self):
        node, clock = harness(grouped)
        node._joy_cb(joy({4: 1.0}, (9,)))
        node._publish_command()
        self.assertTrue(any(node._publisher.publish.call_args.args[0].velocities))
        clock.seconds = 0.3
        node._joy_cb(joy({4: 1.0}))
        node._publish_command()
        self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities), [0.0]*6)

    def test_grouped_axis_moves_all_three_rmds_in_both_directions(self):
        node, _ = harness(grouped)
        for direction in (1.0, -1.0, 0.5):
            speed = direction * 0.12
            self.assertEqual(node._velocities(joy({4: direction})),
                             [0.0, speed, -speed, speed, 0.0, 0.0])

    def test_old_independent_inputs_do_not_move_grouped_rmds(self):
        node, _ = harness(grouped)
        self.assertEqual(node._velocities(joy({1: 1.0, 7: -1.0})), [0.0] * 6)

    def test_grouped_axis_can_be_remapped(self):
        node, _ = harness(grouped, rmd_pitch_axis=7)
        self.assertEqual(node._velocities(joy({7: 1.0})),
                         [0.0, 0.12, -0.12, 0.12, 0.0, 0.0])
        self.assertEqual(node._velocities(joy({4: 1.0})), [0.0] * 6)

    def test_sep_preserves_three_independent_inputs_and_directions(self):
        node, _ = harness(separate)
        for axis, expected in (
            (4, [0.0, 0.12, 0.0, 0.0, 0.0, 0.0]),
            (1, [0.0, 0.0, -0.12, 0.0, 0.0, 0.0]),
            (7, [0.0, 0.0, 0.0, 0.12, 0.0, 0.0]),
        ):
            for direction in (1.0, -1.0):
                self.assertEqual(node._velocities(joy({axis: direction})),
                                 [direction * value for value in expected])

    def test_other_joint_controls_are_preserved_in_both_versions(self):
        for module in (grouped, separate):
            node, _ = harness(module)
            self.assertEqual(node._velocities(joy({6: 1.0}, (4, 3))),
                             [0.12, 0.0, 0.0, 0.0, 0.12, 0.12])
            self.assertEqual(node._velocities(joy({6: -1.0}, (5, 1))),
                             [-0.12, 0.0, 0.0, 0.0, -0.12, -0.12])

    def test_neutral_missing_deadzone_and_nonfinite_inputs_stop(self):
        for module in (grouped, separate):
            node, _ = harness(module)
            for message in (joy(), module.Joy(), joy({4: 0.05}),
                            joy({4: float("nan"), 1: float("inf")})):
                self.assertEqual(node._velocities(message), [0.0] * 6)

    def test_focus_timeout_and_release_stop_both_versions(self):
        for module in (grouped, separate):
            with self.subTest(module=module.__name__):
                node, clock = harness(module)
                node._joy_cb(joy({4: 1.0}))
                node._publish_command()
                node._publisher.publish.assert_not_called()
                node._joy_cb(joy({4: 1.0}, (9,)))
                node._publish_command()
                self.assertTrue(any(node._publisher.publish.call_args.args[0].velocities))
                node._joy_cb(joy())
                node._publish_command()
                self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities),
                                 [0.0] * 6)
                node._joy_cb(joy({4: 1.0}))
                node._publish_command()
                clock.seconds = 0.3
                node._publish_command()
                self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities),
                                 [0.0] * 6)
                node._joy_cb(joy({4: 1.0}))
                node._publish_command()
                node._focus_cb(module.String(data="drive"))
                self.assertEqual(list(node._publisher.publish.call_args.args[0].velocities),
                                 [0.0] * 6)


if __name__ == "__main__":
    unittest.main()
