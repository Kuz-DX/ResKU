"""Exercise production callbacks without ROS or hardware, following action tests."""
import ast
from concurrent.futures import Future
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest


source = Path(__file__).resolve().parents[1] / "scripts/joystick_manipulator.py"
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
cls.body = [n for n in cls.body if not isinstance(n, ast.FunctionDef) or n.name != "__init__"]
scope = dict(Node=object, math=math, time=time, String=NS, Duration=NS,
             JointTrajectoryPoint=NS, GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5),
             FollowJointTrajectory=NS(Goal=lambda: NS(trajectory=NS()), Result=NS(SUCCESSFUL=0)))
exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), "exec"), scope)


def harness():
    node = scope["JoystickManipulator"]()
    node.settings = dict(focus_button=9, deadman_button=4, jog_axis=4, select_axis=6,
                         joy_timeout_s=0.25, feedback_timeout_s=0.5, deadzone=0.12,
                         invert_jog_axis=False, limit_margin_rad=0.01,
                         step_duration_s=0.2, action_timeout_s=2.0)
    node.names = ["joint"]
    node.model = NS(limits={"joint": (-1, 1)}, within_limits=lambda q, m: all(-1+m <= v <= 1-m for v in q))
    node.speeds = [0.08]
    node.selected = 0
    node.focus = "arm"
    node.focus_pub = None
    node.previous_focus = node.selection_latched = False
    node.armed = True
    node.joy = NS(buttons=[0]*4 + [1] + [0]*5, axes=[0.0]*4 + [1.0, 0.0, 0.0])
    node.joy_at = node.feedback_at = time.monotonic()
    node.completed_at = 0
    node.positions = [0.0]
    node.pending = node.stop_requested = node.cancel_sent = False
    node.handle = None
    node.deadline = time.monotonic() + 2
    node.fault = ""
    node.get_logger = lambda: NS(error=lambda m: None, info=lambda m: None)
    node.sent = []
    def send(goal):
        node.sent.append(goal)
        return Future()
    node.client = NS(server_is_ready=lambda: True, send_goal_async=send)
    return node


class JoystickTests(unittest.TestCase):
    def test_bounded_goal_and_no_queue(self):
        node = harness()
        node._tick()
        node._tick()
        self.assertEqual(len(node.sent), 1)
        self.assertAlmostEqual(node.sent[0].trajectory.points[-1].positions[0], 0.016)

    def test_limit_clamps_target(self):
        node = harness()
        node.positions = [0.985]
        node._tick()
        self.assertAlmostEqual(node.sent[0].trajectory.points[-1].positions[0], 0.99)

    def test_requires_post_result_feedback(self):
        node = harness()
        node.completed_at = time.monotonic()
        node._tick()
        self.assertFalse(node.sent)
        node._state(NS(name=["joint"], position=[0.0]))
        node._tick()
        self.assertEqual(len(node.sent), 1)

    def test_stale_joy_requires_deadman_release(self):
        node = harness()
        node.joy_at -= 1
        node._tick()
        node._joy(node.joy)
        node._tick()
        self.assertFalse(node.sent)
        node.joy.buttons[4] = 0
        node._joy(node.joy)
        node.joy.buttons[4] = 1
        node._joy(node.joy)
        node._tick()
        self.assertEqual(len(node.sent), 1)

    def test_late_acceptance_is_cancelled_after_release(self):
        node = harness()
        node._tick()
        node.joy.buttons[4] = 0
        node._joy(node.joy)
        cancellations = []
        def cancel():
            cancellations.append(True)
            return Future()
        handle = NS(accepted=True, get_result_async=Future, cancel_goal_async=cancel)
        future = Future()
        future.set_result(handle)
        node._accepted(future)
        node._stop()
        self.assertEqual(len(cancellations), 1)

    def test_invalid_feedback_disarms(self):
        for message in (NS(name=[], position=[]), NS(name=["joint"], position=[math.nan]),
                        NS(name=["joint"], position=[1.1]),
                        NS(name=["joint", "joint"], position=[0.0, 0.0])):
            node = harness()
            node._state(message)
            node._tick()
            self.assertFalse(node.sent)
            self.assertFalse(node.armed)

    def test_timeout_latches_fault(self):
        node = harness()
        node.pending = True
        node.deadline = 0
        node._tick()
        self.assertTrue(node.fault)
        self.assertTrue(node.stop_requested)

    def test_focus_loss_and_stale_feedback_cancel(self):
        for invalidate in (lambda n: n._focus(NS(data="drive")),
                           lambda n: setattr(n, "feedback_at", 0)):
            node = harness()
            node.pending = True
            invalidate(node)
            node._tick()
            self.assertTrue(node.stop_requested)
            self.assertFalse(node.armed)

    def test_malformed_joy_stops(self):
        node = harness()
        node.pending = True
        node._joy(NS(buttons=[], axes=[]))
        self.assertTrue(node.stop_requested)
        self.assertIsNone(node.joy)


if __name__ == "__main__":
    unittest.main()
