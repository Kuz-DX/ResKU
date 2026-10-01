"""Exercise the real action-wait methods without ROS services or motor devices."""
import ast
from concurrent.futures import Future
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest


def ready(value):
    future = Future()
    future.set_result(value)
    return future


source = Path(__file__).resolve().parents[1] / 'scripts/motion_executor.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MotionExecutor')
# Compile the actual production methods; omit only ROS-dependent initialization.
cls.body = [n for n in cls.body if not isinstance(n, ast.FunctionDef) or n.name != '__init__']
module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), cls], type_ignores=[])
namespace = dict(Node=object, math=math, time=time,
                 FollowJointTrajectory=NS(Goal=NS, Result=NS(SUCCESSFUL=0)),
                 GoalStatus=NS(STATUS_SUCCEEDED=4))
exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), namespace)
Executor = namespace['MotionExecutor']


class Handle:
    accepted = True

    def __init__(self, result):
        self.result = result
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return Future()  # Deliberately no cancellation ACK: must not claim stop.


def harness(acceptance):
    node = Executor()
    node.motion_fault = ''
    node.joint_names = ('joint',)
    node.config = {'motion': {'ik_limit_margin_rad': 0.01}}
    node.model = NS(within_limits=lambda q, margin: True)
    node.client = NS(wait_for_server=lambda **kw: True, send_goal_async=lambda goal: acceptance)
    node.get_logger = lambda: NS(error=lambda message: None)
    trajectory = NS(joint_names=['joint'], points=[NS(positions=[0.0])])
    return node, trajectory


class MotionActionTests(unittest.TestCase):
    def test_success_requires_action_and_controller_success(self):
        for code in (0, -1):
            handle = Handle(ready(NS(status=4, result=NS(error_code=code, error_string='test'))))
            node, trajectory = harness(ready(handle))
            if code == 0:
                self.assertEqual(node._send(trajectory, 1), 'controller trajectory succeeded')
            else:
                with self.assertRaises(RuntimeError):
                    node._send(trajectory, 1)
                self.assertTrue(node.motion_fault)

    def test_result_timeout_cancels_and_blocks_next_motion(self):
        handle = Handle(Future())
        node, trajectory = harness(ready(handle))
        with self.assertRaisesRegex(RuntimeError, 'physical stop unconfirmed'):
            node._send(trajectory, 0.001)
        self.assertEqual(handle.cancels, 1)
        with self.assertRaisesRegex(RuntimeError, 'blocked until restart'):
            node._send(trajectory, 1)

    def test_late_acceptance_is_cancelled(self):
        acceptance = Future()
        node, trajectory = harness(acceptance)
        with self.assertRaisesRegex(RuntimeError, 'acceptance unresolved'):
            node._send(trajectory, 0.001)
        handle = Handle(Future())
        acceptance.set_result(handle)
        self.assertEqual(handle.cancels, 1)

    def test_rejected_goal_does_not_wait_for_result(self):
        handle = Handle(Future())
        handle.accepted = False
        node, trajectory = harness(ready(handle))
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            node._send(trajectory, 1)
        self.assertEqual(handle.cancels, 0)


if __name__ == '__main__':
    unittest.main()
