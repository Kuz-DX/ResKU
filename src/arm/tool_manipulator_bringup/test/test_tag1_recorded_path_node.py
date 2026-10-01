"""Use real ROS message classes but no node, executor, device, or network."""
from concurrent.futures import Future
from pathlib import Path
import sys
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from test_tag1_recorded_path import fixture
try:
    import tag1_recorded_path_node as module
except ModuleNotFoundError as exc:
    raise unittest.SkipTest(f'ROS Python environment required: {exc}')


def ready(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self, result=None):
        self.result = result if result is not None else Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        return ready(None)


class Client:
    def __init__(self, node):
        self.node, self.sent, self.waits = node, [], 0
        self.status = module.GoalStatus.STATUS_SUCCEEDED

    def wait_for_server(self, timeout_sec):
        self.waits += 1
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        self.node.positions = dict(zip(module.ARM_JOINTS, goal.trajectory.points[-1].positions))
        return ready(Handle(ready(SimpleNamespace(status=self.status, result=SimpleNamespace(error_code=0)))))


def harness():
    node = SimpleNamespace(lock=threading.RLock(), cancelled=threading.Event(),
                           busy=True, unresolved_goal=False, handle=None, fault='',
                           positions={n: 0.0 for n in module.ARM_JOINTS},
                           velocities={n: 0.0 for n in module.ARM_JOINTS},
                           stamp=time.monotonic(), active_limits=None, states=[], state='preflight')
    parameters = dict(recorded_path_file='record', hardware_config_file='hardware', tools_config_file='tools',
                      action_wait_timeout_sec=0.01, joint_state_timeout_sec=1.0,
                      max_start_velocity_rad_s=0.02, start_tolerance_rad=0.05,
                      max_joint_speed_rad_s=0.15, min_segment_duration_sec=1.0, goal_tolerance_rad=0.01)
    node.value = parameters.__getitem__
    def publish(state):
        node.state = state
        node.states.append(state)
    node.publish = publish
    for name in ('run_sequence', 'check_current', 'goal_response', 'goal_terminal',
                 'wait_future', 'hold', 'reset', 'watchdog'):
        setattr(node, name, MethodType(getattr(module.RecordedPathNode, name), node))
    node.client = Client(node)
    return node


class RecordedNodeTests(unittest.TestCase):
    def setUp(self):
        record, hardware, tools = fixture()
        self.data = {'record': record, 'hardware': hardware, 'tools': tools}
        self.node = harness()

    def run_node(self):
        with patch.object(module, 'read_yaml', side_effect=self.data.__getitem__):
            self.node.run_sequence()

    def test_one_six_axis_goal_only_after_preflight(self):
        self.run_node()
        self.assertEqual(len(self.node.client.sent), 1)
        goal = self.node.client.sent[0]
        self.assertEqual(goal.trajectory.joint_names, list(module.ARM_JOINTS))
        self.assertEqual(len(goal.trajectory.points), 6)
        times = [p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in goal.trajectory.points]
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
        self.assertEqual(self.node.state, 'complete:trajectory_only')

    def test_invalid_last_point_sends_nothing(self):
        self.data['record']['waypoints'][-1][0] = 3.0
        self.run_node()
        self.assertEqual(self.node.client.sent, [])
        self.assertEqual(self.node.client.waits, 0)
        self.assertTrue(self.node.state.startswith('hold:'))

    def test_unknown_stage_split_sends_nothing(self):
        self.data['record']['execution_sequence'] = []
        self.run_node()
        self.assertEqual(self.node.client.sent, [])
        self.assertIn('execution_sequence is empty', self.node.state)

    def test_stale_feedback_and_wrong_start_send_nothing(self):
        self.node.stamp = 0.0
        self.run_node()
        self.assertEqual(self.node.client.sent, [])
        self.node = harness()
        self.node.positions['base_joint'] = 0.1
        self.run_node()
        self.assertEqual(self.node.client.sent, [])
        self.assertIn('start pose', self.node.state)

    def test_cancelled_before_send(self):
        self.node.cancelled.set()
        self.run_node()
        self.assertEqual(self.node.client.sent, [])

    def test_aborted_action_is_not_success(self):
        self.node.client.status = module.GoalStatus.STATUS_ABORTED
        self.run_node()
        self.assertTrue(self.node.state.startswith('hold:'))

    def test_late_acceptance_is_cancelled_and_reset_waits_for_terminal(self):
        self.node.busy = False
        self.node.unresolved_goal = True
        self.node.hold('cancel during send')
        handle = Handle()
        self.node.goal_response(ready(handle))
        self.assertEqual(handle.cancels, 1)
        response = self.node.reset(None, SimpleNamespace())
        self.assertFalse(response.success)
        handle.result.set_result(SimpleNamespace(status=module.GoalStatus.STATUS_CANCELED))
        self.assertFalse(self.node.unresolved_goal)
        self.assertTrue(self.node.reset(None, SimpleNamespace()).success)

    def test_watchdog_cancels_on_stale_feedback(self):
        _, limits, _ = module.compile_sequence(*fixture())
        self.node.active_limits = limits
        self.node.state = 'executing_recorded_path'
        self.node.stamp = 0.0
        self.node.handle = Handle()
        self.node.watchdog()
        self.assertEqual(self.node.handle.cancels, 1)
        self.assertTrue(self.node.cancelled.is_set())


if __name__ == '__main__':
    unittest.main()
