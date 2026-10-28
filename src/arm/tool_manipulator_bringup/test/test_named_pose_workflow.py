"""Named returns must stop first and verify TCP ready before startup home."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tcp_motion_workflow import TcpMotionWorkflow


def workflow():
    node = TcpMotionWorkflow()
    node.status = dict(ready=True, startup_complete=True, focus='arm', mode='tool_auto',
                       startup_goal_rad=[.1, .2, .3, -.4, .5, .6])
    node.ready_enabled = True
    node.status_time = node.state_time = 10.
    node.ready_checks = []
    node.default_ready_goal = np.zeros(6)
    node.cancel = Mock()
    node.cancel.service_is_ready.return_value = True
    node.cancel.call_async.return_value.done.return_value = True
    node.cancel.call_async.return_value.result.return_value = SimpleNamespace(success=True)
    node.discard = Mock()
    node.measured = np.zeros(6)
    return node


def finish_leg(node, execution_id):
    node.ready_state = 'executing'
    node.ready_execution_id = execution_id
    node.status.update(mode='hold', last_execution_id=execution_id,
                       last_execution_result='succeeded')
    node.drive_ready(10.1)


def test_home_stops_active_path_then_visits_ready_then_startup_home():
    node = workflow()
    home = np.array(node.status['startup_goal_rad'])
    home[[0, 4, 5]] = 0.
    assert node.request_named_pose(True, 10.)[0]
    node.cancel.call_async.assert_called_once()
    assert node.ready_state == 'stopping'
    np.testing.assert_array_equal(node.ready_goal, node.default_ready_goal)
    node.drive_ready(10.05)
    assert node.ready_state == 'stopping'  # must wait for confirmed HOLD
    node.status.update(mode='hold', transitioning=False)
    node.drive_ready(10.1)
    assert node.ready_state == 'idle'
    finish_leg(node, 'ready-leg')
    assert node.ready_state == 'idle'  # next leg gets fresh collision checks
    assert node.returning_home and node.home_leg == 'home'
    np.testing.assert_array_equal(node.ready_goal, home)
    finish_leg(node, 'home-leg')
    assert not node.returning_home and not node.auto_entry_armed
    assert node.status['mode'] == 'hold'


def test_missing_home_and_fault_do_not_command_motion():
    node = workflow()
    node.status.pop('startup_goal_rad')
    assert not node.request_named_pose(True, 10.)[0]
    node.status['mode'] = 'fault'
    assert not node.request_named_pose(False, 10.)[0]
    node.cancel.call_async.assert_not_called()


def test_aborted_ready_leg_never_starts_home():
    node = workflow()
    node.request_named_pose(True, 10.)
    node.ready_state, node.ready_execution_id = 'executing', 'failed-leg'
    node.status.update(mode='hold', last_execution_id='failed-leg', last_execution_result='aborted')
    node.drive_ready(10.1)
    assert node.ready_state == 'failed'
    assert not node.auto_entry_armed and not node.returning_home


def test_explicit_ready_removes_distance_gate_but_not_collision_gate():
    node = workflow()
    node.request_named_pose(False, 10.)
    node.status.update(mode='hold', transitioning=False)
    node.drive_ready(10.05)
    node.measured = np.ones(6)*1.2
    node.full_state = {}
    node.ready_step = .005
    node.ready_max_delta, node.home_max_delta = .35, 1.
    node.ready_timeout = 15.
    node.check = Mock()
    node.check.service_is_ready.return_value = True
    node.check.call_async.return_value.done.return_value = True
    node.check.call_async.return_value.result.return_value = SimpleNamespace(valid=False, contacts=[])
    node.tool_pub = Mock()
    node.drive_ready(10.1)
    assert node.ready_state == 'checking'
    node.drive_ready(10.15)
    assert node.ready_state == 'failed'
    assert 'collision/state invalid' in node.reason
    node.tool_pub.publish.assert_not_called()
