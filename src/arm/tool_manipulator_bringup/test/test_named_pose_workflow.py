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
    def settle():
        node.measured = node.ready_goal.copy()
        node.status['velocities_rad_s'] = [0.]*6
        for stamp in (10.2, 10.8):
            node.status_time = node.state_time = stamp
            node.drive_ready(stamp)
    node.ready_snapshot = node.measured.copy()
    node.home_waypoints = [node.ready_goal.tolist()]
    finish_leg(node, 'ready-leg')
    assert node.ready_state == 'home_settle'
    settle()
    assert not node.returning_home and not node.auto_entry_armed
    assert node.status['mode'] == 'hold'


def test_home_preparation_holds_current_base_instead_of_rotating_to_ready():
    node = workflow()
    node.measured[0] = 3.1
    node.request_named_pose(True, 10.)
    node.status.update(mode='hold', transitioning=False)
    node.drive_ready(10.1)
    assert node.home_leg == 'planned'
    assert node.ready_goal[0] == 0.
    np.testing.assert_array_equal(node.measured, [3.1, 0., 0., 0., 0., 0.])


def test_home_rotation_cancels_when_shoulder_droops():
    node = workflow()
    node.request_named_pose(True, 10.)
    node.home_leg = 'rotate'
    node.ready_state = 'executing'
    node.measured[1] = -.011
    node.cancel.reset_mock()
    node.drive_ready(10.1)
    node.cancel.call_async.assert_called_once()
    assert node.ready_state == 'failed'
    assert not node.auto_entry_armed


def test_home_never_rotates_without_stopped_feedback():
    node = workflow()
    node.request_named_pose(True, 10.)
    finish_leg(node, 'prepare')
    node.status['velocities_rad_s'] = [0., -.02, 0., 0., 0., 0.]
    for stamp in (10.2, 11., 16.):
        node.status_time = node.state_time = stamp
        node.drive_ready(stamp)
    assert node.home_leg == 'prepare'
    assert node.ready_state == 'failed'
    assert 'did not settle' in node.reason


def test_home_requests_moveit_plan_and_rejects_failed_result():
    node = workflow()
    node.request_named_pose(True, 10.)
    node.status.update(mode='hold', transitioning=False)
    node.drive_ready(10.05)
    node.full_state = dict(zip(('base_joint', 'shoulder_joint', 'elbow_joint',
                               'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint'), node.measured))
    node.check = Mock()
    node.check.service_is_ready.return_value = True
    node.home_planner = Mock()
    node.home_planner.service_is_ready.return_value = True
    node.drive_ready(10.1)
    assert node.ready_state == 'planning_home'
    request = node.home_planner.call_async.call_args.args[0].motion_plan_request
    assert request.group_name == 'arm'
    assert len(request.goal_constraints[0].joint_constraints) == 6
    node.ready_future.done.return_value = True
    node.ready_future.result.return_value = SimpleNamespace(
        motion_plan_response=SimpleNamespace(error_code=SimpleNamespace(val=-1)))
    node.drive_ready(10.2)
    assert node.ready_state == 'failed'
    assert not node.auto_entry_armed


def test_planned_home_cancels_off_path_motion():
    node = workflow()
    node.request_named_pose(True, 10.)
    node.home_leg, node.ready_state = 'planned', 'executing'
    node.ready_snapshot = np.zeros(6)
    node.home_waypoints = [[1., 0., 0., 0., 0., 0.]]
    node.measured = np.array([.5, -.02, 0., 0., 0., 0.])
    node.cancel.reset_mock()
    node.drive_ready(10.1)
    node.cancel.call_async.assert_called_once()
    assert node.ready_state == 'failed'


def test_missing_home_and_fault_do_not_command_motion():
    node = workflow()
    node.status.pop('startup_goal_rad')
    assert not node.request_named_pose(True, 10.)[0]
    node.status['mode'] = 'fault'
    assert not node.request_named_pose(False, 10.)[0]
    node.cancel.call_async.assert_not_called()


def test_r2_current_pose_entry_checks_hold_without_requesting_motion():
    node = workflow()
    node.status.update(mode='hold', requested='hold', transitioning=False,
                       entry_alignment_complete=True)
    node.ready_state = 'idle'
    node.auto_entry_armed = False
    accepted, reason = node.request_current_tcp_entry(10.)
    assert accepted and 'measured pose' in reason
    assert node.ready_state == 'await_entry' and node.auto_entry_armed
    assert node.entry_deadline == 13.
    node.cancel.call_async.assert_not_called()
    np.testing.assert_array_equal(node.measured, np.zeros(6))

    node.status['mode'] = 'fault'
    accepted, reason = node.request_current_tcp_entry(10.1)
    assert not accepted and 'manager fault' in reason
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
