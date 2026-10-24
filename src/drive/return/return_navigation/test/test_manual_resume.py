import json
from types import SimpleNamespace as NS

import pytest
pytest.importorskip('rclpy')
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from return_navigation.se2 import Pose2D
from return_navigation.rejoin import distance_to_remaining_segment
from return_navigation.return_state_machine_node import ReturnStateMachineNode as Machine
from return_navigation.return_path_follower_node import ReturnPathFollowerNode as Follower
from drive_cmd_mux.drive_cmd_mux_node import DriveCmdMuxNode as Mux


def fixture():
    clock = NS(value=Time(seconds=100, clock_type=ClockType.ROS_TIME))
    commands, states, statuses, requests = [], [], [], []
    path = [Pose2D(2., 0., 0.), Pose2D(0., 0., 0.)]
    node = NS(_state='FOLLOW_RETURN_PATH', _emergency_stop_active=False,
              _return_path=path, _resume_request=None, _resume_sent_at=None,
              _auto_follow_after=None, _stop_settle_since=None,
              _last_odom_time=clock.value, _odom_vx=0., _odom_wz=0.,
              stop_vx_threshold=.03, stop_wz_threshold=.03, stop_settle_duration=.5,
              _follow_cmd_time=clock.value,
              cmd_vel_return_pub=NS(publish=commands.append),
              state_pub=NS(publish=states.append), mode_status_pub=NS(publish=statuses.append),
              resume_request_pub=NS(publish=requests.append),
              get_clock=lambda: NS(now=lambda: clock.value),
              get_parameter=lambda name: NS(value={'odom_timeout_s': .5, 'resume_timeout_s': 5.}[name]),
              _elapsed_in_state=lambda: 0.)
    node._go_to = lambda state: setattr(node, '_state', state)
    node._control_status = lambda *args: Machine._control_status(node, *args)
    node._resume_still = lambda: Machine._resume_still(node)
    return node, clock, commands, statuses, requests


def test_manual_keeps_route_and_auto_requires_stopped_fresh_handshake():
    node, clock, commands, statuses, requests = fixture()
    path = node._return_path
    Machine._on_control_mode(node, String(data='MANUAL'))
    assert node._state == 'RETURN_MANUAL' and node._return_path is path
    assert commands[-1].linear.x == commands[-1].angular.z == 0.
    Machine._on_control_mode(node, String(data='AUTO'))
    assert node._state == 'RETURN_RESUME_STOP'
    Machine._tick_resume_stop(node)
    assert not requests
    clock.value = Time(seconds=100.6, clock_type=ClockType.ROS_TIME)
    node._last_odom_time = clock.value
    Machine._tick_resume_stop(node)
    assert len(requests) == 1
    token = requests[-1].data
    # A previous attempt cannot resume the robot.
    Machine._on_resume_result(node, String(data=json.dumps(dict(request='old', ready=True))))
    assert node._state == 'RETURN_RESUME_STOP'
    Machine._on_resume_result(node, String(data=json.dumps(dict(request=token, ready=True))))
    assert node._state == 'FOLLOW_RETURN_PATH' and node._return_path is path
    assert node._follow_cmd_time is None
    assert json.loads(statuses[-1].data)['status'] == 'auto'


def test_rejected_rejoin_holds_until_explicit_manual_selection():
    node, _, _, statuses, _ = fixture()
    node._state = 'RETURN_RESUME_STOP'
    node._resume_request = '123'
    Machine._on_resume_result(node, String(data=json.dumps(dict(request='123', ready=False, reason='far'))))
    assert node._state == 'RETURN_INTERVENTION'
    assert json.loads(statuses[-1].data)['status'] == 'manual_intervention_required'
    Machine._on_control_mode(node, String(data='MANUAL'))
    assert node._state == 'RETURN_MANUAL'


def test_manual_interrupts_pending_auto_handshake():
    node, _, _, _, _ = fixture()
    node._state = 'RETURN_RESUME_STOP'
    node._resume_request = '123'
    Machine._on_control_mode(node, String(data='MANUAL'))
    Machine._on_resume_result(node, String(data=json.dumps(dict(request='123', ready=True))))
    assert node._state == 'RETURN_MANUAL'


def test_stale_odometry_and_motion_cannot_resume():
    node, clock, _, _, requests = fixture()
    node._state = 'RETURN_RESUME_STOP'
    clock.value = Time(seconds=102, clock_type=ClockType.ROS_TIME)
    Machine._tick_resume_stop(node)
    assert not requests and node._stop_settle_since is None
    node._last_odom_time = clock.value
    node._odom_wz = .1
    Machine._tick_resume_stop(node)
    assert not requests


def test_rejoin_does_not_select_future_branch_at_crossing():
    path = [Pose2D(0, 0), Pose2D(5, 0), Pose2D(5, 5), Pose2D(0, 0)]
    current = Pose2D(2.5, 2.5)
    assert distance_to_remaining_segment(path, current, 0, 1) == pytest.approx(2.5)
    assert distance_to_remaining_segment(path, current, 2, 3) < 1e-10
    assert distance_to_remaining_segment(path, Pose2D(2, .5), 0, 1) == pytest.approx(.5)


@pytest.mark.parametrize('state', ['RETURN_MANUAL', 'RETURN_RESUME_STOP', 'RETURN_INTERVENTION'])
def test_follower_freezes_progress_and_outputs_zero_outside_auto(state):
    output = []
    node = NS(_mission_state=state, _v_cmd=.5, _w_cmd=.2, _progress_index=8,
              _publish=lambda v, w: output.append((v, w)))
    Follower._tick(node)
    assert output == [(0., 0.)] and node._progress_index == 8


@pytest.mark.parametrize('state,expected', [('RETURN_MANUAL', .2), ('RETURN_RESUME_STOP', 0.), ('RETURN_INTERVENTION', 0.)])
def test_mux_routes_manual_and_holds_intervention(state, expected):
    output = []
    manual = Twist()
    manual.linear.x = .2
    node = NS(_state=state, _manual_cmd=manual, _manual_time=1.,
              _capture_hold=NS(active=lambda now: False), _now_s=lambda: 1.,
              _was_holding=False, _fresh=lambda stamp: True, cmd_pub=NS(publish=output.append))
    Mux._publish_cmd(node)
    assert output[-1].linear.x == expected


@pytest.mark.parametrize('x,expected', [(1., True), (10., False)])
def test_follower_handshake_checks_remaining_route_without_republishing_path(x, expected):
    now = Time(seconds=100, clock_type=ClockType.ROS_TIME)
    results = []
    path = [Pose2D(0., 0.), Pose2D(2., 0.)]
    node = NS(_mission_state='RETURN_RESUME_STOP', _path=path, _origin=Pose2D(),
              _odom_pose=Pose2D(x, 0.), _last_odom_time=now, _visual_correction=None,
              _floor=0, _progress_index=0, _corner_ptr=0, odom_timeout_s=.5,
              _next_corner=lambda: None, get_clock=lambda: NS(now=lambda: now),
              get_parameter=lambda name: NS(value=1.), resume_pub=NS(publish=results.append))
    Follower._on_resume_request(node, String(data='123'))
    result = json.loads(results[-1].data)
    assert result['request'] == '123' and result['ready'] == expected
    assert node._path is path


def test_control_switch_requires_post_switch_command():
    outputs = []
    node = NS(_state='FOLLOW_RETURN_PATH', _manual_time=object(), _return_time=object(),
              cmd_pub=NS(publish=outputs.append))
    Mux._on_state(node, String(data='RETURN_MANUAL'))
    assert node._manual_time is None and node._return_time is None
    assert outputs[-1].linear.x == outputs[-1].angular.z == 0.
