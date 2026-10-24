import json
import math
from types import SimpleNamespace as NS

import pytest
from return_navigation.recovery import LandmarkWatch, YawSearch, project_route
from return_navigation.se2 import Pose2D, normalize_angle


def path():
    return [Pose2D(0., 0.), Pose2D(10., 0.), Pose2D(10., 10.)]


def test_route_distance_uses_bend_and_progress_window():
    arc, lateral = project_route(path(), Pose2D(10., 5.))
    assert arc == pytest.approx(15.) and lateral == 0.
    assert project_route(path(), Pose2D(10., 5.), 0, 1)[0] == 10.


def test_watch_does_not_stop_between_landmarks_and_triggers_after_passage():
    watch = LandmarkWatch()
    watch.references['L1'] = Pose2D(10., 5.)
    assert watch.missed(path(), Pose2D(5., 0.), 0., 0, 1) is None
    assert watch.missed(path(), Pose2D(10., 4.), 10., 1, 2) is None
    assert watch.missed(path(), Pose2D(10., 7.1), 12., 1, 2) == 'L1'
    assert watch.missed(path(), Pose2D(10., 8.), 20., 1, 2) is None


def test_watch_wait_timeout_and_already_recognized_checkpoint():
    watch = LandmarkWatch()
    watch.references['L1'] = Pose2D(10., 5.)
    assert watch.missed(path(), Pose2D(10., 4.), 0., 1, 2) is None
    assert watch.missed(path(), Pose2D(10., 4.), 9., 1, 2) == 'L1'
    watch = LandmarkWatch()
    watch.references['L1'] = Pose2D(10., 5.)
    watch.seen.add('L1')
    assert watch.missed(path(), Pose2D(10., 9.), 10., 1, 2) is None


def test_no_references_no_automatic_search():
    assert LandmarkWatch().missed(path(), Pose2D(10., 10.), 99., 1, 2) is None


def test_search_feedback_sweeps_both_sides_and_returns_to_start():
    search = YawSearch(math.radians(179), 0.)
    now = 0.
    yaw = search.start_yaw
    visited = []
    for _ in range(1500):
        speed, phase = search.tick(yaw, 0., 0., now)
        assert abs(speed) <= .15
        if phase == 'observing':
            visited.append(round(math.degrees(normalize_angle(yaw-search.start_yaw))))
        if phase == 'not_found':
            break
        assert phase not in {'timeout', 'angle_limit'}
        yaw = normalize_angle(yaw+speed*.05)
        now += .05
    else:
        pytest.fail('sweep did not finish')
    assert max(visited) >= 28 and min(visited) <= -28
    assert abs(normalize_angle(yaw-search.start_yaw)) <= math.radians(2.)


def test_candidate_freezes_turn_then_resumes_after_unsuccessful_dwell():
    search = YawSearch(0., 0.)
    search.index = 1
    search.candidate(.2)
    assert search.tick(.2, 0., 0., 1.) == (0., 'observing')
    assert search.tick(.2, 0., 0., 4.1)[0] == 0.
    speed, phase = search.tick(.2, 0., 0., 4.2)
    assert speed > 0 and phase == 'turning'


def test_search_time_limit_stuck_robot_and_excessive_angle():
    search = YawSearch(0., 0.)
    assert search.tick(0., 0., 0., 61.) == (0., 'timeout')
    search.index = 1
    assert search.tick(math.radians(40.), 0., 0., 1.) == (0., 'angle_limit')
    search = YawSearch(0., 0.)
    assert search.tick(0., .1, 0., 1.) == (0., 'settling')
    assert search.hold_since is None


def test_search_rejects_invalid_configuration():
    with pytest.raises(ValueError):
        YawSearch(0., 0., angle=-1.)


# Node callbacks are exercised without opening DDS sockets.
pytest.importorskip('rclpy')
from rclpy.time import Time
from rclpy.clock import ClockType
from std_msgs.msg import String
from return_navigation.return_state_machine_node import ReturnStateMachineNode as Machine


def node_for_search():
    now = Time(seconds=100, clock_type=ClockType.ROS_TIME)
    commands, statuses = [], []
    parameters = dict(odom_timeout_s=.5, landmark_search_angle_deg=30.,
                      landmark_search_speed_radps=.15, landmark_search_dwell_s=3.,
                      landmark_search_timeout_s=60.)
    node = NS(_state='FOLLOW_RETURN_PATH', _emergency_stop_active=False,
              _origin_received_at=Time(seconds=90, clock_type=ClockType.ROS_TIME),
              _odom_pose=Pose2D(), _last_odom_time=now, _odom_vx=0., _odom_wz=0.,
              _last_visual_frame=now.nanoseconds, _last_visual_stamp=None,
              stop_settle_duration=.5,
              get_clock=lambda: NS(now=lambda: now),
              get_parameter=lambda name: NS(value=parameters[name]),
              cmd_vel_return_pub=NS(publish=commands.append), state_pub=NS(publish=lambda m: None),
              _control_status=lambda *args: statuses.append(args),
              _publish_search_request=lambda: None)
    node._go_to = lambda state: setattr(node, '_state', state)
    node._finish_search = lambda success, reason: statuses.append((success, reason))
    node._resume_still = lambda: True
    return node, commands, statuses


def test_missed_event_stops_and_starts_expected_landmark_search():
    node, commands, statuses = node_for_search()
    Machine._on_landmark_missed(node, String(data=json.dumps(dict(id='L1', stamp_ns=100_000_000_000))))
    assert node._state == 'RETURN_LANDMARK_SEARCH' and node._search_id == 'L1'
    assert commands[-1].linear.x == commands[-1].angular.z == 0.
    Machine._tick_landmark_search(node)
    assert commands[-1].angular.z == 0.  # first observe while stopped
    assert statuses[0][0] == 'landmark_search'


def test_manual_mode_cannot_start_automatic_sweep_from_late_missed_event():
    node, commands, _ = node_for_search()
    node._state = 'RETURN_MANUAL'
    Machine._on_landmark_missed(node, String(data=json.dumps(dict(id='L1', stamp_ns=100_000_000_000))))
    assert node._state == 'RETURN_MANUAL' and not commands


def test_search_stale_odometry_fails_closed():
    node, commands, statuses = node_for_search()
    node._search = YawSearch(0., 100.)
    node._last_odom_time = Time(seconds=90, clock_type=ClockType.ROS_TIME)
    Machine._tick_landmark_search(node)
    assert statuses[-1] == (False, 'search odometry stale')
    assert commands[-1].angular.z == 0.


def test_search_ignores_old_request_and_wrong_landmark_results():
    node, _, _ = node_for_search()
    node._state = 'RETURN_LANDMARK_SEARCH'
    node._search_id, node._search_request = 'L1', 'current'
    node._search = YawSearch(0., 100.)
    node._search_commit_after, node._search_match = 99_000_000_000, None
    for lid, token in [('L1', 'old'), ('L2', 'current')]:
        Machine._on_localization_event(node, String(data=json.dumps(dict(
            event='localized', id=lid, request=token, stamp_ns=100_000_000_000))))
    assert node._search_match is None
    Machine._on_localization_event(node, String(data=json.dumps(dict(
        event='localized', id='L1', request='current', stamp_ns=100_000_000_000))))
    assert node._search_match == 100_000_000_000


def test_recovery_waits_for_matching_correction_receipt_before_resuming():
    node, _, statuses = node_for_search()
    node._state = 'RETURN_LANDMARK_SEARCH'
    node._search = YawSearch(0., 98.)
    node._search.hold_since = 99.
    node._search_commit_after = 99_000_000_000
    node._search_match = 99_500_000_000
    node._last_visual_stamp = 99_000_000_000  # another correction is insufficient
    Machine._tick_landmark_search(node)
    assert not statuses
    node._last_visual_stamp = node._search_match
    Machine._tick_landmark_search(node)
    assert statuses[-1] == (True, 'landmark localized')


def test_search_camera_pipeline_loss_stops_without_rotating():
    node, commands, statuses = node_for_search()
    node._search = YawSearch(0., 95.)
    node._search.index = 1
    node._last_visual_frame = None
    Machine._tick_landmark_search(node)
    assert statuses[-1] == (False, 'RGB-D or camera TF unavailable during search')
    assert all(msg.linear.x == msg.angular.z == 0. for msg in commands)


def test_mux_permits_only_search_controller_command_in_search_state():
    from geometry_msgs.msg import Twist
    from drive_cmd_mux.drive_cmd_mux_node import DriveCmdMuxNode
    command = Twist()
    command.angular.z = .15
    output = []
    node = NS(_state='RETURN_LANDMARK_SEARCH', _return_time=object(), _return_cmd=command,
              _fresh=lambda timestamp: True, cmd_pub=NS(publish=output.append))
    DriveCmdMuxNode._publish_cmd(node)
    assert output[-1].linear.x == 0. and output[-1].angular.z == .15
