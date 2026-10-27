"""Scheduling contracts; no hardware or ROS daemon required."""
import math
import pytest
from visual_landmark.adaptive import AutoKeyframes, CheckWatchdog, candidate_window, reference_arcs, fresh
from return_navigation.se2 import Pose2D


def test_keyframes_distance_heading_timeout_and_stationary():
    auto = AutoKeyframes()
    assert auto.reason(0, 0, 0) == 'initial'
    auto.accept(0, 0, 0)
    assert auto.reason(1, 1.24, 0) is None
    assert auto.reason(1, 1.25, 0) == 'distance'
    assert auto.reason(1, .1, math.radians(12)) == 'yaw'
    assert auto.reason(20, 0, 0) is None
    assert auto.reason(6, .5, 0) == 'timeout'
    auto.accept(7, .5, 0)
    assert auto.reason(8, .6, 0) is None
    auto.reset()
    assert auto.reason(8, .6, 0) == 'initial'


def drive(watch, metres):
    for i in range(round(metres*10)+1):
        watch.odom(i/10, 0)


def test_distance_success_and_failure_cooldown():
    w = CheckWatchdog()
    drive(w, 7)
    assert w.due(0) is None
    w.odom(7.5, 0); w.odom(8.1, 0)
    assert w.due(0) == 'distance'
    w.start('a', 0)
    assert w.due(1) is None
    assert not w.finish('old', 1, True)
    assert w.distance > 8
    assert w.finish('a', 1, False)
    assert w.distance > 8 and w.due(2) is None
    assert w.due(6) == 'distance'
    w.start('b', 6)
    assert w.finish('b', 7, True)
    assert w.distance == 0 and w.score == 0


def test_unavailable_node_times_out_without_resetting_distance():
    w = CheckWatchdog(); drive(w, 9)
    w.start('missing-node', 0)
    assert w.expire(4) is None
    assert w.expire(5) == 'missing-node'
    assert w.distance >= 9 and w.due(6) is None
    assert w.due(10) == 'distance'


def test_normal_rates_and_short_spike_do_not_trigger():
    w = CheckWatchdog()
    for i in range(10): w.risk(i*.1, .3, .3, True)
    w.risk(1., 1., 0., True)
    assert w.due(1) is None
    for i in range(11, 100): w.risk(i*.1, .3, .3, True)
    assert w.score == 0


def test_sustained_mismatch_triggers_and_success_resets():
    w = CheckWatchdog()
    for i in range(10): w.risk(i*.1, .9, 0., True)
    assert w.due(1) == 'slip' and w.distance == 0
    w.start('slip', 1); w.finish('slip', 2, True)
    assert w.score == 0 and w.distance == 0


@pytest.mark.parametrize('healthy', [False])
def test_unhealthy_or_gap_cannot_accumulate(healthy):
    w = CheckWatchdog()
    for i in range(30): w.risk(i*.1, 5., 0., healthy)
    assert w.due(3) is None
    w.risk(10, 5., 0., True)
    assert w.score == 0


def test_100_references_bounded_and_expansion_preserves_anchor():
    arcs = {f'K{i:03}': float(100-i) for i in range(100)}
    primary = candidate_window(arcs, 20, last_arc=19)
    assert len(primary) == 3
    assert primary[0][0] == 'K080'  # reverse of outbound order
    expanded = candidate_window(arcs, 20, last_arc=19, expansion=2)
    assert len(expanded) == 5 and set(primary).issubset(expanded)
    assert all(s >= 17 for _, s in expanded)
    assert len(candidate_window(arcs, 20, expansion=100)) <= 7


def test_self_crossing_reference_uses_outbound_arc_and_return_order():
    path = [Pose2D(-2, 0), Pose2D(2, 0), Pose2D(2, 2),
            Pose2D(0, 2), Pose2D(0, -2)]
    # Return arc 2 and 10 have the exact same XY (0,0).
    refs = dict(late=dict(order=9, s_along_path=10., mission_pose=dict(x=0., y=0.)),
                early=dict(order=1, s_along_path=2., mission_pose=dict(x=0., y=0.)))
    arcs = reference_arcs(path, refs)
    assert arcs['late'] == pytest.approx(2.)
    assert arcs['early'] == pytest.approx(10.)
    assert candidate_window(arcs, 10., last_arc=9.) == [('early', 10.)]


@pytest.mark.parametrize('stamp', [None, 9., 11., float('nan')])
def test_freshness_rejects_missing_stale_future_nonfinite(stamp):
    assert not fresh(10., stamp, .5)


def test_late_success_is_not_a_valid_anchor():
    w=CheckWatchdog(); drive(w,9); w.start('late',0)
    assert not w.finish('late',5.1,True)
    assert w.distance>=9
    assert w.expire(5.1)=='late'
