"""Ownership, cancellation and watchdog tests without ROS or hardware."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from arm_mode_core import MANUAL_EE_MAX_VELOCITY_RAD_S, ModeCore


def ready(mode='manual'):
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.focus, c.ready = 'arm', True
    c.feedback([0.]*6, [0.]*6, 0.)
    c.request(mode, 0.)
    for i in range(1, 6):
        c.feedback([0.]*6, [0.]*6, i*.02)
        c.tick(i*.02, .02)
    assert c.mode == mode and not c.transitioning
    return c


def test_only_owner_and_current_epoch_can_command():
    c = ready()
    assert not c.command('tcp_ik', c.epoch, [.05]*6, 0., .1)
    assert not c.command('manual', 'old', [.05]*6, 0., .1)
    assert c.command('manual', c.epoch, [.05]*6, .1, .1)
    assert max(c.tick(.12, .02)) <= .12*.02


def test_manual_ee_accepts_decimal_400_but_rejects_higher_speed():
    c = ready()
    assert c.command('manual', c.epoch, [0.]*6, MANUAL_EE_MAX_VELOCITY_RAD_S, .1)
    assert c.ee == pytest.approx(MANUAL_EE_MAX_VELOCITY_RAD_S)
    assert c.command('manual', c.epoch, [0.]*6, -MANUAL_EE_MAX_VELOCITY_RAD_S, .12)
    assert c.ee == pytest.approx(-MANUAL_EE_MAX_VELOCITY_RAD_S)
    with pytest.raises(ValueError, match='limits exceeded'):
        c.command('manual', c.epoch, [0.]*6, 401 * 0.0239691227, .14)


def test_tcp_ee_keeps_012_speed_limit():
    c = ready('tcp_ik')
    assert c.command('tcp_ik', c.epoch, [0.]*6, .12, .1)
    with pytest.raises(ValueError, match='limits exceeded'):
        c.command('tcp_ik', c.epoch, [0.]*6, .121, .12)


def test_managed_ee_reaches_feedback_goal_then_holds_arm():
    c = ready('tool_auto')
    c.ee_feedback(0., 0., .1)
    assert not c.close_ee('old', 'close', .2, .1)
    assert c.close_ee(c.epoch, 'close', .2, .1)
    position = 0.
    for step in range(1, 1001):
        now = .1+step*.02
        position += c.ee*.02
        c.ee_feedback(position, c.ee, now)
        c.feedback([0.]*6, [0.]*6, now)
        assert c.tick(now, .02) == [0.]*6
        assert abs(c.ee) <= .12
        if c.last_execution_result is not None:
            break
    assert c.last_execution_result == 'succeeded'
    assert abs(position-.2) <= .01
    assert c.mode == 'hold' and c.ee == 0. and c.ee_goal is None


@pytest.mark.parametrize('failure', ['stale', 'stall', 'arm_drift', 'cancel'])
def test_managed_ee_failure_stops_without_success(failure):
    c = ready('tool_auto')
    c.ee_feedback(0., 0., .1)
    assert c.close_ee(c.epoch, 'close', .2, .1)
    for step in range(1, 201):
        now = .1+step*.02
        if failure != 'stale':
            c.ee_feedback(0., 0., now)
        arm = [0., .006 if failure == 'arm_drift' else 0., 0., 0., 0., 0.]
        c.feedback(arm, [0.]*6, now)
        if failure == 'cancel':
            assert c.cancel_path(c.epoch, 'close', now)
        c.tick(now, .02)
        if c.last_execution_result is not None:
            break
    assert c.last_execution_result == 'aborted'
    assert c.mode == 'hold' and c.ee == 0. and c.ee_goal is None


def test_managed_ee_requires_fresh_feedback_and_valid_goal():
    c = ready('tool_auto')
    with pytest.raises(ValueError, match='fresh ee'):
        c.close_ee(c.epoch, 'close', .2, .1)
    c.ee_feedback(0., 0., .1)
    for goal in (True, float('nan'), 3., -3.):
        with pytest.raises(ValueError, match='invalid ee goal'):
            c.close_ee(c.epoch, 'close', goal, .1)
    c.ee_feedback(float('nan'), 0., .1)
    with pytest.raises(ValueError, match='fresh ee'):
        c.close_ee(c.epoch, 'close', .2, .1)


@pytest.mark.parametrize('distance,duration', [(.004, 10.), (.04, 100.)])
def test_slow_tracking_path_does_not_report_motor_stall(distance, duration):
    c = ready('tool_auto')
    assert c.path(c.epoch, 'slow', [[distance, 0., 0., 0., 0., 0.]], .1,
                  durations=[duration])
    for step in range(1, int((duration+5.)/.02)):
        now = .1+step*.02
        c.feedback(c.target, [0.]*6, now)
        c.tick(now, .02)
        if c.last_execution_result is not None:
            break
    assert c.last_execution_result == 'succeeded', c.reason


def test_blocked_joint_still_aborts_even_while_other_joints_move():
    c = ready('tool_auto')
    assert c.path(c.epoch, 'blocked', [[.4, .5, 0., 0., 0., 0.]], .1)
    for step in range(1, 501):
        now = .1+step*.02
        measured = list(c.target)
        measured[0] = 0.
        c.feedback(measured, [0.]*6, now)
        c.tick(now, .02)
        if c.last_execution_result is not None:
            break
    assert c.last_execution_result == 'aborted'
    assert 'stalled: joint_0' in c.reason
    assert c.mode == 'hold'


def test_first_arm_focus_aligns_dxl_before_managed_mode_entry():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.entry_alignment_enabled = True
    c.entry_alignment_complete = False
    c.ready, c.focus = True, 'arm'
    measured = [.06, .4, -.3, -.5, -.04, .02]
    c.feedback(measured, [0.]*6, 1.)
    with pytest.raises(ValueError, match='DXL entry alignment'):
        c.request('tcp_ik', 1.)
    for step in range(1, 130):
        now = 1.+step*.02
        c.feedback(measured, [0.]*6, now)
        previous = list(c.target)
        target = c.tick(now, .02)
        if c.entry_alignment_started == now:
            assert c.entry_alignment_deadline-now == pytest.approx(15.+.06/.09)
        assert max(abs(target[i]-previous[i]) for i in (0, 4, 5)) <= .0018+1e-12
        assert [target[i] for i in (1, 2, 3)] == [previous[i] for i in (1, 2, 3)]
        measured = list(target)
        if c.entry_alignment_complete:
            break
    assert c.entry_alignment_complete
    assert [c.target[i] for i in (0, 4, 5)] == [0.]*3
    assert c.mode == 'hold'
    assert c.tick(now+.02, .02) == c.target  # no return to the pre-alignment goal


@pytest.mark.parametrize('base, completes', [(-.00766990393963507, True), (-.012, False)])
def test_dxl_entry_stopped_residual_is_bounded(base, completes):
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.entry_alignment_enabled = True
    c.entry_alignment_complete = False
    c.ready, c.focus = True, 'arm'
    measured = [base, .4, -.3, -.5, .0030679615755646594, -.001533980787678857]
    for step in range(1, 801):
        now = step*.02
        c.feedback(measured, [0.]*6, now)
        c.tick(now, .02)
        if c.entry_alignment_complete or c.mode == 'fault':
            break
    assert c.entry_alignment_complete == completes
    if completes:
        assert c.mode == 'hold'
        assert [c.target[i] for i in (0, 4, 5)] == [0.]*3
    else:
        assert c.mode == 'fault'
        assert 'alignment timed out' in c.reason


def test_reset_expired_alignment_starts_new_attempt_from_feedback():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.entry_alignment_enabled = True
    c.entry_alignment_complete = False
    c.ready, c.focus = True, 'arm'
    c.feedback([.03, 0., 0., 0., 0., 0.], [0.]*6, 1.)
    c.tick(1., .02)
    old_deadline = c.entry_alignment_deadline
    now = old_deadline + 1.
    c.feedback([.03, 0., 0., 0., 0., 0.], [0.]*6, now)
    c.tick(now, .02)
    assert c.mode == 'fault'
    c.reset(now)
    assert c.entry_alignment_deadline is None
    assert c.entry_stable_since is None
    for step in range(1, 101):
        stamp = now+step*.02
        c.feedback(list(c.target), [0.]*6, stamp)
        c.tick(stamp, .02)
        assert c.mode != 'fault'
        if c.entry_alignment_complete:
            break
    assert c.entry_alignment_complete


def test_managed_dxl_entry_aligns_beyond_old_start_limit():
    c = ModeCore([(-2., 2.)]*6, [.12]*6)
    c.entry_alignment_enabled = True
    c.entry_alignment_complete = False
    c.ready, c.focus = True, 'arm'
    c.feedback([.36, 0., 0., 0., 0., 0.], [0.]*6, 1.)
    assert c.tick(1.02, .02)[0] < .36
    assert c.mode == 'hold' and not c.entry_alignment_complete


def test_transition_blocks_commands_and_invalidates_pending_messages():
    c = ready()
    old = c.epoch
    c.request('tcp_ik', .1)
    assert c.mode == 'hold' and c.transitioning and c.epoch != old
    assert not c.command('manual', old, [.05]*6, 0., .1)


def test_cancel_discards_remaining_waypoints_and_late_paths():
    c = ready('tool_auto')
    old = c.epoch
    assert c.path(old, 'run1', [[.01]*6, [.2]*6], .1)
    c.request('hold', .12)
    assert c.queue == [] and c.execution_id is None
    assert not c.path(old, 'late', [[1.]*6], .12)
    assert not c.command('manual', c.epoch, [.05]*6, .1, .12)


def test_fault_latches_and_reset_requires_healthy_feedback():
    c = ready()
    c.fault('motor error')
    with pytest.raises(ValueError):
        c.request('manual', .1)
    c.ready = False
    with pytest.raises(ValueError):
        c.reset(.1)
    c.ready = True
    c.reset(.1)
    assert c.mode == 'hold'


def test_stale_feedback_blocks_output_and_zeroes_ee():
    c = ready()
    c.command('manual', c.epoch, [.05]*6, .1, .1)
    assert c.tick(1., .02) is None
    assert c.mode == 'fault' and c.ee == 0.


def test_lost_stream_goes_to_hold():
    c = ready()
    c.feedback([0.]*6, [0.]*6, .5)
    c.tick(.5, .02)
    assert c.mode == 'hold' and c.ee == 0.


def test_limits_and_feedback_lead_are_enforced():
    c = ready()
    for target in ([float('nan')]*6, [3.]*6, [.11]*6, [0.]):
        with pytest.raises(ValueError):
            c.command('manual', c.epoch, target, 0., .1)


def test_following_error_identifies_joint_and_values():
    names = ['base', 'shoulder', 'elbow', 'pitch', 'roll', 'yaw']
    c = ModeCore([(-2., 2.)]*6, [.12]*6, joint_names=names)
    c.ready = True
    c.focus = 'arm'
    c.feedback([0.]*6, [0.]*6, 0.)
    c.target = c.goal = [0., .16, 0., 0., 0., 0.]
    c.tick(.02, .02)
    assert c.mode == 'fault'
    assert 'shoulder' in c.reason
    assert 'target=0.160000' in c.reason
    assert 'measured=0.000000' in c.reason


def test_tool_path_command_never_runs_far_ahead_of_feedback():
    c = ready('tool_auto')
    assert c.path(c.epoch, 'loaded', [[.5]*6], .1, durations=[1.])
    for i in range(1, 100):
        now = .1+i*.02
        c.feedback([0.]*6, [0.]*6, now)
        c.tick(now, .02)
    assert all(target <= lead+1e-12
               for target, lead in zip(c.target, c.path_feedback_leads))
    assert c.mode == 'tool_auto'


def test_tool_path_following_error_aborts_to_reusable_hold_not_global_fault():
    c = ready('tool_auto')
    assert c.path(c.epoch, 'bad', [[.2]*6], .1)
    c.target = [.16]*6
    c.feedback([0.]*6, [0.]*6, .12)
    c.tick(.12, .02)
    assert c.mode == c.requested == 'hold'
    assert c.last_execution_id == 'bad'
    assert c.last_execution_result == 'aborted'
    assert c.target == c.goal == [0.]*6


def test_loaded_shoulder_can_use_tighter_path_limits():
    c = ModeCore([(-2., 2.)]*6, [.12]*6,
                 path_speeds=[.08, .04, .06, .06, .08, .08],
                 path_accelerations=[.08, .03, .06, .06, .08, .08],
                 path_jerks=[.3, .04, .15, .15, .3, .3],
                 path_feedback_leads=[.04, .015, .025, .025, .04, .04])
    assert c.path_speed_limits[1] == .04
    assert c.path_accelerations[1] == .03
    assert c.path_jerks[1] == .04
    assert c.path_feedback_leads[1] == .015


def test_outward_motion_near_soft_limit_faults_before_crossing_limit():
    c = ModeCore([(-2., 2.)]*6, [.12]*6,
                 joint_names=['base', 'shoulder', 'elbow', 'pitch', 'roll', 'yaw'])
    c.feedback([0., 1.96, 0., 0., 0., 0.], [0., .03, 0., 0., 0., 0.], 0.)
    assert c.mode == 'fault'
    assert 'shoulder' in c.reason
    assert 'moving outward near soft limit' in c.reason


def test_path_continues_through_intermediate_waypoint_without_stopping():
    c = ready('tool_auto')
    c.path(c.epoch, 'run1', [[.01]*6, [.02]*6], .1)
    for i in range(1, 100):
        now = .1+i*.02
        c.feedback([0.]*6, [.03]*6, now)
        c.tick(now, .02)
        if not c.queue:
            break
    assert c.queue == [] and c.goal == [.02]*6
    assert c.execution_id == 'run1'
    # The command crosses the intermediate sample with non-zero velocity.
    assert all(v > 0. for v in c.path_velocity)


def test_path_requires_measured_settlement_only_at_final_waypoint():
    c = ready('tool_auto')
    c.path(c.epoch, 'run1', [[.01]*6, [.02]*6], .1)
    for i in range(1, 80):
        now = .1+i*.02
        c.feedback([0.]*6, [.03]*6, now)
        c.tick(now, .02)
        if c.target == [.02]*6:
            break
    assert c.execution_id == 'run1'
    for i in range(1, 6):
        now = .3+i*.02
        c.feedback([.02]*6, [0.]*6, now)
        c.tick(now, .02)
    assert c.execution_id is None
    assert c.last_execution_id == 'run1'
    assert c.last_execution_result == 'succeeded'


def test_tool_path_command_acceleration_and_final_braking_are_bounded():
    c = ready('tool_auto')
    c.path_accelerations = [.2]*6
    c.path_jerks = [1.]*6
    c.path(c.epoch, 'smooth', [[.2]*6], .1, durations=[2.])
    velocities = []
    previous = c.target[0]
    for i in range(1, 201):
        now = .1+i*.02
        c.feedback([c.target[0]]*6, [c.path_velocity[0]]*6, now)
        current = c.tick(now, .02)[0]
        velocities.append((current-previous)/.02)
        previous = current
        if current == pytest.approx(.2):
            break
    accelerations = [(b-a)/.02 for a, b in zip(velocities, velocities[1:])]
    assert max(map(abs, accelerations)) <= .2+1e-9
    assert max(abs(b-a) for a, b in zip(accelerations, accelerations[1:])) <= 1.*.02+1e-8
    assert velocities[0] > 0.
    assert velocities[-1] < max(velocities)
    assert c.target[0] == pytest.approx(.2)


def test_explicit_intermediate_stop_waits_for_stopped_feedback():
    c = ready('tool_auto')
    assert c.path(c.epoch, 'staged', [[.01]*6, [.02]*6], .1,
                  durations=[1., 1.], stop_at_waypoints=[True, False])
    c.target = c.goal = [.01]*6
    c.path_velocity = [0.]*6
    c.path_acceleration = [0.]*6
    for i in range(1, 8):
        now = .1+i*.01
        c.feedback([.01]*6, [.03]*6, now)
        c.tick(now, .01)
    assert c.goal == [.01]*6
    for i in range(1, 6):
        now = .2+i*.01
        c.feedback([.01]*6, [0.]*6, now)
        c.tick(now, .01)
    assert c.goal == [.02]*6


def test_tcp_full_input_is_acceleration_bounded_without_packet_dropping():
    c = ready('tcp_ik')
    c.tcp_accelerations = [.25]*6
    velocities = []
    previous = c.target[0]
    for i in range(1, 60):
        now = .1+i*.02
        c.feedback(list(c.target), list(c.tcp_velocity), now)
        assert c.command('tcp_ik', c.epoch, [.08]*6, 0., now)
        current = c.tick(now, .02)[0]
        velocities.append((current-previous)/.02)
        previous = current
        if current == pytest.approx(.08):
            break
    assert velocities[0] == pytest.approx(.5*.25*.02)
    assert max(abs(b-a) for a, b in zip(velocities, velocities[1:])) <= .25*.02+1e-9
    assert velocities[-1] < max(velocities)
    assert c.target[0] == pytest.approx(.08)


def test_tcp_pause_immediately_discards_stream_velocity():
    c = ready('tcp_ik')
    c.tcp_velocity = [.1]*6
    c.ee = .08
    assert c.pause_tcp(c.epoch, .11)
    assert c.tcp_velocity == [0.]*6
    assert c.ee == 0.


def test_focus_and_controller_gates():
    c = ready()
    c.focus = 'drive'
    assert not c.command('manual', c.epoch, [0.]*6, 0., .1)
    with pytest.raises(ValueError):
        c.request('tool_auto', .1)


def recovery_core():
    c = ModeCore([(-3.503, 3.614), (-1.592, 1.518), (-1.497, 1.542),
                  (-1.592, 1.470), (-.75, .63), (-5.76, 6.5)], [.12]*6,
                 startup_recovery_enabled=True)
    measured = [ -.00766990394, 1.5666074407163109, 1.5433946628649078,
                -1.597848850305976, -.0889708857, -.0674951547]
    c.feedback(measured, [0.]*6, 0.)
    assert c.mode == 'hold'
    c.begin_startup(0.)
    c.ready = True
    return c, measured


def test_reported_hardware_pose_recovers_inward_without_joystick():
    c, initial = recovery_core()
    assert c.focus == 'drive' and c.startup_recovering and c.transitioning
    for n in range(1, 751):
        now = n*.02
        before = c.target[:]
        c.feedback(before, [0.]*6, now)
        target = c.tick(now, .02)
        assert target is not None
        for i in (0, 4, 5):
            assert target[i] == initial[i]
        for i in (1, 2, 3):
            assert abs(target[i]-before[i]) <= .03*.02+1e-12
            assert (target[i]-before[i])*(c.startup_goal[i]-before[i]) >= -1e-12
        if c.startup_complete:
            break
    assert c.startup_complete and not c.startup_recovering
    assert c.mode == 'hold' and not c.transitioning and c.ee == 0.
    import math
    assert c.target[1:4] == pytest.approx([
        1.518-math.radians(2), 1.542-math.radians(2), -1.592+math.radians(2)])
    # Startup allowance must not apply again during normal operation.
    c.feedback(initial, [0.]*6, now+.02)
    assert c.mode == 'fault'


@pytest.mark.parametrize('index,value', [(0, 3.7), (4, .7), (5, 6.6),
                                       (1, 1.7), (2, float('nan'))])
def test_recovery_rejects_dxl_excursions_large_rmd_errors_and_nan(index, value):
    c, initial = recovery_core()
    last_valid = c.position[:]
    initial[index] = value
    c.feedback(initial, [0.]*6, .02)
    assert c.mode == 'fault' and c.tick(.02, .02) == last_valid


def test_startup_blocks_producers_and_cancel_never_resumes():
    c, _ = recovery_core()
    c.focus = 'arm'
    with pytest.raises(ValueError, match='startup'):
        c.request('manual', 0.)
    assert not c.command('manual', c.epoch, [0.]*6, .1, 0.)
    assert not c.path(c.epoch, 'run', [[0.]*6], 0.)
    c.request('hold', 0.)
    assert c.mode == 'fault'
    assert c.tick(.02, .02) == c.position
    with pytest.raises(ValueError, match='restart'):
        c.reset(.02)


def test_startup_pauses_for_stale_feedback_and_times_out():
    c, initial = recovery_core()
    c.tick(.02, .02)
    frozen = c.target[:]
    assert c.tick(.6, .02) == frozen
    assert c.tick(15.1, .02) == c.position
    assert c.mode == 'fault' and 'timeout' in c.reason


def test_startup_slow_feedback_progress_is_not_an_absolute_15_second_fault():
    c, initial = recovery_core()
    measured = initial[:]
    for i in range(1, 1501):
        now = i*.02
        # Model a loaded joint that moves slowly but continuously inward.
        measured[1] = max(c.startup_goal[1], measured[1]-.00001)
        c.feedback(measured, [0.]*6, now)
        c.tick(now, .02)
        assert c.mode != 'fault'
    assert now == pytest.approx(30.)
    assert c.startup_recovering


def test_startup_fault_replaces_retained_command_with_measured_hold():
    c, initial = recovery_core()
    c.tick(.02, .02)
    c.target[1] -= .05
    c.feedback(initial, [0.]*6, .04)
    c.fault('test fault')
    assert c.goal == initial
    assert c.target == initial
    assert c.tick(.06, .02) == initial


def test_startup_stalls_at_feedback_lead_and_requires_measured_completion():
    c, initial = recovery_core()
    for i in range(1, 701):
        now = i*.02
        c.feedback(initial, [0.]*6, now)
        c.tick(now, .02)
    assert not c.startup_complete
    c.feedback(initial, [0.]*6, 15.1)
    c.tick(15.1, .02)
    assert c.mode == 'fault'
