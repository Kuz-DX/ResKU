from visual_landmark.capture_logic import (
    IDLE, SETTLE, WAIT_FRAME, WAIT_STILL, CaptureSequencer, Odometer)


def run_still(seq, t0, t1, dt=0.05):
    t = t0
    while t <= t1 + 1e-9:
        seq.on_odom(t, 0.0, 0.0)
        assert seq.tick(t) is None
        t += dt
    return t


def test_happy_path_stop_settle_frame():
    seq = CaptureSequencer(still_time_s=0.5, settle_s=0.3, timeout_s=5.0)
    seq.start(0.0)
    # 아직 움직이는 중
    seq.on_odom(0.0, 0.3, 0.0)
    assert seq.tick(0.1) is None and seq.phase == WAIT_STILL
    t = run_still(seq, 0.2, 0.75)          # 0.55 s 정지 -> SETTLE
    assert seq.phase == SETTLE
    assert not seq.accept_frame(t)         # 정착 중 프레임은 안 씀
    run_still(seq, t, t + 0.35)
    assert seq.phase == WAIT_FRAME
    frame_ok_time = t + 0.4
    assert not seq.accept_frame(seq._frame_after - 0.01)  # 정착 끝나기 전 도착 프레임
    assert seq.accept_frame(frame_ok_time)
    seq.finish()
    assert seq.phase == IDLE and not seq.busy


def test_motion_during_settle_restarts_still_check():
    seq = CaptureSequencer(still_time_s=0.5, settle_s=0.3)
    seq.start(0.0)
    t = run_still(seq, 0.0, 0.6)
    assert seq.phase == SETTLE
    seq.on_odom(t, 0.0, 0.2)               # 다시 회전
    assert seq.phase == WAIT_STILL
    assert not seq.accept_frame(t + 1.0)


def test_timeout_when_robot_never_stops():
    seq = CaptureSequencer(timeout_s=2.0)
    seq.start(0.0)
    reason = None
    t = 0.0
    while reason is None and t < 3.0:
        seq.on_odom(t, 0.3, 0.0)
        reason = seq.tick(t)
        t += 0.1
    assert reason is not None and 'timeout' in reason
    assert seq.phase == IDLE


def test_idle_accepts_no_frames():
    seq = CaptureSequencer()
    assert not seq.accept_frame(100.0)
    assert seq.tick(100.0) is None


def test_odometer_accumulates_and_ignores_jumps():
    o = Odometer(max_step_m=0.5)
    for i in range(11):
        o.update(0.1 * i, 0.0)              # 1.0 m 직진
    assert abs(o.s - 1.0) < 1e-9
    o.update(10.0, 0.0)                     # odom 점프 -> 무시
    assert abs(o.s - 1.0) < 1e-9
    o.update(10.0, 0.3)
    assert abs(o.s - 1.3) < 1e-9
    o.reset()
    assert o.s == 0.0


def test_moving_mode_takes_first_frame_without_stopping():
    seq = CaptureSequencer()
    seq.start(10.0, wait_still=False)
    assert seq.phase == WAIT_FRAME
    seq.on_odom(10.05, 0.3, 0.1)          # 계속 주행 중이어도
    assert seq.phase == WAIT_FRAME and seq.tick(10.05) is None
    assert not seq.accept_frame(9.99)     # 누르기 전 프레임은 안 씀
    assert seq.accept_frame(10.06)


def test_press_detector_matches_mux_rule():
    from visual_landmark.capture_logic import PressDetector
    d = PressDetector(rearm_s=0.5)
    assert d.update(0.0, True)
    assert not d.update(0.1, True)
    assert d.update(0.7, True)
