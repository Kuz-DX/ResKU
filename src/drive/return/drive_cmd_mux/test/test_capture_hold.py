from drive_cmd_mux.capture_hold import CaptureHold


def test_inactive_by_default():
    assert not CaptureHold().active(0.0)


def test_press_holds_immediately_and_releases_on_done():
    h = CaptureHold(grace_s=1.0, lease_s=0.5, max_hold_s=6.0)
    h.press(10.0)
    assert h.active(10.0)
    for t in (10.2, 10.6, 11.0, 11.4):     # 촬영 노드가 계속 hold=True 갱신
        h.lease(t, True)
        assert h.active(t + 0.1)
    h.lease(11.5, False)                   # 촬영 끝
    assert not h.active(11.5)


def test_no_capture_node_releases_after_grace():
    h = CaptureHold(grace_s=1.0)
    h.press(0.0)
    assert h.active(0.9)
    assert not h.active(1.0)


def test_capture_node_dies_mid_capture_releases_after_lease():
    h = CaptureHold(grace_s=1.0, lease_s=0.5)
    h.press(0.0)
    h.lease(0.3, True)
    assert h.active(0.7)
    assert not h.active(0.81)


def test_max_hold_caps_even_with_lease():
    h = CaptureHold(lease_s=0.5, max_hold_s=6.0)
    h.press(0.0)
    t = 0.0
    while t < 5.9:
        h.lease(t, True)
        assert h.active(t)
        t += 0.1
    h.lease(6.0, True)
    assert not h.active(6.0)


def test_repress_during_hold_does_not_extend():
    h = CaptureHold(grace_s=1.0)
    h.press(0.0)
    h.press(0.8)
    assert not h.active(1.0)


def test_stray_lease_without_press_is_ignored():
    h = CaptureHold()
    h.lease(0.0, True)
    assert not h.active(0.1)


def test_press_detector_edges_and_rearm():
    from drive_cmd_mux.capture_hold import PressDetector
    d = PressDetector(rearm_s=0.5)
    assert d.update(0.0, True)            # False->True
    assert not d.update(0.1, True)        # 누른 채 반복
    assert not d.update(0.2, False)
    assert d.update(0.3, True)            # 정상적인 두 번째 누름
    # UI가 False를 빼먹고 True만 다시 보냄 -> 0.5 s 이상 간격이면 새 누름
    assert not d.update(0.6, True)
    assert d.update(1.2, True)
