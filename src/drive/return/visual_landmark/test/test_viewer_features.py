"""Exercise actual live-preview SIFT without ROS initialization or GUI."""
import cv2
import numpy as np
from visual_landmark.viewer import FeaturePreview


def test_sift_preview_draws_real_features_and_keeps_input_intact():
    rng = np.random.default_rng(321)
    gray = rng.integers(0, 256, (480, 640), dtype=np.uint8)
    frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    before = frame.copy()
    rendered, count = FeaturePreview(640, 120).render(frame)
    assert count > 20
    assert np.array_equal(frame, before)
    assert not np.array_equal(rendered, frame)


def test_blank_image_and_disabled_preview_do_not_invent_features():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    preview = FeaturePreview(320, 100)
    rendered, count = preview.render(frame)
    assert rendered.shape == (240, 320, 3)
    assert count == 0
    frame[100:300, 100:300] = 255
    rendered, count = preview.render(frame, enabled=False)
    assert count == 0


def test_frozen_frame_keeps_refreshing_stale_indicator_and_missing_front(monkeypatch):
    from types import SimpleNamespace
    import visual_landmark.viewer as viewer
    captured = []
    windows = []
    monkeypatch.setattr(cv2, 'imshow', lambda name, frame: windows.append((name, frame.shape)))
    monkeypatch.setattr(cv2, 'waitKey', lambda *_: -1)
    monkeypatch.setattr(cv2, 'putText', lambda image, text, *args: captured.append(text))
    node = SimpleNamespace(pending={}, frames={'Rear live': (np.zeros((100, 200, 3), np.uint8), 1.)},
                           image_stamps={'Rear live': 10.}, feature_counts={'Rear live': 3},
                           receive_counts={'Rear live': 1}, lines={},
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000)))
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: 1.5)
    assert viewer.Viewer.show(node)
    assert windows == [('VPR Dashboard', (864, 1280, 3))]
    assert any('Front live: WAITING' in text for text in captured)
    assert not any('STALE' in text for text in captured)
    captured.clear()
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: 5.)
    assert viewer.Viewer.show(node)
    assert any('STALE' in text and '4.0s' in text for text in captured)
