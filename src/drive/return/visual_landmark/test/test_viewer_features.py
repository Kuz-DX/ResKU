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


def test_snapshot_is_held_without_stale_camera_warning_and_missing_current(monkeypatch):
    from types import SimpleNamespace
    import visual_landmark.viewer as viewer
    captured = []
    windows = []
    monkeypatch.setattr(cv2, 'imshow', lambda name, frame: windows.append((name, frame.shape)))
    monkeypatch.setattr(cv2, 'waitKey', lambda *_: -1)
    monkeypatch.setattr(cv2, 'putText', lambda image, text, *args: captured.append(text))
    node = SimpleNamespace(pending={}, frames={'Recorded snapshot': (np.zeros((100, 200, 3), np.uint8), 1.)},
                           image_stamps={'Recorded snapshot': 10.}, feature_counts={'Rear live': 3},
                           receive_counts={'Rear live': 1}, lines={},
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000)))
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: 1.5)
    assert viewer.Viewer.show(node)
    assert windows == [('VPR Dashboard', (864, 1280, 3))]
    assert any('Current snapshot: WAITING' in text for text in captured)
    assert not any('STALE' in text for text in captured)
    captured.clear()
    monkeypatch.setattr(viewer.time, 'monotonic', lambda: 5.)
    assert viewer.Viewer.show(node)
    assert not any('STALE' in text for text in captured)
    assert any('4.0s' in text and 'held until next result' in text for text in captured)


def test_return_never_falls_back_to_live_rear_and_explains_missing_matches(monkeypatch):
    from types import SimpleNamespace
    import visual_landmark.viewer as viewer
    texts = []
    monkeypatch.setattr(cv2, 'imshow', lambda *_: None)
    monkeypatch.setattr(cv2, 'waitKey', lambda *_: -1)
    monkeypatch.setattr(cv2, 'putText', lambda image, text, *args: texts.append(text))
    node = SimpleNamespace(pending={}, frames={'Rear live': (np.zeros((100,200,3), np.uint8), 1.)},
        mission_state='RETURN_MANUAL', image_stamps={}, feature_counts={}, receive_counts={}, lines={},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
    viewer.Viewer.show(node)
    assert any('WAITING FOR VPR CHECK / SELECTED REFERENCE' in text for text in texts)
    assert any('compare at each reverse-order checkpoint' in text for text in texts)
    assert not any(text.startswith('Rear live:') for text in texts)


def test_reference_snapshot_uses_cached_points_and_only_publishes_on_change(monkeypatch):
    from types import SimpleNamespace
    from visual_landmark.visual_localization_node import VisualLocalizationNode
    output = []
    node = SimpleNamespace(debug_enabled=True,
        reference_images={'L1': np.zeros((100,200), np.uint8)},
        landmarks={'L1': ((np.array([[60.,60.]]), None, None, None), None)},
        reference_pub=SimpleNamespace(publish=output.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time())))
    VisualLocalizationNode.publish_reference_snapshot(node, 'L1')
    VisualLocalizationNode.publish_reference_snapshot(node, 'L1')
    assert len(output) == 1
    image = cv2.imdecode(np.frombuffer(output[0].data, np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    assert np.any(image[57:64,57:64])  # Cached reference keypoint is visible.
