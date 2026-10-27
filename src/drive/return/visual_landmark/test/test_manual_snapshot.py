"""Manual capture must produce a usable, cached reference without automatic capture."""
import json
import cv2
import numpy as np
from types import SimpleNamespace as NS
from sensor_msgs.msg import CameraInfo, CompressedImage
from rclpy.time import Time
from visual_landmark.localization import FeatureOptions
from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture


def test_manual_frame_saves_features_and_announces_only_after_cache(tmp_path, monkeypatch):
    import visual_landmark.visual_landmark_node as module
    events, holds, previews, saved_samples = [], [], [], []
    ci = CameraInfo()
    ci.header.frame_id = 'rear'
    ci.width, ci.height = 640, 480
    ci.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
    ci.d = [0.] * 5
    ci.distortion_model = 'plumb_bob'
    msg = CompressedImage()
    msg.header.frame_id = 'rear'
    msg.header.stamp = Time(seconds=9.385).to_msg()
    msg.data = cv2.imencode('.jpg', np.zeros((480, 640), np.uint8))[1].tobytes()
    points = np.zeros((30, 2), np.float32)
    descriptors = np.ones((30, 128), np.float32)
    monkeypatch.setattr(module, 'features', lambda image, options, stats: (points, descriptors))
    seq = NS(busy=True, _frame_after=9., accept_frame=lambda now: True,
             finish=lambda: setattr(seq, 'busy', False))
    node = NS(seq=seq, auto_enabled=False, image_stale_s=1., _camera_info=ci,
              feature_options=FeatureOptions(), stop_mode=True, odometer=NS(s=1.2),
              _last_image_arrival=None, _capture_quality=None,
              _now=lambda: 10., get_parameter=lambda name: NS(value=20),
              _select_image_odom=lambda stamp, now: ('historical sample', 9.4, 1.2),
              _save_landmark=lambda image: saved_samples.append(node._capture_odom_sample) or
                  dict(id='L000', order=0, dir=str(tmp_path), x=1., y=0., yaw=0.,
                       s_along_path=1.2, localization_ready=True, extrinsic_ok=True),
              hold_pub=NS(publish=lambda msg: holds.append(msg.data)),
              capture_preview_pub=NS(publish=lambda msg: previews.append(msg)),
              _event=lambda kind, **fields: events.append((kind, fields)),
              get_logger=lambda: NS(info=lambda text: None, warn=lambda text: None))
    node._publish_captured_snapshot = lambda image, pts, header, lid: Capture._publish_captured_snapshot(node, image, pts, header, lid)
    Capture._on_image(node, msg)
    assert saved_samples == [('historical sample', 9.4, 1.2)]
    assert not seq.busy and holds == [False]
    assert [kind for kind, _ in events] == ['captured']
    assert previews and previews[0].data
    with np.load(tmp_path / 'features.npz', allow_pickle=False) as cache:
        assert cache['points'].shape == (30, 2)
        assert cache['descriptors'].shape == (30, 128)
        assert str(cache['feature_config'].item()) == FeatureOptions().signature()
    assert node._capture_odom_sample is None


def test_manual_frame_without_tf_releases_hold_and_never_announces_reference(tmp_path, monkeypatch):
    import visual_landmark.visual_landmark_node as module
    monkeypatch.setattr(module, 'features', lambda image, options, stats:
                        (np.zeros((30, 2), np.float32), np.zeros((30, 128), np.float32)))
    ci = CameraInfo()
    ci.header.frame_id = 'rear'
    ci.width, ci.height = 640, 480
    ci.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
    ci.d = [0.] * 5
    ci.distortion_model = 'plumb_bob'
    msg = CompressedImage()
    msg.header.frame_id = 'rear'
    msg.header.stamp = Time(seconds=9.385).to_msg()
    msg.data = cv2.imencode('.jpg', np.zeros((480, 640), np.uint8))[1].tobytes()
    events, holds = [], []
    seq = NS(busy=True, _frame_after=9., accept_frame=lambda now: True,
             finish=lambda: setattr(seq, 'busy', False))
    node = NS(seq=seq, auto_enabled=False, image_stale_s=1., _camera_info=ci,
              feature_options=FeatureOptions(), stop_mode=True, _last_image_arrival=None,
              _now=lambda: 10., get_parameter=lambda name: NS(value=20),
              _select_image_odom=lambda stamp, now: ('historical sample', 9.4, 1.2),
              _save_landmark=lambda image: dict(id='L000', dir=str(tmp_path), localization_ready=False, extrinsic_ok=False),
              hold_pub=NS(publish=lambda msg: holds.append(msg.data)),
              _event=lambda kind, **fields: events.append((kind, fields)),
              get_logger=lambda: NS(warn=lambda text: None))
    node._fail = lambda reason, hold_started: Capture._fail(node, reason, hold_started)
    Capture._on_image(node, msg)
    assert not seq.busy and holds == [False]
    assert [kind for kind, _ in events] == ['failed']
    assert not (tmp_path / 'features.npz').exists()


def test_captured_event_accepts_numpy_boolean_from_geometry():
    published = []
    node = NS(event_pub=NS(publish=lambda msg: published.append(msg.data)))
    Capture._event(node, 'captured', id='L000', localization_ready=np.bool_(True),
                   x=np.float32(1.25))
    assert json.loads(published[0]) == {
        'event': 'captured', 'id': 'L000', 'localization_ready': True, 'x': 1.25,
    }
