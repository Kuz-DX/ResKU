"""Regressions for continued manual recording after return becomes available."""
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml
pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from drive_cmd_mux.capture_hold import CaptureHold, PressDetector
from drive_cmd_mux.drive_cmd_mux_node import DriveCmdMuxNode
from return_navigation.se2 import Pose2D
from visual_landmark.capture_logic import CaptureSequencer, Odometer
from visual_landmark.visual_landmark_node import VisualLandmarkNode
from visual_landmark.visual_localization_node import VisualLocalizationNode


STATES = ['MANUAL_RECORDING', 'WAIT_RETURN_COMMAND', 'IDLE',
          'STOP_BEFORE_TURN', 'TURN_180', 'FOLLOW_RETURN_PATH', 'FINISHED']
RECORDING = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}


def logger():
    return SimpleNamespace(info=lambda *args: None, warn=lambda *args: None)


@pytest.mark.parametrize('state', STATES)
def test_capture_trigger_and_mux_hold_agree(state):
    failures = []
    capture = SimpleNamespace(
        _state=state, _now=lambda: 10.0, _press=PressDetector(), seq=CaptureSequencer(),
        _origin=Pose2D(), _odom=object(), _last_image_arrival=9.9, image_stale_s=1.,
        rear_image_topic='/rear', stop_mode=True, get_logger=logger,
        hold_pub=SimpleNamespace(publish=lambda msg: None),
        _fail=lambda reason, **kwargs: failures.append(reason))
    VisualLandmarkNode._on_trigger(capture, Bool(data=True))
    mux = SimpleNamespace(_state=state, _now_s=lambda: 10.0,
                          _landmark_press=PressDetector(), _landmark_stop=True,
                          _capture_hold=CaptureHold(), get_logger=logger)
    DriveCmdMuxNode._on_landmark_trigger(mux, Bool(data=True))
    assert capture.seq.busy == (state in RECORDING)
    assert mux._capture_hold.active(10.0) == (state in RECORDING)
    assert bool(failures) == (state not in RECORDING)


def test_ready_state_preserves_mission_numbering_and_accumulates_distance():
    node = SimpleNamespace(_state='MANUAL_RECORDING', _mission_dir='/existing',
                           _next_order=3, odometer=Odometer(), seq=CaptureSequencer())
    VisualLandmarkNode._on_state(node, String(data='WAIT_RETURN_COMMAND'))
    assert node._mission_dir == '/existing' and node._next_order == 3
    node._origin = Pose2D()
    node._now = lambda: 10.0
    node.odometer.update(0., 0.)
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0
    odom.pose.pose.position.x = 0.4
    VisualLandmarkNode._on_odom(node, odom)
    assert node.odometer.s == pytest.approx(0.4)


def test_capture_cancelled_when_return_starts():
    failures = []
    seq = CaptureSequencer()
    seq.start(10.)
    node = SimpleNamespace(_state='WAIT_RETURN_COMMAND', seq=seq,
                           _fail=lambda reason, **kwargs: failures.append(reason))
    VisualLandmarkNode._on_state(node, String(data='STOP_BEFORE_TURN'))
    assert failures == ['recording ended during capture']


def test_localization_loads_reference_in_ready_state(tmp_path):
    image = np.zeros((480, 640), np.uint8)
    assert cv2.imwrite(str(tmp_path/'rear.jpg'), image)
    meta = dict(id='L001', image_file='rear.jpg', camera_frame='rear_optical',
                mission_camera_transform=np.eye(4).tolist(), camera_info=dict(
                    frame_id='rear_optical', width=640, height=480,
                    distortion_model='plumb_bob',
                    k=[500., 0., 320., 0., 500., 240., 0., 0., 1.], d=[0.]*5))
    (tmp_path/'meta.yaml').write_text(yaml.safe_dump(meta))
    events = []
    node = SimpleNamespace(state='WAIT_RETURN_COMMAND', landmarks={},
                           event=lambda kind, **kwargs: events.append(kind))
    msg = String(data=json.dumps(dict(event='captured', dir=str(tmp_path))))
    VisualLocalizationNode.on_landmark(node, msg)
    assert 'L001' in node.landmarks
    assert events == ['reference_loaded']
    node.pending = object()
    node.state = 'MANUAL_RECORDING'
    node.correction = object()
    VisualLocalizationNode.on_state(node, String(data='WAIT_RETURN_COMMAND'))
    assert 'L001' in node.landmarks
    # A new outbound mission must clear the previous reference set.
    node.state = 'IDLE'
    VisualLocalizationNode.on_state(node, String(data='MANUAL_RECORDING'))
    assert not node.landmarks and node.correction is None
