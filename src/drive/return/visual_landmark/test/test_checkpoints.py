"""Checkpoint arrival, reverse ordering and real node callback wiring without motors."""
import json
import math
from types import SimpleNamespace as NS
import pytest
import numpy as np
from std_msgs.msg import String
from visual_landmark.checkpoints import CheckpointSequence
from visual_landmark.adaptive import AutoKeyframes, reference_arcs
from visual_landmark.visual_check_scheduler_node import VisualCheckScheduler as Scheduler
from visual_landmark.visual_localization_node import VisualLocalizationNode as Localizer
from return_navigation.se2 import Pose2D
from test_adaptive_nodes import scheduler
from test_cross_camera import live_fixture


def points():
    return [dict(id=f'L{i}', order=i, arc=(2-i)*1.8) for i in range(3)]


def test_capture_distance_only_no_yaw_or_timeout_duplicates():
    auto = AutoKeyframes(time_only=False)
    assert auto.distance == 1.0
    auto.accept(0, 0, 0)
    assert auto.reason(100, .99, math.pi) is None
    assert auto.reason(100, 1.0, math.pi) == 'distance'
    auto.accept(100, 1.8, math.pi)
    assert auto.reason(200, 1.8, 0) is None


def test_reverse_once_and_skip_instead_of_match_wrong_place():
    sequence = CheckpointSequence()
    sequence.load(points())
    point, missed = sequence.due(0)
    assert point['id'] == 'L2' and not missed
    sequence.active = point['id']
    assert sequence.due(1.8) == (None, [])
    sequence.finish()
    assert sequence.due(1.0) == (None, [])
    assert sequence.due(1.5)[0]['id'] == 'L1'
    # Passing L1 with no check must never compare it at L0.
    point, missed = sequence.due(3.6)
    assert point['id'] == 'L0' and missed == ['L1']
    sequence.active = 'L0'
    sequence.finish()
    assert sequence.due(3.6) == (None, [])
    sequence.load(points())  # Re-published route must not repeat finished checks.
    assert sequence.due(3.6) == (None, [])


def test_project_route_checkpoints_reversed_order():
    route = [Pose2D(3.6,0,math.pi), Pose2D(0,0,math.pi)]
    metadata = {f'L{i}':dict(order=i, s_along_path=i*1.8, mission_pose=dict(x=i*1.8,y=0)) for i in range(3)}
    arcs = reference_arcs(route, metadata)
    assert list(arcs) == ['L2', 'L1', 'L0']
    assert list(arcs.values()) == pytest.approx([0,1.8,3.6])


def setup_scheduler():
    node, values, requests, events = scheduler()
    node.checkpoint_mode = True
    node.checkpoints = CheckpointSequence()
    node.checkpoints.load(points())
    node.progress = (0., 10.)
    node.tick_checkpoints = lambda now: Scheduler.tick_checkpoints(node, now)
    return node, values, requests, events


def test_checkpoint_trigger_ignores_eight_meter_watchdog_and_slip_and_old_results():
    node, _, requests, events = setup_scheduler()
    Scheduler.tick(node)
    request = requests[-1]
    assert request['trigger'] == 'checkpoint' and request['checkpoint_id'] == 'L2'
    assert node.watch.distance == 0
    Scheduler.tick(node)
    assert len(requests) == 1
    Scheduler.on_result(node, String(data=json.dumps(dict(check_complete=True,event='localized',check_request='old'))))
    assert node.checkpoints.active == 'L2'
    Scheduler.on_result(node, String(data=json.dumps(dict(check_complete=True,event='localized',check_request=request['request']))))
    assert node.checkpoints.done == {'L2'}
    node.progress = (1.8,10.)
    Scheduler.tick(node)  # No 5s delay between distinct checkpoints.
    assert requests[-1]['checkpoint_id'] == 'L1'


def test_manual_cancels_then_resume_retries_same_point_without_new_references():
    node, _, requests, _ = setup_scheduler()
    Scheduler.tick(node)
    Scheduler.on_state(node, String(data='RETURN_MANUAL'))
    assert requests[-1]['active'] is False
    assert not node.checkpoints.done and node.checkpoints.active is None
    Scheduler.on_state(node, String(data='FOLLOW_RETURN_PATH'))
    Scheduler.tick(node)
    assert requests[-1]['checkpoint_id'] == 'L2'


def test_stale_progress_does_not_start_and_timeout_advances():
    node, _, requests, _ = setup_scheduler()
    node.progress = (0, 0.)
    Scheduler.tick(node)
    assert not requests
    node.progress = (0, 10.)
    Scheduler.tick(node)
    node.now = lambda: 15.
    node.odom_time = 15.
    node.progress = (1.8,15.)
    Scheduler.tick(node)
    assert 'L2' in node.checkpoints.done
    assert requests[-1]['checkpoint_id'] == 'L1'


def test_fixed_checkpoint_still_requires_two_frames_to_correct(monkeypatch):
    node, output, events, pair = live_fixture(monkeypatch, apply=True)
    node.check.update(checkpoint_id='L1', checkpoint_arc=0., checkpoint_passed_m=.75)
    pair()
    assert not output
    pair()
    assert len(output) == 1 and events[-1]['event'] == 'localized'


def test_another_reference_cannot_be_committed_for_requested_checkpoint(monkeypatch):
    import visual_landmark.visual_localization_node as module
    node, output, events, pair = live_fixture(monkeypatch, apply=True)
    node.check.update(checkpoint_id='L1', checkpoint_arc=0., checkpoint_passed_m=.75)
    node.landmarks['L2'] = node.landmarks['L1']
    node.reference_meta['L2'] = node.reference_meta['L1']
    node.cached_arcs['L2'] = .1
    counter = []
    def match(*args, **kwargs):
        counter.append(True)
        return (np.eye(4), 30 if len(counter)%2 else 60, .2)
    monkeypatch.setattr(module, 'match', match)
    pair()
    assert not output
    assert events[-1]['reason'] == 'checkpoint mismatch: another reference ranked higher'


def test_passed_checkpoint_rejects_before_sift(monkeypatch):
    import visual_landmark.visual_localization_node as module
    node, output, events, pair = live_fixture(monkeypatch, apply=True)
    node.check.update(checkpoint_id='L1', checkpoint_arc=-1., checkpoint_passed_m=.75)
    monkeypatch.setattr(module, 'features', lambda *_: pytest.fail('passed checkpoint must not extract SIFT'))
    pair()
    assert not output and events[-1]['reason'] == 'checkpoint arrival window passed'


def test_snapshot_viewer_subscribes_to_no_live_camera_topics():
    import inspect
    from visual_landmark.viewer import Viewer
    source = inspect.getsource(Viewer)
    assert '/rear/camera/image_raw' not in source and '/arm/camera/color/image_raw' not in source
    assert 'self.preview.render' not in source


@pytest.mark.parametrize('missing,expected', [
    ('_origin', 'missing mission origin'), ('_odom', 'missing odometry'),
    ('_last_image_arrival', 'missing or stale rear image'),
    ('_camera_info', 'missing rear CameraInfo')])
def test_capture_waiting_reason_reaches_ui_without_image_callbacks(missing, expected):
    from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture
    events = []
    node = NS(_state='MANUAL_RECORDING', auto_enabled=True, _origin=object(), _odom=object(),
        _last_image_arrival=10., image_stale_s=1., _camera_info=object(),
        _event=lambda kind, **fields: events.append(dict(event=kind, **fields)))
    setattr(node, missing, None)
    Capture._publish_capture_status(node, 10.)
    assert events[-1]['reason'] == expected
    Capture._publish_capture_status(node, 10.5)
    assert len(events) == 1


def test_capture_snapshot_publish_does_not_depend_on_localizer_or_reextract_features(monkeypatch):
    import cv2
    from sensor_msgs.msg import CompressedImage
    from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture
    import visual_landmark.visual_landmark_node as module
    output = []
    monkeypatch.setattr(module, 'features', lambda *_: pytest.fail('must use cached points'))
    node = NS(capture_preview_pub=NS(publish=output.append))
    Capture._publish_captured_snapshot(node, np.zeros((100,200),np.uint8),
        np.array([[60.,60.]],np.float32), CompressedImage().header, 'L0')
    assert len(output) == 1
    image = cv2.imdecode(np.frombuffer(output[0].data,np.uint8), cv2.IMREAD_COLOR)
    assert image is not None and np.any(image[57:64,57:64])


def test_time_only_two_seconds_including_stationary_and_no_distance_trigger():
    auto = AutoKeyframes()
    assert auto.time_only and auto.interval == 2.
    assert auto.reason(10.,0.,0.) == 'initial'
    auto.accept(10.,0.,0.)
    assert auto.reason(11.99,100.,3.) is None  # Distance / yaw cannot trigger early.
    assert auto.reason(12.,0.,0.) == 'interval'  # Stationary capture is allowed.
    auto.accept(12.,0.,0.)
    assert auto.reason(13.99,0.,0.) is None
    assert auto.reason(14.,0.,0.) == 'interval'
    assert auto.reason(9.,0.,0.) is None  # No early capture on a backward clock step.


def test_failed_timed_frame_does_not_advance_anchor_or_capture_during_return(monkeypatch):
    from test_adaptive_nodes import capture
    from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture
    node, msg, events, saved = capture(monkeypatch)
    node.auto_policy = AutoKeyframes()
    node.auto_policy.accept(8.,0.,0.)
    msg.header.stamp.sec = 0
    Capture._try_auto(node,msg,10.)
    assert not saved and node.auto_policy.anchor[0] == 8.
    assert events[-1][0] == 'automatic_rejected'
    node._state = 'RETURN_MANUAL'
    Capture._try_auto(node,msg,12.)
    assert not saved and node.auto_policy.anchor[0] == 8.
