from types import SimpleNamespace as NS
import json
import math

import cv2
import numpy as np
import pytest
from visual_landmark.calibration import camera_model
from visual_landmark.localization import estimate_pose, match, transform, planar, valid_transform
from visual_landmark.calibration_tool import validate
from visual_landmark.evaluate_matching import prepared


def synthetic():
    rng = np.random.default_rng(11)
    xyz = rng.uniform([-1.2, -.8, 2.5], [1.2, .8, 5.], (180, 3))
    ck = np.array([[520., 0, 320], [0, 510., 240], [0, 0, 1]])
    rk = np.array([[690., 0, 640], [0, 680., 360], [0, 0, 1]])
    cd = np.array([-.08, .01, .001, -.001, .001])
    rd = np.array([-.18, .04, -.002, .001, -.01])
    rot, tr = np.array([.08, -.1, .05]), np.array([.1, -.05, .2])
    cp = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), ck, cd)[0].reshape(-1, 2)
    rp = cv2.projectPoints(xyz, rot, tr, rk, rd)[0].reshape(-1, 2)
    expected = np.eye(4)
    expected[:3, :3], expected[:3, 3] = cv2.Rodrigues(rot)[0], tr
    return xyz, cp, rp, ck, cd, rk, rd, expected


def test_different_intrinsics_resolution_and_nonzero_distortion():
    xyz, cp, rp, ck, cd, rk, rd, expected = synthetic()
    result = estimate_pose(xyz, rp, rk, rd)
    assert result is not None
    np.testing.assert_allclose(result[0], expected, atol=1e-5)
    depth = np.zeros((480, 640), np.float32)
    keep = []
    for i, (u, v) in enumerate(cp):
        x, y = np.round([u, v]).astype(int)
        if 3 < x < 636 and 3 < y < 476 and not np.any(depth[y-3:y+4, x-3:x+4]):
            depth[y-1:y+2, x-1:x+2] = xyz[i, 2]
            keep.append(i)
    desc = np.random.default_rng(3).normal(size=(len(keep), 128)).astype(np.float32)
    debug = {}
    ref = (rp[keep].astype(np.float32), desc, rk, rd)
    current = (cp[keep].astype(np.float32), desc.copy())
    result = match(ref, current, depth, ck, cd, diagnostics=debug)
    assert result is not None
    np.testing.assert_allclose(result[0], expected, atol=1e-4)
    assert debug['raw_matches'] >= debug['mutual_ratio_matches'] >= debug['inliers'] >= 20
    assert debug['inlier_ratio'] > .95 and debug['reject_reason'] is None
    assert match(ref, current, np.full_like(depth, np.nan), ck, cd) is None
    # Same descriptor correspondences with wrong image geometry must not localize.
    bad = (np.random.default_rng(9).uniform([0, 0], [1280, 720], (len(keep), 2)).astype(np.float32), desc, rk, rd)
    assert match(bad, current, depth, ck, cd) is None


def test_rear_180_mount_current_fixed_mount_and_planar_correction():
    # Optical z points backward for rear, forward for front in body x-forward axes.
    bc_rear = np.eye(4)
    bc_rear[:3, :3] = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
    bc_rear[:3, 3] = [-.3, 0, .7]
    bc_front = np.eye(4)
    bc_front[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    bc_front[:3, 3] = [.4, 0, .7]
    reference_base = transform([5, 1, 0], [0, 0, 0, 1])
    yaw = math.pi+.05
    true_base = transform([5.2, 1.1, 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
    odom_base = transform([5.5, 1.3, 0], [0, 0, math.sin((yaw+.03)/2), math.cos((yaw+.03)/2)])
    mc_reference = reference_base@bc_rear
    relative = np.linalg.inv(mc_reference)@true_base@bc_front
    recovered_base = mc_reference@relative@np.linalg.inv(bc_front)
    correction = planar(recovered_base)@np.linalg.inv(planar(odom_base))
    np.testing.assert_allclose(correction@planar(odom_base), planar(true_base), atol=1e-9)
    assert valid_transform(bc_front) and valid_transform(bc_rear)


@pytest.mark.parametrize('model,d', [('equidistant', [0.]*4), ('plumb_bob', []), ('plumb_bob', [float('nan')]*5)])
def test_invalid_calibration_is_rejected(model, d):
    with pytest.raises(ValueError):
        camera_model(640, 480, np.eye(3), d, model)


def test_resolution_mismatch_and_invalid_transform_rejected():
    k = [[500., 0, 320], [0, 500., 240], [0, 0, 1]]
    with pytest.raises(ValueError):
        camera_model(640, 480, k, [0.]*5, 'plumb_bob', (720, 1280))
    assert not valid_transform(np.zeros((4, 4)))
    with pytest.raises(ValueError):
        transform([0, 0, 0], [0, 0, 0, 0])


def test_heldout_calibration_report_and_undistortion_preview(tmp_path):
    board = np.zeros((54, 3), np.float32)
    board[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2)*.03
    k = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
    d = np.array([-.1, .02, .001, -.001, 0.])
    pixels = cv2.projectPoints(board, np.array([.1, .2, .05]), np.array([-.1, -.06, .8]), k, d)[0].astype(np.float32)
    image = tmp_path/'image.png'
    assert cv2.imwrite(str(image), np.zeros((480, 640, 3), np.uint8))
    report = validate(board, [(str(image), pixels)], (640, 480), k, d, tmp_path/'preview')
    assert report['reprojection_rms_px'] < .01
    assert report['coverage_bins_4x3'] > 0
    assert (tmp_path/'preview'/'undistorted_000.png').is_file()


pytest.importorskip('rclpy')
from visual_landmark.visual_localization_node import VisualLocalizationNode


def test_optional_pose_guard_rejects_arm_motion():
    params = dict(lock_pose_guard=True, expected_lock_pose_file='', lock_translation_tolerance_m=.02,
                  lock_rotation_tolerance_rad=.035)
    node = NS(lock_pose=None, p=lambda name: params[name])
    VisualLocalizationNode.check_lock_pose(node, np.eye(4))
    VisualLocalizationNode.check_lock_pose(node, np.eye(4))
    changed = np.eye(4)
    changed[0, 3] = .03
    with pytest.raises(ValueError, match='locked'):
        VisualLocalizationNode.check_lock_pose(node, changed)


def live_fixture(monkeypatch, apply=False, relative_x=.1):
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    import visual_landmark.visual_localization_node as module
    from visual_landmark.localization import MatchOptions
    tick = NS(seconds=100.)
    clock = lambda: Time(seconds=tick.seconds, clock_type=ClockType.ROS_TIME)
    image = np.random.default_rng(42).integers(0, 256, (480, 640), np.uint8)
    color = CompressedImage()
    color.header.frame_id = 'arm_camera_color_optical_frame'
    color.format = 'jpeg'
    color.data = cv2.imencode('.jpg', image)[1].tobytes()
    depth = Image()
    depth.header.frame_id = color.header.frame_id
    depth.encoding = '16UC1'
    ci = CameraInfo()
    ci.header.frame_id = color.header.frame_id
    ci.width, ci.height, ci.distortion_model = 640, 480, 'plumb_bob'
    ci.k, ci.d = [500., 0., 320., 0., 500., 240., 0., 0., 1.], [0.]*5
    values = dict(processing_period_s=1., base_frame='base_link', odom_frame='odom',
                  candidate_radius_m=12., max_correction_m=3., max_correction_yaw_rad=.7,
                  ambiguity_ratio=.85, agreement_translation_m=.25, agreement_yaw_rad=.1,
                  agreement_max_interval_s=2.5, apply_correction=apply, lock_pose_guard=True,
                  expected_lock_pose_file='', lock_translation_tolerance_m=.02,
                  lock_rotation_tolerance_rad=.035)
    events, output = [], []
    relative = np.eye(4)
    relative[0, 3] = relative_x
    monkeypatch.setattr(module, 'match', lambda *args, **kwargs: (relative, 30, .2))
    def lookup(*args):
        tf = TransformStamped()
        tf.transform.rotation.w = 1.
        return tf
    node = NS(state='FOLLOW_RETURN_PATH', origin=np.eye(4), info=ci, last=-float('inf'),
              landmarks={'L1': ((np.zeros((30, 2)), None, np.eye(3), np.zeros(5)), np.eye(4))},
              reference_meta={'L1': {'camera_info': dict(distortion_model='plumb_bob', width=640, height=480)}},
              p=lambda name: values[name], get_clock=lambda: NS(now=clock), pending=None, pending_stamp=None,
              lock_pose=None, correction=None, route=[], last_reference_arc=None, search=None,
              match_options=MatchOptions(), pub=NS(publish=output.append),
              event=lambda event, **fields: events.append(dict(event=event, **fields)),
              bridge=NS(imgmsg_to_cv2=lambda *args, **kwargs: np.full((480, 640), 2000, np.uint16)),
              buffer=NS(lookup_transform=lookup))
    node.check_lock_pose = lambda t: VisualLocalizationNode.check_lock_pose(node, t)
    def pair():
        color.header.stamp = clock().to_msg()
        depth.header.stamp = color.header.stamp
        VisualLocalizationNode.on_pair(node, color, depth)
        tick.seconds += 1.
    return node, output, events, pair


@pytest.mark.parametrize('apply', [False, True])
def test_live_two_frame_acceptance_and_observe_only_never_publishes(monkeypatch, apply):
    node, output, events, pair = live_fixture(monkeypatch, apply=apply)
    pair()
    assert not output
    pair()
    assert len(output) == int(apply)
    assert events[-1]['event'] == ('localized' if apply else 'validated')
    assert events[-1]['correction_applied'] == apply
    assert (node.correction is not None) == apply


def test_live_large_false_correction_and_resolution_mismatch_reject(monkeypatch):
    node, output, events, pair = live_fixture(monkeypatch, apply=True, relative_x=10.)
    pair()
    pair()
    assert not output and events[-1]['event'] == 'no_match'
    node.info.width = 1280
    pair()
    assert not output and events[-1]['event'] == 'rejected'
    assert 'resolution' in events[-1]['reason']


def test_missing_image_time_tf_rejects_without_resetting_last_correction(monkeypatch):
    from tf2_ros import TransformException
    node, output, events, pair = live_fixture(monkeypatch, apply=True)
    previous = np.eye(4)
    node.correction = previous
    def missing(*args):
        raise TransformException('extrapolation: image-time TF unavailable')
    node.buffer.lookup_transform = missing
    pair()
    assert not output and node.correction is previous
    assert events[-1]['event'] == 'rejected' and 'TF' in events[-1]['reason']


def test_missing_calibration_allows_archive_but_rejects_localization(tmp_path):
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import TransformException
    from visual_landmark.visual_landmark_node import VisualLandmarkNode
    from return_navigation.se2 import Pose2D
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 1.
    image = CompressedImage()
    image.header.frame_id = 'rear_camera_optical_frame'
    image.header.stamp = Time(seconds=10).to_msg()
    image.format = 'jpeg'
    image.data = cv2.imencode('.jpg', np.zeros((480, 640), np.uint8))[1].tobytes()
    def missing(*args):
        raise TransformException('missing TF')
    capture = NS(_ensure_mission_dir=lambda: str(tmp_path), _next_order=0, _odom=odom,
                 _origin=Pose2D(), _camera_info=None, _lookup_extrinsic=lambda *args: None,
                 tf_buffer=NS(lookup_transform=missing), odometer=NS(s=0.), capture_mode='stop',
                 _now=lambda: 10., _odom_arrival=10.)
    info = VisualLandmarkNode._save_landmark(capture, image)
    assert (tmp_path/'L000'/'rear.jpg').is_file() and not info['localization_ready']
    events = []
    node = NS(state='WAIT_RETURN_COMMAND', landmarks={}, reference_meta={},
              event=lambda event, **kwargs: events.append((event, kwargs)))
    VisualLocalizationNode.on_landmark(node, String(data=json.dumps(dict(event='captured', dir=info['dir']))))
    assert not node.landmarks and events[-1][0] == 'reference_rejected'


def test_visual_node_absent_does_not_block_local_return_when_recovery_disabled():
    from return_navigation.return_path_follower_node import ReturnPathFollowerNode
    from return_navigation.se2 import Pose2D
    from rclpy.time import Time
    from rclpy.clock import ClockType
    now = Time(seconds=100., clock_type=ClockType.ROS_TIME)
    tracked = []
    node = NS(_mission_state='FOLLOW_RETURN_PATH', _path=[Pose2D(0., 0., 0.), Pose2D(10., 0., 0.)],
              _odom_pose=Pose2D(1., 0., 0.), _origin=Pose2D(), _last_odom_time=now,
              get_clock=lambda: NS(now=lambda: now), odom_timeout_s=.5, _visual_correction=None,
              _next_corner=lambda: None, _floor=0, _progress_index=0,
              _nearest_index=lambda *args: 0, _landmark_hold=None,
              get_parameter=lambda name: NS(value=False), _mode='TRACK_PATH',
              _target_heading=lambda idx: 0., rotate_enter_angle=1.,
              _tick_track_path=lambda *args: tracked.append(True))
    ReturnPathFollowerNode._tick(node)
    assert tracked == [True]


def test_current_camera_frame_mismatch_rejects(monkeypatch):
    node, output, events, pair = live_fixture(monkeypatch, apply=True)
    node.info.header.frame_id = 'different_optical_frame'
    pair()
    assert not output and events[-1]['event'] == 'rejected'
    assert 'optical frame' in events[-1]['reason']
