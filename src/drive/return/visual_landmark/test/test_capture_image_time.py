"""Replay measured APC850 delay; bind snapshots to source-time odometry."""
from collections import deque
from types import SimpleNamespace as NS
import numpy as np
import pytest
from nav_msgs.msg import Odometry
from rclpy.time import Time
from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture


def odom(stamp):
    msg=Odometry()
    msg.header.stamp=Time(seconds=stamp).to_msg()
    msg.header.frame_id='odom'
    msg.pose.pose.orientation.w=1.
    return msg


def history_node():
    samples=deque((odom(100+i*.05),100+i*.05,i*.01) for i in range(21))
    return NS(_odom=samples[-1][0], _odom_arrival=101., _odom_history=samples)


def test_measured_615ms_camera_delay_uses_nearest_historical_sample():
    node=history_node()
    sample=Capture._select_image_odom(node,101.-.615,101.)
    assert sample[0].header.stamp == Time(seconds=100.4).to_msg()
    assert sample[2] == pytest.approx(.08)
    assert sample[0] is not node._odom


def test_live_odometry_outage_remains_rejected_even_with_good_history():
    node=history_node()
    with pytest.raises(ValueError,match='no fresh live callback'):
        Capture._select_image_odom(node,100.385,101.6)


def test_history_gap_cannot_be_hidden_by_changing_image_timestamp():
    node=history_node()
    node._odom_history=deque([(odom(101.),101.,.2)])
    with pytest.raises(ValueError,match='skew 0.615s'):
        Capture._select_image_odom(node,100.385,101.)


def test_delayed_image_reaches_save_with_original_stamp_and_historical_odom(monkeypatch):
    from test_adaptive_nodes import capture
    node,msg,events,saved=capture(monkeypatch)
    node._odom=odom(10.)
    node._odom_history=deque([(odom(9.4),9.4,.4),(node._odom,10.,1.)])
    msg.header.stamp=Time(seconds=9.385).to_msg()
    original=msg.header.stamp
    selected=[]
    def save(image):
        selected.append(node._capture_odom_sample)
        saved.append(image)
        return dict(localization_ready=True,extrinsic_ok=True,dir='/tmp',yaw=0.)
    node._save_landmark=save
    Capture._try_auto(node,msg,10.)
    assert events[-1][0]=='captured'
    assert len(saved)==1 and msg.header.stamp==original
    assert selected[0][0].header.stamp==Time(seconds=9.4).to_msg()
    assert node._capture_odom_sample is None


def test_archive_uses_selected_sample_distance_and_twist(tmp_path):
    import yaml
    import cv2
    from sensor_msgs.msg import CompressedImage
    from tf2_ros import TransformException
    from return_navigation.se2 import Pose2D
    older=odom(100.4)
    older.twist.twist.linear.x=.2
    def missing_tf(*_):
        raise TransformException('test has no TF')
    node=NS(_next_order=0, _ensure_mission_dir=lambda:str(tmp_path), _odom=odom(101.),
        _odom_arrival=101., _capture_odom_sample=(older,100.4,.4), odometer=NS(s=1.),
        _origin=Pose2D(), _camera_info=None, _lookup_extrinsic=lambda *_:None,
        tf_buffer=NS(lookup_transform=missing_tf), _now=lambda:101., capture_mode='stop')
    msg=CompressedImage()
    msg.header.stamp=Time(seconds=100.385).to_msg()
    msg.header.frame_id='rear'
    msg.format='jpeg'
    msg.data=cv2.imencode('.jpg',np.zeros((100,100),np.uint8))[1].tobytes()
    info=Capture._save_landmark(node,msg)
    meta=yaml.safe_load((tmp_path/'L000'/'meta.yaml').read_text())
    assert info['s_along_path']==meta['s_along_path']==.4
    assert meta['odom_sample_stamp']==pytest.approx(100.4)
    assert meta['odom_twist_at_capture']['vx']==.2
    assert meta['odom_image_skew_s']==pytest.approx(.015)
    assert meta['odom_sample_selection']=='nearest_buffered'
    assert not info['localization_ready']  # Missing TF is never treated as a valid reference.
