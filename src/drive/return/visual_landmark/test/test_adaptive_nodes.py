"""ROS message integration with deterministic clocks; no motors or ROS graph."""
import json
from types import SimpleNamespace as NS
import pytest
import numpy as np
import cv2
pytest.importorskip('rclpy')
from rclpy.time import Time
from std_msgs.msg import String, Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage
from visual_landmark.visual_check_scheduler_node import VisualCheckScheduler as Scheduler
from visual_landmark.adaptive import CheckWatchdog, AutoKeyframes
from visual_landmark.visual_landmark_node import VisualLandmarkNode as Capture
from return_navigation.se2 import Pose2D


def scheduler():
    values=dict(enable_adaptive_visual_check=True, enable_slip_early_trigger=True,
                wheel_timeout_s=.5, gyro_timeout_s=.5, diagnostics_timeout_s=2.,
                odom_timeout_s=.5, visual_check_timeout_s=5., visual_check_on_return_start=False)
    req, events = [], []
    node=NS(watch=CheckWatchdog(), state='FOLLOW_RETURN_PATH', origin=None,
            now=lambda:10., p=lambda n:values[n], start_check=False,
            odom_time=10., wheel=1., gyro=0., wheel_time=10., gyro_time=10.,
            health=True, diag_time=10., pub=NS(publish=lambda msg:req.append(json.loads(msg.data))),
            event=lambda kind, **kw:events.append((kind,kw)))
    node.cancel=lambda:Scheduler.cancel(node)
    return node,values,req,events


@pytest.mark.parametrize('field', ['health','wheel_time','gyro_time','diag_time'])
def test_scheduler_stale_or_fallback_disables_early_trigger(field):
    n,p,req,_=scheduler()
    setattr(n,field,False if field=='health' else 0.)
    n.watch.risk_time=9.9
    n.watch.score=.34
    Scheduler.tick(n)
    assert not req and n.watch.score==0


@pytest.mark.parametrize('calibrated,source', [(0,1),(1,0),(0,0),(1,1)])
def test_reduced_odom_diagnostics_health_contract(calibrated,source):
    n,_,_,_=scheduler()
    msg=DiagnosticArray()
    msg.header.stamp=Time(seconds=10).to_msg()
    status=DiagnosticStatus(name='reduced_odom/estimator')
    status.values=[KeyValue(key=k,value=str(v)) for k,v in dict(gyro_calibrated=calibrated,wz_source_gyro=source,imu_stale=0).items()]
    msg.status=[status]
    Scheduler.on_diag(n,msg)
    assert n.health == bool(calibrated and source)


def test_manual_auto_retains_watchdog_and_cancel_pending_request():
    n,p,req,_=scheduler()
    n.watch.distance=9.; n.watch.start('old',9.)
    Scheduler.on_state(n,String(data='RETURN_MANUAL'))
    assert n.watch.distance==9 and n.watch.pending is None
    assert req[-1]==dict(active=False,request='old')
    Scheduler.on_state(n,String(data='FOLLOW_RETURN_PATH'))
    assert n.watch.distance==9
    Scheduler.on_state(n,String(data='IDLE'))
    Scheduler.on_state(n,String(data='STOP_BEFORE_TURN'))
    assert n.watch.distance==0 and n.watch.score==0


def test_matching_result_id_required_and_validation_resets_without_correction():
    n,p,req,_=scheduler(); n.watch.distance=9.
    Scheduler.tick(n)
    token=req[-1]['request']
    Scheduler.on_result(n,String(data=json.dumps(dict(event='validated',check_complete=True,check_request='old'))))
    assert n.watch.distance==9
    Scheduler.on_result(n,String(data=json.dumps(dict(event='validated',check_complete=True,check_request=token,correction_applied=False))))
    assert n.watch.distance==0 and n.watch.pending is None


def test_scheduler_does_not_spam_missing_visual_node():
    n,p,req,_=scheduler(); n.watch.distance=9.
    Scheduler.tick(n); Scheduler.tick(n)
    assert len(req)==1
    n.now=lambda:15.
    Scheduler.tick(n)
    assert req[-1]['active'] is False and n.watch.distance==9.
    assert n.watch.due(16) is None


def capture(monkeypatch):
    import visual_landmark.visual_landmark_node as module
    values=dict(auto_keyframe_retry_s=1.,auto_keyframe_min_features=20,auto_keyframe_blur_min=0.)
    ci=CameraInfo(); ci.width,ci.height=640,480; ci.header.frame_id='rear'
    ci.distortion_model='plumb_bob';ci.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.];ci.d=[0.]*5
    od=Odometry();od.header.stamp=Time(seconds=10).to_msg();od.pose.pose.orientation.w=1.
    msg=CompressedImage();msg.header.frame_id='rear';msg.header.stamp=Time(seconds=10).to_msg()
    msg.data=cv2.imencode('.jpg',np.zeros((480,640),np.uint8))[1].tobytes()
    events=[]; saved=[]
    n=NS(_state='MANUAL_RECORDING',_origin=Pose2D(),_odom=od,_odom_arrival=10.,
         auto_next_try=0.,auto_policy=AutoKeyframes(),image_stale_s=1.,_camera_info=ci,
         odometer=NS(s=0.), get_parameter=lambda k:NS(value=values[k]),
         _event=lambda kind,**kw:events.append((kind,kw)))
    n._save_landmark=lambda img:saved.append(img) or dict(localization_ready=True,extrinsic_ok=True,dir='/tmp',yaw=0.)
    monkeypatch.setattr(module,'features',lambda im:(np.zeros((30,2),np.float32),np.zeros((30,128),np.float32)))
    monkeypatch.setattr(module.np,'savez_compressed',lambda *a,**kw:None)
    return n,msg,events,saved


def test_auto_moving_capture_never_requests_hold_and_reuses_descriptors(monkeypatch):
    n,msg,events,saved=capture(monkeypatch)
    # No hold publisher or sequencer supplied: automatic capture must not access them.
    Capture._try_auto(n,msg,10.)
    assert len(saved)==1 and events[-1][0]=='captured'
    assert events[-1][1]['capture_source']=='automatic'
    assert n.auto_policy.anchor is not None
    Capture._try_auto(n,msg,30.)
    assert len(saved)==1  # stationary duplicate, even after 20 seconds


@pytest.mark.parametrize('bad', ['stale','camera_info','features','tf','return'])
def test_bad_auto_frame_never_becomes_reference(monkeypatch,bad):
    n,msg,events,saved=capture(monkeypatch)
    if bad=='stale': msg.header.stamp=Time(seconds=8).to_msg()
    if bad=='camera_info': n._camera_info=None
    if bad=='return': n._state='RETURN_MANUAL'
    if bad=='features':
        monkeypatch.setattr('visual_landmark.visual_landmark_node.features',lambda im:(np.zeros((2,2)),None))
    if bad=='tf': n._save_landmark=lambda img:dict(localization_ready=False,extrinsic_ok=False)
    Capture._try_auto(n,msg,10.)
    assert not any(event=='captured' for event,_ in events)
    assert n.auto_policy.anchor is None


def test_capture_new_mission_resets_auto_but_ready_state_preserves_it(monkeypatch):
    from visual_landmark.capture_logic import Odometer
    n,_,_,_=capture(monkeypatch)
    n.seq=NS(busy=False);n._mission_dir='/old';n._next_order=8;n.odometer=Odometer()
    n.auto_policy.accept(10.,2.,0.)
    Capture._on_state(n,String(data='WAIT_RETURN_COMMAND'))
    assert n.auto_policy.anchor is not None and n._next_order==8
    Capture._on_state(n,String(data='IDLE'))
    Capture._on_state(n,String(data='MANUAL_RECORDING'))
    assert n.auto_policy.anchor is None and n._next_order==0 and n._mission_dir is None
