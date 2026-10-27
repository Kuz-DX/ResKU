"""Geometric regression tests: real SIFT/OpenCV, no motors or external camera."""
from dataclasses import replace
import threading
import math

import cv2
import numpy as np
import pytest

from visual_landmark.localization import (FeatureOptions, MatchOptions, PriorOptions, features,
    grid_select, geometric_mask, estimate_pose, check_pose_prior, transform)
from visual_landmark.latest_pair import LatestPair


def test_grid_caps_total_cells_and_keeps_descriptor_indices(monkeypatch):
    opts = FeatureOptions(feature_grid_rows=2, feature_grid_cols=2,
                          feature_grid_max_per_cell=2, feature_max_total=5)
    xy = np.array([[10,10],[11,11],[12,12],[60,10],[61,11],[10,60],[11,61],[60,60]])
    response = np.array([1,9,4,3,8,7,2,6], float)
    indices = grid_select(xy, response, (100,100), opts)
    assert len(indices) == 5
    assert set(indices) == {1,4,5,7,2}
    cells = xy[indices]//50
    assert max(np.unique(cells, axis=0, return_counts=True)[1]) <= 2
    kps = [cv2.KeyPoint(float(x), float(y), 3., response=float(r)) for (x,y),r in zip(xy,response)]
    desc = np.repeat(np.arange(len(xy), dtype=np.float32)[:, None], 128, axis=1)
    monkeypatch.setattr(cv2, 'SIFT_create', lambda **kw: type('Sift', (), {'detectAndCompute':lambda *args:(kps,desc)})())
    stats = {}
    points, rows = features(np.zeros((100,100), np.uint8), opts, stats)
    np.testing.assert_array_equal(points, xy[indices])
    np.testing.assert_array_equal(rows, desc[indices])
    assert stats == dict(feature_count_raw=8, feature_count_grid_filtered=5, raw_features=8, filtered_features=5)


def test_epipolar_ransac_removes_false_cross_camera_correspondences():
    rng = np.random.default_rng(919)
    xyz = rng.uniform([-1.8,-1.,2.], [1.8,1.,6.], (160,3))
    ck = np.array([[520.,0,320.],[0,510.,240.],[0,0,1.]])
    rk = np.array([[690.,0,640.],[0,680.,360.],[0,0,1.]])
    cd, rd = np.array([-.08,.01,.001,-.001,.001]), np.array([-.18,.04,-.002,.001,-.01])
    cp = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), ck, cd)[0].reshape(-1,2)
    rp = cv2.projectPoints(xyz, np.array([.08,-.1,.05]), np.array([.3,-.05,.2]), rk, rd)[0].reshape(-1,2)
    rp[-35:] = rng.uniform([0,0],[1280,720],(35,2))
    mask = geometric_mask(cp,rp,ck,cd,rk,rd,MatchOptions())
    assert mask[:125].sum() >= 120
    assert mask[-35:].sum() <= 5


def test_pnp_accepts_distributed_scene_and_rejects_one_corner_cluster():
    rng = np.random.default_rng(44)
    k = np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
    uv = rng.uniform([30,25],[610,455],(160,2))
    z = rng.uniform(2.,5.,160)
    xyz = np.c_[(uv[:,0]-320)*z/500,(uv[:,1]-240)*z/500,z]
    diag = {}
    assert estimate_pose(xyz,uv,k,None,diagnostics=diag,image_shape=(480,640)) is not None
    assert diag['inlier_grid_cells'] >= 20
    uv = rng.uniform([12,12],[110,100],(100,2))  # still >60px span and 3D spread
    z = rng.uniform(2.,6.,100)
    xyz = np.c_[(uv[:,0]-320)*z/500,(uv[:,1]-240)*z/500,z]
    diag = {}
    assert estimate_pose(xyz,uv,k,None,diagnostics=diag,image_shape=(480,640)) is None
    assert diag['reject_reason'] == 'insufficient_inlier_spatial_coverage'


@pytest.mark.parametrize('offset,angle,allowed', [(.2,2.,True),(.6,2.,False),(.2,25.,False)])
def test_prior_uses_body_yaw_with_optical_camera_axes(offset, angle, allowed):
    # Camera optical z points along body x. Optical Euler yaw is NOT body yaw.
    bc = np.eye(4);bc[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];bc[:3,3]=[.17,0,.65]
    a = math.radians(angle)
    body_error = transform([offset,0,0],[0,0,math.sin(a/2),math.cos(a/2)])
    predicted = transform([.1,.2,.3],[0,0,0,1])
    estimated = predicted @ np.linalg.inv(bc) @ body_error @ bc
    diag = {}
    assert check_pose_prior(estimated,predicted,bc,PriorOptions(),diag) == allowed
    assert diag['pose_prior_translation_error'] == pytest.approx(offset)
    assert diag['pose_prior_yaw_error'] == pytest.approx(a)
    assert diag['pose_prior_roll_error'] < 1e-10


def test_latest_pair_overwrites_backlog_while_single_worker_is_busy():
    slot = LatestPair()
    started, finish = threading.Event(), threading.Event()
    slot.put(0,0,0.)
    consumed = []
    def worker():
        consumed.append(slot.take())
        started.set();assert finish.wait(2.)
        consumed.append(slot.take())
    thread = threading.Thread(target=worker)
    thread.start();assert started.wait(2.)
    for i in range(1,101):slot.put(i,-i,float(i))
    finish.set();thread.join(2.)
    assert not thread.is_alive()
    assert consumed == [(0,0,0.),(100,-100,100.)]
    assert slot.take() is None and slot.overwritten == 99


def test_depth_boundary_radius_rejects_outer_ring_discontinuity():
    from test_localization import scene
    from visual_landmark.localization import match
    xyz, uv, k, _ = scene()
    cp = cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,None)[0].reshape(-1,2)
    depth = np.zeros((480,640),np.float32)
    keep = []
    for i,(u,v) in enumerate(cp):
        x,y = np.round([u,v]).astype(int)
        if 4 <= x < 636 and 4 <= y < 476 and not np.any(depth[y-4:y+5,x-4:x+5]):
            depth[y-2:y+3,x-2:x+3] = xyz[i,2]*1.2
            depth[y-1:y+2,x-1:x+2] = xyz[i,2]
            keep.append(i)
    desc = np.random.default_rng(1).normal(size=(len(keep),128)).astype(np.float32)
    ref=(uv[keep].astype(np.float32),desc,k,None)
    cur=(cp[keep].astype(np.float32),desc.copy())
    diagnostics={}
    assert match(ref,cur,depth,k,None,diagnostics=diagnostics) is None
    assert diagnostics['depth_valid_matches'] == 0
    assert match(ref,cur,depth,k,None,options=replace(MatchOptions(),depth_patch_radius=1)) is not None


def test_latest_slot_does_not_replace_new_source_with_delayed_old_pair():
    slot = LatestPair()
    slot.put('new','depth',1.,stamp_ns=200)
    slot.put('old','depth',2.,stamp_ns=100)
    assert slot.take()[0] == 'new' and slot.out_of_order == 1


def test_dashboard_shows_production_not_preview_counts_and_prior_rejection():
    from visual_landmark.viewer import quality_lines
    data = dict(event='no_match', feature_count_raw=800, feature_count_grid_filtered=480,
                current_sift_s=.04, processing_latency_s=.1, attempts=[dict(id='L1',
                    raw_matches=480, mutual_ratio_matches=55, geometric_inliers=40,
                    depth_valid_matches=35, pnp_points=35,pnp_inliers=33,inlier_ratio=33/35,
                    inlier_grid_cells=9,reprojection_rmse=.8,pose_prior_translation_error=.2,
                    pose_prior_yaw_error=.5,reject_reason='pose_prior_mismatch')])
    lines = '\n'.join(quality_lines(data))
    assert '800 / 480' in lines and '35 / 33' in lines
    assert 'pose_prior_mismatch' in lines and '28.6deg' in lines
    assert 'PRODUCTION' in lines


def test_live_wrong_yaw_prior_rejects_without_mutating_correction(monkeypatch):
    from test_cross_camera import live_fixture
    import visual_landmark.visual_localization_node as module
    n, out, events, pair = live_fixture(monkeypatch, apply=True)
    previous = np.eye(4)
    n.correction = previous
    a = math.radians(25)
    estimated = transform([0,0,0],[0,0,math.sin(a/2),math.cos(a/2)])
    monkeypatch.setattr(module, 'match', lambda *args, **kw:(estimated, 40, .3))
    pair();pair()
    assert not out and n.correction is previous
    assert events[-1]['attempts'][0]['reject_reason'] == 'pose_prior_mismatch'


def test_stale_frame_rejected_before_sift_even_if_it_was_latest_received(monkeypatch):
    from test_cross_camera import live_fixture
    import visual_landmark.visual_localization_node as module
    from sensor_msgs.msg import CompressedImage, Image
    from rclpy.time import Time
    n,out,events,_=live_fixture(monkeypatch)
    color,depth=CompressedImage(),Image()
    color.header.stamp=Time(seconds=99.).to_msg();depth.header.stamp=color.header.stamp
    monkeypatch.setattr(module,'features',lambda *args:pytest.fail('SIFT on stale input'))
    module.VisualLocalizationNode.on_pair(n,color,depth)
    assert not out and 'stale RGB-D pair' in events[-1]['reason']


def test_debug_disabled_never_renders_or_queries_subscribers(monkeypatch):
    from test_cross_camera import live_fixture
    n,out,events,pair=live_fixture(monkeypatch)
    n.debug_enabled=False
    n.publish_matches=lambda *args:pytest.fail('debug renderer called')
    # Deliberately omit debug_pub: short circuit must avoid even querying it.
    pair();pair()
    assert events[-1]['event']=='validated' and not out


def test_legacy_cache_recomputed_once_and_versioned_cache_reused(tmp_path, monkeypatch):
    from types import SimpleNamespace as NS
    from std_msgs.msg import String
    from visual_landmark.visual_localization_node import VisualLocalizationNode as Localizer
    import visual_landmark.visual_localization_node as module
    import json,yaml
    image=np.random.default_rng(42).integers(0,256,(480,640),np.uint8)
    assert cv2.imwrite(str(tmp_path/'rear.png'),image)
    meta=dict(id='L1',image_file='rear.png',mission_pose=dict(x=0.,y=0.),camera_frame='rear',
        mission_camera_transform=np.eye(4).tolist(), camera_info=dict(frame_id='rear',width=640,height=480,
        distortion_model='plumb_bob',k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],d=[0.]*5))
    (tmp_path/'meta.yaml').write_text(yaml.safe_dump(meta))
    pts,desc=features(image)
    np.savez_compressed(tmp_path/'features.npz',points=pts,descriptors=desc)
    events=[]
    n=NS(state='MANUAL_RECORDING',landmarks={},reference_meta={},feature_options=FeatureOptions(),
         reference_feature_stats={},event=lambda kind,**kw:events.append((kind,kw)))
    msg=String(data=json.dumps(dict(event='captured',dir=str(tmp_path))))
    Localizer.on_landmark(n,msg)
    assert events[-1][0]=='reference_loaded' and not events[-1][1]['cache_reused']
    np.savez_compressed(tmp_path/'features.npz',points=pts,descriptors=desc,
                       feature_config=np.asarray(FeatureOptions().signature()),feature_count_raw=800)
    monkeypatch.setattr(module,'features',lambda *args:pytest.fail('cache recomputed'))
    Localizer.on_landmark(n,msg)
    assert events[-1][0]=='reference_loaded' and events[-1][1]['cache_reused']
    assert events[-1][1]['feature_count_grid_filtered'] == len(pts)


def test_quality_yaml_matches_feature_and_gate_defaults():
    import yaml
    from pathlib import Path
    from dataclasses import asdict
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/vpr_quality.yaml').read_text())
    capture=config['visual_landmark_node']['ros__parameters']
    local=config['visual_localization_node']['ros__parameters']
    for k,v in asdict(FeatureOptions()).items():assert capture[k]==local[k]==v
    for k,v in asdict(MatchOptions()).items():assert local['match.'+k]==v
    for k,v in asdict(PriorOptions()).items():assert local['pose_prior.'+k]==v
    assert local['apply_correction'] is False


def test_viewer_has_no_control_or_mission_publishers():
    import ast
    from pathlib import Path
    import visual_landmark.viewer as viewer
    tree=ast.parse(Path(viewer.__file__).read_text())
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                   and n.func.attr=='create_publisher' for n in ast.walk(tree))
