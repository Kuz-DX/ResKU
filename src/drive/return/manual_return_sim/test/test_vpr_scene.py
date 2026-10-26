import ast
from pathlib import Path
import numpy as np
import pytest
from manual_return_sim.vpr_scene import Scene, K, camera_pose
from visual_landmark.localization import features, match


@pytest.mark.parametrize('lateral,yaw', [(.35,0.),(-.25,.04)])
def test_rendered_rgbd_production_pnp_recovers_ground_truth(lateral,yaw):
    scene=Scene()
    reference_pose=[1.,0.,0.]
    current_pose=[1.5,lateral,np.pi+yaw]
    a,_=scene.render(reference_pose,True)
    b,depth=scene.render(current_pose)
    rp,rd=features(a);current=features(b)
    diagnostics={}
    result=match((rp,rd,K,np.zeros(5)),current,depth,K,np.zeros(5),diagnostics=diagnostics)
    assert result is not None,diagnostics
    expected=np.linalg.inv(camera_pose(reference_pose,True))@camera_pose(current_pose)
    assert np.linalg.norm(result[0][:3,3]-expected[:3,3]) < .03
    assert np.linalg.norm(result[0][:3,:3]-expected[:3,:3]) < .02
    assert diagnostics['inlier_ratio'] >= .45
    assert match((rp,rd,K,np.zeros(5)),current,np.zeros_like(depth),K,np.zeros(5)) is None


def test_isolated_launch_contains_no_hardware_drivers():
    root=Path(__file__).resolve().parents[1]
    text=(root/'launch/vpr_rough_terrain.launch.py').read_text()
    tree=ast.parse(text)
    nodes=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Node']
    packages={k.value.value for n in nodes for k in n.keywords if k.arg=='package'}
    assert not packages & {'rmd_x8_driver','realsense2_camera','tool_manipulator_bringup','robot_bringup'}
    assert "os.environ.get('ROS_DOMAIN_ID') != '109'" in text
