"""Validate the LOCK transform without starting sensors, TF, or motors."""
import importlib.util
import math
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch')
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'camera/config/front_d435i_vpr_lock_tf.yaml'


@pytest.fixture
def launcher(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    spec = importlib.util.spec_from_file_location(
        'front_vpr_tf_launch_test', ROOT/'camera/launch/front_d435i_vpr_tf.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'LogInfo', lambda **kwargs: kwargs)
    return module


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(config=str(CONFIG), x='', y='', z='', roll='', pitch='', yaw=''))
    ctx.launch_configurations.update(overrides)
    return ctx


def transform(launcher, **overrides):
    actions = launcher.configure(context(**overrides))
    nodes = [a for a in actions if a.get('package')]
    assert len(nodes) == 1
    assert nodes[0]['executable'] == 'static_transform_publisher'
    args = nodes[0]['arguments']
    return dict(zip(args[::2], args[1::2]))


def test_back_face_not_tripod_origin_and_no_added_lateral_offset(launcher):
    tf = transform(launcher)
    assert float(tf['--x']) == pytest.approx(.152853 + (.02505-.0001-.0042))
    assert float(tf['--y']) == 0.
    assert float(tf['--z']) == .650674
    assert [float(tf['--'+k]) for k in ('roll', 'pitch', 'yaw')] == [0., 0., 0.]
    assert tf['--frame-id'] == 'base_link'
    assert tf['--child-frame-id'] == 'arm_camera_cam_link'


def test_body_offset_rotates_with_explicit_mount_orientation(launcher):
    tf = transform(launcher, yaw=str(math.pi/2))
    assert float(tf['--x']) == pytest.approx(.152853)
    assert float(tf['--y']) == pytest.approx(.02075)
    tf = transform(launcher, pitch=str(math.pi/2))
    assert float(tf['--z']) == pytest.approx(.650674-.02075)


def test_complete_measured_root_override_and_reject_partial(launcher):
    tf = transform(launcher, x='.18', y='0', z='.65')
    assert float(tf['--x']) == .18
    with pytest.raises(ValueError, match='together'):
        transform(launcher, x='.18')


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf'])
def test_nonfinite_never_published(launcher, value):
    with pytest.raises(ValueError, match='finite'):
        transform(launcher, roll=value)


def test_unknown_rotation_or_internal_child_refused(launcher, tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['rotation_rpy_rad']['roll'] = None
    path = tmp_path/'unknown.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='explicit finite'):
        transform(launcher, config=str(path))
    data['rotation_rpy_rad']['roll'] = 0.
    data['child_frame'] = 'arm_camera_color_optical_frame'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='Only vehicle'):
        transform(launcher, config=str(path))


def test_launch_assets_are_packaged_by_existing_globs():
    # A build must include both the mount data and launch, not merely source files.
    from glob import glob
    assert str(CONFIG.relative_to(ROOT)) in glob('camera/config/*.yaml', root_dir=ROOT)
    assert 'camera/launch/front_d435i_vpr_tf.launch.py' in glob('camera/launch/*.launch.py', root_dir=ROOT)


def test_arm_tf_remaps_persist_for_delayed_nodes(monkeypatch, tmp_path):
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch_ros.actions import SetRemap
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))
    spec = importlib.util.spec_from_file_location(
        'vpr_arm_wrapper_test', ROOT/'camera/launch/vpr_lock_arm_control.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(tmp_path))
    description = module.generate_launch_description()
    group = next(a for a in description.entities if isinstance(a, GroupAction))
    actions = group.get_sub_entities()
    # No scope pop: IK's OnProcessExit callbacks launch children after inclusion returns.
    assert not any(type(a).__name__ in {'PopLaunchConfigurations', 'PushLaunchConfigurations'}
                   for a in actions)
    ctx = LaunchContext()
    for action in actions:
        if isinstance(action, SetRemap):
            action.execute(ctx)
    assert ctx.launch_configurations['ros_remaps'] == [
        ('/tf', '/arm_model/tf'), ('/tf_static', '/arm_model/tf_static')]
    assert sum(isinstance(a, IncludeLaunchDescription) for a in actions) == 1
