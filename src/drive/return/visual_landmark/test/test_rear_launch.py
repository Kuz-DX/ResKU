"""Inspect launch actions without starting a camera or publishing TF."""
import importlib.util
from pathlib import Path

import pytest
import yaml
pytest.importorskip('launch')
from launch import LaunchContext


@pytest.fixture(autouse=True)
def writable_ros_logs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))


def rear_module(monkeypatch):
    path = Path(__file__).resolve().parents[4]/'vision/camera/launch/apc850_rear.launch.py'
    spec = importlib.util.spec_from_file_location('rear_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'LogInfo', lambda **kwargs: kwargs)
    return module


def context(**overrides):
    ctx = LaunchContext()
    values = dict(device='/dev/v4l/by-id/TEST_DEVICE', width='1280', height='720', fps='15.',
                  camera_info_url='', publish_extrinsic='false', parent_frame='base_link',
                  frame_id='rear_camera_optical_frame', pixel_format='mjpeg2rgb',
                  x='', y='', z='', roll='', pitch='', yaw='')
    values.update(overrides)
    ctx.launch_configurations.update(values)
    return ctx


def test_no_calibration_warns_and_does_not_invent_extrinsic(monkeypatch):
    module = rear_module(monkeypatch)
    actions = module.configure(context())
    assert any('WARNING' in action.get('msg', '') for action in actions)
    nodes = [action for action in actions if 'package' in action]
    assert len(nodes) == 1
    assert nodes[0]['package'] == 'usb_cam' and nodes[0]['namespace'] == 'rear/camera'
    params = nodes[0]['parameters'][0]
    assert params['pixel_format'] == 'mjpeg2rgb' and params['camera_info_url'] == ''


def test_rejects_video_number_and_unmeasured_optical_transform(monkeypatch):
    module = rear_module(monkeypatch)
    with pytest.raises(ValueError, match='by-id'):
        module.configure(context(device='/dev/video0'))
    with pytest.raises(ValueError, match='measured'):
        module.configure(context(publish_extrinsic='true'))


def test_calibration_resolution_mismatch_refuses_start(monkeypatch, tmp_path):
    module = rear_module(monkeypatch)
    measured = tmp_path/'calibration.yaml'
    measured.write_text(yaml.safe_dump(dict(image_width=640, image_height=480)))
    with pytest.raises(ValueError, match='resolution'):
        module.configure(context(camera_info_url=measured.as_uri()))


def test_partial_measurements_require_remaining_extrinsics(monkeypatch):
    module = rear_module(monkeypatch)
    declarations = module.generate_launch_description().entities
    defaults = {action.name: ''.join(part.text for part in action.default_value)
                for action in declarations
                if hasattr(action, 'default_value') and action.default_value is not None}
    assert defaults['y'] == ''
    assert defaults['z'] == ''
    assert defaults['x'] == '' and defaults['publish_extrinsic'] == 'false'
    with pytest.raises(ValueError, match='measured x'):
        module.configure(context(publish_extrinsic='true'))
    with pytest.raises(ValueError, match='measured roll'):
        module.configure(context(publish_extrinsic='true', x='-0.3', y='0', z='0.65'))
    actions = module.configure(context(publish_extrinsic='true', x='-0.3', y='0.0', z='0.650674',
                                       roll='-1.570796', pitch='0', yaw='1.570796'))
    tf = next(action for action in actions
              if action.get('executable') == 'static_transform_publisher')
    assert tf['arguments'][:6] == ['--x', '-0.3', '--y', '0.0', '--z', '0.650674']
