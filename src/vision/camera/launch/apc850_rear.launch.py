"""Rear APC850 RGB only. Partially measured mounting; no guessed TF."""
import math
from pathlib import Path
from urllib.parse import urlparse, unquote

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    device = value('device')
    if not device.startswith('/dev/v4l/by-id/'):
        raise ValueError('device must be an explicit /dev/v4l/by-id/... path')
    width, height, fps = int(value('width')), int(value('height')), float(value('fps'))
    if min(width, height) <= 0 or not math.isfinite(fps) or fps <= 0:
        raise ValueError('width/height/fps must be positive')
    url = value('camera_info_url')
    actions = []
    if not url:
        actions.append(LogInfo(msg='WARNING: APC850 has no calibration URL. Capture archive only; localization rejects uncalibrated references.'))
    else:
        parsed = urlparse(url)
        path = None
        if parsed.scheme == 'file':
            path = Path(unquote(parsed.path))
        elif parsed.scheme == 'package':
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory(parsed.netloc))/unquote(parsed.path).lstrip('/')
        else:
            actions.append(LogInfo(msg='WARNING: calibration URL not locally inspectable; verify runtime CameraInfo.'))
        if path is None:
            pass
        elif not path.is_file():
            actions.append(LogInfo(msg=f'WARNING: calibration file missing: {path}'))
        else:
            import yaml
            data = yaml.safe_load(path.read_text())
            if (data.get('image_width'), data.get('image_height')) != (width, height):
                raise ValueError('APC850 calibration resolution differs from runtime resolution')
    actions.append(Node(
        package='usb_cam', executable='usb_cam_node_exe',
        namespace='rear/camera', name='apc850_rear', output='screen',
        parameters=[{'video_device': device, 'image_width': width, 'image_height': height,
                     'framerate': fps, 'pixel_format': value('pixel_format'),
                     'io_method': 'mmap', 'brightness': -1,
                     'camera_name': 'apc850_rear', 'frame_id': value('frame_id'),
                     'camera_info_url': url}]))
    if value('publish_extrinsic').lower() == 'true':
        keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for key in keys:
            if not value(key) or not math.isfinite(float(value(key))):
                raise ValueError(f'Explicit measured {key} required to publish rear optical TF')
        args = [arg for key in keys for arg in ['--'+key, value(key)]]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='rear_apc850_extrinsics', output='screen', arguments=args+[
                                '--frame-id', value('parent_frame'), '--child-frame-id', value('frame_id')]))
    return actions


def generate_launch_description():
    arguments = [DeclareLaunchArgument('device', description='Required /dev/v4l/by-id/... APC850 device')]
    defaults = {'width': '1280', 'height': '720', 'fps': '15.0',
                'pixel_format': 'mjpeg2rgb', 'frame_id': 'rear_camera_optical_frame',
                'camera_info_url': '', 'publish_extrinsic': 'false', 'parent_frame': 'base_link',
                # Housing-back measurements are recorded separately; they do
                # not establish optical-origin translation. Require all optical
                # coordinates explicitly before publishing a TF.
                'x': '', 'y': '', 'z': '',
                'roll': '', 'pitch': '', 'yaw': ''}
    arguments.extend(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items())
    return LaunchDescription(arguments+[OpaqueFunction(function=configure)])
