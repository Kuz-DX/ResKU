"""One vehicle -> D435i root TF, valid only at the surveyed arm LOCK pose.

No camera driver, arm model, internal optical TF, or motion nodes are launched.
The default geometry is an explicitly provisional mechanical candidate.
"""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def finite(value, name):
    if value is None or isinstance(value, bool):
        raise ValueError(f'{name} requires an explicit finite number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f'{name} requires an explicit finite number')
    return number


def configure(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    config = yaml.safe_load(Path(value('config')).read_text())
    # Never allow this launcher to replace a driver-owned optical transform.
    if (config['parent_frame'], config['child_frame']) != ('base_link', 'arm_camera_cam_link'):
        raise ValueError('Only vehicle base_link -> arm_camera_cam_link is supported')
    rpy = [finite(value(key) if value(key).strip() else config['rotation_rpy_rad'][key], key)
           for key in ('roll', 'pitch', 'yaw')]
    back = [finite(config['housing_back_translation_m'][key], f'back.{key}')
            for key in ('x', 'y', 'z')]
    offset = [finite(config['back_to_root_translation_m'][key], f'offset.{key}')
              for key in ('x', 'y', 'z')]
    # Rotate mechanical body offsets into the vehicle axes (Rz(yaw) Ry(pitch) Rx(roll)).
    sr, cr = math.sin(rpy[0]), math.cos(rpy[0])
    sp, cp = math.sin(rpy[1]), math.cos(rpy[1])
    sy, cy = math.sin(rpy[2]), math.cos(rpy[2])
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                (-sp, cp*sr, cp*cr))
    xyz = [back[i] + sum(rotation[i][j]*offset[j] for j in range(3)) for i in range(3)]
    overrides = [value(key).strip() for key in ('x', 'y', 'z')]
    if any(overrides):
        if not all(overrides):
            raise ValueError('Override final root x/y/z together, or leave all empty')
        xyz = [finite(item, key) for item, key in zip(overrides, ('x', 'y', 'z'))]
    args = [item for key, number in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz+rpy)
            for item in ('--'+key, str(number))]
    args += ['--frame-id', config['parent_frame'], '--child-frame-id', config['child_frame']]
    return [
        LogInfo(msg=f'TRIAL/UNVERIFIED D435i LOCK TF: xyz={xyz}, rpy={rpy}. '
                    'Keep visual_apply_correction=false and enable_landmark_recovery=false. '
                    'Arm movement invalidates this static TF; no other parent/publisher may own the root.'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='front_d435i_vpr_lock_tf', arguments=args, output='screen'),
    ]


def generate_launch_description():
    config = str(Path(get_package_share_directory('vision'))/'config/camera/front_d435i_vpr_lock_tf.yaml')
    declarations = [DeclareLaunchArgument('config', default_value=config)]
    declarations += [DeclareLaunchArgument(key, default_value='',
                      description='Override config rotation (radians) or complete final root xyz (metres)')
                     for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    return LaunchDescription(declarations+[OpaqueFunction(function=configure)])
