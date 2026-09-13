"""공용 YAML에서 RealSense 카메라별 launch 설정을 읽는다."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_launch_value(value) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _include_realsense(context, camera_key: str):
    config_path = Path(LaunchConfiguration('config_file').perform(context))
    with config_path.open(encoding='utf-8') as stream:
        all_camera_configs = yaml.safe_load(stream) or {}

    config = all_camera_configs.get(camera_key)
    if not isinstance(config, dict):
        raise RuntimeError(
            f"'{camera_key}' must be a mapping in RealSense config: {config_path}")

    realsense_launch = (
        Path(get_package_share_directory('realsense2_camera')) / 'launch' / 'rs_launch.py'
    )
    launch_arguments = {
        str(name): _as_launch_value(value)
        for name, value in config.items()
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(realsense_launch)),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_camera_launch_description(camera_key: str) -> LaunchDescription:
    """camera_key에 해당하는 설정으로 RealSense launch를 구성한다."""
    default_config = (
        Path(get_package_share_directory('dolbotz'))
        / 'config'
        / 'realsense_cameras.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=str(default_config),
            description='Drive/arm RealSense launch arguments YAML file',
        ),
        OpaqueFunction(
            function=_include_realsense,
            kwargs={'camera_key': camera_key},
        ),
    ])
