"""Deprecated compatibility entry point for tool_change.launch.py."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'tool_change.launch.py')),
    )])
