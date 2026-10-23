"""Full separate-joint position-control bringup; defaults to mock hardware."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('real_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('hardware_config', default_value=str(share/'config/hardware.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/manual_control.launch.py')),
            launch_arguments={
                'control_mode': 'separate',
                'real_hardware': LaunchConfiguration('real_hardware'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'hardware_config': LaunchConfiguration('hardware_config'),
            }.items()),
    ])
