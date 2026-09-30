"""Run existing vision.apriltag from docking.yaml without duplicating tag settings."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('docking_config', default_value=str(share / 'config' / 'docking.yaml')),
        Node(
            package='vision',
            executable='apriltag',
            name='vision_apriltag',
            output='screen',
            parameters=[LaunchConfiguration('docking_config')],
        ),
    ])
