"""UI tag-1 approach state machine; run beside real control and MoveIt Servo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(get_package_share_directory('tool_manipulator_bringup')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config_dir / 'ui_tag1_docking.yaml')),
        Node(
            package='tool_manipulator_bringup', executable='ui_tag1_docking_node.py',
            name='ui_tag1_docking', output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
