"""Reviewed tag-1 recorded replay; hardware is NOT started by this launch."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(get_package_share_directory('tool_manipulator_bringup')) / 'config'
    # Existing symlink-install can run the new Python script without a ROS build.
    source = Path(__file__).resolve().parents[1]
    script = source / 'scripts' / 'tag1_recorded_path_node.py'
    if script.is_file():
        config_dir = source / 'config'
    else:
        script = (Path(get_package_prefix('tool_manipulator_bringup')) / 'lib' /
                  'tool_manipulator_bringup' / 'tag1_recorded_path_node.py')
    if not script.is_file():
        raise RuntimeError('recorded path node not installed; run its source Python script explicitly')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config_dir / 'ui_tag1_docking.yaml')),
        DeclareLaunchArgument('recorded_path', default_value=str(config_dir / 'tag1_recorded_path.yaml')),
        DeclareLaunchArgument('hardware_config', default_value=str(config_dir / 'hardware.yaml')),
        DeclareLaunchArgument('tools_config', default_value=str(config_dir / 'tools.yaml')),
        Node(
            executable=sys.executable, arguments=[str(script)],
            name='ui_tag1_docking', output='screen',
            parameters=[LaunchConfiguration('config'), {
                'recorded_path_file': LaunchConfiguration('recorded_path'),
                'hardware_config_file': LaunchConfiguration('hardware_config'),
                'tools_config_file': LaunchConfiguration('tools_config'),
            }],
        ),
    ])
