"""Launch only the taught-path commissioning UI; real_control must run separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("tool_manipulator_bringup")) / "config"
    return LaunchDescription([
        DeclareLaunchArgument("sequence", default_value="tagid0_gripper"),
        DeclareLaunchArgument("paths", default_value=str(config / "manual_docking_paths.yaml")),
        DeclareLaunchArgument("hardware", default_value=str(config / "hardware.yaml")),
        Node(
            package="tool_manipulator_bringup",
            executable="manual_docking_path_ui.py",
            output="screen",
            arguments=[
                "--sequence", LaunchConfiguration("sequence"),
                "--paths", LaunchConfiguration("paths"),
                "--hardware", LaunchConfiguration("hardware"),
            ],
        ),
    ])
