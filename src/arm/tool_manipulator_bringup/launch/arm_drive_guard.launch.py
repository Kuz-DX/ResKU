"""Launch the fail-closed arm drive-safe readiness monitor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pose_name", default_value="drive_safe"),
        DeclareLaunchArgument("srdf_path", default_value=""),
        DeclareLaunchArgument("hardware_path", default_value=""),
        Node(
            package="tool_manipulator_bringup",
            executable="arm_drive_guard.py",
            name="arm_drive_guard",
            output="screen",
            parameters=[{
                "pose_name": LaunchConfiguration("pose_name"),
                "srdf_path": LaunchConfiguration("srdf_path"),
                "hardware_path": LaunchConfiguration("hardware_path"),
            }],
        ),
    ])

