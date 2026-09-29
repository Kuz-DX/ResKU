"""Run on the operator PC that owns the physical joystick."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "dev": LaunchConfiguration("joy_dev"),
                "deadzone": 0.05,
                "autorepeat_rate": 50.0,
            }],
            output="screen",
        ),
    ])
