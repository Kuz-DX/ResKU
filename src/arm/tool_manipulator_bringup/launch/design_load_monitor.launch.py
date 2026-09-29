from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='tool_manipulator_bringup',
                                   executable='design_load_monitor.py', output='screen')])
