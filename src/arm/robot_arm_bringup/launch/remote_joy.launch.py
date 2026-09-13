"""Publish the shared joystick from the PC that owns the USB device.

Run this launch on the remote/operator PC for arm-only operation.  When the
drive ``manual_control.launch.py`` is already running, do not run this launch;
the drive launch already owns the single ``/joy`` publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device on the remote/operator PC',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen',
        ),
    ])
