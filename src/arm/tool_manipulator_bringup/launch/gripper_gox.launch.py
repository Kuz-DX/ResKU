"""Launch the /arm/gripper_gox Arduino serial bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    topic = LaunchConfiguration('topic')
    reset_delay_sec = LaunchConfiguration('reset_delay_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/GRIPPER_BOX',
            description='Gripper_BOX Arduino serial device',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Gripper_BOX Arduino serial baud rate',
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/gripper/box',
            description='std_msgs/Int32 gripper command topic',
        ),
        DeclareLaunchArgument(
            'reset_delay_sec',
            default_value='4.0',
            description='Delay before publishing command 0 after command 1 or 2',
        ),
        Node(
            package='tool_manipulator_bringup',
            executable='gripper_gox.py',
            output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=str),
                'baud': ParameterValue(baud, value_type=int),
                'topic': ParameterValue(topic, value_type=str),
                'reset_delay_sec': ParameterValue(
                    reset_delay_sec, value_type=float),
            }],
        ),
    ])
