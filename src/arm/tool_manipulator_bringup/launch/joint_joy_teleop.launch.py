"""Start single-joint joystick teleoperation on the robot PC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument(
            "joint_jog_topic", default_value="/servo_node/delta_joint_cmds"),
        DeclareLaunchArgument("joint_speed_rad_s", default_value="0.12"),
        DeclareLaunchArgument("manage_focus", default_value="true"),
        Node(
            package="tool_manipulator_bringup",
            executable="joint_joy_teleop.py",
            name="joint_joy_teleop",
            parameters=[{
                "joy_topic": LaunchConfiguration("joy_topic"),
                "joint_jog_topic": LaunchConfiguration("joint_jog_topic"),
                "joint_speed_rad_s": ParameterValue(
                    LaunchConfiguration("joint_speed_rad_s"), value_type=float),
                "manage_focus": ParameterValue(
                    LaunchConfiguration("manage_focus"), value_type=bool),
            }],
            output="screen",
        ),
    ])
