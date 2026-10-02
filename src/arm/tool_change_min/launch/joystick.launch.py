"""Manual arm control; controllers and the joystick publisher run separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution(
            [FindPackageShare("tool_change_min"), "config", "joystick.yaml"])),
        DeclareLaunchArgument("hardware_yaml", default_value=PathJoinSubstitution(
            [FindPackageShare("tool_manipulator_bringup"), "config", "hardware.yaml"])),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("arm_action", default_value="/arm_controller/follow_joint_trajectory"),
        Node(package="tool_change_min", executable="joystick_manipulator.py",
             name="joystick_manipulator", output="screen", parameters=[
                 LaunchConfiguration("params_file"),
                 {"hardware_yaml": LaunchConfiguration("hardware_yaml"),
                  "joy_topic": LaunchConfiguration("joy_topic"),
                  "arm_action": LaunchConfiguration("arm_action")},
             ]),
    ])
