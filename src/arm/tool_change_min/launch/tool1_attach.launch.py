"""Launch the tool 1 IK, motion executor and state machine.

The arm controller, robot TF, cameras and perception nodes are launched separately.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    poses = LaunchConfiguration("poses_yaml")
    hardware = LaunchConfiguration("hardware_yaml")
    ee_action = LaunchConfiguration("ee_action")
    return LaunchDescription([
        DeclareLaunchArgument("ee_action", default_value="/ee_controller/follow_joint_trajectory"),
        DeclareLaunchArgument("poses_yaml", default_value=PathJoinSubstitution(
            [FindPackageShare("tool_change_min"), "config", "poses.yaml"])),
        DeclareLaunchArgument("hardware_yaml", default_value=PathJoinSubstitution(
            [FindPackageShare("tool_manipulator_bringup"), "config", "hardware.yaml"])),
        # Motion path.  No robot_state_publisher or camera-TF node is here.
        Node(package="tool_change_min", executable="ik_node.py", name="ik_node", output="screen",
             parameters=[{"poses_yaml": poses, "hardware_yaml": hardware}]),
        Node(package="tool_change_min", executable="motion_executor.py", name="motion_executor", output="screen",
             parameters=[{"poses_yaml": poses, "hardware_yaml": hardware, "ee_action": ee_action}]),
        Node(package="tool_change_min", executable="tool_change_fsm.py", name="tool_change_fsm", output="screen",
             parameters=[{"poses_yaml": poses}]),
    ])
