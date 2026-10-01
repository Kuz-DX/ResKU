"""Tool 1 attachment path with the shared vision camera bringup.

The arm controller and robot TF remain external.  Camera drivers are included
from vision/camera/launch/cameras.launch.py so the perception nodes receive
their configured arm-camera topics from the same top-level launch.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        # Own the camera drivers through the existing vision launch.  It starts
        # the configured arm RealSense as well as its other declared cameras.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("vision"), "launch", "cameras.launch.py"])])),
        # Existing source, with the center output named explicitly for this path.
        Node(package="vision", executable="apriltag", name="apriltag_node", output="screen",
             parameters=[{"centers_topic": "/arm/apriltag/centers", "tag_size_cm": 2.0}]),
        # Existing source; it consumes the externally-owned arm TF tree.
        Node(package="vision", executable="supply", name="supply_node", output="screen"),
        # Motion path.  No robot_state_publisher or camera-TF node is here.
        Node(package="tool_change_min", executable="ik_node.py", name="ik_node", output="screen",
             parameters=[{"poses_yaml": poses, "hardware_yaml": hardware}]),
        Node(package="tool_change_min", executable="motion_executor.py", name="motion_executor", output="screen",
             parameters=[{"poses_yaml": poses, "hardware_yaml": hardware, "ee_action": ee_action}]),
        Node(package="tool_change_min", executable="tool_change_fsm.py", name="tool_change_fsm", output="screen",
             parameters=[{"poses_yaml": poses}]),
    ])
