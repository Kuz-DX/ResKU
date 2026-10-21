"""Mock joystick jogging with a live URDF in RViz, without move_group."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("tool_manipulator_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("joint_speed_rad_s", default_value="0.24"),
        DeclareLaunchArgument("check_collisions", default_value="true",
                              description="Set false only for mock collision diagnosis."),
        DeclareLaunchArgument("launch_joy", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "tcp_joy_teleop.launch.py")),
            launch_arguments={
                "real_hardware": "false",
                "use_mesh": "true",
                "mock_joint_teleop": "true",
                "mock_joint_speed_rad_s": LaunchConfiguration("joint_speed_rad_s"),
                "mock_check_collisions": LaunchConfiguration("check_collisions"),
                "launch_joy": LaunchConfiguration("launch_joy"),
            }.items(),
        ),
        # The CLI waits for the service, so no startup sleep is needed.
        ExecuteProcess(
            cmd=["ros2", "service", "call", "/servo_node/start_servo",
                 "std_srvs/srv/Trigger", "{}"], output="screen"),
        Node(
            package="rviz2", executable="rviz2", name="rviz",
            arguments=["-d", str(share / "config" / "joint_rviz.rviz")],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            output="screen",
        ),
    ])
