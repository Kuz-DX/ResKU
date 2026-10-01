"""Start the TCP MoveToPose node with mock or explicitly requested hardware."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('tool_manipulator_bringup')
    real_hardware = LaunchConfiguration('real_hardware')
    target_poses_yaml = LaunchConfiguration('target_poses_yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'real_hardware', default_value='false',
            description=(
                'true starts real_control and permits MoveIt trajectories to '
                'reach /arm_controller/follow_joint_trajectory.')),
        DeclareLaunchArgument(
            'target_poses_yaml', default_value='{}',
            description=(
                'YAML map of tool ID to a taught TCP pose; empty is fail-safe.')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'mock.launch.py')),
            condition=UnlessCondition(real_hardware)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'real_control.launch.py')),
            condition=IfCondition(real_hardware)),
        Node(
            package='tool_manipulator_bringup',
            executable='tool_dock_tcp_sim_node.py',
            name='move_to_pose',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'tool_dock_tcp_sim.yaml'),
                {'target_poses_yaml': target_poses_yaml},
            ],
        ),
    ])
