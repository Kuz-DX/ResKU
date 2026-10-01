"""Start the TCP MoveToPose node with mock or explicitly requested hardware."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('tool_manipulator_bringup')
    real_hardware = LaunchConfiguration('real_hardware')
    return LaunchDescription([
        DeclareLaunchArgument(
            'real_hardware', default_value='false',
            description=(
                'true starts real_control and permits MoveIt trajectories to '
                'reach /arm_controller/follow_joint_trajectory.')),
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
                {'execute': ParameterValue(real_hardware, value_type=bool)},
            ],
        ),
    ])
