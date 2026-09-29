"""Start the primitive-geometry MoveIt mock plus the tool-ID TCP test node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tool_manipulator_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'mock.launch.py'))),
        Node(
            package='tool_manipulator_bringup',
            executable='tool_dock_tcp_sim_node.py',
            name='tool_dock_tcp_sim',
            output='screen',
            parameters=[os.path.join(share, 'config', 'tool_dock_tcp_sim.yaml')],
        ),
    ])
