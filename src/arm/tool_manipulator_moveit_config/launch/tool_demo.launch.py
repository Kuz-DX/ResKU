"""Launch the MoveIt demo with one static visualization/planning tool model.

This is for offline inspection only.  Runtime tool changes still use the
planning-scene attachment manager and do not restart MoveIt.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    tool_id = LaunchConfiguration('tool_id')
    moveit_config = (
        MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
        .robot_description(mappings={'tool_id': tool_id, 'use_mesh': 'true'})
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_id', default_value='0',
            description='Static model for offline MoveIt inspection: 0, 1, 2, or 3.'),
        *generate_demo_launch(moveit_config).entities,
    ])
