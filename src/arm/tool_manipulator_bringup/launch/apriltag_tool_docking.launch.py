"""Start MoveIt Servo and the AprilTag two-stage docking controller.

This launch expects move_group, ros2_control, robot_state_publisher and the
AprilTag detector to already be running (e.g. from the MoveIt demo/bringup).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    share = get_package_share_directory('tool_manipulator_bringup')
    config = (MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
              .robot_description(mappings={'use_mock_hardware': 'true', 'use_mesh': 'true'})
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    params = os.path.join(share, 'config', 'docking.yaml')
    tools = os.path.join(share, 'config', 'tools.yaml')
    servo = os.path.join(share, 'config', 'servo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('docking_params', default_value=params),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node', output='screen',
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, servo]),
        Node(package='tool_manipulator_bringup', executable='apriltag_tool_docking_node.py',
             name='apriltag_tool_docking', output='screen',
             parameters=[LaunchConfiguration('docking_params'), {'tools_config_file': tools}]),
        Node(package='tool_manipulator_bringup', executable='tool_attachment_manager.py',
             name='tool_attachment_manager', output='screen',
             parameters=[{'tools_config_file': tools}]),
    ])
