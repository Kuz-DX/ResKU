"""Tool-change docking stack for the real controller started by tool_change.launch.py.

This launch deliberately owns only MoveIt Servo and the docking nodes.  The
real ros2_control node and move_group are owned by real_control.launch.py, so
there is exactly one owner of the arm hardware and controller actions.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('tool_manipulator_bringup')
    hardware_config = LaunchConfiguration('hardware_config')
    config = (MoveItConfigsBuilder('tool_manipulator', package_name='tool_manipulator_moveit_config')
              .robot_description(mappings={
                  'use_mock_hardware': 'false',
                  'hardware_config_file': hardware_config,
                  'tool_id': '0',
                  'use_mesh': 'true',
              })
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    docking = os.path.join(share, 'config', 'docking.yaml')
    servo = os.path.join(share, 'config', 'servo.yaml')
    with open(servo, encoding='utf-8') as stream:
        servo_params = {'moveit_servo': yaml.safe_load(stream) or {}}
    docking_params = LaunchConfiguration('docking_params')
    tools_config = LaunchConfiguration('tools_config')
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=os.path.join(share, 'config', 'hardware.yaml')),
        DeclareLaunchArgument('docking_params', default_value=docking),
        DeclareLaunchArgument('tools_config', default_value=os.path.join(share, 'config', 'tools.yaml')),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node', output='screen',
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, servo_params]),
        Node(package='tool_manipulator_bringup', executable='tag_pose_filter.py',
             name='tag_pose_filter', output='screen',
             parameters=[docking_params, {'tools_config_file': tools_config}]),
        Node(package='tool_manipulator_bringup', executable='visual_servo_node.py',
             name='visual_servo_node', output='screen', parameters=[docking_params]),
        Node(package='tool_manipulator_bringup', executable='apriltag_tool_docking_node.py',
             name='docking_motion_executor', output='screen',
             parameters=[docking_params, {'tools_config_file': tools_config}]),
        Node(package='tool_manipulator_bringup', executable='docking_manager.py',
             name='docking_manager', output='screen', parameters=[{'tools_config_file': tools_config}]),
        Node(package='tool_manipulator_bringup', executable='drill_tool_change_coordinator.py',
             name='drill_tool_change_coordinator', output='screen'),
        Node(package='tool_manipulator_bringup', executable='tool_scene_manager.py',
             name='tool_scene_manager', output='screen', parameters=[{'tools_config_file': tools_config}]),
    ])
