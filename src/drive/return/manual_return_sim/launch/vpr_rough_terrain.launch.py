"""Synthetic camera/terrain integration demo. Domain 109, NO real CAN/IMU drivers."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('manual_return_sim'))
    def isolated(context):
        if os.environ.get('ROS_DOMAIN_ID') != '109':
            raise RuntimeError('Run this automatic simulation in isolated ROS_DOMAIN_ID=109 only.')
        return []
    return LaunchDescription([
        DeclareLaunchArgument('auto_start',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('viewer',default_value='true'),
        DeclareLaunchArgument('apply_after_debug',default_value='true'),
        OpaqueFunction(function=isolated),
        Node(package='manual_return_sim',executable='virtual_vpr_rig',output='screen',parameters=[{
            'auto_start':ParameterValue(LaunchConfiguration('auto_start'),value_type=bool),
            'resume_apply_correction':ParameterValue(LaunchConfiguration('apply_after_debug'),value_type=bool)}]),
        Node(package='drive_cmd_mux',executable='drive_cmd_mux_node',output='screen',parameters=[{'manual_topic':'/motor_speed_cmd_virtual'}]),
        Node(package='return_navigation',executable='manual_path_recorder_node',output='screen'),
        Node(package='return_navigation',executable='return_state_machine_node',output='screen'),
        Node(package='return_navigation',executable='return_path_follower_node',output='screen'),
        Node(package='visual_landmark',executable='visual_landmark_node',output='screen',parameters=[{
            'enable_auto_keyframes':True,'auto_keyframe_distance_m':100.,
            'auto_keyframe_yaw_deg':180.,'auto_keyframe_max_interval_s':100.}]),
        Node(package='manual_return_sim',executable='vpr_localizer_probe',output='screen',parameters=[{
            'publish_debug_matches':True,'enable_stationary_vpr_debug':True,'apply_correction':False,
            'enable_adaptive_visual_check':True}]),
        Node(package='manual_return_sim',executable='vpr_scheduler_probe',output='screen',parameters=[{
            'enable_slip_early_trigger':False,'visual_check_distance_m':.5,'visual_check_on_auto_resume':True}]),
        Node(package='manual_return_sim',executable='sim_debug_viz',output='screen'),
        Node(package='visual_landmark',executable='vpr_viewer',condition=IfCondition(LaunchConfiguration('viewer'))),
        Node(package='rviz2',executable='rviz2',name='vpr_sim_rviz',condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d',str(share/'config/vpr_rough_terrain.rviz')]),
    ])
