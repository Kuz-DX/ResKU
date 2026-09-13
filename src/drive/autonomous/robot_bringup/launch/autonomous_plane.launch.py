"""
autonomous_plane.launch.py

autonomous_pland.launch.py의 "PlanE" 변형. path_control_pland.launch.py
(PlanD, slope_traverse_pland_node) 대신 path_control_plane.launch.py
(PlanE, slope_traverse_plane_node -- planD 코드 그대로 + SLOPE_EXIT
slope_exit_side_motor_dps만 400->300)를 include한다는 것만
autonomous_pland.launch.py와 다름. 나머지(reduced_odom_bringup/nav2/
current_ramp_node/stability_monitor_node)는 완전히 동일.

PlanD와의 차이는 slope_traverse_plane_node.cpp 상단 docstring 참고.

사용:
    ros2 launch robot_bringup autonomous_plane.launch.py
    ros2 launch robot_bringup autonomous_plane.launch.py can_interface:=vcan0  # dry-run
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch')
    # [2026-09-04] 값은 robot_bringup/config/autonomous_plane_params.yaml 참고.
    autonomous_plane_params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'autonomous_plane_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'nav2.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'path_control_plane.launch.py'))),

        # 아래 두 노드는 autonomous.launch.py와 완전히 동일 (모든 Plan 공통).
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[autonomous_plane_params],
            output='screen',
        ),

        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[autonomous_plane_params],
            output='screen',
        ),
    ])
