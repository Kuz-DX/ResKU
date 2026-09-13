"""
path_control_blend.launch.py

path_control.launch.py의 "PlanB" 변형 -- slope_traverse_node(PlanA, SLOPE_EXIT
에서 TF 기반 pure pursuit) 대신 slope_traverse_blend_node(PlanB, TF 없이
/path의 raw 좌표로 동일한 pure pursuit 곡률식을 쓰는 버전)를 띄운다. 두
노드의 유일한 차이는 slope_traverse_blend_node.cpp 상단 docstring 참고.

path_relay_node는 PlanA/PlanB 공통으로 그대로 띄운다(MPPI FollowPath 경로 중계는
동일하게 필요).

사용법은 path_control.launch.py와 동일 -- reduced_odom_bringup.launch.py +
nav2.launch.py를 먼저/같이 띄워야 함. 이 셋을 한 번에 띄우려면
autonomous_blend.launch.py를 쓸 것(autonomous.launch.py의 PlanB 버전).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-08-29] autonomous_blend.launch.py(PlanB) 전용 파라미터 -- 이
    # launch는 autonomous_blend.launch.py에서만 include되므로, 그쪽 소유의
    # yaml을 그대로 참조한다.
    autonomous_blend_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'autonomous_blend_params.yaml')

    return LaunchDescription([

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계 (PlanA/PlanB 공통)
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # [PlanB] SLOPE_EXIT에서 TF 없이 pure pursuit 곡률식을 쓰는 대안
        # 노드 (slope_traverse_blend_node.cpp 참고).
        Node(
            package='robot_bringup',
            executable='slope_traverse_blend_node',
            name='slope_traverse_blend_node',
            parameters=[autonomous_blend_params],
            output='screen',
        ),
    ])
