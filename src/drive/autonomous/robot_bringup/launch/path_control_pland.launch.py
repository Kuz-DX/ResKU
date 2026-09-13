"""
path_control_pland.launch.py

path_control_planc.launch.py의 "PlanD" 변형 -- slope_traverse_planc_node
대신 slope_traverse_pland_node(planC 코드 그대로 + SLOPE_DRIVE의 side==0->IDLE
폴백만 제거한 노드, SLOPE_EXIT/RECOVERY를 반드시 거쳐야 IDLE 복귀)를
띄운다. 차이는 slope_traverse_pland_node.cpp 상단 docstring 참고.

path_relay_node는 PlanA/PlanB/PlanC/PlanD 공통으로 그대로 띄운다(MPPI
FollowPath 경로 중계는 동일하게 필요).

사용법은 path_control.launch.py와 동일 -- reduced_odom_bringup.launch.py +
nav2.launch.py를 먼저/같이 띄워야 함. 이 셋을 한 번에 띄우려면
autonomous_pland.launch.py를 쓸 것.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-09-05] autonomous_pland.launch.py(PlanD) 전용 파라미터 -- 이
    # launch는 autonomous_pland.launch.py에서만 include되므로, 그쪽 소유의
    # yaml을 그대로 참조한다.
    autonomous_pland_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'autonomous_pland_params.yaml')

    return LaunchDescription([

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계 (PlanA/B/C/D 공통)
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # [PlanD] planC 그대로 + SLOPE_DRIVE의 side==0->IDLE 폴백만 제거한 노드
        # (slope_traverse_pland_node.cpp 참고).
        Node(
            package='robot_bringup',
            executable='slope_traverse_pland_node',
            name='slope_traverse_pland_node',
            parameters=[autonomous_pland_params],
            output='screen',
        ),
    ])
