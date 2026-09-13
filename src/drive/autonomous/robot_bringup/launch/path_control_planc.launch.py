"""
path_control_planc.launch.py

path_control.launch.py의 "PlanC" 변형 -- slope_traverse_node(PlanA, SLOPE_EXIT
에서 TF 기반 pure pursuit) 대신 slope_traverse_planc_node(PlanC, 커밋
c5aef4d1b0c374abda2a688b04c020ff34b62e8a 시점의 옛 SLOPE_EXIT -- kp_exit_
고정 게인 P제어 + 400/150dps 기준 wz 상한)를 띄운다. 세 노드의 차이는
slope_traverse_planc_node.cpp 상단 docstring 참고.

path_relay_node는 PlanA/PlanB/PlanC 공통으로 그대로 띄운다(MPPI FollowPath
경로 중계는 동일하게 필요).

사용법은 path_control.launch.py와 동일 -- reduced_odom_bringup.launch.py +
nav2.launch.py를 먼저/같이 띄워야 함. 이 셋을 한 번에 띄우려면
autonomous_planc.launch.py를 쓸 것(autonomous.launch.py의 PlanC 버전).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-09-04] autonomous_planc.launch.py(PlanC) 전용 파라미터 -- 이
    # launch는 autonomous_planc.launch.py에서만 include되므로, 그쪽 소유의
    # yaml을 그대로 참조한다.
    autonomous_planc_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'autonomous_planc_params.yaml')

    return LaunchDescription([

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계 (PlanA/PlanB/PlanC 공통)
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # [PlanC] 커밋 c5aef4d 시점의 옛 SLOPE_EXIT(고정 게인 P제어) 노드
        # (slope_traverse_planc_node.cpp 참고).
        Node(
            package='robot_bringup',
            executable='slope_traverse_planc_node',
            name='slope_traverse_planc_node',
            parameters=[autonomous_planc_params],
            output='screen',
        ),
    ])
