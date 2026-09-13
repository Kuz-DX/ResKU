"""
path_control_planc_spring.launch.py

path_control_planc.launch.py의 봄 미션 전용 변형 -- slope_traverse_planc_node
(PlanC) 대신 slope_traverse_planc_spring_node(PlanC + STARTUP: 노드 시작 즉시
initial_straight_drive_sec 동안 무조건 직진, slope_traverse_planc_spring_node.cpp
상단 docstring 참고)를 띄운다. 그 외(path_relay_node)는
path_control_planc.launch.py와 완전히 동일.

사용법은 path_control_planc.launch.py와 동일 -- reduced_odom_bringup.launch.py +
nav2.launch.py를 먼저/같이 띄워야 함. 이 셋을 한 번에 띄우려면
mission_spring_autonomous_drive.launch.py를 쓸 것(autonomous_planc.launch.py의
봄 미션 버전).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-09-05] mission_spring_autonomous_drive.launch.py(봄 미션 PlanC)
    # 전용 파라미터 -- 이 launch는 mission_spring_autonomous_drive.launch.py
    # 에서만 include되므로, 그쪽 소유의 yaml을 그대로 참조한다.
    spring_autonomous_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'mission_spring_autonomous_drive_params.yaml')

    initial_straight_drive_sec = LaunchConfiguration('initial_straight_drive_sec')
    initial_straight_speed_mps = LaunchConfiguration('initial_straight_speed_mps')

    return LaunchDescription([
        # [CLI] 초반 무조건 직진 구간 -- 재빌드 없이 바로 조정 가능.
        # 기본값은 mission_spring_autonomous_drive_params.yaml 값과 동일하게
        # 맞춰둠(mission_spring_drive.launch.py와 동일한 관례).
        DeclareLaunchArgument(
            'initial_straight_drive_sec',
            default_value='3.0',
            description='STARTUP 상태에서 무조건 직진할 시간(초)',
        ),
        DeclareLaunchArgument(
            'initial_straight_speed_mps',
            default_value='0.4',
            description='STARTUP 상태에서 직진할 속도(m/s)',
        ),

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계 (PlanA/PlanB/PlanC 공통)
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # [봄 미션] PlanC + STARTUP (slope_traverse_planc_spring_node.cpp 참고).
        Node(
            package='robot_bringup',
            executable='slope_traverse_planc_spring_node',
            name='slope_traverse_planc_spring_node',
            parameters=[
                spring_autonomous_params,
                {
                    'initial_straight_drive_sec':
                        ParameterValue(initial_straight_drive_sec, value_type=float),
                    'initial_straight_speed_mps':
                        ParameterValue(initial_straight_speed_mps, value_type=float),
                },
            ],
            output='screen',
        ),
    ])
