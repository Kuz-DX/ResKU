"""
mission_spring_autonomous_drive.launch.py

autonomous_planc.launch.py의 "봄 미션" 변형 -- path_control_planc.launch.py
(PlanC, slope_traverse_planc_node) 대신 path_control_planc_spring.launch.py
(PlanC + STARTUP: slope_traverse_planc_spring_node)를 include한다는 것만
autonomous_planc.launch.py와 다름. mission_spring_drive.launch.py가 PlanD
(slope_traverse_pland_node) 위에 STARTUP을 얹어 slope_traverse_spring_node를
만든 것과 동일한 방식을, nav2/MPPI를 그대로 쓰는 PlanC 스택에 적용한 버전이다
-- nav2/MPPI를 완전히 건너뛰는 mission_spring_drive.launch.py와 달리, 이
launch는 autonomous_planc.launch.py와 동일하게 nav2(controller_server/MPPI)를
그대로 띄운다는 점이 핵심 차이.

STARTUP: 노드 시작 즉시 side/path 등 아무 조건도 안 보고
initial_straight_drive_sec(기본 3초) 동안 무조건 직진(initial_straight_speed_mps,
기본 0.4m/s)한 뒤 IDLE로 넘어가서, 그 이후부터는 PlanC와 완전히 동일하게
동작한다(side 신호 대기 -> SLOPE_DRIVE -> SLOPE_EXIT -> RECOVERY, IDLE
동안은 nav2 MPPI가 이어받음). 배경(mission_spring_drive.launch.py와 동일):
대회 트랙 출발선에서 인지팀 신호/경로가 아직 안정화되기 전에 초반 몇 초는
그냥 밀고 나가야 하는 구간이 있어서(사용자 확인).

나머지(reduced_odom_bringup/nav2/current_ramp_node/stability_monitor_node)는
autonomous_planc.launch.py와 완전히 동일.

Plan들의 SLOPE_EXIT 조향 방식 차이는 slope_traverse_planc_node.cpp 상단
docstring 참고, STARTUP 추가 내용은 slope_traverse_planc_spring_node.cpp
상단 docstring 참고.

사용:
    ros2 launch robot_bringup mission_spring_autonomous_drive.launch.py
    ros2 launch robot_bringup mission_spring_autonomous_drive.launch.py initial_straight_drive_sec:=5.0
    ros2 launch robot_bringup mission_spring_autonomous_drive.launch.py can_interface:=vcan0  # dry-run
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch')
    # [2026-09-05] 값은 robot_bringup/config/mission_spring_autonomous_drive_params.yaml 참고.
    spring_autonomous_params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config',
        'mission_spring_autonomous_drive_params.yaml')

    initial_straight_drive_sec = LaunchConfiguration('initial_straight_drive_sec')
    initial_straight_speed_mps = LaunchConfiguration('initial_straight_speed_mps')

    return LaunchDescription([
        # [CLI] 초반 무조건 직진 구간 -- 재빌드 없이 바로 조정 가능.
        # 기본값은 mission_spring_autonomous_drive_params.yaml 값과 동일하게
        # 맞춰둠(mission_spring_drive.launch.py와 동일한 관례). 실제로는
        # path_control_planc_spring.launch.py에 전달돼 그쪽에서 노드
        # 파라미터로 오버라이드된다.
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'nav2.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'path_control_planc_spring.launch.py')),
            launch_arguments={
                'initial_straight_drive_sec': initial_straight_drive_sec,
                'initial_straight_speed_mps': initial_straight_speed_mps,
            }.items(),
        ),

        # 아래 두 노드는 autonomous_planc.launch.py와 완전히 동일 (모든 Plan 공통).
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[spring_autonomous_params],
            output='screen',
        ),

        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[spring_autonomous_params],
            output='screen',
        ),
    ])
