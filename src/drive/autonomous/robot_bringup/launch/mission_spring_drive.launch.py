"""
mission_spring_drive.launch.py

봄 미션(피아식별 구간) 전용 자율주행 진입점 -- [2026-09-05 재작성]
기존 버전(slope_traverse_node/PlanA 재사용)을 지우고, PlanD
(slope_traverse_pland_node.cpp)를 그대로 복사한 전용 노드
slope_traverse_spring_node로 교체했다. PlanD 위에 STARTUP 상태 하나만
추가돼 있다 -- 노드 시작 즉시 side/path 등 아무 조건도 안 보고
initial_straight_drive_sec(기본 3초)초 동안 무조건 직진한 뒤 IDLE로
넘어가서, 그 이후부터는 PlanD와 완전히 동일하게 동작한다(side 신호 대기
-> SLOPE_DRIVE -> SLOPE_EXIT -> RECOVERY). 배경: 대회 트랙 출발선에서
인지팀 신호/경로가 아직 안정화되기 전에 초반 몇 초는 그냥 밀고 나가야
하는 구간이 있어서(사용자 확인).

MPPI(controller_server)/path_relay_node/current_ramp_node는 이전 버전과
동일하게 안 띄운다 -- slope_traverse_spring_node(PlanD와 동일)는 nav2에
코드 레벨로 전혀 의존하지 않고 /cmd_vel_safety 하나로 직접 구동한다
(자세한 근거는 git 이력의 이전 mission_spring_drive.launch.py 참고).

노드 구성:
    reduced_odom_bringup.launch.py (모터/IMU/오도메트리, rmd_x8_driver_node 포함)
    slope_traverse_spring_node (PlanD + STARTUP, mission_spring_drive_params.yaml)
    stability_monitor_node (통신두절/하드웨어 에러 감지)

사용:
    ros2 launch robot_bringup mission_spring_drive.launch.py
    ros2 launch robot_bringup mission_spring_drive.launch.py initial_straight_drive_sec:=5.0
    ros2 launch robot_bringup mission_spring_drive.launch.py can_interface:=vcan0  # dry-run
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch')
    spring_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'mission_spring_drive_params.yaml')

    initial_straight_drive_sec = LaunchConfiguration('initial_straight_drive_sec')
    initial_straight_speed_mps = LaunchConfiguration('initial_straight_speed_mps')

    return LaunchDescription([
        # [CLI] 초반 무조건 직진 구간 -- 재빌드 없이 바로 조정 가능.
        # 기본값은 mission_spring_drive_params.yaml 값과 동일하게 맞춰둠.
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

        # PlanD(slope_traverse_pland_node) + STARTUP 상태 추가 버전.
        # parameters 리스트에서 yaml 다음에 dict를 얹으면 그 안의 키만
        # 오버라이드된다(뒤에 오는 값이 우선) -- CLI 인자를 yaml 기본값
        # 위에 덮어씀. [주의] LaunchConfiguration은 항상 문자열로 해석되므로
        # float로 선언된 파라미터에 그냥 넘기면 타입이 안 맞아 예외가 난다
        # (autonomous.planz.launch.py에서 이미 겪은 문제) --
        # ParameterValue(value_type=float)로 명시 변환.
        Node(
            package='robot_bringup',
            executable='slope_traverse_spring_node',
            name='slope_traverse_spring_node',
            parameters=[
                spring_params,
                {
                    'initial_straight_drive_sec':
                        ParameterValue(initial_straight_drive_sec, value_type=float),
                    'initial_straight_speed_mps':
                        ParameterValue(initial_straight_speed_mps, value_type=float),
                },
            ],
            output='screen',
        ),

        # 구동 모터 통신두절/하드웨어 에러(STALE/ERROR) 감지 -> /cmd_vel_safety
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[spring_params],
            output='screen',
        ),
    ])
