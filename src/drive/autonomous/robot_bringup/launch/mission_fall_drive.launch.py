"""
mission_fall_drive.launch.py

가을 미션(비전마커 순차 인식 구간) 전용 자율주행 진입점 -- [2026-09-01 신규]
autonomous.launch.py(PlanA)에서 MPPI(controller_server)/path_relay_node를
아예 뺀 경량화 버전. mission_spring_drive.launch.py와 노드 구성이 완전히
동일하다 -- 자세한 근거(slope_traverse_node.cpp가 nav2에 코드 레벨로
의존하지 않는다는 것 등)는 그쪽 모듈 docstring 참고, 여기서는 중복 설명
생략.

[중요] config/mission_fall_drive_params.yaml(fall_marker_node 전용 파라미터
문서)과는 무관하다 -- 그 파일은 인지 노드(fall_marker_node, mission_fall.launch.py가
띄움) 파라미터를 정리한 문서이지 이 launch(구동)와는 별개다. 그 파일 상단에
"가을 구간 실제 주행 속도는 controller_server(MPPI)의 vx_max"라고 적혀
있는데, [2026-09-01] 이 launch가 MPPI를 아예 안 쓰게 되면서 그 설명은
더 이상 맞지 않는다 -- 이제 가을 구간 속도는 slope_traverse_node의
slope_exit_speed_/SLOPE_DRIVE 관련 파라미터(autonomous_params.yaml)가
정한다. 그 yaml 파일 주석도 같이 갱신해둘 것.

노드 구성 (mission_spring_drive.launch.py와 동일):
    reduced_odom_bringup.launch.py (모터/IMU/오도메트리, rmd_x8_driver_node 포함)
    slope_traverse_node (PlanA와 동일 노드/파라미터, autonomous_params.yaml 그대로 사용)
    stability_monitor_node (통신두절/하드웨어 에러 감지, 안전 필수라 유지)

사용:
    ros2 launch robot_bringup mission_fall_drive.launch.py
    ros2 launch robot_bringup mission_fall_drive.launch.py can_interface:=vcan0  # dry-run
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
    autonomous_params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'autonomous_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),

        Node(
            package='robot_bringup',
            executable='slope_traverse_node',
            name='slope_traverse_node',
            parameters=[autonomous_params],
            output='screen',
        ),

        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[autonomous_params],
            output='screen',
        ),
    ])
