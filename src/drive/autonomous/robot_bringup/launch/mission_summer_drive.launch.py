"""
mission_summer_drive.launch.py

여름 미션(구호물자 피킹 구간) 전용 주행 진입점. autonomous.launch.py와
같은 하드웨어 체인(reduced_odom_bringup.launch.py로 모터/IMU/오도메트리 +
current_ramp_node 소프트 전류 램프 + stability_monitor_node 긴급정지)을
그대로 쓴다. nav2/MPPI는 띄우지 않고 summer_supply_drive_node가 인지의
/path를 직접 pure pursuit으로 추종해 직진 방향 오차를 보정한다.

    summer_supply_drive_node --[/cmd_vel_auto]--> current_ramp_node
        -> /cmd_vel -> rmd_x8_driver_node (reduced_odom_bringup.launch.py에서 이미 뜸)

summer_supply_drive_node(robot_bringup)가 로봇팔 picking 명령 +
traffic stop/go 상태머신을 직접 돌리고 /cmd_vel_auto의 유일한 publisher가
된다(상세 상태머신은 summer_supply_drive_node.cpp 상단 docstring 참고).

[2026-09-05] 박스는 시작부터 팔 가동범위 안에 배치한다. 파지 완료 전에는
정지하며 /arm/picking_command를 수신하면 주행한다. 차체 접근 동작은 없다.

인지팀 쪽 mission_summer.launch.py(dolbotz 패키지 -- segmentation/
side_cameras/flat_drive/elevation_map/gradient_map/slope_decision/
summer_traffic/summer_supply)는 이 launch와 별개로
같이 띄워야 한다 -- summer_traffic_node가 발행하는
/mission/summer_traffic/result를 이 launch의 summer_supply_drive_node가
구독하므로, 신호등 정지/진행 반응을 보려면 두 launch를 함께 실행해야 함.
로봇팔 picking 명령 토픽은 summer_supply/ik_node와 연결돼 있다.

[2026-08-27 신규, 사용자 요청] 여름 구간은 경로 추종이 아니라 로봇팔
팀/신호등 신호로만 제어되는 직진 전용 주행이라 별도 launch로 분리.

사용:
    ros2 launch robot_bringup mission_summer_drive.launch.py
    ros2 launch robot_bringup mission_summer_drive.launch.py can_interface:=vcan0  # dry-run
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
    # [2026-08-29] 이 미션의 모든 커스텀 노드 파라미터를 한 파일로 모아뒀다
    # -- robot_bringup/config/mission_summer_drive_params.yaml 참고.
    summer_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'mission_summer_drive_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),

        # 여름 구간 직진 전용 주행 상태머신 -> /cmd_vel_auto (파라미터 다수가
        # summer_supply_drive_node.cpp 참고)
        Node(
            package='robot_bringup',
            executable='summer_supply_drive_node',
            name='summer_supply_drive_node',
            parameters=[summer_params],
            output='screen',
        ),

        # [하림 수정] 소프트 전류 램프 -- 값은 mission_summer_drive_params.yaml 참고.
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[summer_params],
            output='screen',
        ),

        # 구동 모터 통신두절/하드웨어 에러(STALE/ERROR) 감지 -> /cmd_vel_safety
        # (stability_monitor_node.cpp 상단 docstring 참고)
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[summer_params],
            output='screen',
        ),
    ])
