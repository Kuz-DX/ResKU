"""
mission_dog_follow.launch.py

로봇개 추종 미션 전용 주행 진입점. autonomous.launch.py와 같은 하드웨어
체인(reduced_odom_bringup.launch.py로 모터/IMU/오도메트리 + current_ramp_node
소프트 전류 램프 + stability_monitor_node 긴급정지)을 그대로 쓰지만,
nav2.launch.py(MPPI)와 path_control.launch.py(perception 경로 추종/경사
대응)는 아예 안 띄운다 -- 이 미션은 path가 아니라 로봇개 상대위치를 직접
보고 반응하는 target-following 제어라 경로 추종 자체가 필요 없다
(mission_summer_drive.launch.py와 동일한 이유로 동일하게 구성).

    dog_follow_node --[/cmd_vel_auto]--> current_ramp_node
        -> /cmd_vel -> rmd_x8_driver_node (reduced_odom_bringup.launch.py에서 이미 뜸)

dog_follow_node(robot_bringup)가 로봇개 위치(PLACEHOLDER 인터페이스, 파일
상단 docstring 참고)를 구독해서 거리/방향 유지 + 가림막 대응 상태머신을
직접 돌리고 /cmd_vel_auto의 유일한 publisher가 된다(상세 상태머신은
dog_follow_node.cpp 상단 docstring 참고).

인지팀 쪽 로봇개 인식 노드(아직 미확정 -- dog_pose_topic_ 토픽명/메시지
타입 확정되면 이 launch 또는 dog_follow_node 기본 파라미터를 맞춰 조정할
것)는 이 launch와 별개로 같이 띄워야 한다.

[2026-08-29 신규] 로봇개 추종 미션은 path/MPPI가 아니라 로봇개 상대위치
직접 추종이라 별도 launch로 분리.

사용:
    ros2 launch robot_bringup mission_dog_follow.launch.py
    ros2 launch robot_bringup mission_dog_follow.launch.py can_interface:=vcan0  # dry-run
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
    # -- robot_bringup/config/mission_dog_follow_params.yaml 참고.
    dog_follow_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'mission_dog_follow_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),

        # 로봇개 추종 상태머신 -> /cmd_vel_auto (파라미터 다수가 PLACEHOLDER,
        # dog_follow_node.cpp 참고 -- 인지팀 인터페이스 확정/실측 후 조정 필요)
        Node(
            package='robot_bringup',
            executable='dog_follow_node',
            name='dog_follow_node',
            parameters=[dog_follow_params],
            output='screen',
        ),

        # [하림 수정] 소프트 전류 램프 -- 값은 mission_dog_follow_params.yaml 참고.
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[dog_follow_params],
            output='screen',
        ),

        # 구동 모터 통신두절/하드웨어 에러(STALE/ERROR) 감지 -> /cmd_vel_safety
        # (stability_monitor_node.cpp 상단 docstring 참고)
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[dog_follow_params],
            output='screen',
        ),
    ])
