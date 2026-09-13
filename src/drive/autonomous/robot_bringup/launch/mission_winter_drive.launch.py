"""
mission_winter_drive.launch.py

겨울 미션(윤활제 도포된 철 트랙 경사 구간) 전용 자율주행 진입점.
autonomous.launch.py(PlanA -- MPPI + path_relay_node + slope_traverse_node)와
노드 구성은 완전히 동일하다. 유일한 차이는 slope_traverse_node의 SLOPE_EXIT
wz 안전 상한 기준(slope_exit_wz_base_side_dps/other_dps)을 이 launch에서
파라미터로 오버라이드한다는 것뿐이다.

[2026-08-29 신규, 사용자 요청] 배경: slope_exit_wz_base_side_dps/other_dps
기본값(300/200)은 사용자가 실제 주행으로 검증한 "일반 트랙 기준 안정적인
차동조향" 값이었는데, 이후 "path가 원하는 회전을 더 잘 따라가게 해달라"는
요청으로 기본값을 500/0(wz 상한 ~2.0rad/s, 이론상 pure pursuit 최대치보다
여유를 둔 값)까지 올렸다(slope_traverse_node.cpp 참고). 그런데 겨울 미션은
윤활제가 발린 철 트랙이라 마찰력이 훨씬 낮고, 마찰력이 낮을수록 슬립 없이
버틸 수 있는 안전 차동 한계도 낮아진다(Coulomb 마찰 F_max=μN) -- 즉 500/0
같은 공격적인 차동은 이 표면에서 오히려 슬립을 유발할 위험이 크다.
아직 이 표면에서의 안전 한계를 실측하지 못한 상태라, 사용자가 실측으로
검증한 유일한 값인 300/200(일반 표면 기준)을 그대로 가져와 "조심스럽게"
쓴다 -- 500/0보다는 훨씬 보수적이지만, 실측 전까지의 최선 추정치일 뿐이므로
겨울 트랙 실측 후 이 값도 재검증 필요.

slope_exit_speed_(vx, 기본 0.4m/s)나 SLOPE_DRIVE의 kp_steer_/wz_steer_max_는
이번 요청 범위 밖이라 건드리지 않음 -- 필요해지면 이 launch의 파라미터
딕셔너리에 추가할 것.

노드 구성 (autonomous.launch.py와 동일):
    reduced_odom_bringup.launch.py (모터/IMU/오도메트리)
    nav2.launch.py (MPPI controller_server)
    path_relay_node (/path -> FollowPath 중계)
    slope_traverse_node (PlanA, pure pursuit -- 이 launch에서 wz 클램프만 오버라이드)
    current_ramp_node (소프트 전류 램프)
    stability_monitor_node (통신두절/하드웨어 에러 감지)

사용:
    ros2 launch robot_bringup mission_winter_drive.launch.py
    ros2 launch robot_bringup mission_winter_drive.launch.py can_interface:=vcan0  # dry-run
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
    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-08-29] 이 미션의 모든 커스텀 노드 파라미터를 한 파일로 모아뒀다
    # -- robot_bringup/config/mission_winter_drive_params.yaml 참고(겨울
    # 트랙 전용 SLOPE_EXIT wz 오버라이드 포함).
    winter_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'mission_winter_drive_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'nav2.launch.py'))),

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계 (autonomous.launch.py의
        # path_control.launch.py와 동일, 파라미터 오버라이드 없음)
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # PlanA(pure pursuit) 그대로 쓰되, SLOPE_EXIT wz 안전 상한만 겨울
        # 트랙(윤활제 철판, 저마찰) 기준으로 보수적으로 낮춘다 -- 값은
        # mission_winter_drive_params.yaml 참고.
        Node(
            package='robot_bringup',
            executable='slope_traverse_node',
            name='slope_traverse_node',
            parameters=[winter_params],
            output='screen',
        ),

        # 소프트 전류 램프 (autonomous.launch.py와 동일 구성)
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[winter_params],
            output='screen',
        ),

        # 구동 모터 통신두절/하드웨어 에러(STALE/ERROR) 감지 -> /cmd_vel_safety
        # (autonomous.launch.py와 동일 구성)
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[winter_params],
            output='screen',
        ),
    ])
