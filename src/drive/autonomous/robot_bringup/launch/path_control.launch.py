"""
path_control.launch.py

Perception-fed path following + slope traversal. Step 4 of the
doldrive_ws pipeline:
    path_relay_node          -- /path (인지팀 track centerline) -> FollowPath 액션
    slope_traverse_node      -- 인지팀 signed left/right 경사 신호 ->
                                 정렬(ALIGNING)-직진(CLIMBING)-차동조향탈출
                                 (EXITING)-안정화(RECOVERY) 상태머신, /cmd_vel_safety
                                 최우선 발행. MPPI/perception을 완전히 우회
                                 (slope_traverse_node.cpp 상단 docstring 참고).

두 노드 다 pure "제안/중계"만 하고 직접 모터를 제어하지 않는다 --
controller_server(nav2.launch.py, Step 3)가 최종적으로 소비한다.

Run reduced_odom_bringup.launch.py + nav2.launch.py first/alongside this (odom, controller_server
액션 서버, /imu 필요). 이 셋을 한 번에 띄우려면 autonomous.launch.py를 쓸 것.

[하림 수정] path_relay/slope_speed_limiter 둘 다 지금까지 launch 파일이 없어서
`ros2 run`으로 수동 실행해야 했음 -- 이번에 처음 launch로 묶음.
[2026-08-26] imu_slope_mode_node -> slope_traverse_node로 교체(완전 대체,
imu_slope_mode_node는 삭제). 기능 1(IMU roll -> /drive/mode_command)도 함께
삭제 -- slope_decision.py는 이제 force_mode/auto(depth 기반 자체 판단)만
사용. 대신 인지팀이 새로 발행할 signed left/right 신호를 받아 진입 헤딩을
직접 정하고, 탈출은 IMU 경사 크기 대신 /path 곡률(20도)로 판단.
[2026-08-27] slope_speed_limiter_node 패키지 전체 삭제 -- roll/pitch 임계값
테이블이 2026-08-18(EKF 발산 대응)부터 빈 배열이라 실질적으로 아무 동작도
안 하는 상태로 방치돼 있었음(AxisLimiter가 항상 100%=무제한 반환). 그 뒤로
vx_max 인하, PreferForwardCritic, slope_traverse_node(REVERSING/
EMERGENCY_REVERSE 포함)까지 안전장치가 계속 쌓여서 이 노드가 채울 여백이
줄었다고 판단, 실측 전 상태로 방치하느니 제거(사용자 확인). 필요해지면 git
이력에서 패키지 통째로 복구 가능.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')
    # [2026-08-29] autonomous.launch.py(PlanA) 전용 파라미터 -- 이 launch는
    # autonomous.launch.py에서만 include되므로, 그쪽 소유의 yaml을 그대로
    # 참조한다(robot_bringup/config/autonomous_params.yaml).
    autonomous_params = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'autonomous_params.yaml')

    return LaunchDescription([

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        # 인지팀 signed left/right 경사 신호 -> SLOPE_DRIVE/SLOPE_EXIT(pure
        # pursuit) 상태머신 -> /cmd_vel_safety (파라미터 다수가 PLACEHOLDER,
        # slope_traverse_node.cpp 참고 -- 실측 후 조정 필요. slope_side_topic
        # 인터페이스는 인지팀과 아직 미확정)
        Node(
            package='robot_bringup',
            executable='slope_traverse_node',
            name='slope_traverse_node',
            parameters=[autonomous_params],
            output='screen',
        ),
    ])
