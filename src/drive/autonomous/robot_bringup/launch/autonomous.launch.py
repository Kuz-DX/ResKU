"""
autonomous.launch.py

자율주행 최상위 진입점. reduced_odom_bringup.launch.py(Step 2, 옛 이름
ekf.launch.py -- 2026-08-25 개명, 옛 15-state robot_localization EKF는
이미 완전히 제거됐고 지금은 reduced_odom만 띄우므로 실제 내용에 맞게
정리) + nav2.launch.py(Step 3) + path_control.launch.py(Step 4)에 더해,
MPPI 명령을 조이스틱 없이 모터까지 연결하는 체인을 직접 띄운다:

    controller_server --[remap]--> /cmd_vel_auto
        -> current_ramp_node (소프트 전류 램프, 40A)
        -> /cmd_vel
        -> rmd_x8_driver_node (ekf.launch.py에서 이미 뜸)

[2026-08-25] auto_cmd_relay_node 제거. 원래 joy_mux_node의 SHARE 버튼
게이팅을 우회하려고 만든 순수 pass-through(/cmd_vel_auto -> /cmd_vel_auto_raw,
로직 없음)였는데, joy_mux_node 자체가 이미 삭제돼 있어서(바로 아래 문단)
우회할 대상이 없는 죽은 코드가 돼 있었다 -- 의존성 전수 확인(manual 경로
0건) 후 제거, current_ramp_node가 controller_server의 /cmd_vel_auto를
바로 받도록 정리. 소스(`auto_cmd_relay_node.cpp`)와 CMakeLists.txt 빌드
타겟도 같이 제거함 -- 필요해지면(예: 조이스틱 게이팅 다시 도입) git
이력에서 복구.

stability_monitor_node(긴급정지 전담, IMU pitch/roll + 카메라 roll +
과전류/고장 -> /cmd_vel_safety)도 여기서 같이 띄운다. /cmd_vel_safety는
rmd_x8_driver_node가 위 체인과 무관하게 직접 구독하므로 조이스틱 없이도
항상 동작한다.

[하림 수정] manual 조종은 이제 이 패키지가 아니라 manual_joy_control+can_driver
전담이다 (미션이 원격 1회/자율 1회로 완전히 나뉘어서, robot_bringup은 autonomous
전용으로 정리함). joy_node/joy_mux_node/manual_drive.launch.py는 전부 삭제됨.
stability_monitor_node도 manual_joy_control 쪽에 별도로 만들 예정(이 노드와는
독립된 구현).

[2026-08-27] /speed_limit을 발행하던 slope_speed_limiter_node 패키지 삭제
(임계값 미설정으로 오래 실질 무동작 상태였음, path_control.launch.py 참고)
-- 지금은 /speed_limit에 발행자가 없다. 자세 기반 안전 대응은
slope_traverse_node(경사 진입/차동조향탈출/REVERSING/EMERGENCY_REVERSE)가
전담.

사용:
    ros2 launch robot_bringup autonomous.launch.py
    ros2 launch robot_bringup autonomous.launch.py can_interface:=vcan0  # dry-run
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
    # [2026-08-29] 이 launch가 띄우는 모든 커스텀 노드(slope_traverse_node는
    # path_control.launch.py 안에서, 여기 current_ramp_node/
    # stability_monitor_node는 아래에서 각각 로드) 파라미터를 한 파일로
    # 모아뒀다 -- ros2 launch 다시 실행하면 바로 반영(재빌드 불필요).
    autonomous_params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'autonomous_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'reduced_odom_bringup.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'nav2.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'path_control.launch.py'))),

        # [하림 수정] 소프트 전류 램프 -- 자율주행은 별도 인스턴스. 값은
        # robot_bringup/config/autonomous_params.yaml 참고(soft_current_limit_a
        # 등 "정지가 거의 안 걸리게" 완화된 임시값 -- 그 파일 주석 참고).
        Node(
            package='robot_bringup',
            executable='current_ramp_node',
            name='current_ramp_node',
            parameters=[autonomous_params],
            output='screen',
        ),

        # [하림 수정 2026-08-23] 자세/과전류 긴급정지 제거, 구동 모터 통신두절/
        # 하드웨어 에러(STALE/ERROR) 감지만 담당 (stability_monitor_node.cpp
        # 클래스 상단 docstring 참고). 값은 autonomous_params.yaml 참고.
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            parameters=[autonomous_params],
            output='screen',
        ),
    ])
