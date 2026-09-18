"""
autonomous.launch.py

[2026 사용자 결정, 경량화 + 계절 미션 정리] MPPI 자율주행 진입점.
reduced_odom_bringup.launch.py(Step 2) + nav2.launch.py(Step 3,
controller_server/MPPI) + path_control.launch.py(Step 4, path_relay_node)만
남기고, current_ramp_node(전류 제한 램프)와 stability_monitor_node(IMU/모터
통신 긴급정지)는 삭제했다 -- 소스 자체를 제거함(git 이력에서 복구 가능).
계절별 미션 전용 노드(slope_traverse_*, dog_follow_node,
summer_supply_drive_node)와 그 launch/config도 전부 삭제됨 -- 지금은
manual+return 미션(robot_bringup/launch/manual_return_bringup.launch.py)만
운용하고, MPPI는 나중에 recorded return path와 비교하는 2단계 평가용으로만
소스를 보존한다(nav2_mppi_controller/our_mppi_critics/path_relay).

current_ramp_node 제거로 controller_server의 출력은 더 이상 /cmd_vel_auto로
우회하지 않고 곧장 /cmd_vel로 나간다(nav2.launch.py 참고) -- manual+return
미션의 drive_cmd_mux_node/rmd_x8_driver_node와 이 launch를 절대 동시에
띄우지 말 것(같은 CAN 버스를 두고 충돌).

사용:
    ros2 launch robot_bringup autonomous.launch.py
    ros2 launch robot_bringup autonomous.launch.py can_interface:=vcan0  # dry-run
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch')

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
    ])
