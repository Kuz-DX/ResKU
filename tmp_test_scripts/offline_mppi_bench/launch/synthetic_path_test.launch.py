"""synthetic_path_test.launch.py

[테스트용] perception 없이, RViz용 합성(synthetic) 경로 하나로 MPPI 폐루프를
검증하기 위한 launch. mock_localization.launch.py(고정 pose)와 달리
sim_diff_drive_odom.py가 controller_server의 cmd_vel을 실제로 적분해서
로봇이 진짜로 움직이는 것처럼 odom<->base_link가 갱신된다.

띄우는 것:
    sim_diff_drive_odom.py  -- /offline/cmd_vel 적분 -> dynamic TF + /odometry/filtered
    path_relay_node         -- /path(합성 경로, odom frame으로 직접 발행됨) -> FollowPath
    offline_controller      -- controller_server + MPPI (mppi_params_file로 A/B/C 스왑 가능)

이 launch 자체는 /path를 아무도 발행하지 않는다 -- 별도 터미널에서
mppi_repro_harness/scripts/synthetic_quarter_arc_path.py(또는
synthetic_sine_path.py)를 topic:=/path로 실행해서 넣어줄 것. 두 스크립트
다 frame_id 기본값이 'odom'이라 path_relay_node의 camera_link->odom TF
변환도 사실상 identity(odom->odom)로 바로 통과된다 -- perception/카메라
자체가 이 테스트에 전혀 관여하지 않는다.

실제 모터는 이 launch에서 전혀 실행하지 않는다 (autonomous.launch.py의
current_ramp_node/rmd_x8_driver_node 없음).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    mppi_params_file = LaunchConfiguration('mppi_params_file')

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'), 'config', 'path_relay_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mppi_params_file',
            default_value=os.path.join(
                get_package_share_directory('robot_bringup'),
                'config', 'our_mppi_params.yaml'),
            description='테스트할 MPPI 파라미터 yaml 경로 (mppi_params_A/B/C.yaml 등)'),

        ExecuteProcess(
            cmd=['python3', os.path.join(_TOOLS_DIR, 'sim_diff_drive_odom.py')],
            output='screen',
        ),

        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_TOOLS_DIR, 'launch', 'offline_controller.launch.py')),
            launch_arguments={'mppi_params_file': mppi_params_file}.items()),
    ])
