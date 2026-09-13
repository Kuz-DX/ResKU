"""offline_controller.launch.py

nav2.launch.py의 offline 버전 -- controller_server + lifecycle_manager만
그대로 재사용하되, mppi_params_file을 launch argument로 노출해서 A/B/C
config를 재빌드 없이 바로 스왑할 수 있게 한다.

odom remap은 nav2.launch.py와 동일(/odometry/filtered)하지만, cmd_vel remap은
'/cmd_vel_auto' 대신 '/offline/cmd_vel'로 바꿨다 -- 혹시 실제 로봇(같은
ROS_DOMAIN_ID)이 켜져 있어도 autonomous.launch.py의 current_ramp_node/
rmd_x8_driver_node가 절대 이 토픽을 소비할 일이 없도록 토픽 이름 자체를
다르게 만든다(motor safety). 이 launch는 그 두 노드도 stability_monitor_node도
전혀 포함하지 않는다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('robot_bringup'), 'config', 'our_mppi_params.yaml')

    mppi_params_file = LaunchConfiguration('mppi_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mppi_params_file', default_value=default_params,
            description='A/B/C 비교용 MPPI 파라미터 yaml 경로'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[mppi_params_file, {'use_sim_time': False}],
            remappings=[
                ('odom', '/odometry/filtered'),
                ('/cmd_vel', '/offline/cmd_vel'),
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),
    ])
