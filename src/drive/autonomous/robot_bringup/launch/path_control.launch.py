"""
path_control.launch.py

[2026 사용자 결정, 계절 미션 정리] 원래 path_relay_node(인지팀 /path ->
FollowPath 액션 중계) + slope_traverse_node(경사 대응 상태머신) 둘을
같이 띄웠으나, slope_traverse_node 계열(PlanA~PlanE, 봄 변형 포함)을 전부
삭제하면서 path_relay_node만 남았다 -- MPPI(nav2_mppi_controller)를 나중에
recorded return path 비교용(2단계 평가)으로 쓸 때 FollowPath 액션에 경로를
넣어주는 역할은 계속 필요하기 때문에 path_relay는 보존.

Run reduced_odom_bringup.launch.py + nav2.launch.py first/alongside this
(odom, controller_server 액션 서버 필요). 이 셋을 한 번에 띄우려면
autonomous.launch.py를 쓸 것.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    path_relay_params = os.path.join(
        get_package_share_directory('path_relay'),
        'config', 'path_relay_params.yaml')

    return LaunchDescription([

        # 인지팀 트랙 경로(/path) -> FollowPath 액션 중계
        Node(
            package='path_relay',
            executable='path_relay_node',
            name='path_relay_node',
            parameters=[path_relay_params],
            output='screen',
        ),
    ])
