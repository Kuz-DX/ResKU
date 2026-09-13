"""
reduced_odom_validation_robot.launch.py -- ROBOT PC 전용.

조이스틱은 Bluetooth로 LOCAL PC에 물려있고 로봇 PC엔 joystick device가
없다(실측 확인됨) -- 그래서 joy_node/teleop_twist_joy는 이 launch에 없다.
LOCAL PC 쪽은 reduced_odom_validation_teleop.launch.py에서 별도로 띄우고,
ROS2 DDS 네트워크로 /cmd_vel이 여기까지 넘어온다.

    [LOCAL PC]  joy_node -> teleop_twist_joy_node -> /cmd_vel ---(DDS)---+
                                                                          v
    [ROBOT PC]  reduced_odom_bringup.launch.py(rmd_x8_driver+myahrs+static TF+reduced_odom)
                + stability_monitor_node(모터 통신두절/에러 안전장치)

reduced_odom_bringup.launch.py는 원래부터 조이스틱/네트워크 토폴로지와 무관해서(수정 없이)
그대로 include만 한다. stability_monitor_node만 추가 -- autonomous.launch.py가
reduced_odom_bringup.launch.py와 별도로 띄우는 것과 동일한 이유(기존 하드웨어 안전장치
재사용, 파라미터 없음).

manual can_driver_node/manual_joy_control_node는 여기서도 절대 같이 띄우지
않는다(CAN/motor ID 충돌, VALIDATION.md §-1 참고).

사용 (ROBOT PC에서):
    ros2 launch src/drive/autonomous/reduced_odom/launch/reduced_odom_validation_robot.launch.py

    # CAN dry-run: can_interface:=vcan0 추가

기동 후 확인 (ROBOT PC):
    ros2 node list                # can_driver/manual_joy_control/nav2 없어야 함
    ros2 topic info /cmd_vel -v   # LOCAL PC의 teleop_twist_joy_node publisher가
                                   # discovery되는지 확인 (안 보이면 ROS_DOMAIN_ID/
                                   # ROS_LOCALHOST_ONLY/RMW_IMPLEMENTATION부터 점검)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    imu_port = LaunchConfiguration('imu_port')

    odom_launch_dir = os.path.join(
        get_package_share_directory('robot_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value='can_drive',
            description="reduced_odom_bringup.launch.py로 그대로 전달 (실물 'can_drive', dry-run 'vcan0')",
        ),
        DeclareLaunchArgument(
            'imu_port', default_value='/dev/ttyACM0',
            description='reduced_odom_bringup.launch.py로 그대로 전달',
        ),

        # sensors + rmd_x8_driver + reduced_odom (production과 완전히 동일,
        # 수정 없이 그대로 include)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(odom_launch_dir, 'reduced_odom_bringup.launch.py')),
            launch_arguments={
                'can_interface': can_interface,
                'imu_port': imu_port,
            }.items(),
        ),

        # 기존 하드웨어 안전장치 재사용 (모터 통신두절/에러 시 /cmd_vel_safety로
        # 정지 -- 파라미터 없음, autonomous.launch.py와 동일하게 그대로 씀).
        Node(
            package='robot_bringup',
            executable='stability_monitor_node',
            name='stability_monitor_node',
            output='screen',
        ),
    ])
