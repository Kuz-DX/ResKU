"""
reduced_odom_validation_teleop.launch.py -- LOCAL PC 전용.

Bluetooth joystick이 물려있는 PC(로봇 PC 아님, 실측 확인됨)에서 띄운다.
rmd_x8_driver/myahrs/reduced_odom은 전혀 안 띄우고(그건 로봇 PC의
reduced_odom_validation_robot.launch.py가 담당) /cmd_vel만 ROS2 DDS
네트워크로 로봇 PC까지 내보낸다.

    Joystick(/dev/input/js0, Bluetooth)
      -> joy_node (pkg 'joy')
      -> teleop_twist_joy_node (pkg 'teleop_twist_joy')
      -> /cmd_vel (geometry_msgs/Twist) ---(DDS, 네트워크 조건 아래 참고)---> 로봇 PC의 rmd_x8_driver_node

axis/버튼 매핑, 속도 상한은 manual_joy_control_node.py 실측 매핑 +
todrive.md/our_mppi_params.yaml 첫 실차 시험값을 그대로 재사용(추측/변경
없음) -- 이전 reduced_odom_validation.launch.py에 있던 것과 동일한 값.

사용 전 필수 확인 (LOCAL PC/ROBOT PC 둘 다 동일해야 함):
    echo $ROS_DOMAIN_ID $ROS_LOCALHOST_ONLY $RMW_IMPLEMENTATION
    (ROS_LOCALHOST_ONLY=0 또는 미설정 -- 1이면 이 PC 안에서만 통신됨)

사용 (LOCAL PC에서):
    ros2 launch src/drive/autonomous/reduced_odom/launch/reduced_odom_validation_teleop.launch.py

기동 후 확인 (LOCAL PC):
    ros2 topic echo /joy       # 조이스틱 raw 입력 확인
    ros2 topic echo /cmd_vel   # teleop 출력이 실제로 나오는지 확인

기동 후 확인 (ROBOT PC, 이 PC가 아님 -- reduced_odom_validation_robot.launch.py
띄운 쪽에서):
    ros2 topic info /cmd_vel -v   # 이 PC(LOCAL)의 teleop_twist_joy_node publisher가
                                   # 보여야 discovery 성공
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    max_angular_radps = LaunchConfiguration('max_angular_radps')
    enable_button = LaunchConfiguration('enable_button')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='조이스틱 장치 경로 (manual_control.launch.py와 동일 기본값)',
        ),
        DeclareLaunchArgument(
            'max_linear_mps', default_value='0.2',
            description=(
                "풀 스틱(1.0)일 때 최대 전진속도. 기본값은 production 첫 실차 "
                "시험값(todrive.md/our_mppi_params.yaml vx_max=0.2)과 동일 -- "
                "임의로 올리지 말 것."
            ),
        ),
        DeclareLaunchArgument(
            'max_angular_radps', default_value='0.5',
            description=(
                "풀 스틱(1.0)일 때 최대 회전속도. 기본값은 production 첫 실차 "
                "시험값(wz_max=0.5)과 동일."
            ),
        ),
        DeclareLaunchArgument(
            'enable_button', default_value='6',
            description=(
                "deadman switch 버튼 인덱스 (기본 6=L2, manual_joy_control_node.py "
                "실측 매핑 재사용). 이 버튼을 누르고 있을 때만 /cmd_vel이 나간다."
            ),
        ),

        # 조이스틱 raw 입력 -> /joy (표준 joy 패키지, manual_control.launch.py와
        # 동일한 방식으로 재사용, 새로 안 만듦)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),

        # /joy -> /cmd_vel (표준 teleop_twist_joy 패키지). axis 매핑은
        # manual_joy_control_node.py 실측값 재사용(축1=왼쪽 스틱 상하 ->
        # linear.x, 축3=오른쪽 스틱 좌우 -> angular.z).
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,
                'scale_linear.x': max_linear_mps,
                'axis_angular.yaw': 3,
                'scale_angular.yaw': max_angular_radps,
                'enable_button': enable_button,
                'require_enable_button': True,
            }],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),
    ])
