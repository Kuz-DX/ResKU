"""
manual.launch.py (can_driver) -- 로봇(온보드 PC) 쪽 전용

[하림 수정] 2026-08-14: 조이스틱이 로봇 온보드 PC가 아니라 별도 원격 PC에
물려있는 구조로 확인되어, joy_node/manual_joy_control_node를 이 launch에서
분리했다. 이제 두 PC에서 각각 띄워야 한다:

    [원격 PC]  joy_node -> manual_joy_control_node -> /motor_speed_cmd ---(네트워크)---+
                                                                                        v
    [로봇 PC]  myahrs_driver_node(/imu) -> manual_stability_node -> /motor_speed_cmd_safety (우선)
                                                                                        v
                                                                          can_driver_node -> CAN -> 모터

manual_stability_node은 A안(IMU pitch/roll 임계값)만 구현 -- 전류 기반 보호는
없음 (can_driver_node가 모터 응답을 안 읽어서 전류 피드백 자체가 없음, 필요하면
추후 rmd_x8_driver_node의 CAN reply 파싱 로직을 이식해서 추가 가능).

[하림 수정] robot_bringup의 stability_monitor_node/joy_mux_node/current_ramp_node와
완전히 독립된 코드다 (미션 구조상 manual/autonomous가 동시에 뜰 일이 없어서
공유할 필요가 없다고 판단, 의도적으로 중복 허용).

두 PC가 반드시 같은 ROS 네트워크에 있어야 /motor_speed_cmd가 전달된다:
  - ROS_DOMAIN_ID 동일해야 함
  - ROS_LOCALHOST_ONLY=0 (또는 미설정)이어야 함 -- 1이면 자기 PC 안에서만 통신됨
  - 가능하면 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp로 양쪽 통일 권장

사용:
    [원격 PC, 조이스틱이 물린 쪽]
        ros2 launch manual_joy_control manual_control.launch.py

    [로봇 PC]
        ros2 launch can_driver manual.launch.py
        ros2 launch can_driver manual.launch.py can_channel:=vcan0  # dry-run
        ros2 launch can_driver manual.launch.py enable_stability_monitor:=false  # 긴급정지 끄고 테스트

[하림 수정] 2026-08-20: enable_stability_monitor 파라미터 추가. 기본값은 true(안전
켜짐)로 유지 -- 이 launch를 그냥 돌리는 누구에게나 안전장치가 조용히 빠진 채로
나가면 안 되므로, 끄고 싶을 때만 명시적으로 false를 넘기게 했다. myahrs_driver_node
(IMU)는 이 launch 안에서 manual_stability_node의 입력으로만 쓰이므로(모듈 docstring
위쪽 참고) 같은 플래그로 묶어서 같이 껐다 -- 안전장치를 안 쓸 거면 IMU 포트도 굳이
열어둘 이유가 없다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_channel = LaunchConfiguration('can_channel')
    imu_port = LaunchConfiguration('imu_port')
    enable_stability_monitor = LaunchConfiguration('enable_stability_monitor')

    return LaunchDescription([

        DeclareLaunchArgument(
            'can_channel',
            default_value='can_drive',
            description="CAN 인터페이스 (실물 기본값 'can_drive'; 'vcan0'로 dry-run)",
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyACM0',
            description='myahrs_driver_node 시리얼 포트',
        ),
        DeclareLaunchArgument(
            'enable_stability_monitor',
            default_value='true',
            description=(
                "IMU pitch/roll 긴급정지(manual_stability_node)와 그 입력인 "
                "myahrs_driver_node를 같이 켤지 여부. 기본값 true(안전 켜짐) -- "
                "끄면 /motor_speed_cmd_safety로 인한 긴급 개입이 전혀 없어지니 "
                "테스트 등 명확한 이유가 있을 때만 false로 넘길 것."
            ),
        ),

        # [CAN 드라이버] /motor_speed_cmd(원격 PC에서 네트워크로 옴), /motor_speed_cmd_safety 둘 다 구독
        # [하림 수정] cmd_timeout_sec/control_rate_hz/cmd_safety_timeout_sec는
        # 코드 기본값과 같지만, autonomous 쪽(rmd_x8_driver)과 짝이 안 맞게
        # 나중에 한쪽만 바뀌는 걸 놓치지 않도록 명시적으로 적어둠.
        Node(
            package='can_driver',
            executable='can_driver_node',
            name='can_driver_node',
            parameters=[{
                'can_channel': can_channel,
                'left_can_id': 1,
                'right_can_id': 2,
                'cmd_timeout_sec': 0.3,
                'control_rate_hz': 50.0,
                'cmd_safety_timeout_sec': 0.5,
            }],
            output='screen',
        ),

        # [IMU] manual_stability_node의 입력 -- enable_stability_monitor:=false면
        # 같이 안 뜬다 (이 launch 안에서 다른 용도가 없어서 굳이 포트 열어둘 이유 없음)
        Node(
            package='myahrs_driver',
            executable='myahrs_driver_node',
            name='myahrs_driver',
            parameters=[{
                'port': imu_port,
                'baudrate': 460800,
                'frame_id': 'imu_link',
                'orientation_covariance_roll': 0.00000594,
                'orientation_covariance_pitch': 0.00003487,
                'orientation_covariance_yaw': 0.00051956,
            }],
            output='screen',
            condition=IfCondition(enable_stability_monitor),
        ),

        # [긴급정지] IMU pitch/roll 임계값 -> /motor_speed_cmd_safety
        # enable_stability_monitor:=false로 끄면 이 노드가 아예 안 뜨고,
        # can_driver_node는 /motor_speed_cmd_safety를 아무도 안 보내니 그냥
        # /motor_speed_cmd만 그대로 통과시킨다 (긴급 개입 없음 -- 의도된 동작).
        Node(
            package='can_driver',
            executable='manual_stability_node',
            name='manual_stability_node',
            parameters=[{
                'critical_pitch_deg': 25.0,
                'critical_roll_deg': 20.0,
                'check_rate_hz': 100.0,
            }],
            output='screen',
            condition=IfCondition(enable_stability_monitor),
        ),
    ])
