"""
mission_winter_drive_planz.launch.py

[2026-09-05 신규, 사용자 요청] 겨울 미션(윤활제 도포된 철 트랙 경사 구간)을
기존 PlanA 기반(mission_winter_drive.launch.py, MPPI+slope_traverse_node)
대신 PlanZ(autonomous.planz.launch.py, pure pursuit + can_driver_node) 구조로
띄우는 별도 진입점. 기존 mission_winter_drive.launch.py(PlanA)는 그대로
유지되고 이 파일과 서로 수정을 공유하지 않는다 -- 어느 쪽을 쓸지는 실기에서
선택.

autonomous.planz.launch.py와 노드 구성은 동일(정적 TF 2개 + can_driver_node +
purepursuit_node)하되, purepursuit_node 대신 STRAIGHT_LOCK이 추가된
autonomous.planz_winter.py를 띄운다: straight_after_sec(기본 35초)까지는
평소처럼 /path 기반 pure pursuit으로 주행하고, 그 이후로는 영구적으로(1회성
래치, 다시 pure pursuit으로 안 돌아옴) straight_drive_dps(기본 200dps)로
직진 고정한다(autonomous.planz_winter.py 상단 docstring 참고).

[정적 TF 값] autonomous.planz.launch.py 자체에 있는 TF 값은 2026-08-27
시점 것으로 이미 stale이라고 그 파일에 경고가 남아있다 -- 이 launch는 대신
reduced_odom_bringup.launch.py에 있는 현재(2026-09-04 재실측) CAD 값을
그대로 복사해서 쓴다. reduced_odom_bringup.launch.py 자체를 include하지는
않는다 -- 그러면 rmd_x8_driver_node가 같이 떠서 can_driver_node와 같은 CAN
버스(left=1/right=2)를 동시에 잡아 충돌한다(아래 경고 참고).

[중요, autonomous.planz.launch.py와 동일한 경고] robot_bringup의
autonomous*.launch.py/mission_*_drive.launch.py(reduced_odom_bringup.launch.py
로 rmd_x8_driver_node가 이미 떠 있는 것들)와 동시에 띄우지 않는다 --
can_driver_node와 같은 CAN 버스/CAN ID(left=1/right=2)를 동시에 잡아 서로
다른 속도 명령을 쏘는 충돌이 실물에서 발생한다.

사용:
    ros2 launch robot_bringup mission_winter_drive_planz.launch.py
    ros2 launch robot_bringup mission_winter_drive_planz.launch.py can_channel:=vcan0  # dry-run
    ros2 launch robot_bringup mission_winter_drive_planz.launch.py straight_after_sec:=35.0 straight_drive_dps:=200.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    can_channel = LaunchConfiguration('can_channel')
    use_outer_wheel_boost = LaunchConfiguration('use_outer_wheel_boost')
    target_linear_speed_m_s = LaunchConfiguration('target_linear_speed_m_s')
    lookahead_distance_m = LaunchConfiguration('lookahead_distance_m')
    # [2026-09-05 신규] STRAIGHT_LOCK -- autonomous.planz_winter.py 참고.
    straight_after_sec = LaunchConfiguration('straight_after_sec')
    straight_drive_dps = LaunchConfiguration('straight_drive_dps')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('robot_bringup'), 'config',
                'mission_winter_drive_planz_params.yaml'),
            description=(
                "purepursuit_node(autonomous.planz_winter.py) 파라미터 yaml 경로 -- "
                "config/mission_winter_drive_planz_params.yaml 상단 주석 참고. CLI "
                "인자로 노출된 키는 이 파일 값을 덮어쓴다(커맨드라인이 항상 이김)."
            ),
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can_drive',
            description="CAN 인터페이스 (실물 기본값 'can_drive'; 'vcan0'로 dry-run)",
        ),

        DeclareLaunchArgument(
            'use_outer_wheel_boost',
            default_value='false',
            description=(
                "true면 회전 시 안쪽 바퀴를 깎는 대신 바깥쪽만 부스트(경사 "
                "주행용, 매뉴얼 실측 300/200dps 조합 참고). false면 기존 "
                "대칭 스큐-스티어 그대로. straight_after_sec 이후 STRAIGHT_LOCK "
                "구간(wz=0)에는 영향 없음."
            ),
        ),
        DeclareLaunchArgument(
            'target_linear_speed_m_s',
            default_value='0.3',
            description=(
                "straight_after_sec 전까지 pure pursuit 목표 선속도[m/s] -- "
                "경사 주행 시 안쪽 바퀴 목표(0.4 권장). STRAIGHT_LOCK 구간의 "
                "속도는 이것과 별개로 straight_drive_dps가 정한다."
            ),
        ),
        DeclareLaunchArgument(
            'lookahead_distance_m',
            default_value='0.5',
            description="straight_after_sec 전까지 pure pursuit lookahead 거리[m]",
        ),
        DeclareLaunchArgument(
            'straight_after_sec',
            default_value='35.0',
            description=(
                "이 시간(초)이 지나면 영구적으로(1회성 래치, 다시 pure "
                "pursuit으로 안 돌아옴) /path를 무시하고 straight_drive_dps로 "
                "직진 고정한다 (autonomous.planz_winter.py STRAIGHT_LOCK 참고)."
            ),
        ),
        DeclareLaunchArgument(
            'straight_drive_dps',
            default_value='200.0',
            description=(
                "straight_after_sec 이후 고정 직진 시 좌우 바퀴 dps 크기 "
                "(속도 단위가 아니라 바퀴 dps 그대로 -- 좌우 부호는 "
                "left_motor_sign/right_motor_sign이 알아서 적용)."
            ),
        ),

        # ---- 정적 TF (reduced_odom_bringup.launch.py의 현재/2026-09-04
        # 재실측 CAD 값 그대로 복사 -- 그 launch 자체를 include하면
        # rmd_x8_driver_node가 같이 떠서 can_driver_node와 CAN 버스가
        # 충돌하므로 복사해서 쓴다. 노드 이름을 다른 launch와 다르게 둬서
        # 혹시라도 실수로 같이 뜨더라도 이름 충돌(중복 기동 에러)만은 안
        # 나게 함). ----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='winter_planz_base_to_imu_tf',
            arguments=[
                '--x', '-0.180', '--y', '0', '--z', '0.375',
                '--roll', '0.0555', '--pitch', '0.0134', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='winter_planz_base_to_camera_tf',
            arguments=[
                '--x', '0.34815', '--y', '0', '--z', '0.25795',
                '--roll', '0.0030', '--pitch', '0.8106', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        # ---- CAN 드라이버 (manual.launch.py/autonomous.planz.launch.py와 동일 파라미터) ----
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

        # ---- purepursuit_node (autonomous.planz_winter.py, STRAIGHT_LOCK 추가) ----
        Node(
            package='robot_bringup',
            executable='autonomous.planz_winter.py',
            name='purepursuit_node',
            parameters=[
                params_file,
                {
                    'track_width_m': 0.4904,
                    'wheel_radius_m': 0.1125,
                    'max_wheel_speed_dps': 360.0,
                    # [주의] LaunchConfiguration은 항상 문자열로 해석되므로,
                    # bool/float로 선언된 파라미터에 그냥 넘기면 rclpy가
                    # String 타입으로 잘못 설정해 declare_parameter의
                    # bool/float 기본값과 타입이 안 맞아 예외가 난다 --
                    # ParameterValue(value_type=...)로 명시 변환.
                    'use_outer_wheel_boost': ParameterValue(use_outer_wheel_boost, value_type=bool),
                    'target_linear_speed_m_s': ParameterValue(target_linear_speed_m_s, value_type=float),
                    'lookahead_distance_m': ParameterValue(lookahead_distance_m, value_type=float),
                    'straight_after_sec': ParameterValue(straight_after_sec, value_type=float),
                    'straight_drive_dps': ParameterValue(straight_drive_dps, value_type=float),
                },
            ],
            output='screen',
        ),
    ])
