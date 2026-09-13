"""
autonomous.planz.launch.py

[2026-09-02 복원] dolbotz/purepursuit.launch.py의 2026-08-27 시점(커밋
3e3d81782f39b3a6ffb6887bf9ea74f5e4350664) 구성을 robot_bringup 패키지의
scripts/autonomous.planz.py(같은 시점 purepursuit.py 코드를 그대로 되살린
독립 노드)용으로 재구성한 launch. 그 이후 dolbotz 쪽에 추가된
escort_follow 전용 파라미터/인자(거리 비례 속도, bearing steering, 피벗 턴,
가감속 분리 등)는 이 노드 코드 자체에 없으므로 여기도 없다. 현재의
dolbotz/purepursuit.launch.py와는 완전히 별개이며 서로 수정을 공유하지
않는다.

아래 세 가지만 띄운다 (경로 소스는 이 launch에 안 넣음 -- perception 쪽
/path 발행 launch를 별도로 같이 띄워야 함):

    1) base_link -> imu_link / camera_link 정적 TF (2026-08-27 시점 CAD/IMU
       실측값 -- autonomous.planz.py가 /path(camera_link)를 base_link로
       변환하는 데 필요)
    2) can_driver_node (can_driver 패키지, manual.launch.py가 조종 모드에서
       쓰는 그 노드) -- /motor_speed_cmd(dps) -> CAN
    3) purepursuit_node (robot_bringup 패키지, autonomous.planz.py) --
       /path -> pure pursuit -> 스키드조향 역기구학 -> /motor_speed_cmd

[중요, 원본과 동일한 경고] robot_bringup의 autonomous.launch.py/
reduced_odom_bringup.launch.py(rmd_x8_driver_node가 이미 떠 있음)와 동시에
띄우지 않는다 -- can_driver_node와 같은 CAN 버스/CAN ID(left=1/right=2)를
동시에 잡아 서로 다른 속도 명령을 쏘는 충돌이 실물에서 발생한다.

[주의] 아래 정적 TF 값은 2026-08-27 시점 실측값 그대로다. 실제 카메라
마운트는 이후(2026-09-01) 재변경되어 위치/각도가 달라졌다(현재
dolbotz/purepursuit.launch.py의 camera_pitch_rad/camera_roll_rad 및 x/z 값
참고) -- 이 launch를 실물에서 그대로 쓰려면 재실측 없이는 base_link 좌표
변환이 실제 기하와 어긋날 수 있다.

사용:
    ros2 launch robot_bringup autonomous.planz.launch.py
    ros2 launch robot_bringup autonomous.planz.launch.py can_channel:=vcan0  # dry-run
    ros2 launch robot_bringup autonomous.planz.launch.py use_outer_wheel_boost:=true target_linear_speed_m_s:=0.4
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

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('robot_bringup'), 'config',
                'autonomous.planz_params.yaml'),
            description=(
                "purepursuit_node(autonomous.planz.py) 파라미터 yaml 경로 -- "
                "config/autonomous.planz_params.yaml 상단 주석 참고. CLI "
                "인자로 노출된 키는 이 파일 값을 덮어쓴다(커맨드라인이 항상 "
                "이김)."
            ),
        ),
        DeclareLaunchArgument(
            'can_channel',
            default_value='can_drive',
            description="CAN 인터페이스 (실물 기본값 'can_drive'; 'vcan0'로 dry-run)",
        ),

        # [2026-08-27 시점 그대로] 경사 주행 모드 -- 기본값(false/0.3/0.5)은
        # 평지 주행과 동일, 경사 주행 시 커맨드라인에서 오버라이드.
        DeclareLaunchArgument(
            'use_outer_wheel_boost',
            default_value='false',
            description=(
                "true면 회전 시 안쪽 바퀴를 깎는 대신 바깥쪽만 부스트(경사 "
                "주행용, 매뉴얼 실측 300/200dps 조합 참고). false면 기존 "
                "대칭 스큐-스티어 그대로."
            ),
        ),
        DeclareLaunchArgument(
            'target_linear_speed_m_s',
            default_value='0.3',
            description="목표 선속도[m/s] -- 경사 주행 시 안쪽 바퀴 목표(0.4 권장)",
        ),
        DeclareLaunchArgument(
            'lookahead_distance_m',
            default_value='0.5',
            description="pure pursuit lookahead 거리[m]",
        ),

        # ---- 정적 TF (2026-08-27 시점 CAD/IMU 실측값 -- 위 [주의] 참고) ----
        # 노드 이름을 다른 launch(reduced_odom_bringup.launch.py, dolbotz의
        # purepursuit.launch.py 등)와 다르게 둬서, 혹시라도 실수로 같이 뜨더라도
        # 노드 이름 충돌(중복 기동 에러)만은 안 나게 함.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='autonomous_planz_base_to_imu_tf',
            arguments=[
                '--x', '-0.180', '--y', '0', '--z', '0.375',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='autonomous_planz_base_to_camera_tf',
            arguments=[
                # 2026-08-27 IMU 실측값 (재측정 전 마운트 위치/각도 -- 위
                # [주의] 참고).
                '--x', '0.090461', '--y', '0', '--z', '0.805188',
                '--roll', '-0.0011', '--pitch', '0.8063', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        # ---- CAN 드라이버 (manual.launch.py와 동일 파라미터) ----
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

        # ---- purepursuit_node (autonomous.planz.py) ----
        # track_width_m/max_wheel_speed_dps는 코드 기본값(0.50/800.0) 대신
        # reduced_odom_bringup.launch.py 캘리브레이션 값(0.4904)과 2026-08-27
        # 시점 자율주행 하드 클램프(360.0 dps)를 명시적으로 맞춘다 -- 원본
        # dolbotz/purepursuit.launch.py의 그 시점 값과 동일.
        Node(
            package='robot_bringup',
            executable='autonomous.planz.py',
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
                },
            ],
            output='screen',
        ),
    ])
