"""
manual_return_bringup.launch.py

Robot-PC side entry point for the manual-drive + autonomous-return mission
(see repo root planning doc for the full scenario). Replaces the old
can_driver/manual.launch.py + reduced_odom_bringup.launch.py combo with a
single CAN-owning driver shared by manual driving and the autonomous return:

    Joystick (remote PC, manual_joy_control/launch/manual_control.launch.py)
        -> /motor_speed_cmd_manual (dps)
                              -+
    return_state_machine_node -+-> drive_cmd_mux_node (dps->Twist + 중재) -> /cmd_vel -> rmd_x8_driver_node -> CAN
        -> /cmd_vel_return    -+                                            |
                                                                      /wheel/odom
                                                                            |
                                                                     reduced_odom_node
                                                                            |
                                                                  /odometry/filtered, tf odom->base_link

[중요] can_driver_node(구 manual 전용 CAN 드라이버)는 여기서 launch하지 않는다
-- rmd_x8_driver_node가 CAN 버스(can_drive)/motor ID를 소유하는 유일한 노드다.
이 launch 파일을 autonomous.launch.py(MPPI, 2단계 평가용 보존)와 동시에
실행하지 말 것 (동일 CAN 버스/`/cmd_vel` 충돌).

[2026 사용자 결정, 경량화] manual_stability_node(IMU pitch/roll 긴급정지)는
이 launch에서 제외했다 -- 실기 검증 결과 전복 위험 자세가 나타나지 않는
운용 환경으로 판단되어, 안전 여유보다 구성 단순화를 우선한 명시적 선택.
소스(src/drive/manual/can_driver/can_driver/manual_stability_node.py)는
남아있고 /cmd_vel_safety(Twist)로 발행하도록 이미 재배선되어 있으므로,
필요해지면 아래에 Node()만 다시 추가하면 된다. rmd_x8_driver_node의
/cmd_vel_safety 구독 자체는 그대로 남아있다 (아무도 발행하지 않으면 그냥
비어있는 안전 입력일 뿐, side effect 없음).

원격 조이스틱 쪽은 기존 그대로 별도 PC에서
manual_joy_control/launch/manual_control.launch.py로 띄운다 (변경 없음 --
publish하는 토픽만 /motor_speed_cmd_manual로 바뀌었을 뿐, 타입은 기존과
동일한 Float32MultiArray dps 유지. dps->Twist 변환은 drive_cmd_mux_node
담당, 2026 사용자 결정).

Verify with:
    ros2 topic echo /wheel/odom
    ros2 topic echo /odometry/filtered
    ros2 run tf2_ros tf2_echo odom base_link
    ros2 topic echo /mission/return/state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    imu_port = LaunchConfiguration('imu_port')
    turn_direction = LaunchConfiguration('turn_direction')
    track_width = LaunchConfiguration('effective_track_width_m')
    slip_factor = LaunchConfiguration('angular_slip_compensation_factor')
    use_imu_yaw = LaunchConfiguration('use_imu_yaw')

    reduced_odom_bringup_launch = os.path.join(
        get_package_share_directory('robot_bringup'),
        'launch', 'reduced_odom_bringup.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can_drive'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM0'),
        # RETURN 직전 180도 정렬 회전 방향 ('left'=반시계, 'right'=시계).
        DeclareLaunchArgument('turn_direction', default_value='left'),
        # [실차 실측 2026-09] 스키드 조향 제자리 회전이 바퀴 기준 명령/오도메트리의
        # 약 0.31배만 실제로 회전함(왼쪽 90도 vs 바퀴 267.5도, 오른쪽 82도 vs 289.1도
        # -> 평균 0.31). 유효 트랙 폭을 0.4904/0.31=1.58m로 잡으면 명령(w)과 바퀴
        # 오도메트리 yaw가 모두 실제 회전과 맞는다. drive_cmd_mux의 값도 반드시
        # 같이 바꿔야 수동 dps 명령이 그대로 통과한다(mux->driver 왕복이 항등).
        # myAHRS+ yaw는 모터 구동 후 분 단위로 초당 2도씩 흘러서 이 미션에서는
        # 쓰지 않는다(use_imu_yaw=false). 옛 동작으로 되돌리려면:
        #   effective_track_width_m:=0.4904 angular_slip_compensation_factor:=1.05 use_imu_yaw:=true
        DeclareLaunchArgument('effective_track_width_m', default_value='1.58'),
        DeclareLaunchArgument('angular_slip_compensation_factor', default_value='1.0'),
        DeclareLaunchArgument('use_imu_yaw', default_value='false'),

        # [단계 1] rmd_x8_driver_node(CAN, 유일한 소유자) + myahrs_driver_node
        # + static TF(base_link->imu_link/camera_link) + reduced_odom_node.
        # 그대로 재사용 -- manual/return 어느 쪽이든 이 파이프라인은 항상 켜져
        # 있어야 한다 (odom 자체를 reset하지 않는다).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(reduced_odom_bringup_launch),
            launch_arguments={
                'can_interface': can_interface,
                'imu_port': imu_port,
                'effective_track_width_m': track_width,
                'angular_slip_compensation_factor': slip_factor,
                'use_imu_yaw': use_imu_yaw,
            }.items(),
        ),

        # [단계 2] manual(dps)->Twist 변환 + manual/return 명령 중재 -> /cmd_vel
        Node(
            package='drive_cmd_mux',
            executable='drive_cmd_mux_node',
            name='drive_cmd_mux_node',
            parameters=[{'effective_track_width_m': ParameterValue(track_width, value_type=float)}],
            output='screen',
        ),

        # [단계 3] 경로 기록 (manual 주행 중 /odometry/filtered 기록)
        Node(
            package='return_navigation',
            executable='manual_path_recorder_node',
            name='manual_path_recorder_node',
            output='screen',
        ),

        # [단계 4] RETURN 상태 머신 (STOP_BEFORE_TURN/TURN_180/FOLLOW_RETURN_PATH
        # 상태 및 /cmd_vel_return 발행)
        Node(
            package='return_navigation',
            executable='return_state_machine_node',
            name='return_state_machine_node',
            parameters=[{'turn_direction': turn_direction}],
            output='screen',
        ),

        # [단계 5] 고정 mission-frame Path를 추종하는 복귀 컨트롤러
        Node(
            package='return_navigation',
            executable='return_path_follower_node',
            name='return_path_follower_node',
            output='screen',
        ),
    ])
