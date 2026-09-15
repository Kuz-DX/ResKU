"""
reduced_odom_bringup.launch.py (2026-08-25 개명, 옛 이름 ekf.launch.py --
production odom이 reduced_odom으로 완전히 넘어간 지 오래라 실제 내용에
맞게 정리. 15-state robot_localization EKF 자체는 이미 완전히 제거됨,
HANDOFF.md/archive/robot_localization_ekf.cpp도 같이 삭제됨)

Sensor + production reduced-state odometry bringup. Step 2 of the pipeline:
    rmd_x8_driver_node (/wheel/odom) -+
    myahrs_driver_node (/imu)        -+-> reduced_odom_node -> /odometry/filtered, tf odom->base_link

Deliberately excludes Nav2/MPPI (see nav2.launch.py, step 3 - not wired
up/verified yet) and manual-drive joystick nodes (see manual_drive.launch.py).

No URDF/xacro/robot_state_publisher here (URDF dropped, see project
decision). base_link -> imu_link is provided by a static_transform_publisher
below instead.

Verify with:
    ros2 topic echo /odometry/filtered
    ros2 run tf2_ros tf2_echo odom base_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    imu_port = LaunchConfiguration('imu_port')

    odom_params = os.path.join(
        get_package_share_directory('reduced_odom'),
        'config', 'reduced_odom.yaml')

    return LaunchDescription([

        # [하림 수정] 기본값을 can_drive로 변경 (can_driver 패키지와 동일 운영 관례 -
        # 실물 어댑터가 can0로 잡힌 뒤 can_drive로 리네임되어 쓰임)
        DeclareLaunchArgument(
            'can_interface',
            default_value='can_drive',
            description=(
                "SocketCAN interface for rmd_x8_driver_node. Default is "
                "the real bus ('can_drive'); override with "
                "'can_interface:=vcan0' to dry-run against a virtual bus."
            ),
        ),

        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyACM0',
            description=(
                "Serial port for myahrs_driver_node. Override with "
                "'imu_port:=/tmp/.../imu_slave' to dry-run against a "
                "socat-fed virtual serial pair."
            ),
        ),

        # [단계 1] RMD-X8 CAN 구동 드라이버 노드 -> /wheel/odom
        # 지상 주행용 hard wheel-speed clamp. 최초 저속 시험값에서 3배 상향.
        # cmd_vel_timeout_s/cmd_vel_safety_timeout_s는 코드 기본값(0.3s/0.5s)과
        # 같지만, 나중에 한쪽만 바뀌는 걸 놓치지 않도록 명시적으로 적어둠.
        # [2026-08-27] control_rate_hz 50.0 -> 20.0 -- MPPI(controller_frequency,
        # our_mppi_params.yaml)와 myAHRS+ IMU(output_divider, 아래)를 전부
        # 20Hz로 통일하려는 사용자 요청. cmd_vel_timeout_s(0.3s)/
        # cmd_vel_safety_timeout_s(0.5s) 모두 새 주기(0.05s)보다 훨씬 커서
        # 안전 마진 문제 없음.
        Node(
            package='rmd_x8_driver',
            executable='rmd_x8_driver_node',
            name='rmd_x8_driver',
            parameters=[{
                'can_interface': can_interface,
                'left_motor_can_id': 1,
                'right_motor_can_id': 2,
                # [2026-08-27 기하학적 추론 기각, 원래 값으로 복귀]
                # "궤도뭉치 가운데(365mm)가 기구학적으로 맞다"는 추론이 실제
                # MPPI 반원 경로 주행(전진+회전 동시) 테스트로는 틀린 것으로
                # 확인됨 -- 0.365로 바꾸니 원래 0.4904(주석 없이 박혀있던,
                # 바깥쪽≈500mm 실측에 가까운 값)보다 CTE가 더 나빠짐(0.722m
                # -> 1.16~1.42m). 이 값을 물리적 의미 없이 계속 스윕해서
                # 맞추는 건 하지 않기로 함 -- self.track_width가
                # _skid_steer_inverse()의 명령 생성뿐 아니라 /wheel/odom의
                # wz 계산(위 510행)에도 그대로 쓰여서, 조향 보정 목적으로
                # 임의로 부풀리면 오도메트리 자체가 물리량과 무관하게
                # 왜곡된다. 그래서 원래 값(0.4904, CTE 실측 최저)으로 되돌리고,
                # 추가 보정이 필요하면 명령 생성에만 영향을 주는(오도메트리는
                # 안 건드리는) angular_slip_compensation_factor 쪽에서 작은
                # 폭으로 스윕할 것.
                'effective_track_width_m': 0.4904,
                # [2026-08-27 스윕 잠정 중단, 1.05로 복귀] track_width=0.4904
                # (원래 값, CTE 실측 최저)로 고정. 지금까지: 1.0->0.722m,
                # 1.05->0.445m(최저), 1.10->0.507m. 1.07 세밀화 테스트 도중
                # 오도메트리가 1.5초 만에 6m 이상 튀는 이상(원인 미확인 --
                # slip factor 자체와는 무관해 보임, 로봇 실물 상태 확인 필요)이
                # 발생해서 CTE 비교가 무의미해짐 -- 그 원인 규명 전까지 세밀화
                # 스윕은 보류하고, 지금까지 나온 값 중 가장 좋았던 1.05로
                # 되돌려둠.
                'angular_slip_compensation_factor': 1.05,
                'wheel_radius_m': 0.1125,
                'external_gear_ratio': 1.0,
                'max_wheel_speed_dps': 360.0,
                'cmd_vel_timeout_s': 0.3,
                'cmd_vel_safety_timeout_s': 0.5,
                'control_rate_hz': 20.0,
            }],
            output='screen',
        ),

        # [단계 2] myAHRS+ IMU 드라이버 노드 -> /imu
        Node(
            package='myahrs_driver',
            executable='myahrs_driver_node',
            name='myahrs_driver',
            parameters=[{
                'port': imu_port,
                'baudrate': 460800,
                'frame_id': 'imu_link',
                # [2026-08-23] 기존 10Hz(divider=10 상당) -> 50Hz(divider=2).
                # EKF sensor_timeout(0.3s로 같이 상향, ekf_local.yaml 참고)
                # 대비 마진을 넉넉히 확보 -- EKF 발산 진단 세션 근거.
                # myahrs_driver_node.cpp 상단 docstring 참고: 지원값은
                # 1/2/4/5/10(divider) -> 100/50/25/20/10(Hz)뿐, 다른 값은
                # 노드가 시작 자체를 거부함.
                # [2026-08-27] 50Hz(divider=2) -> 20Hz(divider=5) -- MPPI/
                # rmd_x8_driver(위)와 동일하게 20Hz로 통일하려는 사용자 요청.
                # 20Hz는 지원 목록에 있는 값이라 정확히 맞출 수 있음(30Hz는
                # 지원 안 돼서 불가능했음, 그래서 20Hz로 합의).
                'output_divider': 5,
                'orientation_covariance_roll': 0.00000594,
                'orientation_covariance_pitch': 0.00003487,
                'orientation_covariance_yaw': 0.00051956,
            }],
            output='screen',
        ),

        # [단계 3] base_link -> imu_link static TF
        # [하림 수정] CAD 실측값 반영 (base_link 기준, x=전방+/y=좌측/z=위):
        # IMU는 base_link보다 뒤쪽(-180mm), 높이 375mm.
        # [하림 수정] 2026-08-18: roll/pitch 실측 반영 (기존 0/0 placeholder ->
        # 3.18/0.77도). 로봇을 평평한 바닥에 완전히 정지시킨 상태에서
        # /imu(쿼터니언 -> roll/pitch/yaw 변환)를 몇 분 관찰, 값이 안정된
        # 구간(roll 3.14~3.21도, pitch 0.74~0.80도)의 대표값 사용
        # (imu_calibration_debug.launch.py로 CAN/모터 없이 IMU만 띄워서 측정 --
        # 로봇이 실제로 안 움직이는 상태를 보장). yaw는 나침반이 없어 절대
        # 기준이 없으므로(자이로 드리프트로 계속 변함) 실측 대상에서 제외하고
        # 0 유지.
        # [하림 수정] 2026-08-18 버그 수정: static_transform_publisher의
        # --roll/--pitch/--yaw는 라디안 단위인데 위에서 잰 값(3.18/0.77)은
        # "도" 단위였음 -> 그대로 넣으면 roll이 약 182°, pitch가 약 44°로
        # 들어가서 IMU 자세가 EKF에서 거의 뒤집힌 채로 융합됨(경로 발산/뒤집힘
        # 원인). math.radians()로 변환한 값(0.0555/0.0134 rad)으로 수정.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '--x', '-0.180', '--y', '0', '--z', '0.375',
                '--roll', '0.0555', '--pitch', '0.0134', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),

        # [단계 3.5] base_link -> camera_link static TF
        # [하림 수정] 기존엔 이 TF가 아예 없었음 (URDF 제거 후 IMU만 추가돼 있었음).
        # [2026-08-25 마운트 변경] 카메라가 x,y는 base_link 중앙(0,0)에 맞춰
        # 재장착됨, z만 지면 기준 85cm(base_link 지상고 11.5cm 기준
        # 0.85-0.115=0.735m). 예전 값(x=+348.515mm, z=257.95mm)은 이전
        # 마운트 위치였음.
        # [2026-08-27 마운트 재측정] base_link 기준 x=+90.461mm(전방),
        # y=0(정렬), z=805.188mm(base_link 원점 기준, 지면 기준 아님 --
        # 사용자 실측값 그대로 사용). purepursuit.launch.py에 동일 TF가
        # 중복 선언돼 있어서 같이 갱신함.
        # [2026-09-01 마운트 재변경] base_link 기준 x=+348.15mm, z=+257.95mm로
        # 다시 변경(사용자 지시). roll/pitch는 이번 변경 대상이 아니라
        # 기존 실측값(-0.0011/0.8063) 그대로 유지 -- 위치가 바뀌었으니 각도도
        # 재측정이 필요할 수 있음. purepursuit.launch.py도 같이 갱신.
        # child-frame-id를 'camera_link'로 잡은 이유: path_relay_node.cpp가 인지팀
        # Path의 frame_id를 'camera_link'로 가정하고 있고(코드 주석 참고), realsense2_camera
        # 노드도 이 이름을 자기 TF 트리의 루트로 쓰므로, 카메라 노드를 띄우면
        # camera_link 밑의 camera_color_optical_frame 등은 realsense 쪽이 알아서 채워줌.
        # [2026-08-27 회전 실측] 로봇을 평평한 바닥에 정지시킨 상태에서
        # /drive/camera/imu 원시 가속도(4개 유효 샘플, MIPI 에러로 스트림이
        # 끊겨서 12번 중 4번만 성공했지만 std는 낮음)를 body 좌표로 변환해 실측:
        # roll=-0.001103rad(-0.063deg), pitch=0.806281rad(46.196deg, 기수 하향
        # 이 양수). yaw는 가속도만으로는 원리상 측정 불가(마그네토미터 없음)이라
        # 기존대로 0 유지.
        # [2026-09-04 재실측, 주행구간] imu_pitch_roll_probe.py로 재측정
        # (std roll=0.0009rad/pitch=0.0008rad로 매우 낮아 신뢰 가능):
        # roll=+0.0030rad(+0.17deg), pitch=+0.8106rad(+46.45deg). 같은 날
        # 앞서 재측정했던 값(roll=+0.0060/pitch=+0.8250)을 덮어씀 -- 위치
        # 변경 없이(x/z 그대로) 각도만 갱신. [주의] 이 값은 "주행구간"
        # 마운트 각도다 -- escort_follow(추종구간)는 별개 측정값을 쓰며
        # purepursuit.launch.py의 camera_pitch_rad/camera_roll_rad
        # DeclareLaunchArgument 기본값으로 관리된다(혼동 금지).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.34815', '--y', '0', '--z', '0.25795',
                '--roll', '0.0030', '--pitch', '0.8106', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        # [단계 4] 관측 가능한 [x,y,yaw,vx,wz]만 유지하는 production 추정기.
        # 이 노드가 odom->base_link의 유일한 publisher다. generic 15-state
        # robot_localization은 동시에 실행하지 않는다.
        Node(
            package='reduced_odom',
            executable='reduced_odom_node',
            name='reduced_odom_node',
            parameters=[odom_params],
            output='screen',
        ),
    ])
