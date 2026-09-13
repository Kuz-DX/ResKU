"""
manual_drive_sensors.launch.py

[2026-09-04 신규, 사용자 요청] "메뉴얼 주행에서도 rosbag으로 TF/카메라를
같이 딸 수 있게" -- can_driver/manual.launch.py(로봇 PC 쪽 메뉴얼 주행)는
모터 구동/IMU 긴급정지만 신경 쓰고 TF나 카메라는 원래 안 띄운다. 이
launch는 그 둘(TF, 카메라)만 따로 켜서 record_manual_drive.sh가 기록할
토픽을 채워준다.

[중요, CAN 충돌 주의] reduced_odom_bringup.launch.py(자율주행 쪽)는 절대
같이 쓰지 않는다 -- 그 안의 rmd_x8_driver_node가 can_driver_node와 완전히
같은 CAN 채널/ID(left=1/right=2)를 잡아서, 서로 다른 속도 명령을 동시에
쏘는 충돌이 실물에서 이미 확인된 바 있다(autonomous.planz.launch.py 상단
[중요] 문단과 동일 이유). 그래서 이 launch는 CAN을 전혀 건드리지 않는
것만 담는다:
    1) base_link -> imu_link / camera_link 정적 TF (reduced_odom_bringup.
       launch.py [단계 3]/[단계 3.5]와 완전히 동일한 CAD/IMU 실측값 --
       거기서 바뀌면 여기도 같이 바꿀 것. 노드 이름은 다르게 둬서 혹시라도
       실수로 같이 뜨더라도 이름 충돌만은 안 나게 함, purepursuit.launch.py
       와 동일한 관례)
    2) drive_cam.launch.py(dolbotz 패키지) -- RealSense 주행카메라. 카메라
       자체 TF(camera_link 아래 optical frame들)와 /drive/camera/imu,
       color/depth 스트림이 이걸로 뜬다.

[의도적으로 안 띄우는 것] myahrs_driver_node(/imu, base_link<-imu_link의
IMU 소스) -- can_driver/manual.launch.py가 기본값(enable_stability_monitor:=
true)으로 이미 띄우고 있어서 여기서 또 띄우면 같은 시리얼 포트(기본
/dev/ttyACM0)를 두 프로세스가 열려다가 실패한다. manual.launch.py를
enable_stability_monitor:=false로 껐다면 /imu 자체가 없는 상태이니, 기록에서
/imu가 빠지는 건 이 launch가 아니라 그쪽 설정 때문이다.

reduced_odom_node(odom->base_link 동적 TF, /odometry/filtered)도 의도적으로
안 띄운다 -- 그건 rmd_x8_driver_node의 /wheel/odom을 입력으로 쓰는데,
can_driver_node는 CAN 응답을 안 읽어서 애초에 /wheel/odom을 아무도 발행하지
않는다(메뉴얼 주행 자체에 바퀴 오도메트리 피드백이 없음, can_driver_node.py
참고). 그래서 메뉴얼 주행 중엔 odom->base_link 동적 TF/`/odometry/filtered`/
`/wheel/odom`는 구조적으로 기록 불가능 -- record_manual_drive.sh도 이 셋은
필수 목록에서 뺐다.

사용:
    ros2 launch robot_bringup manual_drive_sensors.launch.py
    (can_driver/manual.launch.py와 별도 터미널에서 같이 띄울 것 -- 순서 무관)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drive_cam_launch = get_package_share_directory('dolbotz') + '/launch/drive_cam.launch.py'

    return LaunchDescription([
        # [단계 1] base_link -> imu_link 정적 TF (reduced_odom_bringup.
        # launch.py [단계 3]과 동일 값 -- 값이 바뀌면 거기도 같이 바꿀 것)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='manual_base_to_imu_tf',
            arguments=[
                '--x', '-0.180', '--y', '0', '--z', '0.375',
                '--roll', '0.0555', '--pitch', '0.0134', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),

        # [단계 2] base_link -> camera_link 정적 TF (reduced_odom_bringup.
        # launch.py [단계 3.5]와 동일 값 -- 값이 바뀌면 거기도 같이 바꿀 것)
        # [2026-09-04 재실측] roll=+0.0030/pitch=+0.8106로 갱신 (reduced_odom_
        # bringup.launch.py와 동일 근거).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='manual_base_to_camera_tf',
            arguments=[
                '--x', '0.34815', '--y', '0', '--z', '0.25795',
                '--roll', '0.0030', '--pitch', '0.8106', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),

        # [단계 3] 주행 카메라 (dolbotz 패키지) -- color/depth 스트림 +
        # /drive/camera/imu + camera_link 아래 optical frame TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_cam_launch)),
    ])
