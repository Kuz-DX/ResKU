"""
manual_drive_sensors.launch.py

[manual+return 통합 이후 갱신] 예전엔 can_driver_node(CAN 응답을 안 읽음) 기반
manual 주행이라 TF/odom을 이 launch가 대신 채워줘야 했지만, 이제
manual_return_bringup.launch.py 자체가 rmd_x8_driver_node + reduced_odom_node를
포함해서 base_link->imu_link/camera_link 정적 TF, odom->base_link 동적 TF,
/odometry/filtered, /wheel/odom을 전부 정상 발행한다. 그래서 이 launch는
그것들 중복 발행을 그만두고, **아직 아무도 안 띄우는 주행 카메라 스트림만**
채우는 역할로 축소했다.

사용:
    ros2 launch robot_bringup manual_drive_sensors.launch.py
    (manual_return_bringup.launch.py와 별도 터미널에서 같이 띄울 것 -- 순서 무관.
    record_manual_drive.sh가 카메라가 안 보이면 대신 이 launch를 띄워준다.)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drive_cam_launch = get_package_share_directory('dolbotz') + '/launch/drive_cam.launch.py'

    return LaunchDescription([
        # 주행 카메라 (dolbotz 패키지) -- color/depth 스트림 + /drive/camera/imu +
        # camera_link 아래 optical frame TF. base_link->camera_link 자체는
        # manual_return_bringup.launch.py(reduced_odom_bringup.launch.py 포함)가
        # 이미 발행하므로 여기서 다시 만들지 않는다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_cam_launch)),
    ])
