"""D435i 2대와 Logitech C920 2대를 함께 실행한다.

주요 이미지 토픽:
  /drive/camera/color/image_raw       (RGB 전용)
  /arm/camera/color/image_raw
  /arm/camera/depth/image_rect_raw
  /left/camera/image_raw
  /right/camera/image_raw

각 image_transport 플러그인이 설치되어 있으면 대응하는 ``.../compressed``
토픽도 카메라 드라이버에서 함께 제공한다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_config(filename: str) -> str:
    return str(
        Path(get_package_share_directory("vision"))
        / "config"
        / "camera"
        / filename
    )


def generate_launch_description() -> LaunchDescription:
    drive_config = LaunchConfiguration("drive_config")
    arm_config = LaunchConfiguration("arm_config")
    left_config = LaunchConfiguration("left_config")
    right_config = LaunchConfiguration("right_config")

    drive_serial_no = LaunchConfiguration("drive_serial_no")
    arm_serial_no = LaunchConfiguration("arm_serial_no")
    left_device = LaunchConfiguration("left_device")
    right_device = LaunchConfiguration("right_device")

    declared_arguments = [
        DeclareLaunchArgument(
            "drive_config",
            default_value=_default_config("drive_d435i.yaml"),
            description="주행용 D435i 파라미터 YAML",
        ),
        DeclareLaunchArgument(
            "arm_config",
            default_value=_default_config("arm_d435i.yaml"),
            description="로봇팔용 D435i 파라미터 YAML",
        ),
        DeclareLaunchArgument(
            "left_config",
            default_value=_default_config("left_c920.yaml"),
            description="좌측 C920 파라미터 YAML",
        ),
        DeclareLaunchArgument(
            "right_config",
            default_value=_default_config("right_c920.yaml"),
            description="우측 C920 파라미터 YAML",
        ),
        DeclareLaunchArgument(
            "drive_serial_no",
            default_value="_117222251401",
            description="주행용 D435i 시리얼 번호",
        ),
        DeclareLaunchArgument(
            "arm_serial_no",
            default_value="_243322074693",
            description="로봇팔용 D435i 시리얼 번호",
        ),
        DeclareLaunchArgument(
            "left_device",
            default_value="/dev/video0",
            description="좌측 C920 V4L2 장치. /dev/v4l/by-id/... 사용 권장",
        ),
        DeclareLaunchArgument(
            "right_device",
            default_value="/dev/video2",
            description="우측 C920 V4L2 장치. /dev/v4l/by-id/... 사용 권장",
        ),
    ]

    cameras = [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="drive",
            name="camera",
            output="screen",
            emulate_tty=True,
            parameters=[
                drive_config,
                {"serial_no": ParameterValue(drive_serial_no, value_type=str)},
            ],
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="arm",
            name="camera",
            output="screen",
            emulate_tty=True,
            parameters=[
                arm_config,
                {"serial_no": ParameterValue(arm_serial_no, value_type=str)},
            ],
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace="left/camera",
            name="usb_cam",
            output="screen",
            parameters=[
                left_config,
                {"video_device": ParameterValue(left_device, value_type=str)},
            ],
        ),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace="right/camera",
            name="usb_cam",
            output="screen",
            parameters=[
                right_config,
                {"video_device": ParameterValue(right_device, value_type=str)},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + cameras)
