"""Launch the drive RealSense camera without a workspace-path dependency."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config = str(
        Path(get_package_share_directory("vision"))
        / "config"
        / "camera"
        / "drive_d435i.yaml"
    )
    serial_no = LaunchConfiguration("serial_no")
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="_117222251401",
            description="Drive RealSense serial number",
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="drive",
            name="camera",
            output="screen",
            emulate_tty=True,
            parameters=[
                config,
                {"serial_no": ParameterValue(serial_no, value_type=str)},
            ],
        ),
    ])
