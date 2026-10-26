"""Launch the arm D435i without depending on a user's workspace path."""

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
        / "arm_d435i.yaml"
    )
    serial_no = LaunchConfiguration("serial_no")
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_no",
            default_value="_339222071362",
            description="Arm D435i serial number",
        ),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="arm",
            name="camera",
            output="screen",
            emulate_tty=True,
            parameters=[
                config,
                {"serial_no": ParameterValue(serial_no, value_type=str)},
            ],
        ),
    ])
