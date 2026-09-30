"""Launch the taught semi-automatic supply-box delivery UI."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("tool_manipulator_bringup")) / "config"
    return LaunchDescription([
        Node(
            package="tool_manipulator_bringup",
            executable="manual_docking_path_ui.py",
            name="semi_auto_supply_delivery_ui",
            output="screen",
            arguments=[
                "--sequence", "supplybox_delivery",
                "--paths", str(config / "manual_docking_paths.yaml"),
                "--hardware", str(config / "hardware.yaml"),
            ],
        )
    ])
