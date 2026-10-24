"""Shared hardware/description/controller construction for arm control modes.

This is an internal launch helper, not a standalone launch entry point.  It
ensures every mode uses one robot description, one controller manager and one
robot state publisher construction path.
"""
from pathlib import Path
from types import SimpleNamespace
import runpy
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def resolve_existing_path(value: str) -> str:
    """Resolve a caller-provided path before xacro changes path context."""
    return str(Path(value).expanduser().resolve(strict=True))


def validate_hardware(hardware_file: str, error_prefix: str = "real control blocked") -> None:
    """Run the package's single hardware preflight without opening devices."""
    helper = (Path(get_package_prefix("tool_manipulator_bringup")) /
              "lib/tool_manipulator_bringup/hardware_preflight.py")
    config = yaml.safe_load(Path(hardware_file).read_text(encoding="utf-8")) or {}
    errors = runpy.run_path(str(helper))["validate"](config)
    if errors:
        raise RuntimeError(f"{error_prefix}: " + "; ".join(errors))


def _mock_velocity_ee(description: dict) -> dict:
    """Make mock ee_joint match the real velocity-only interface."""
    result = dict(description)
    root = ET.fromstring(result["robot_description"])
    command = root.find("ros2_control/joint[@name='ee_joint']/command_interface")
    if command is None:
        raise RuntimeError("Mock EE command interface missing")
    command.set("name", "velocity")
    result["robot_description"] = ET.tostring(root, encoding="unicode")
    return result


def build_arm_base(
    *,
    real_hardware: bool,
    hardware_file: str,
    tool_id: str = "0",
    use_mesh: str = "true",
    mock_initial_positions_file: str = "",
    mock_ee_velocity: bool = False,
    error_prefix: str = "real control blocked",
):
    """Build common description and process actions for one selected mode."""
    hardware_file = resolve_existing_path(hardware_file)

    if real_hardware:
        if mock_initial_positions_file:
            raise RuntimeError(
                "mock_initial_positions_file is only supported with mock hardware")
        validate_hardware(hardware_file, error_prefix)

    bringup_share = Path(get_package_share_directory("tool_manipulator_bringup"))
    moveit_share = Path(get_package_share_directory("tool_manipulator_moveit_config"))

    mappings = {
        "use_mock_hardware": "false" if real_hardware else "true",
        "hardware_config_file": hardware_file,
        "tool_id": tool_id,
        "use_mesh": use_mesh,
    }
    if mock_initial_positions_file:
        mappings["initial_positions_file"] = resolve_existing_path(
            mock_initial_positions_file)

    moveit_config = (
        MoveItConfigsBuilder(
            "tool_manipulator", package_name="tool_manipulator_moveit_config")
        .robot_description(mappings=mappings)
        .to_moveit_configs()
    )
    description = dict(moveit_config.robot_description)
    if not real_hardware and mock_ee_velocity:
        description = _mock_velocity_ee(description)

    controller_files = [
        str(moveit_share / "config/ros2_controllers.yaml"),
        str(bringup_share / "config/manual_controllers.yaml"),
    ]
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[description, *controller_files],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[description],
        output="screen",
    )

    def spawner(name: str, inactive: bool = False):
        arguments = [
            name,
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ]
        if inactive:
            arguments.append("--inactive")
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            output="screen",
        )

    return SimpleNamespace(
        bringup_share=bringup_share,
        hardware_file=hardware_file,
        moveit_config=moveit_config,
        description=description,
        control_node=control_node,
        state_publisher=state_publisher,
        spawner=spawner,
    )
