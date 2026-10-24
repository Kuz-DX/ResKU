"""Offline checks for the common arm launch construction path."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "arm_bringup_common", ROOT / "launch/arm_bringup_common.py")
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


ROBOT = """<robot name="test"><ros2_control name="mock" type="system">
  <joint name="ee_joint"><command_interface name="position"/></joint>
</ros2_control></robot>"""


def fake_builder(description=ROBOT):
    config = SimpleNamespace(robot_description={"robot_description": description})
    builder = Mock()
    builder.robot_description.return_value.to_moveit_configs.return_value = config
    return builder, config


def test_mock_manual_uses_velocity_ee_and_common_processes():
    builder, config = fake_builder()
    nodes = []

    def node(**kwargs):
        value = dict(kwargs)
        nodes.append(value)
        return value

    with patch.object(common, "MoveItConfigsBuilder", return_value=builder), \
            patch.object(common, "Node", side_effect=node), \
            patch.object(common, "get_package_share_directory", return_value=str(ROOT)):
        result = common.build_arm_base(
            real_hardware=False,
            hardware_file=str(ROOT / "config/hardware.yaml"),
            mock_ee_velocity=True,
        )
        inactive = result.spawner("manual_position_controller", inactive=True)

    mappings = builder.robot_description.call_args.kwargs["mappings"]
    assert mappings["use_mock_hardware"] == "true"
    assert 'command_interface name="velocity"' in result.description["robot_description"]
    assert result.moveit_config is config
    assert [node["executable"] for node in nodes] == [
        "ros2_control_node", "robot_state_publisher", "spawner"]
    assert inactive["arguments"][-1] == "--inactive"


def test_real_mode_runs_shared_preflight_and_passes_tool_mapping():
    builder, _ = fake_builder()
    hardware = str(ROOT / "config/hardware.yaml")
    with patch.object(common, "MoveItConfigsBuilder", return_value=builder), \
            patch.object(common, "Node", side_effect=lambda **kwargs: kwargs), \
            patch.object(common, "get_package_share_directory", return_value=str(ROOT)), \
            patch.object(common, "validate_hardware") as validate:
        common.build_arm_base(
            real_hardware=True,
            hardware_file=hardware,
            tool_id="1",
            use_mesh="false",
        )

    validate.assert_called_once_with(str(Path(hardware).resolve()), "real control blocked")
    mappings = builder.robot_description.call_args.kwargs["mappings"]
    assert mappings["use_mock_hardware"] == "false"
    assert mappings["tool_id"] == "1"
    assert mappings["use_mesh"] == "false"


def test_real_mode_rejects_mock_initial_positions_before_hardware_access():
    with pytest.raises(RuntimeError, match="only supported with mock hardware"), \
            patch.object(common, "validate_hardware") as validate:
        common.build_arm_base(
            real_hardware=True,
            hardware_file=str(ROOT / "config/hardware.yaml"),
            mock_initial_positions_file=str(ROOT / "test/storage_pose_initial_positions.yaml"),
        )
    validate.assert_not_called()
