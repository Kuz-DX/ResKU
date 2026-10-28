"""Offline checks for the common arm launch construction path."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_wrist_roll_limits_remain_inside_measured_safe_endpoints():
    spec = yaml.safe_load((ROOT/'config/hardware.yaml').read_text())['joints']['wrist_roll_joint']
    raw_low, raw_high = spec['soft_limit_raw']
    assert -628 < raw_low < -572 < raw_high < 443
    scale = 2*math.pi/4096
    assert abs((raw_low-spec['zero_raw'])*scale-spec['soft_limit_rad'][0]) < .001
    assert abs((raw_high-spec['zero_raw'])*scale-spec['soft_limit_rad'][1]) < .002
    measured_start = (-572-spec['zero_raw'])*scale
    assert spec['soft_limit_rad'][0]+.02 < measured_start < spec['soft_limit_rad'][1]-.02
    urdf = ET.parse(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro')
    joint = urdf.find(".//joint[@name='wrist_roll_joint']")
    assert joint is not None
    bounds = joint.find('limit')
    assert [float(bounds.get(k)) for k in ('lower', 'upper')] == spec['soft_limit_rad']
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
