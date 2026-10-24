"""Ensure the working manual launch keeps its startup/controller contract."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "manual_launch", ROOT / "launch/manual.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_manual_mode_uses_common_base_and_inactive_seed_controller():
    values = {
        "real_hardware": "false",
        "hardware_config": str(ROOT / "config/hardware.yaml"),
        "mock_initial_positions_file": "",
        "launch_joy": "false",
        "launch_rviz": "false",
    }
    spawner = Mock(side_effect=lambda name, inactive=False: {
        "kind": "spawner", "name": name, "inactive": inactive})
    nodes = []

    def node(**kwargs):
        value = {"kind": "node", **kwargs}
        nodes.append(value)
        return value

    base = SimpleNamespace(
        bringup_share=ROOT,
        hardware_file=values["hardware_config"],
        description={"robot_description": ROBOT_DESCRIPTION},
        control_node={"kind": "node", "executable": "ros2_control_node"},
        state_publisher={"kind": "node", "executable": "robot_state_publisher"},
        spawner=spawner,
    )
    with patch.object(module.common, "build_arm_base", return_value=base) as build, patch.multiple(
            module,
            LaunchConfiguration=lambda name: SimpleNamespace(perform=lambda ctx: values[name]),
            Node=node,
            RegisterEventHandler=lambda handler: {"kind": "handler", **handler},
            OnProcessExit=lambda **kwargs: kwargs,
            OnProcessIO=lambda **kwargs: kwargs,
            LogInfo=lambda **kwargs: {"kind": "log", **kwargs}):
        result = module._bringup(None)

    build.assert_called_once_with(
        real_hardware=False,
        hardware_file=values["hardware_config"],
        mock_initial_positions_file="",
        mock_ee_velocity=True,
        error_prefix="Servo-free real control blocked",
    )
    assert spawner.call_args_list[0].args == ("joint_state_broadcaster",)
    assert spawner.call_args_list[1].args == ("ee_velocity_controller",)
    assert spawner.call_args_list[2].args == ("manual_position_controller",)
    assert spawner.call_args_list[2].kwargs == {"inactive": True}
    manual = next(item for item in nodes if item.get("executable") == "manual.py")
    assert manual["parameters"][0] == base.description


ROBOT_DESCRIPTION = "<robot name='test'/>"
