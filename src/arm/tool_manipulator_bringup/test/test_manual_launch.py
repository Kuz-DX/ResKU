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
        moveit_config=SimpleNamespace(
            to_dict=lambda: {},
            robot_description_semantic={'robot_description_semantic': '<robot name="test"/>'}),
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
    assert not any(item.get('executable') == 'move_group' for item in nodes)
    assert base.moveit_config.robot_description_semantic['robot_description_semantic'] == '<robot name="test"/>'


ROBOT_DESCRIPTION = "<robot name='test'/>"


def test_real_rmd_transport_keeps_can_bus_headroom():
    control = (ROOT.parent/'tool_manipulator_moveit_config/config/'
               'tool_manipulator.ros2_control.xacro').read_text()
    assert '<param name="cycle_time">5</param>' in control
    assert '<param name="can_receive_timeout_ms">50</param>' in control
    assert '<param name="status1_period_ms">500</param>' in control
    assert '<param name="cycle_time">2</param>' not in control
