"""Inspect the core real-control launch without opening hardware."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "real_control_launch", ROOT / "launch/real_control.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def actions(**overrides):
    values = dict(
        hardware_config=str(ROOT / "config/hardware.yaml"),
        tool_id="0",
        use_mesh="true",
        move_home_on_start="false",
        home_duration="8.0",
        servo_config=str(ROOT / "config/servo.yaml"),
    )
    values.update(overrides)
    config = SimpleNamespace(robot_description={})
    builder = Mock()
    builder.robot_description.return_value.to_moveit_configs.return_value = config
    with patch.multiple(
        module,
        LaunchConfiguration=lambda name: SimpleNamespace(perform=lambda ctx: values[name]),
        MoveItConfigsBuilder=Mock(return_value=builder),
        get_package_share_directory=lambda name: str(ROOT),
        _validate=lambda path: None,
        generate_move_group_launch=lambda config: SimpleNamespace(entities=[]),
        RegisterEventHandler=lambda handler: dict(kind="handler", **handler),
        OnProcessExit=lambda **kwargs: kwargs,
        Node=lambda **kwargs: dict(kind="node", **kwargs),
        TimerAction=lambda **kwargs: dict(kind="timer", **kwargs),
    ):
        return module._start_real(None)


class RealControlLaunchTest(unittest.TestCase):
    def setUp(self):
        logger = patch.object(module, "LogInfo", lambda **kwargs: dict(kind="log", **kwargs))
        logger.start()
        self.addCleanup(logger.stop)

    def test_core_controllers_and_move_group_are_started(self):
        result = actions()
        executables = [item.get("executable") for item in result if item.get("kind") == "node"]
        self.assertIn("ros2_control_node", executables)
        self.assertIn("robot_state_publisher", executables)
        self.assertEqual(sum(item.get("kind") == "timer" for item in result), 4)

    def test_recovery_runs_even_without_home(self):
        result = actions()
        gate = next(item for item in result if item.get("kind") == "handler")
        startup = gate["on_exit"](SimpleNamespace(returncode=0), None)[0]
        self.assertEqual(startup["executable"], "move_to_named_pose.py")
        self.assertIn("--recover-only", startup["arguments"])
        self.assertNotIn("home", startup["arguments"])

    def test_home_argument_is_selected_when_requested(self):
        result = actions(move_home_on_start="true")
        gate = next(item for item in result if item.get("kind") == "handler")
        startup = gate["on_exit"](SimpleNamespace(returncode=0), None)[0]
        self.assertIn("home", startup["arguments"])
        self.assertNotIn("--recover-only", startup["arguments"])

    def test_failed_controller_activation_blocks_startup_motion(self):
        result = actions()
        gate = next(item for item in result if item.get("kind") == "handler")
        blocked = gate["on_exit"](SimpleNamespace(returncode=1), None)
        self.assertEqual(blocked[0]["kind"], "log")


if __name__ == "__main__":
    unittest.main()
