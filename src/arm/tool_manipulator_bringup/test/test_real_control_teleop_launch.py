"""Inspect launch actions with hardware and process creation mocked out."""
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
    values = dict(hardware_config=str(ROOT / "config/hardware.yaml"), tool_id="0",
                  use_mesh="true", move_home_on_start="false", home_duration="8.0", launch_servo="false",
                  launch_tcp_joy_teleop="false", launch_joint_joy_teleop="false",
                  auto_start_servo="false", launch_joy="false",
                  servo_config=str(ROOT / "config/servo.yaml"))
    values.update(overrides)
    config = SimpleNamespace(robot_description={}, robot_description_semantic={},
                             robot_description_kinematics={})
    builder = Mock()
    builder.robot_description.return_value.to_moveit_configs.return_value = config
    with patch.multiple(
        module,
        LaunchConfiguration=lambda name: SimpleNamespace(perform=lambda ctx: values[name]),
        MoveItConfigsBuilder=Mock(return_value=builder),
        get_package_share_directory=lambda name: str(ROOT),
        generate_move_group_launch=lambda config: SimpleNamespace(entities=[]),
        RegisterEventHandler=lambda handler: dict(kind="handler", **handler),
        OnProcessExit=lambda **kwargs: kwargs,
        Node=lambda **kwargs: dict(kind="node", **kwargs),
        ExecuteProcess=lambda **kwargs: dict(kind="process", **kwargs),
        TimerAction=lambda **kwargs: dict(kind="timer", **kwargs),
    ):
        return module._start_real(None)


class RealControlTeleopLaunchTest(unittest.TestCase):
    def setUp(self):
        logger = patch.object(module, "LogInfo", lambda **kwargs: dict(kind="log", **kwargs))
        logger.start()
        self.addCleanup(logger.stop)

    def test_bringup_defaults_do_not_start_teleop(self):
        result = actions()
        self.assertFalse(any(a.get("period") == 8.0 for a in result))

    def test_launch_arm_starts_servo_joy_joint_teleop_and_start_service(self):
        result = actions(launch_servo="true", launch_joy="true",
                         launch_joint_joy_teleop="true", auto_start_servo="true")
        gate = next(a for a in result if a.get("kind") == "handler" and
                    a["target_action"].get("executable") == "move_to_named_pose.py")
        delayed = gate["on_exit"](SimpleNamespace(returncode=0), None)
        self.assertEqual(len(gate["on_exit"](SimpleNamespace(returncode=1), None)), 1)
        self.assertEqual([a["executable"] for a in delayed if a["kind"] == "node"],
                         ["servo_node_main", "joy_node", "joint_joy_teleop.py"])
        self.assertEqual(delayed[-1]["cmd"],
                         ["ros2", "service", "call", "/servo_node/start_servo",
                          "std_srvs/srv/Trigger", "{}"])

    def test_recovery_runs_even_without_home(self):
        result = actions()
        gate = next(a for a in result if a.get("kind") == "handler")
        startup = gate["on_exit"](SimpleNamespace(returncode=0), None)[0]
        self.assertEqual(startup["executable"], "move_to_named_pose.py")
        self.assertIn("--recover-only", startup["arguments"])
        self.assertNotIn("home", startup["arguments"])
        self.assertNotEqual(gate["on_exit"](SimpleNamespace(returncode=1), None), [startup])

    def test_servo_required_for_both_teleop_modes_and_auto_start(self):
        for option in ("launch_joint_joy_teleop", "launch_tcp_joy_teleop", "auto_start_servo"):
            with self.subTest(option=option), self.assertRaisesRegex(RuntimeError, "requires launch_servo"):
                actions(**{option: "true"})

    def test_conflicting_teleop_modes_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "select only one"):
            actions(launch_servo="true", launch_joint_joy_teleop="true",
                    launch_tcp_joy_teleop="true")


if __name__ == "__main__":
    unittest.main()
