"""Passive message mapping, missing schemas and quiet idle checks."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from control_msgs.msg import JointJog, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

spec = importlib.util.spec_from_file_location('trajectory_diagnostics',
    Path(__file__).resolve().parents[1] / 'scripts/trajectory_diagnostics.py')
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


class DiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.time = 1.
        self.logger = Mock()
        self.node = NS(create_subscription=Mock(), create_timer=Mock(),
                       get_logger=lambda: self.logger)
        self.d = diag.TrajectoryDiagnostics(self.node, '/jog', '/joint_states', lambda: self.time)

    def motion(self):
        self.d.on_jog(JointJog(joint_names=['wrist_yaw_joint', 'base_joint'], velocities=[-.12, .12]))

    def test_idle_is_silent_and_no_publisher_exists(self):
        self.d.report()
        self.logger.info.assert_not_called()
        self.assertEqual(self.node.create_subscription.call_count, 5)
        self.assertEqual(self.node.create_timer.call_args.args[0], .5)

    def test_permuted_names_delta_and_current_humble_state_schema(self):
        self.motion()
        msg = JointTrajectory(joint_names=['wrist_yaw_joint', 'base_joint'],
                              points=[JointTrajectoryPoint(positions=[-.1, .2])])
        self.d.on_trajectory(msg)
        self.time += .05
        msg.points[0].positions = [-.106, .206]
        self.d.on_trajectory(msg)
        state = JointTrajectoryControllerState(joint_names=['base_joint'])
        state.desired.positions = [.205]
        state.actual.positions = [.201]
        state.error.positions = [.004]
        self.d.on_controller(state, '/arm_controller/state')
        self.d.on_joints(JointState(name=['base_joint'], position=[.201], velocity=[.01]))
        self.d.report()
        lines = [c.args[0] for c in self.logger.info.call_args_list]
        self.assertIn('traj_index=1', lines[0])
        self.assertIn('dp_msg=0.006000000 expected_dp=0.006000000', lines[0])
        self.assertIn('desired=0.205000000 actual=0.201000000', lines[0])
        self.assertIn('js_p=0.201000000 js_v=0.010000000', lines[0])
        self.assertIn('traj_index=None', lines[1])
        self.assertIn('p0=NA v0=NA', lines[1])
        self.assertIn('expected_dp=-0.006000000', lines[2])

    def test_reference_preferred_and_missing_field_is_na(self):
        msg = NS(reference=NS(positions=[.8]), desired=NS(positions=[.2]))
        self.assertEqual(diag.state_point(msg, 'reference', 'desired')[1], 'reference')
        self.assertIsNone(diag.field(None, 'positions', 0))
        self.assertIsNone(diag.number([float('nan')], 0))

    def test_empty_trajectory_and_stop_tail_then_silence(self):
        self.motion()
        self.d.on_trajectory(JointTrajectory())
        self.d.report()
        self.assertEqual(self.logger.info.call_count, 4)
        self.assertIn('p0=NA', self.logger.info.call_args_list[0].args[0])
        self.time += 1.1
        self.logger.info.reset_mock()
        self.d.report()
        self.logger.info.assert_not_called()


if __name__ == '__main__':
    unittest.main()
