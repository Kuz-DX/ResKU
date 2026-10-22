"""Offline trajectory checks; no ROS nodes or motor commands are created."""
import math
import sys
import unittest
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from move_to_named_pose import ARM_JOINTS, make_smooth_points, make_startup_points, recovery_target, NamedPoseRunner


class NamedPoseTrajectoryTest(unittest.TestCase):
    def test_rest_to_rest_motion_stays_within_speed_and_position_bounds(self):
        current = dict.fromkeys(ARM_JOINTS, 0.0)
        target = [1.0, -1.0, 0.5, -0.5, 0.0, 0.1]
        speeds = dict.fromkeys(ARM_JOINTS, 0.3)
        points, duration = make_smooth_points(current, target, 2.0, speeds)
        self.assertAlmostEqual(duration, 12.5)
        self.assertEqual(list(points[0].positions), [0.0] * 6)
        self.assertEqual(list(points[-1].positions), target)
        for endpoint in (points[0], points[-1]):
            self.assertEqual(list(endpoint.velocities), [0.0] * 6)
            self.assertEqual(list(endpoint.accelerations), [0.0] * 6)
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        self.assertAlmostEqual(times[-1], duration)
        for point in points:
            for index, name in enumerate(ARM_JOINTS):
                self.assertLessEqual(abs(point.velocities[index]), speeds[name] * 0.5 + 1e-12)
                self.assertGreaterEqual(point.positions[index], min(0.0, target[index]) - 1e-12)
                self.assertLessEqual(point.positions[index], max(0.0, target[index]) + 1e-12)

    def test_long_requested_duration_and_stationary_joints(self):
        current = dict.fromkeys(ARM_JOINTS, 0.4)
        points, duration = make_smooth_points(current, [0.4] * 6, 8.0,
                                              dict.fromkeys(ARM_JOINTS, 0.3))
        self.assertEqual(duration, 8.0)
        self.assertTrue(all(list(p.positions) == [0.4] * 6 for p in points))

    def test_startup_recovers_only_outside_joints_before_target(self):
        current = dict(zip(ARM_JOINTS, [-1.5, 1.4, 0.2, 0.0, -1.0, 1.0]))
        limits = dict.fromkeys(ARM_JOINTS, (-1.0, 1.0))
        speeds = dict.fromkeys(ARM_JOINTS, 0.3)
        points, duration, boundary = make_startup_points(
            current, [0.0] * 6, 2.0, speeds, limits)
        expected = [-1.0, 1.0, 0.2, 0.0, -1.0, 1.0]
        index = next(i for i, p in enumerate(points) if list(p.positions) == expected)
        self.assertEqual(list(points[0].positions), list(current.values()))
        self.assertEqual(list(points[-1].positions), [0.0] * 6)
        self.assertEqual(list(points[index].velocities), [0.0] * 6)
        for point in points[:index + 1]:
            self.assertEqual(list(point.positions)[2:], list(current.values())[2:])
            self.assertGreaterEqual(point.positions[0], current[ARM_JOINTS[0]])
            self.assertLessEqual(point.positions[0], -1.0)
            self.assertGreaterEqual(point.positions[1], 1.0)
            self.assertLessEqual(point.positions[1], current[ARM_JOINTS[1]])
        for point in points[index:]:
            self.assertTrue(all(-1.0 <= v <= 1.0 for v in point.positions))
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        for point in points:
            self.assertTrue(all(abs(v) <= 0.15 + 1e-12 for v in point.velocities))

    def test_rmd_servo_margin_leaves_dynamixel_unchanged(self):
        limits = dict.fromkeys(ARM_JOINTS, (-1.0, 1.0))
        current = dict(zip(ARM_JOINTS, [-1.0, 1.4, -1.2, 0.99, 1.0, 0.5]))
        target = recovery_target(current, limits)
        self.assertAlmostEqual(target['shoulder_joint'], 1.0 - math.radians(3))
        self.assertAlmostEqual(target['elbow_joint'], -1.0 + math.radians(3))
        self.assertAlmostEqual(target['wrist_pitch_joint'], 1.0 - math.radians(3))
        for name in ('base_joint', 'wrist_roll_joint', 'wrist_yaw_joint'):
            self.assertEqual(target[name], current[name])
        current['base_joint'] = -1.01
        with self.assertRaisesRegex(ValueError, 'Dynamixel'):
            recovery_target(current, limits)

    def test_recovery_uses_supplied_servo_margin(self):
        limits = dict.fromkeys(ARM_JOINTS, (-1.0, 1.0))
        current = dict.fromkeys(ARM_JOINTS, 0.0)
        current['shoulder_joint'] = 1.1
        target = recovery_target(current, limits, inset=0.08)
        self.assertAlmostEqual(target['shoulder_joint'], 0.92)
        for margin in (-0.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                recovery_target(current, limits, inset=margin)

    def test_recovery_only_sends_goal_and_checks_fresh_feedback(self):
        import move_to_named_pose as module
        runner = Mock()
        runner.run = MethodType(NamedPoseRunner.run, runner)
        current = dict.fromkeys(ARM_JOINTS, 0.0)
        current['shoulder_joint'] = 1.1
        limits = dict.fromkeys(ARM_JOINTS, (-1.0, 1.0))
        target = recovery_target(current, limits)
        stopped = dict.fromkeys(ARM_JOINTS, 0.0)
        runner._wait_for_joint_state.side_effect = [
            (current, stopped), (current, stopped), (target, stopped)]
        runner.client.wait_for_server.return_value = True
        handle = Mock(accepted=True)
        handle.get_result_async.return_value.result.return_value = SimpleNamespace(
            status=module.GoalStatus.STATUS_SUCCEEDED)
        runner.client.send_goal_async.return_value.result.return_value = handle
        with patch.object(module.rclpy, 'spin_until_future_complete'), \
                patch.object(module.rclpy, 'ok', return_value=True):
            result = runner.run(None, 2.0, 1.0, limits, 1.0, 0.02,
                                dict.fromkeys(ARM_JOINTS, 0.3))
        self.assertEqual(result, 0)
        runner.client.send_goal_async.assert_called_once()
        goal = runner.client.send_goal_async.call_args.args[0]
        self.assertEqual(list(goal.trajectory.points[-1].positions), list(target.values()))
        self.assertEqual(runner._wait_for_joint_state.call_count, 3)

    def test_startup_rejects_invalid_feedback(self):
        for value in (float('nan'), float('inf'), None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                make_startup_points(dict.fromkeys(ARM_JOINTS, value), [0.0] * 6,
                                    2.0, dict.fromkeys(ARM_JOINTS, 0.3),
                                    dict.fromkeys(ARM_JOINTS, (-1.0, 1.0)))


if __name__ == '__main__':
    unittest.main()
