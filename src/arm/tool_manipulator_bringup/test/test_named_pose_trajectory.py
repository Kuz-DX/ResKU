"""Offline trajectory checks; no ROS nodes or motor commands are created."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from move_to_named_pose import ARM_JOINTS, make_smooth_points


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


if __name__ == '__main__':
    unittest.main()
