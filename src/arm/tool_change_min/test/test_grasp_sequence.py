"""Unit tests for ROS-independent supply grasp calculations."""

import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.grasp_sequence import (  # noqa: E402
    checked_joint_vector,
    fixed_grasp_z,
    pitch_offset_quaternion,
    required_duration,
)


class GraspSequenceTest(unittest.TestCase):
    def test_fixed_z_matches_legacy_geometry_contract(self):
        self.assertAlmostEqual(fixed_grasp_z(0.350, 0.095, 0.5, -0.0055), -0.3080)

    def test_pitch_offset_is_normalized(self):
        quaternion = pitch_offset_quaternion((0.0, 0.0, 0.0, 1.0), math.pi / 2.0)
        self.assertAlmostEqual(sum(value * value for value in quaternion), 1.0)
        self.assertAlmostEqual(quaternion[1], math.sqrt(0.5))
        self.assertAlmostEqual(quaternion[3], math.sqrt(0.5))

    def test_joint_vector_rejects_limit_violation(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            checked_joint_vector(("joint",), [1.1], {"joint": (-1.0, 1.0)}, "test")

    def test_duration_honors_slowest_hardware_joint(self):
        duration = required_duration(
            ("fast", "slow"), (0.0, 0.0), (0.2, 0.4),
            {"fast": 1.0, "slow": 0.2}, requested_s=1.0, padding_s=0.5)
        self.assertAlmostEqual(duration, 2.5)


if __name__ == "__main__":
    unittest.main()
