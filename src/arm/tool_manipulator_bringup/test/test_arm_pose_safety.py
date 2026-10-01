#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from arm_pose_safety import pose_errors, validate_joint_positions  # noqa: E402


class ArmPoseSafetyTest(unittest.TestCase):
    def setUp(self):
        self.limits = {"a": (-1.0, 1.0), "b": (-2.0, 2.0)}

    def test_accepts_pose_with_margin(self):
        validate_joint_positions({"a": 0.0, "b": 1.5}, self.limits, 0.1)

    def test_rejects_pose_inside_limit_but_outside_margin(self):
        with self.assertRaisesRegex(ValueError, "a=0.950000"):
            validate_joint_positions({"a": 0.95, "b": 0.0}, self.limits, 0.1)

    def test_rejects_missing_joint(self):
        with self.assertRaisesRegex(ValueError, "b: finite position is required"):
            validate_joint_positions({"a": 0.0}, self.limits)

    def test_pose_errors(self):
        self.assertEqual(
            pose_errors({"a": 0.1, "b": -0.2}, {"a": 0.4, "b": 0.0}, ("a", "b")),
            {"a": 0.30000000000000004, "b": 0.2},
        )


if __name__ == "__main__":
    unittest.main()
