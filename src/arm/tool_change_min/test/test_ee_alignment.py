"""Exercise real EE limits, phase selection, and fail-closed edge cases."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.ee_alignment import EeAlignment


class EeAlignmentTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.model = EeAlignment(
            root / "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro",
            root / "tool_manipulator_bringup/config/hardware.yaml")

    def test_current_limits_allow_only_2048_for_zero_phase(self):
        for raw in (500, 1992, 2363, 2920, 3800):
            self.assertAlmostEqual(
                self.model.target(self.model.to_rad(raw), 0, 0.01),
                self.model.to_rad(2048))

    def test_outside_start_rejected_even_if_target_exists(self):
        with self.assertRaisesRegex(ValueError, "current EE"):
            self.model.target(self.model.to_rad(1), 0, 0.01)

    def test_nearest_candidate_with_multiple_turns(self):
        self.model.lower, self.model.upper = -10, 10
        ref = self.model.to_rad(0)
        for n in (-1, 0, 1, 2):
            expected = ref + n * math.pi
            self.assertAlmostEqual(self.model.target(expected + 0.2, 0, 0), expected)

    def test_no_candidate_rejected(self):
        self.model.lower, self.model.upper = 0.5, 0.6
        with self.assertRaisesRegex(ValueError, "no EE"):
            self.model.target(0.55, 0, 0)

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            self.model.target(float("nan"), 0, 0)


if __name__ == "__main__":
    unittest.main()
