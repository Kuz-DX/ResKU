import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from zero_calibration import CalibrationError, plan_zero_update  # noqa: E402


class ZeroCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "joints": {
                "base_joint": {
                    "zero_raw": 100,
                    "soft_limit_raw": [-900, 1200],
                    "soft_limit_rad": [-1.0, 1.1],
                },
                "shoulder_joint": {
                    "q_offset_rad": 0.25,
                    "soft_limit_rad": [-1.2, 1.3],
                },
            }
        }

    def test_dxl_zero_shifts_absolute_raw_limits_only(self):
        updated, changes = plan_zero_update(
            self.config, ["base_joint"], {}, {"base_joint": 125})

        base = updated["joints"]["base_joint"]
        self.assertEqual(base["zero_raw"], 125)
        self.assertEqual(base["soft_limit_raw"], [-875, 1225])
        self.assertEqual(base["soft_limit_rad"], [-1.0, 1.1])
        self.assertEqual(changes[0].new_raw_limits, (-875, 1225))
        self.assertEqual(self.config["joints"]["base_joint"]["zero_raw"], 100)

    def test_rmd_current_raw_angle_becomes_offset(self):
        updated, _ = plan_zero_update(
            self.config, ["shoulder_joint"], {"shoulder_joint": 90.0}, {})

        shoulder = updated["joints"]["shoulder_joint"]
        self.assertAlmostEqual(shoulder["q_offset_rad"], math.pi / 2.0)
        self.assertEqual(shoulder["soft_limit_rad"], [-1.2, 1.3])

    def test_invalid_raw_limits_fail_closed(self):
        self.config["joints"]["base_joint"]["soft_limit_raw"] = [10]
        with self.assertRaises(CalibrationError):
            plan_zero_update(
                self.config, ["base_joint"], {}, {"base_joint": 125})


if __name__ == "__main__":
    unittest.main()
