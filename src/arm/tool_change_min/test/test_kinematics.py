"""Known-joint FK -> IK restoration check for the shared tool-manipulator URDF."""
from pathlib import Path
import sys
import unittest
import importlib.util

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.kinematics import ArmKinematics  # noqa: E402


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.model = ArmKinematics(
            root / "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro",
            root / "tool_manipulator_bringup/config/hardware.yaml")

    def test_manual_raw_captures_fit_with_jog_margin(self):
        package = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            "convert_raw", package / "scripts/convert_raw_poses.py")
        converter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(converter)
        hardware = yaml.safe_load((package.parent /
            "tool_manipulator_bringup/config/hardware.yaml").read_text())
        captures = {
            "joint_names": [*self.model.joint_names, "ee_joint"],
            "raw_units": ["raw_pulse", "raw_deg", "raw_deg", "raw_deg",
                          "raw_pulse", "raw_pulse", "raw_pulse"],
            "captures": {
                "d": {"raw": [-27, 1.82, -87.339996, 143.020004, 14, 49, 3365]},
                "l": {"raw": [8, 2.0, -86.580002, 143.209991, -56, 75, 3365]},
            },
        }
        converted, errors = converter.convert(captures, hardware, self.model.joint_names, 0.01)
        self.assertEqual(errors, [])
        for sample in converted.values():
            self.assertTrue(self.model.within_limits(
                [sample[name] for name in self.model.joint_names], 0.01))

    def test_fk_then_ik_restores_known_pose(self):
        known = np.array([0.35, -0.35, 0.45, -0.40, 0.12, 0.55])
        target = self.model.fk(known)
        recovered = self.model.solve(
            target, known + np.array([0.02, -0.02, 0.02, -0.02, 0.01, -0.02]))
        error = self.model.pose_error(recovered, target)
        self.assertLess(np.linalg.norm(error[:3]), 1.0e-5)
        self.assertLess(np.linalg.norm(error[3:]), 1.0e-4)

    def test_pose_path_waypoints_follow_cartesian_line(self):
        start = np.array([0.20, -0.30, 0.40, -0.25, 0.10, 0.20])
        target_joints = start + np.array([0.01, -0.015, 0.01, -0.01, 0.005, 0.01])
        start_pose = self.model.fk(start)
        target_pose = self.model.fk(target_joints)
        path = self.model.linear_path_to_pose(
            start, target_pose, step_m=0.002, orientation_step_rad=0.03,
            margin_rad=0.0)

        translation = target_pose[:3, 3] - start_pose[:3, 3]
        for index, joints in enumerate(path, start=1):
            expected = start_pose[:3, 3] + translation * index / len(path)
            actual = self.model.fk(joints)[:3, 3]
            self.assertLess(np.linalg.norm(actual - expected), 1.0e-5)
            self.assertTrue(self.model.within_limits(joints))
        endpoint_error = self.model.pose_error(path[-1], target_pose)
        self.assertLess(np.linalg.norm(endpoint_error[:3]), 1.0e-5)
        self.assertLess(np.linalg.norm(endpoint_error[3:]), 1.0e-4)


if __name__ == "__main__":
    unittest.main()
