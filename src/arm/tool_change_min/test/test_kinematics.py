"""Known-joint FK -> IK restoration check for the shared tool-manipulator URDF."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.kinematics import ArmKinematics  # noqa: E402


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[2]
        self.model = ArmKinematics(
            root / "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro",
            root / "tool_manipulator_bringup/config/hardware.yaml")

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
