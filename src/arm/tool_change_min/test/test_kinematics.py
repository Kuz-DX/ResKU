"""Known-joint FK -> IK restoration check for the shared tool-manipulator URDF."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.kinematics import ArmKinematics, _bounded_damped_step  # noqa: E402


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

    def test_ik_limit_error_names_candidate_and_allowed_range(self):
        q = np.array([(lo+hi)/2 for lo, hi in self.model.limits.values()])
        q[0] = self.model.limits['base_joint'][1] - .02
        with patch.object(self.model, 'pose_error', return_value=np.array([1., 0, 0, 0, 0, 0])), \
                patch.object(self.model, 'jacobian', return_value=np.eye(6)):
            with self.assertRaisesRegex(ValueError,
                                        r'base_joint candidate=.*allowed=\[.*\] rad'):
                self.model.solve(np.eye(4), q, margin_rad=.01)

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

    def test_bounded_step_uses_other_joint_when_one_reaches_bound(self):
        step = _bounded_damped_step(np.array([[1., 1.]]), np.array([1.]),
                                    np.array([-.2, -2.]), np.array([.2, 2.]), .003)
        self.assertAlmostEqual(step[0], .2)
        self.assertAlmostEqual(step[1], .8, places=4)

    def test_axis_ik_accepts_different_axial_rotation(self):
        known = np.array([.2, -.3, .4, -.25, .1, .2])
        target = self.model.fk(known)
        rotated = known.copy()
        rotated[-1] += .4
        result = self.model.solve_axis(target[:3, 3], target[:3, 2], [0, 0, 1], rotated,
                                       .02, max_time_s=.5)
        np.testing.assert_allclose(result, rotated)
        self.assertGreater(np.linalg.norm(self.model.pose_error(result, target)[3:]), .3)

    def test_axis_ik_solves_translation_with_position_and_direction_verified(self):
        q = np.array([.2, -.3, .4, -.25, .1, .2])
        origin = self.model.fk(q)
        for index in range(1, 6):
            position = origin[:3, 3] + origin[:3, 2] * .0002 * index
            previous = q.copy()
            q = self.model.solve_axis(position, origin[:3, 2], [0, 0, 1], q, .02,
                                      max_time_s=.5)
            actual = self.model.fk(q)
            self.assertLess(np.linalg.norm(position-actual[:3, 3]), 1e-5)
            self.assertLess(np.linalg.norm(origin[:3, 2]-actual[:3, 2]), 1e-4)
            self.assertLessEqual(np.max(np.abs(q-previous)), .03+1e-12)
            self.assertTrue(self.model.within_limits(q, .02))

    def test_logged_tool1_advance_still_rejects_unreachable_axis_path(self):
        q = np.array([3.2858, .4927, 1.4024, -1.5429, -.4709, -1.9558])
        origin = self.model.fk(q)
        normal = np.array([.5388, .6549, -.5299])
        normal /= np.linalg.norm(normal)
        first = origin[:3, 2]
        angle = np.arccos(first @ normal)
        with self.assertRaisesRegex(ValueError, 'within joint bounds'):
            for index in range(1, 39):
                fraction = index/38
                direction = (np.sin((1-fraction)*angle)*first + np.sin(fraction*angle)*normal)/np.sin(angle)
                q = self.model.solve_axis(origin[:3, 3]+normal*.075*fraction, direction,
                                          [0, 0, 1], q, .02, max_time_s=.5)
                self.assertTrue(self.model.within_limits(q, .02))


if __name__ == "__main__":
    unittest.main()
