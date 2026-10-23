"""Numerical and stop-path checks without hardware or a live ROS graph."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from builtin_interfaces.msg import Time

script = Path(__file__).resolve().parents[1]/'scripts/manual_controller.py'
if not script.exists():
    script = Path(__file__).with_name('manual_controller.py')
spec = importlib.util.spec_from_file_location('manual_under_test', script)
manual = importlib.util.module_from_spec(spec)
spec.loader.exec_module(manual)


def planar_robot():
    # Deliberately include a non-RMD base and distal wrist yaw in FK.
    parts = ['<robot name="test">']
    parent = 'base_actuator'
    for index, name in enumerate(manual.JOINTS):
        child = 'tcp_link' if index == 5 else f'link{index}'
        axis = '0 0 1' if index in (0, 4, 5) else '0 1 0'
        parts.append(f'<joint name="{name}" type="revolute"><parent link="{parent}"/>'
                     f'<child link="{child}"/><origin xyz="0.2 0 0"/>'
                     f'<axis xyz="{axis}"/></joint>')
        parent = child
    return ''.join(parts) + '</robot>'


class KinematicsTest(unittest.TestCase):
    def test_jacobian_matches_finite_difference_and_uses_base_pose(self):
        chain = manual.ArmChain(planar_robot())
        positions = dict(zip(manual.JOINTS, [.4, -.3, .6, -.2, .2, .1]))
        transform, jacobian = chain.pose_and_jacobian(positions)
        for column, name in enumerate(manual.RMD):
            moved = dict(positions)
            moved[name] += 1e-6
            tip, _ = chain.pose_and_jacobian(moved)
            np.testing.assert_allclose((tip[:3, 3]-transform[:3, 3])/1e-6,
                                       jacobian[:, column], atol=1e-6)
        moved = dict(positions, base_joint=0.)
        self.assertFalse(np.allclose(chain.pose_and_jacobian(moved)[1], jacobian))

    def test_velocity_changes_with_posture_and_respects_each_speed_limit(self):
        chain = manual.ArmChain(planar_robot())
        a = dict(zip(manual.JOINTS, [0., -.3, .6, -.2, 0., 0.]))
        b = dict(a, shoulder_joint=.4, elbow_joint=-.7)
        limits = np.array([.04, .07, .03])
        velocities = [manual.coordinated_velocity(chain.pose_and_jacobian(q)[1],
                      [.03, 0., .02], .02, limits) for q in (a, b)]
        self.assertFalse(np.allclose(*velocities))
        for velocity in velocities:
            self.assertTrue(np.all(np.abs(velocity) <= limits+1e-12))

    def test_singularity_and_zero_input_are_finite(self):
        np.testing.assert_array_equal(manual.coordinated_velocity(np.zeros((3, 3)),
            [1., 0., 0.], .05, [.1]*3), [0.]*3)
        np.testing.assert_array_equal(manual.coordinated_velocity(np.eye(3),
            [0.]*3, .05, [.1]*3), [0.]*3)

    def test_boundaries_keep_joint_ratio_and_allow_inward_motion(self):
        lo, hi = np.array([-1.]*3), np.array([1.]*3)
        target = np.array([.97, 0., 0.])
        out = manual.bounded_step(target, [.1, .2, -.1], lo, hi, 1., .02)
        np.testing.assert_allclose(out-target, [.01, .02, -.01])
        inside = manual.bounded_step(np.array([.99, 0., 0.]), [-.1, 0., 0.], lo, hi, 1., .02)
        self.assertLess(inside[0], .99)

    def test_nonfinite_solver_input_rejected(self):
        with self.assertRaises(ValueError):
            manual.coordinated_velocity(np.eye(3), [float('nan'), 0., 0.], .05, [.1]*3)


def harness():
    node = manual.ManualController.__new__(manual.ManualController)
    node.params = {'joint_state_timeout_sec': .5, 'joy_timeout_sec': .25,
                   'publish_rate_hz': 50., 'max_target_lead_rad': .1,
                   'invert_forward': False, 'invert_wrist_yaw': True,
                   'invert_wrist_roll': False, 'invert_ee_velocity': False,
                   'ee_velocity_rad_s': .12,
                   'linear_speed_mps': .03, 'damping': .05, 'limit_margin_rad': .02}
    node.now_seconds = lambda: 1.
    node.last_time = .98
    node.chain = manual.ArmChain(planar_robot())
    node.positions = dict(zip(manual.JOINTS, [0., -.3, .6, -.2, 0., 0.]))
    node.state_times = dict.fromkeys(manual.JOINTS, 1.)
    node.target = np.array(list(node.positions.values()))
    node.lower, node.upper = np.full(6, -2.), np.full(6, 2.)
    node.speeds = np.full(6, .12)
    node.focus, node.fault, node.joy_time = 'arm', False, 1.
    node.publisher = SimpleNamespace(publish=Mock())
    node.jog_publisher = None
    node.ee_velocity_publisher = SimpleNamespace(publish=Mock())
    node.get_logger = Mock()
    node.axis = lambda name: 1. if name == 'forward_axis' else 0.
    node.button = lambda name: False
    return node


class StopPathTest(unittest.TestCase):
    def test_three_dxl_position_jogs_do_not_depend_on_rmd_input(self):
        node = harness()
        node.axis = lambda name: 1. if name in ('wrist_roll_axis', 'wrist_yaw_axis') else 0.
        node.button = lambda name: name == 'base_ccw_button'
        velocity = node.command_velocity(np.array(list(node.positions.values())))
        np.testing.assert_allclose(velocity, [.12, 0., 0., 0., .12, -.12])

    def test_ee_velocity_requires_circle_and_uses_dpad_vertical(self):
        node = harness()
        pressed = set()
        node.button = lambda name: name in pressed
        node.axis = lambda name: 1. if name == 'ee_velocity_axis' else 0.
        self.assertEqual(node.ee_velocity(), 0.)
        pressed.add('ee_enable_button')
        self.assertAlmostEqual(node.ee_velocity(), .12)
        node.axis = lambda name: -1. if name == 'ee_velocity_axis' else 0.
        self.assertAlmostEqual(node.ee_velocity(), -.12)
        pressed.clear()
        self.assertEqual(node.ee_velocity(), 0.)

    def test_servo_input_gap_publishes_zero_joint_jog(self):
        node = harness()
        node.jog_publisher = SimpleNamespace(publish=Mock())
        node._jog_active = True
        node.params['root_link'] = 'base_actuator'
        node.get_clock = Mock(return_value=SimpleNamespace(now=Mock(return_value=SimpleNamespace(to_msg=Mock(return_value=Time())))))
        node.joy_time = .5
        node.on_timer()
        message = node.jog_publisher.publish.call_args.args[0]
        self.assertEqual(message.joint_names, list(manual.JOINTS))
        self.assertEqual(list(message.velocities), [0.] * 6)
        self.assertEqual(list(node.ee_velocity_publisher.publish.call_args.args[0].data), [0.])
        node.on_timer()
        self.assertEqual(node.jog_publisher.publish.call_count, 1)

    def test_options_logs_each_focus_change(self):
        node = harness()
        node.params.update({'manage_focus': True, 'focus_button': 9})
        node.focus_publisher = SimpleNamespace(publish=Mock())
        node.previous_focus = False
        node.joy = None
        node.button = lambda name: bool(node.joy.buttons[node.params[name]])
        node.on_joy(manual.Joy(buttons=[0]*9 + [1]))
        node.get_logger().info.assert_called_once_with(
            'Options selected control mode: drive')
        node.get_logger().info.reset_mock()
        node.on_joy(manual.Joy(buttons=[0]*10))
        node.on_joy(manual.Joy(buttons=[0]*9 + [1]))
        node.get_logger().info.assert_called_once_with(
            'Options selected control mode: arm')

    def test_stale_input_holds_without_integrating(self):
        node = harness()
        before = node.target.copy()
        node.joy_time = .5
        node.on_timer()
        np.testing.assert_array_equal(node.publisher.publish.call_args.args[0].data, before)
        self.assertIsNone(node.target)

    def test_partial_feedback_cannot_mask_stale_joint(self):
        node = harness()
        node.state_times['elbow_joint'] = .1
        node.on_timer()
        self.assertIsNone(node.target)

    def test_feedback_gap_reseeds_without_time_jump(self):
        node = harness()
        node.target = None
        node.last_time = .1
        node.on_timer()
        node.publisher.publish.assert_not_called()
        node.last_time = .98
        node.on_timer()
        self.assertTrue(np.max(np.abs(node.target-np.array(list(node.positions.values())))) < .003)

    def test_following_error_latches_until_explicit_focus_cycle(self):
        node = harness()
        node.target[1] += .15
        node.on_timer()
        self.assertTrue(node.fault)
        node.on_focus(manual.String(data='drive'))
        self.assertFalse(node.fault)
        self.assertIsNone(node.target)

    def test_real_command_coordinates_only_rmd(self):
        node = harness()
        before = node.target.copy()
        node.on_timer()
        np.testing.assert_array_equal(node.target[[0, 4, 5]], before[[0, 4, 5]])
        self.assertTrue(np.any(np.abs(node.target[1:4]-before[1:4]) > 1e-8))

    def test_left_stick_yaw_and_dpad_roll_are_independent(self):
        node = harness()
        node.axis = lambda name: 1. if name == 'wrist_yaw_axis' else 0.
        velocity = node.command_velocity(np.array(list(node.positions.values())))
        np.testing.assert_allclose(velocity, [0., 0., 0., 0., 0., -.12])
        node.axis = lambda name: -1. if name == 'wrist_roll_axis' else 0.
        velocity = node.command_velocity(np.array(list(node.positions.values())))
        np.testing.assert_allclose(velocity, [0., 0., 0., 0., -.12, 0.])


if __name__ == '__main__':
    unittest.main()
