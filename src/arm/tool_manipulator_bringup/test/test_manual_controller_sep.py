"""Separate position-control mappings, independent limits and common hold guards."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('manual_separate', SCRIPTS/'manual_controller.sep.py')
separate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(separate)
from test_manual_controller import harness as coordinated_harness
from manual_controller import ManualController


def harness():
    node = separate.SeparateManualController.__new__(separate.SeparateManualController)
    node.__dict__.update(coordinated_harness().__dict__)
    node.chain = None
    node.axis = lambda name: 0.
    return node


class SeparateManualTest(unittest.TestCase):
    def test_axis_mapping_controls_only_requested_joint(self):
        node = harness()
        for axis, index, sign in (('shoulder_axis', 1, 1.), ('elbow_axis', 2, -1.),
                                  ('wrist_pitch_axis', 3, 1.), ('wrist_yaw_axis', 5, 1.)):
            for direction in (-1., 1.):
                node.axis = lambda name: direction if name == axis else 0.
                expected = np.zeros(6)
                expected[index] = sign*direction*.12
                if axis == 'wrist_yaw_axis':
                    expected[index] *= -1
                np.testing.assert_allclose(node.command_velocity(None), expected)

    def test_base_button_mapping_and_opposite_buttons_cancel(self):
        node = harness()
        for pressed, expected in (({'base_ccw_button'}, [.12, 0, 0, 0, 0, 0]),
                                  ({'base_cw_button'}, [-.12, 0, 0, 0, 0, 0]),
                                  ({'base_ccw_button', 'base_cw_button'}, [0]*6)):
            node.button = lambda name: name in pressed
            np.testing.assert_allclose(node.command_velocity(None), expected)

    def test_constructor_uses_left_dpad_for_yaw_without_model(self):
        node = separate.SeparateManualController.__new__(separate.SeparateManualController)
        params = {}
        node.declare_parameter = lambda key, value: SimpleNamespace(value=params.setdefault(key, value))
        node.now_seconds = lambda: 1.
        node.create_publisher = Mock(side_effect=lambda *a: Mock())
        node.create_subscription = Mock()
        node.create_timer = Mock()
        node.get_logger = Mock()
        with patch.object(separate.ManualController.__bases__[0], '__init__', return_value=None):
            separate.SeparateManualController.__init__(node)
        self.assertEqual(params['wrist_yaw_axis'], 6)
        self.assertTrue(params['invert_wrist_yaw'])
        self.assertNotIn('wrist_yaw_ccw_button', params)
        self.assertNotIn('wrist_yaw_cw_button', params)
        self.assertIsNone(node.chain)
        self.assertNotIn('/robot_description', [call.args[1] for call in node.create_subscription.call_args_list])

    def test_no_ik_or_model_required_to_publish_positions(self):
        node = harness()
        before = node.target.copy()
        node.axis = lambda name: 1. if name == 'shoulder_axis' else 0.
        node.on_timer()
        self.assertGreater(node.target[1], before[1])
        np.testing.assert_array_equal(node.target[[0, 2, 3, 4, 5]], before[[0, 2, 3, 4, 5]])
        self.assertEqual(len(node.publisher.publish.call_args.args[0].data), 6)

    def test_one_joint_at_limit_does_not_block_another(self):
        node = harness()
        node.positions['shoulder_joint'] = 1.98
        node.target[1] = 1.98
        before = node.target.copy()
        node.axis = lambda name: 1. if name in ('shoulder_axis', 'elbow_axis') else 0.
        node.on_timer()
        self.assertEqual(node.target[1], before[1])
        self.assertLess(node.target[2], before[2])

    def test_stale_feedback_or_input_holds(self):
        for stale in ('joy', 'feedback'):
            node = harness()
            if stale == 'joy':
                node.joy_time = .1
            else:
                node.state_times['elbow_joint'] = .1
            before = node.target.copy()
            node.on_timer()
            np.testing.assert_array_equal(node.publisher.publish.call_args.args[0].data, before)
            self.assertIsNone(node.target)

    def test_focus_off_and_following_error_block_motion(self):
        node = harness()
        before = node.target.copy()
        node.focus = 'drive'
        node.axis = lambda name: 1.
        node.on_timer()
        np.testing.assert_array_equal(node.target, before)
        node.focus = 'arm'
        node.target[1] += .15
        node.last_time = .98
        node.on_timer()
        self.assertTrue(node.fault)


if __name__ == '__main__':
    unittest.main()
