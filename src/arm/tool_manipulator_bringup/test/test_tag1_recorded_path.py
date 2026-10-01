import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from tag1_recorded_path import ARM_JOINTS, compile_sequence, convert_pose, read_yaml, segment_duration


def fixture():
    joints, calibration = {}, {}
    for name in ARM_JOINTS:
        spec = {'soft_limit_rad': [-2.0, 2.0], 'velocity_limit_rad_s': 0.2}
        if name in ('base_joint', 'wrist_roll_joint', 'wrist_yaw_joint'):
            spec.update(vendor='dynamixel', raw_increases_ccw=True, zero_raw=0,
                        soft_limit_raw=[-10000, 10000])
            calibration[name] = dict(zero_offset_rad=-math.pi, direction=1.0, wraparound=False)
        else:
            spec.update(vendor='rmd', sign=-1, q_offset_rad=0.0)
            calibration[name] = dict(sign=-1.0, q_offset_rad=0.0)
        joints[name] = spec
    record = dict(format='tool_manipulator_recorded_path/v1', tag_id=1,
                  executable=True, calibration_verified=True, execution_reviewed=True,
                  lock_direction_confirmed=True, ee_policy='no_new_command',
                  joint_names=list(ARM_JOINTS)+['ee_joint'],
                  capture_source={'calibration': calibration},
                  confirmed_lock={'attach_yaw_delta_rad': 0.5},
                  waypoints=[[0.0]*6+[99.0], [0.1]*5+[0.0, 99.0],
                             [0.0]*5+[0.5, 99.0], [0.0]*5+[0.5, 99.0]],
                  docking_samples=[], execution_sequence=[
                      {'stage': 'approach', 'waypoint_index': 1},
                      {'stage': 'dock', 'waypoint_index': 2}, {'stage': 'lock'},
                      {'stage': 'retreat', 'waypoint_index': 3},
                      {'stage': 'return', 'waypoint_index': 4}])
    tools = {'real_docking': {'enabled_tool_ids': [1]}, 'tools': {1: {
        'enabled': True, 'lock': {'joint': 'wrist_yaw_joint', 'joint_path': None,
                                 'attach_yaw_delta_rad': 0.5}}}}
    return record, {'joints': joints}, tools


class RecordedPathTests(unittest.TestCase):
    def setUp(self):
        self.record, self.hw, self.tools = fixture()

    def compile(self):
        return compile_sequence(self.record, self.hw, self.tools)

    def test_six_axes_only_and_lock_holds_other_joints(self):
        points, _, _ = self.compile()
        self.assertEqual(len(points), 5)
        self.assertTrue(all(len(p['positions']) == 6 for p in points))
        self.assertEqual(points[2]['positions'][:5], points[1]['positions'][:5])
        self.assertAlmostEqual(points[2]['positions'][5]-points[1]['positions'][5], 0.5)

    def test_every_approval_required(self):
        for flag in ('executable', 'calibration_verified', 'execution_reviewed', 'lock_direction_confirmed'):
            with self.subTest(flag=flag):
                record = copy.deepcopy(self.record)
                record[flag] = False
                with self.assertRaisesRegex(ValueError, flag):
                    compile_sequence(record, self.hw, self.tools)

    def test_no_guessed_split(self):
        self.record['execution_sequence'] = []
        with self.assertRaisesRegex(ValueError, 'execution_sequence is empty'):
            self.compile()

    def test_invalid_index_and_stage_order(self):
        self.record['execution_sequence'][0]['waypoint_index'] = 0
        with self.assertRaisesRegex(ValueError, 'sample index'):
            self.compile()
        self.record['execution_sequence'][0] = {'stage': 'return', 'waypoint_index': 1}
        with self.assertRaisesRegex(ValueError, 'stage order'):
            self.compile()

    def test_rejects_entire_path_when_last_point_invalid(self):
        self.record['waypoints'][-1][0] = 3.0
        with self.assertRaisesRegex(ValueError, 'step 5.*base_joint'):
            self.compile()

    def test_rejects_lock_target_outside_limits(self):
        self.hw['joints']['wrist_yaw_joint']['soft_limit_rad'] = [-0.2, 0.2]
        with self.assertRaisesRegex(ValueError, 'step 3.*wrist_yaw_joint'):
            self.compile()

    def test_raw_limits_are_enforced(self):
        self.hw['joints']['base_joint']['soft_limit_raw'] = [-1, 1]
        with self.assertRaisesRegex(ValueError, 'base_joint'):
            self.compile()

    def test_rebases_rmd_and_dxl_without_wrapping(self):
        self.hw['joints']['elbow_joint']['q_offset_rad'] = 0.077
        self.hw['joints']['wrist_yaw_joint']['zero_raw'] = 4096
        result = convert_pose([0.0]*7, self.record, self.hw)
        self.assertAlmostEqual(result[2], 0.077)
        self.assertAlmostEqual(result[5], -2*math.pi)
        with self.assertRaisesRegex(ValueError, 'wrist_yaw_joint'):
            self.compile()

    def test_rejects_nan_and_wrapped_capture(self):
        self.record['waypoints'][0][0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.compile()
        self.record['waypoints'][0][0] = 0.0
        self.record['capture_source']['calibration']['base_joint']['wraparound'] = True
        with self.assertRaisesRegex(ValueError, 'unwrapped'):
            self.compile()

    def test_delta_must_match_confirmation(self):
        self.tools['tools'][1]['lock']['attach_yaw_delta_rad'] = -0.5
        with self.assertRaisesRegex(ValueError, 'does not match'):
            self.compile()

    def test_duration_accounts_for_peak_speed(self):
        t = segment_duration([0.0]*6, [1.0]*6, [0.1]*6, 0.15, 1.0)
        self.assertEqual(t, 20.0)
        self.assertLessEqual(1.5/t, 0.1)

    def test_repository_record_fails_closed(self):
        config = Path(__file__).resolve().parents[1] / 'config'
        with self.assertRaisesRegex(ValueError, 'execution_sequence is empty'):
            compile_sequence(read_yaml(config/'tag1_recorded_path.yaml'),
                             read_yaml(config/'hardware.yaml'), read_yaml(config/'tools.yaml'))


if __name__ == '__main__':
    unittest.main()
