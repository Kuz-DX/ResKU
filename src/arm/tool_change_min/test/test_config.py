"""The tool 1 configuration must not require unrelated mission poses."""
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.config import require_for_stop_state


class Tool1ConfigTest(unittest.TestCase):
    def test_tool1_fsm_named_poses_are_present(self):
        config_path = Path(__file__).resolve().parents[1] / 'config' / 'poses.yaml'
        with config_path.open(encoding='utf-8') as stream:
            poses = yaml.safe_load(stream)['named_poses']
        self.assertTrue({'home', 'docking_wait', 'tool1_pre'}.issubset(poses))

    def test_home_stage_does_not_require_later_motion_values(self):
        data = {
            'development': {'stop_after_state': 'home'},
            'feedback': {'joint_state_max_age_s': 1.0},
            'joint_names': ['base_joint'],
            'named_poses': {'home': {'base_joint': 0.0}},
            'motion': {
                'ik_limit_margin_rad': 0.0,
                'named_pose_duration_s': 1.0,
            },
            'timeouts_s': {'home': 1.0},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'poses.yaml'
            path.write_text(yaml.safe_dump(data), encoding='utf-8')
            self.assertEqual(require_for_stop_state(path), data)


if __name__ == '__main__':
    unittest.main()
