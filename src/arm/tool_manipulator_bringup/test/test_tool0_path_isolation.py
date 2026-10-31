"""Production and experimental tool0 paths must remain independent."""
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_trajectory_joy import ArmKinematics, load_path as production_load
from tool_trajectory_joy_test import load_path as experimental_load
from resample_tool_paths import resample


@pytest.mark.parametrize('directory', ['config', 'config/resampled'])
def test_tool0_loader_isolation(directory):
    model = ArmKinematics(
        ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
        ROOT/'config/hardware.yaml')
    path = ROOT/directory/'tool0_trajectory.yaml'
    production = production_load(path, model)
    experimental = experimental_load(path, model)
    assert production['waypoints'][-1][-1] == -0.016873788666535106
    source = yaml.safe_load((ROOT/'config/test/tool0_trajectory.yaml').read_text())
    assert len(source['points']) == 19
    generated = resample(source)
    points = np.array([p['positions_rad'] for p in generated['points']])
    np.testing.assert_array_equal(experimental['waypoints'], points[1:])
    a, b = generated['source_point_indices'][2], generated['source_point_indices'][15]
    assert np.all(points[a:b+1, 1:] == points[a, 1:])
    assert np.all(points[b:, [0, 4, 5]] == points[b, [0, 4, 5]])
    assert source['points'][16]['positions_rad'][1] == .326


def test_preparation_stops_before_base_rotation():
    from arm_mode_core import ModeCore
    source = yaml.safe_load((ROOT/'config/test/tool0_trajectory.yaml').read_text())
    generated = resample(source)
    q = np.array([p['positions_rad'] for p in generated['points']])
    a, b = generated['source_point_indices'][1:3]
    assert np.all(q[a:b+1, 0] == q[a, 0])
    assert np.all(q[a:b+1, 4:] == q[a, 4:])
    c = ModeCore([(-7., 7.)]*6, [.12]*6)
    c.focus, c.ready, c.mode = 'arm', True, 'tool_auto'
    c.target = q[0].tolist()
    c.feedback(q[0].tolist(), [0.]*6, 0.)
    assert c.path(c.epoch, 'test-preparation', q[1:].tolist(), 0.,
                  durations=[.5]*(len(q)-1))
    # path() pops the first waypoint: remaining stop flags start at q[2].
    assert c.path_stops[b-2] is True
