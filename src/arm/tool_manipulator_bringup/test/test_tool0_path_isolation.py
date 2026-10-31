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
    assert len(source['points']) == 18
    generated = resample(source)
    points = np.array([p['positions_rad'] for p in generated['points']])
    np.testing.assert_array_equal(experimental['waypoints'], points[1:])
    a, b = generated['source_point_indices'][2], generated['source_point_indices'][14]
    assert np.all(points[a:b+1, 1:] == points[a, 1:])
    assert np.all(points[b:, [0, 4, 5]] == points[b, [0, 4, 5]])
    assert source['points'][15]['positions_rad'][1] == .326
