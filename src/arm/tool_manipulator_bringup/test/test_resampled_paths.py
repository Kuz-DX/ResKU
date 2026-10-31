"""Generated paths retain capture geometry and conservative speed bounds."""
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from resample_tool_paths import resample, SPEEDS


@pytest.mark.parametrize('tool', [0, 1, 2])
def test_generated_path_is_reproducible_uniform_and_preserves_vertices(tool):
    source = yaml.safe_load((ROOT/f'config/tool{tool}_trajectory.yaml').read_text())
    generated = yaml.safe_load((ROOT/f'config/resampled/tool{tool}_trajectory.yaml').read_text())
    assert generated == resample(source)
    points = np.array([p['positions_rad'] for p in generated['points']])
    times = np.array([p['time_from_start_sec'] for p in generated['points']])
    np.testing.assert_allclose(np.diff(times), .5)
    caps = np.minimum(SPEEDS, source.get('joint_speed_limits_rad_s', SPEEDS))
    assert np.all(np.abs(np.diff(points, axis=0))/.5 <= caps+1e-12)
    vertices = np.array([p['positions_rad'] for p in source['points']])
    np.testing.assert_array_equal(points[generated['source_point_indices']], vertices)
    for a, b, start, end in zip(vertices, vertices[1:], generated['source_point_indices'],
                                 generated['source_point_indices'][1:]):
        np.testing.assert_allclose(points[start:end+1], np.linspace(a, b, end-start+1), atol=1e-14)
    assert len(points) <= 1000
    assert generated.get('after_path') == source.get('after_path')


def test_capture_pauses_do_not_affect_generated_motion():
    source = yaml.safe_load((ROOT/'config/tool0_trajectory.yaml').read_text())
    expected = resample(source)
    for i, point in enumerate(source['points']):
        point['time_from_start_sec'] = i*1234.
    assert resample(source) == expected


@pytest.mark.parametrize('tool', [0, 1, 2])
def test_generated_path_final_target_is_within_joint_margin(tool):
    from tool_trajectory_joy import load_path, ArmKinematics
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    path = load_path(ROOT/f'config/resampled/tool{tool}_trajectory.yaml', model)
    final = path['waypoints'][-1]
    assert model.within_limits(final, .02)
