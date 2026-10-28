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
def test_generated_path_finishes_with_feedback_lag(tool):
    from arm_mode_core import ModeCore
    from tool_trajectory_joy import load_path, ARM_JOINTS, ArmKinematics, isolate_shoulder_zero_crossings
    model = ArmKinematics(ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          ROOT/'config/hardware.yaml')
    path = isolate_shoulder_zero_crossings(load_path(ROOT/f'config/resampled/tool{tool}_trajectory.yaml', model))
    core = ModeCore([model.limits[j] for j in ARM_JOINTS], [.12]*6,
                    joint_names=ARM_JOINTS, path_speeds=SPEEDS,
                    path_accelerations=[.08, .03, .06, .06, .08, .08],
                    path_jerks=[.30, .04, .15, .15, .30, .30],
                    path_feedback_leads=[.04, .015, .025, .025, .04, .04])
    measured = path['start'].copy()
    core.ready, core.focus = True, 'arm'
    core.feedback(measured, [0.]*6, 0.)
    core.mode = core.requested = 'tool_auto'
    assert core.path(core.epoch, 'resampled-test', path['waypoints'], 0., path['durations'],
                     path.get('stop_at_waypoints'))
    for step in range(1, 30001):
        now = step*.02
        previous = measured.copy()
        measured += (np.asarray(core.target)-measured)*.25
        core.feedback(measured, (measured-previous)/.02, now)
        core.tick(now, .02)
        if core.mode == 'fault' or core.last_execution_result is not None:
            break
    assert core.last_execution_result == 'succeeded', core.reason
    np.testing.assert_allclose(measured, path['waypoints'][-1], atol=.01)
