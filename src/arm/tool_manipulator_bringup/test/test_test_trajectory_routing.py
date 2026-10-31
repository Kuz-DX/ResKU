"""The test launch selects independent paths without implicit coordinate edits."""
import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from tool_trajectory_joy import load_path as production_load
from tool_trajectory_joy_test import load_path as experimental_load

spec = importlib.util.spec_from_file_location('test_launch', ROOT/'launch/ik_manual_test.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


@pytest.mark.parametrize('tool', range(3))
def test_copy_loads_identically_and_can_change_independently(tmp_path, tool):
    original = ROOT/f'config/resampled/tool{tool}_trajectory.yaml'
    copied = tmp_path/original.name
    copied.write_bytes(original.read_bytes())
    model = Mock()
    model.within_limits.return_value = True
    expected = production_load(original, model)
    actual = experimental_load(copied, model)
    np.testing.assert_array_equal(actual['start'], expected['start'])
    np.testing.assert_array_equal(actual['waypoints'], expected['waypoints'])
    assert actual['durations'] == expected['durations']
    assert actual['after_path'] == expected['after_path']
    data = yaml.safe_load(copied.read_text())
    data['points'][-1]['positions_rad'][0] += .001
    copied.write_text(yaml.safe_dump(data))
    assert experimental_load(copied, model)['waypoints'][-1][0] != expected['waypoints'][-1][0]
    np.testing.assert_array_equal(production_load(original, model)['waypoints'], expected['waypoints'])


def write_set(config):
    directory = config/'test/resampled'
    directory.mkdir(parents=True)
    for tool in range(3):
        (directory/f'tool{tool}_trajectory.yaml').write_text('test copy')
    return directory


def test_source_and_installed_routing(tmp_path):
    source, installed = tmp_path/'source', tmp_path/'installed'
    installed_paths = write_set(installed)
    assert launch.test_trajectory_directory(source, installed) == installed_paths
    source_paths = write_set(source)
    assert launch.test_trajectory_directory(source, installed) == source_paths
    (source_paths/'tool0_trajectory.yaml').unlink()
    with pytest.raises(FileNotFoundError):
        launch.test_trajectory_directory(source, installed)


def test_shared_files_are_not_a_fallback(tmp_path):
    shared = tmp_path/'resampled'
    shared.mkdir()
    for tool in range(3):
        (shared/f'tool{tool}_trajectory.yaml').write_text('production')
    with pytest.raises(FileNotFoundError, match='Test trajectory files missing'):
        launch.test_trajectory_directory(tmp_path, tmp_path)
