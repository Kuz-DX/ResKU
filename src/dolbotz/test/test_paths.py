"""
Unit tests for the ROS-free path-resolution helpers in dolbotz/utils/paths.py.

Run (requires ROS env sourced, since ament_index_python is a ROS package):
    source /opt/ros/humble/setup.bash
    python3 -m pytest test/test_paths.py -v
"""

import pytest

from dolbotz.utils import paths as paths_module
from dolbotz.utils.paths import (
    get_config_dir,
    get_models_dir,
    get_package_share_dir,
    get_repo_root,
)


# ---------------------------------------------------------------------------
# get_repo_root — real filesystem, no mocking (sanity check against this repo)
# ---------------------------------------------------------------------------

class TestGetRepoRoot:
    def test_repo_root_contains_expected_layout(self):
        root = get_repo_root()
        assert (root / 'config').is_dir()
        assert (root / 'dolbotz').is_dir()
        assert (root / 'setup.py').is_file()

    def test_repo_root_is_stable_across_calls(self):
        assert get_repo_root() == get_repo_root()


# ---------------------------------------------------------------------------
# get_package_share_dir — hybrid ament_index / fallback behaviour, mocked
# ---------------------------------------------------------------------------

class TestGetPackageShareDirHybrid:
    def test_uses_ament_index_result_when_available(self, monkeypatch, tmp_path):
        """When ament_index_python resolves 'dolbotz' successfully (e.g. a real
        colcon install), that result must be used directly — the __file__
        upward-search fallback must not even run."""
        fake_share_dir = tmp_path / 'install' / 'dolbotz' / 'share' / 'dolbotz'
        fake_share_dir.mkdir(parents=True)

        import ament_index_python.packages as ament_packages
        monkeypatch.setattr(
            ament_packages, 'get_package_share_directory',
            lambda pkg: str(fake_share_dir))

        # Sabotage the fallback so the test would fail loudly if it were
        # reached instead of the mocked ament_index path being used.
        def _boom():
            raise AssertionError('fallback get_repo_root() should not be called')
        monkeypatch.setattr(paths_module, 'get_repo_root', _boom)

        assert get_package_share_dir() == fake_share_dir

    def test_falls_back_to_repo_root_when_ament_index_cannot_find_package(self, monkeypatch):
        """When ament_index_python is importable but the 'dolbotz' package isn't
        registered in its index (e.g. plain PYTHONPATH=src dev/test — this
        repo's actual current state), fall back to the __file__ search."""
        import ament_index_python.packages as ament_packages

        def _raise_not_found(pkg):
            raise ament_packages.PackageNotFoundError(pkg, [])
        monkeypatch.setattr(ament_packages, 'get_package_share_directory', _raise_not_found)

        assert get_package_share_dir() == get_repo_root()

    def test_raises_with_all_attempts_when_both_paths_fail(self, monkeypatch):
        import ament_index_python.packages as ament_packages

        def _raise_not_found(pkg):
            raise ament_packages.PackageNotFoundError(pkg, [])
        monkeypatch.setattr(ament_packages, 'get_package_share_directory', _raise_not_found)

        def _raise_runtime():
            raise RuntimeError('simulated: no config/ found upward')
        monkeypatch.setattr(paths_module, 'get_repo_root', _raise_runtime)

        with pytest.raises(RuntimeError) as excinfo:
            get_package_share_dir()
        message = str(excinfo.value)
        assert 'ament_index_python' in message
        assert 'simulated: no config/ found upward' in message


# ---------------------------------------------------------------------------
# get_config_dir / get_models_dir
# ---------------------------------------------------------------------------

class TestConfigSubdirs:
    def test_config_subdirs_are_nested_under_config_dir(self):
        config_dir = get_config_dir()
        assert get_models_dir() == config_dir / 'models'
