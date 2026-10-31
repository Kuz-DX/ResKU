"""Resolve the active workspace calibration for real arm control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def default_hardware_config(share=None):
    share = Path(share or get_package_share_directory('tool_manipulator_bringup'))
    # A regular colcon install contains a copy. Prefer the editable calibration
    # in that same workspace, just as a symlink install does. Never pick another
    # workspace from HOME or the caller's current directory.
    for parent in share.parents:
        candidate = parent / 'src/arm/tool_manipulator_bringup/config/hardware.yaml'
        if candidate.is_file():
            return candidate.resolve()
    return (share / 'config/hardware.yaml').resolve(strict=True)
