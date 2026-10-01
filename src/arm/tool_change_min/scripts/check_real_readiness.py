#!/usr/bin/env python3
"""Offline real-bringup checks; never opens devices or sends ROS commands."""
import argparse
from pathlib import Path
import subprocess
import sys

import yaml

# Also works from source before this package's generated services are installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_change_min.config import require_for_stop_state


def main():
    package = Path(__file__).resolve().parents[1]
    bringup = package.parent / 'tool_manipulator_bringup'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, default=package / 'config/poses.yaml')
    parser.add_argument('--hardware', type=Path, default=bringup / 'config/hardware.yaml')
    parser.add_argument('--extrinsics', type=Path,
                        default=bringup / 'config/arm_camera_extrinsics.yaml')
    args = parser.parse_args()
    errors = []

    def check(label, callback):
        try:
            callback()
            print(f'PASS: {label}')
        except Exception as exc:
            errors.append(label)
            print(f'BLOCKED: {label}: {exc}')

    def installed():
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix('tool_change_min'))
        for name in ('ik_node.py', 'motion_executor.py', 'tool_change_fsm.py'):
            if not (prefix / 'lib/tool_change_min' / name).is_file():
                raise ValueError(f'missing installed executable: {name}')
        # Use a clean interpreter: the source config import above must not hide
        # the installed package containing generated ROS service modules.
        result = subprocess.run(
            [sys.executable, '-c',
             'from tool_change_min.srv import MoveNamedPose, LinearMoveToPose, '
             'ExecuteTrajectory, RotateWristYaw\n'
             'for cls in (MoveNamedPose, LinearMoveToPose, ExecuteTrajectory, RotateWristYaw):\n'
             '    cls.__class__.__import_type_support__()'],
            cwd='/tmp', capture_output=True, text=True, timeout=15)
        if result.returncode:
            raise ValueError('generated service/type support import failed: ' + result.stderr.strip())

    def devices():
        hardware = yaml.safe_load(args.hardware.read_text())
        buses = hardware['buses']
        paths = [Path(buses['dynamixel_u2d2']['device']),
                 Path('/sys/class/net') / buses['rmd_can']['interface']]
        missing = [str(path) for path in paths if not path.exists()]
        if missing:
            raise ValueError('missing device/interface: ' + ', '.join(missing))

    def extrinsics():
        params = yaml.safe_load(args.extrinsics.read_text())['arm_camera_extrinsics']['ros__parameters']
        if params.get('enabled') is not True:
            raise ValueError('camera mounting TF disabled; calibrate file or verify an external TF owner')

    check('installed task nodes and generated services', installed)
    check('selected-stage measured poses', lambda: require_for_stop_state(args.poses))
    check('configured serial/CAN paths exist', devices)
    check('camera mounting TF configuration enabled', extrinsics)
    print('Offline checks only: motor registers, live TF, controllers, UI routing and '
          'physical motion are NOT verified. Do not send a motion request based on this check alone.')
    return 2 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
