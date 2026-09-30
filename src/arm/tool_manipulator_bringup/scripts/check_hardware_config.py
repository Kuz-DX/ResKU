#!/usr/bin/env python3
"""Report real-control hardware safety fields without touching hardware."""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml


DXL_JOINTS = ('base_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint')
RMD_JOINTS = ('shoulder_joint', 'elbow_joint', 'wrist_pitch_joint')
DXL_FIELDS = (
    'zero_raw', 'soft_limit_raw', 'soft_limit_rad', 'velocity_limit_rad_s',
    'current_limit_ma', 'current_limit_raw', 'velocity_limit_raw',
    'profile_acceleration_raw', 'profile_velocity_raw',
)
RMD_FIELDS = (
    'sign', 'q_offset_rad', 'soft_limit_rad', 'velocity_limit_rad_s',
    'current_limit_a', 'current_release_threshold_a',
    'current_retreat_velocity_rad_s', 'current_max_retreat_distance_rad',
    'current_limit_duration_ms',
)


def _missing(value) -> bool:
    return value is None or (
        isinstance(value, list) and any(_missing(item) for item in value)
    )


def _default_config() -> Path:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('tool_manipulator_bringup'))
    return share / 'config' / 'hardware.yaml'


def inspect_config(config: dict) -> tuple[list[tuple[str, object]], list[str]]:
    values: list[tuple[str, object]] = []
    buses = config.get('buses', {})
    values.extend((
        ('buses.dynamixel_u2d2.device', buses.get('dynamixel_u2d2', {}).get('device')),
        ('buses.dynamixel_u2d2.baud_rate', buses.get('dynamixel_u2d2', {}).get('baud_rate')),
        ('buses.dynamixel_u2d2.protocol', buses.get('dynamixel_u2d2', {}).get('protocol')),
        ('buses.dynamixel_u2d2.mixed_ttl_rs485',
         buses.get('dynamixel_u2d2', {}).get('mixed_ttl_rs485')),
        ('buses.dynamixel_u2d2.feedback_mode',
         buses.get('dynamixel_u2d2', {}).get('feedback_mode')),
        ('buses.dynamixel_u2d2.command_mode',
         buses.get('dynamixel_u2d2', {}).get('command_mode')),
        ('buses.rmd_can.interface', buses.get('rmd_can', {}).get('interface')),
    ))
    joints = config.get('joints', {})
    groups = tuple((name, DXL_FIELDS) for name in DXL_JOINTS) + tuple(
        (name, RMD_FIELDS) for name in RMD_JOINTS
    )
    for joint, fields in groups:
        spec = joints.get(joint, {})
        if joint in DXL_JOINTS:
            values.append((f'joints.{joint}.bus', spec.get('bus')))
            values.append((
                f'joints.{joint}.transport_channel',
                spec.get('transport_channel'),
            ))
        for field in fields:
            values.append((f'joints.{joint}.{field}', spec.get(field)))
    missing = [path for path, value in values if _missing(value)]
    return values, missing


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', type=Path, default=None,
        help='hardware.yaml path; default is the installed bringup config',
    )
    parser.add_argument(
        '--show-values', action='store_true',
        help='print every required field, including configured values',
    )
    args = parser.parse_args()
    path = args.config or _default_config()
    try:
        with path.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        print(f'ERROR: cannot read {path}: {exc}')
        return 2

    values, missing = inspect_config(config)
    print(f'config: {path}')
    if args.show_values:
        print('\nRequired fields:')
        for field, value in values:
            status = 'MISSING' if _missing(value) else 'OK'
            print(f'  [{status:7}] {field} = {value!r}')
    if missing:
        print(
            f'\nBLOCKED: {len(missing)} required hardware safety '
            'field(s) are missing:'
        )
        for field in missing:
            print(f'  - {field}')
        print(
            '\nFill these values from motor specifications and measurements; '
            'do not guess them.'
        )
        return 2
    print('\nREADY: all required hardware safety fields are configured.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
