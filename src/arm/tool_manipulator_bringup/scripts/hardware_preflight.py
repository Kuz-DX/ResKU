#!/usr/bin/env python3
"""Fail closed before starting the real arm; opens no hardware device."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

ARM_JOINTS = (
    'base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
    'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint',
)


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def bounded_pair(value) -> bool:
    return isinstance(value, list) and len(value) == 2 and finite(value[0]) and finite(value[1]) and value[0] < value[1]


def validate(config: dict) -> list[str]:
    errors: list[str] = []
    buses = config.get('buses', {})
    u2d2 = buses.get('dynamixel_u2d2', {})
    if not isinstance(u2d2.get('device'), str) or not u2d2['device'].startswith('/dev/'):
        errors.append('buses.dynamixel_u2d2.device must be an absolute /dev path')
    if u2d2.get('baud_rate') != 1000000:
        errors.append('buses.dynamixel_u2d2.baud_rate must be the confirmed 1000000')
    if u2d2.get('protocol') != 2.0:
        errors.append('buses.dynamixel_u2d2.protocol must be the confirmed 2.0')
    if u2d2.get('adapter') != 'U2D2':
        errors.append('buses.dynamixel_u2d2.adapter must be U2D2')
    if u2d2.get('mixed_ttl_rs485') is not True:
        errors.append('buses.dynamixel_u2d2.mixed_ttl_rs485 must be true for this wiring')
    if u2d2.get('feedback_mode') != 'individual_read':
        errors.append('buses.dynamixel_u2d2.feedback_mode must be individual_read')
    if u2d2.get('command_mode') != 'sync_write':
        errors.append('buses.dynamixel_u2d2.command_mode must be sync_write')

    can = buses.get('rmd_can', {})
    if not isinstance(can.get('interface'), str) or not can['interface']:
        errors.append('buses.rmd_can.interface is required')
    if can.get('bitrate') != 1000000:
        errors.append('buses.rmd_can.bitrate must be the confirmed 1000000')

    joints = config.get('joints', {})
    seen_dxl_ids: set[int] = set()
    for name in ARM_JOINTS:
        joint = joints.get(name)
        if not isinstance(joint, dict):
            errors.append(f'joints.{name} is missing')
            continue
        if not isinstance(joint.get('id'), int):
            errors.append(f'joints.{name}.id is required')
            continue
        if joint.get('vendor') == 'dynamixel':
            if joint.get('bus') != 'dynamixel_u2d2':
                errors.append(f'joints.{name}.bus must be dynamixel_u2d2')
            if joint.get('transport_channel') not in ('ttl', 'rs485'):
                errors.append(f'joints.{name}.transport_channel must be ttl or rs485')
            if joint['id'] in seen_dxl_ids:
                errors.append(f'joints.{name}.id duplicates a Dynamixel ID')
            seen_dxl_ids.add(joint['id'])
            for key in (
                'zero_raw', 'velocity_limit_rad_s', 'current_limit_ma', 'current_limit_raw',
                'velocity_limit_raw', 'profile_acceleration_raw', 'profile_velocity_raw',
            ):
                if not finite(joint.get(key)):
                    errors.append(f'joints.{name}.{key} must be measured')
            if not bounded_pair(joint.get('soft_limit_raw')):
                errors.append(f'joints.{name}.soft_limit_raw must be [min, max]')
            if not bounded_pair(joint.get('soft_limit_rad')):
                errors.append(f'joints.{name}.soft_limit_rad must be [min, max]')
        elif joint.get('vendor') == 'rmd':
            if joint.get('bus') != 'rmd_can':
                errors.append(f'joints.{name}.bus must be rmd_can')
            for key in (
                'q_offset_rad', 'velocity_limit_rad_s', 'current_limit_a',
                'current_release_threshold_a', 'current_retreat_velocity_rad_s',
                'current_max_retreat_distance_rad', 'current_limit_duration_ms',
            ):
                if not finite(joint.get(key)):
                    errors.append(f'joints.{name}.{key} must be measured')
            if joint.get('sign') not in (-1, 1):
                errors.append(f'joints.{name}.sign must be measured as -1 or +1')
            if not bounded_pair(joint.get('soft_limit_rad')):
                errors.append(f'joints.{name}.soft_limit_rad must be [min, max]')
        else:
            errors.append(f'joints.{name}.vendor must be dynamixel or rmd')
    expected_dynamixel = {
        'base_joint': (1, 'ttl'), 'wrist_roll_joint': (2, 'rs485'),
        'wrist_yaw_joint': (3, 'ttl'), 'ee_joint': (4, 'ttl'),
    }
    for name, (motor_id, channel) in expected_dynamixel.items():
        joint = joints.get(name, {})
        if joint.get('id') != motor_id or joint.get('transport_channel') != channel:
            errors.append(f'joints.{name}.id/transport_channel must match the confirmed U2D2 B topology')
        if joint.get('operating_mode') != 4:
            errors.append(f'joints.{name}.operating_mode must be the confirmed extended-position mode 4')
        if not isinstance(joint.get('raw_increases_ccw'), bool):
            errors.append(f'joints.{name}.raw_increases_ccw must be a measured boolean')
    expected_rmd = {'shoulder_joint': 4, 'elbow_joint': 5, 'wrist_pitch_joint': 6}
    for name, motor_id in expected_rmd.items():
        if joints.get(name, {}).get('id') != motor_id:
            errors.append(f'joints.{name}.id must match the confirmed CAN ID {motor_id}')
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description='Validate real-arm hardware configuration without opening devices.')
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    try:
        with args.config.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        print(f'HARDWARE NOT READY: cannot read configuration: {exc}', file=sys.stderr)
        return 2
    errors = validate(config)
    if errors:
        print('HARDWARE NOT READY — controller_manager must not be started:', file=sys.stderr)
        print('\n'.join(f'- {error}' for error in errors), file=sys.stderr)
        return 1
    print('HARDWARE READY: hardware.yaml contains calibrated safety values for all seven joints.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
