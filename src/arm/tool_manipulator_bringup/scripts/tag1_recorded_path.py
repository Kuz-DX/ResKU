#!/usr/bin/env python3
"""Read-only preflight/compiler for an explicitly reviewed tag-1 taught path."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml

from arm_pose_safety import validate_joint_positions

ARM_JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint',
              'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint')
STAGES = ('approach', 'dock', 'lock', 'retreat', 'return')


def read_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'{path}: YAML mapping required')
    return data


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'finite numeric value required: {value!r}')
    return float(value)


def convert_pose(row, record, hardware):
    """Rebase via raw angle, never modulo-wrap, clamp, or alter calibration."""
    names = record['joint_names']
    if names != list(ARM_JOINTS) + ['ee_joint'] or len(row) != len(names):
        raise ValueError('record must contain the six arm joints followed by ee_joint')
    output = []
    for name, value in zip(ARM_JOINTS, row[:6]):
        q = number(value)
        src = record['capture_source']['calibration'][name]
        dst = hardware['joints'][name]
        if dst['vendor'] == 'rmd':
            old_sign, new_sign = number(src['sign']), number(dst['sign'])
            if old_sign not in (-1, 1) or new_sign not in (-1, 1):
                raise ValueError(f'{name}: sign must be -1 or 1')
            raw = q / old_sign + number(src['q_offset_rad'])
            output.append(new_sign * (raw - number(dst['q_offset_rad'])))
        elif dst['vendor'] == 'dynamixel':
            if src.get('wraparound') is not False:
                raise ValueError(f'{name}: unwrapped capture required')
            direction = number(src['direction'])
            if direction not in (-1, 1) or not isinstance(dst['raw_increases_ccw'], bool):
                raise ValueError(f'{name}: invalid encoder direction')
            raw = q / direction + number(src['zero_offset_rad'])
            zero = number(dst['zero_raw']) * 2 * math.pi / 4096 - math.pi
            output.append((1 if dst['raw_increases_ccw'] else -1) * (raw - zero))
        else:
            raise ValueError(f'{name}: unsupported vendor')
    return output


def compile_sequence(record, hardware, tools):
    """Validate the entire selected path before a caller may send any action."""
    errors = []
    if record.get('format') != 'tool_manipulator_recorded_path/v1' or record.get('tag_id') != 1:
        raise ValueError('tag 1 recorded-path/v1 required')
    for flag in ('executable', 'calibration_verified', 'execution_reviewed', 'lock_direction_confirmed'):
        if record.get(flag) is not True:
            errors.append(f'{flag} must be confirmed before execution')
    if record.get('ee_policy') != 'no_new_command':
        errors.append('ee_policy must be no_new_command')
    tool = tools['tools'][1]
    if tool.get('enabled') is not True or 1 not in tools['real_docking']['enabled_tool_ids']:
        errors.append('tag 1 is not enabled in tools.yaml')
    spec = tool['lock']
    if spec.get('joint') != 'wrist_yaw_joint' or spec.get('joint_path') is not None:
        errors.append('tag 1 must use a wrist_yaw-only relative lock')
    delta = number(spec['attach_yaw_delta_rad'])
    if not math.isclose(delta, number(record['confirmed_lock']['attach_yaw_delta_rad']), abs_tol=1e-9):
        errors.append('tools.yaml lock delta does not match confirmed measurement')
    limits, speeds = {}, []
    for name in ARM_JOINTS:
        joint = hardware['joints'][name]
        lo, hi = map(number, joint['soft_limit_rad'])
        if lo >= hi:
            raise ValueError(f'{name}: invalid limits')
        # Enforce DXL raw limits as well as the configured radian limits.
        if joint['vendor'] == 'dynamixel':
            direction = 1 if joint['raw_increases_ccw'] else -1
            raw_limits = sorted(direction * (number(v) - number(joint['zero_raw'])) *
                                2 * math.pi / 4096 for v in joint['soft_limit_raw'])
            lo, hi = max(lo, raw_limits[0]), min(hi, raw_limits[1])
            if lo >= hi:
                raise ValueError(f'{name}: raw/radian limits have no intersection')
        limits[name] = (lo, hi)
        speed = number(joint['velocity_limit_rad_s'])
        if speed <= 0:
            raise ValueError(f'{name}: positive velocity limit required')
        speeds.append(speed)
    sequence = record.get('execution_sequence')
    if not isinstance(sequence, list) or not sequence:
        errors.append('execution_sequence is empty: explicitly select approach/dock/lock/retreat/return points')
        sequence = []
    points, stages = [], []
    for index, entry in enumerate(sequence):
        try:
            stage = entry['stage']
            if stage not in STAGES:
                raise ValueError('unknown stage')
            if not stages or stages[-1] != stage:
                stages.append(stage)
            if stage == 'lock':
                if set(entry) != {'stage'} or not points or points[-1]['stage'] != 'dock':
                    raise ValueError('one yaw-only lock must immediately follow dock')
                positions = list(points[-1]['positions'])
                # A sign change requires renewed direction commissioning.
                source = record['capture_source']['calibration']['wrist_yaw_joint']['direction']
                target = 1 if hardware['joints']['wrist_yaw_joint']['raw_increases_ccw'] else -1
                if source != target:
                    raise ValueError('yaw direction changed since lock measurement')
                positions[5] += delta
            else:
                refs = set(entry) - {'stage'}
                if refs not in ({'waypoint_index'}, {'docking_sample_index'}):
                    raise ValueError('exactly one explicit 1-based sample reference required')
                key = next(iter(refs))
                n = entry[key]
                samples = record['waypoints'] if key == 'waypoint_index' else record['docking_samples']
                if type(n) is not int or not 1 <= n <= len(samples):
                    raise ValueError('sample index outside recorded range')
                row = samples[n-1] if key == 'waypoint_index' else samples[n-1]['positions']
                positions = convert_pose(row, record, hardware)
            validate_joint_positions(dict(zip(ARM_JOINTS, positions)), limits,
                                     label=f'step {index+1} {stage}')
            points.append({'stage': stage, 'positions': positions})
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(f'step {index+1}: {exc}')
    if tuple(stages) != STAGES:
        errors.append('stage order must be approach -> dock -> lock -> retreat -> return')
    if errors:
        raise ValueError('\n'.join(errors))
    return points, limits, speeds


def segment_duration(before, after, speeds, requested_speed, minimum):
    # Zero endpoint velocities give cubic interpolation with 1.5x peak speed;
    # use 2x displacement / speed to remain below each velocity limit.
    return max(minimum, max(2 * abs(b-a) / min(requested_speed, v)
                           for a, b, v in zip(before, after, speeds)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', required=True)
    parser.add_argument('--hardware', required=True)
    parser.add_argument('--tools', required=True)
    args = parser.parse_args()
    try:
        points, _, _ = compile_sequence(read_yaml(args.record), read_yaml(args.hardware), read_yaml(args.tools))
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'BLOCKED (no commands sent):\n{exc}')
        return 2
    print(f'PASS: {len(points)} six-axis points; static preflight only, no commands sent')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
