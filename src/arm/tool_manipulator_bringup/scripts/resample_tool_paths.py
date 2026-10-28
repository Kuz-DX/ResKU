#!/usr/bin/env python3
"""Print a uniformly sampled joint-line path; never command or modify hardware.

Keep capture YAMLs as the source. Each taught segment gets an integer number
of sample periods, preserving every corner exactly without blending shortcuts.
These are nominal speed-limited times; the manager still applies acceleration,
jerk, feedback pacing and stops at reversals during execution.
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

SPEEDS = np.array([.08, .04, .06, .06, .08, .08])


def resample(data, period=.5):
    if not math.isfinite(period) or period <= 0:
        raise ValueError('period must be finite and positive')
    points = np.asarray([p['positions_rad'] for p in data['points']], dtype=float)
    if points.ndim != 2 or points.shape[1] != 6 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('expected at least two finite six-joint points')
    caps = np.asarray(data.get('joint_speed_limits_rad_s', SPEEDS), dtype=float)
    if caps.shape != (6,) or not np.isfinite(caps).all() or np.any(caps <= 0):
        raise ValueError('invalid joint speed limits')
    caps = np.minimum(caps, SPEEDS)
    output = [dict(time_from_start_sec=0., positions_rad=points[0].tolist())]
    indices = [0]
    for start, goal in zip(points, points[1:]):
        count = max(1, math.ceil(float(np.max(np.abs(goal-start)/caps))/period))
        for index in range(1, count+1):
            position = goal if index == count else start+(goal-start)*(index/count)
            output.append(dict(time_from_start_sec=round(len(output)*period, 9),
                               positions_rad=position.tolist()))
        indices.append(len(output)-1)
    if len(output) > 1000:
        raise ValueError('resampled path exceeds 1000 points; increase sample period')
    result = {key: value for key, value in data.items()
              if key not in ('points', 'timing_mode', 'joint_speed_limits_rad_s')}
    result.update(timing_mode='recorded', sample_period_sec=period,
                  nominal_joint_speed_limits_rad_s=caps.tolist(),
                  source_point_indices=indices, points=output)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--period', type=float, default=.5)
    args = parser.parse_args()
    result = resample(yaml.safe_load(args.source.read_text()), args.period)
    print('# Generated from ../'+args.source.name+'; do not edit sampled points by hand.')
    print('# Nominal times only: manager acceleration/jerk/feedback guards remain active.')
    print(yaml.safe_dump(result, sort_keys=False), end='')
