#!/usr/bin/env python3
"""Read-only IK benchmark: simulated lag, no ROS node, controller or hardware.

Collision acceptance is simulated. These timings do not certify the full ROS
callback, MoveIt round trip, real feedback or loaded robot-computer performance.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import time

import numpy as np
from evaluate_tcp_ready import ArmKinematics, CANDIDATES
from tcp_joy_core import TcpStepper, LimitBlocked, SafetyError


def stats(values):
    a = np.asarray(values)
    return dict(samples=len(a), mean=float(a.mean()), max=float(a.max()),
                p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=1200)
    parser.add_argument('--paced', action='store_true', help='schedule at 50 Hz')
    args = parser.parse_args()
    if args.steps < 1:
        parser.error('--steps must be positive')
    arm = Path(__file__).resolve().parents[2]
    model = ArmKinematics(arm/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                          arm/'tool_manipulator_bringup/config/hardware.yaml')
    measured = np.array(CANDIDATES['candidate_a'])
    step = TcpStepper(model, [.12]*6)
    step.reset(measured)
    jac_times, ik_times, jac_totals, intervals = [], [], [], []
    original = model.jacobian
    def jacobian(q):
        start = time.perf_counter()
        try:
            return original(q)
        finally:
            jac_times.append((time.perf_counter()-start)*1000.)
    model.jacobian = jacobian
    outcomes, errors = Counter(), Counter()
    load_start = os.getloadavg()
    cpu_start, started = time.process_time(), time.perf_counter()
    previous = None
    for i in range(args.steps):
        if args.paced:
            time.sleep(max(0., started+i*.02-time.perf_counter()))
        now = time.perf_counter()
        if previous is not None:
            intervals.append((now-previous)*1000.)
        previous = now
        # Alternating signs bound accumulated displacement; include all six axes.
        twist = np.zeros(6)
        twist[(i//100) % 6] = 1. if (i//25) % 2 == 0 else -1.
        before_jac = len(jac_times)
        try:
            q, pose = step.candidate(measured, twist, .02)
            step.commit(q, pose)  # simulated successful collision check only
            outcomes['accepted'] += 1
        except LimitBlocked as exc:
            outcomes['limit_blocked'] += 1
            errors[str(exc)] += 1
        except SafetyError as exc:
            outcomes['safety_stop'] += 1
            errors[str(exc)] += 1
            break
        except (ValueError, np.linalg.LinAlgError) as exc:
            outcomes['ik_failure'] += 1
            errors[str(exc)] += 1
        finally:
            ik_times.append((time.perf_counter()-now)*1000.)
            jac_totals.append(sum(jac_times[before_jac:]))
        measured += .3*(step.q-measured)
    wall = time.perf_counter()-started
    report = dict(scope='offline IK, simulated 0.3 feedback response, collision not timed',
        paced=args.paced, configured_period_ms=20, ik_budget_ms=step.budget*1000,
        attempts=len(ik_times), outcomes=dict(outcomes), errors=dict(errors),
        general_failure_rate=outcomes['ik_failure']/len(ik_times),
        computation_over_20ms=sum(t > 20 for t in ik_times),
        ik_ms=stats(ik_times), jacobian_call_ms=stats(jac_times),
        all_jacobians_per_candidate_ms=stats(jac_totals),
        interval_ms=stats(intervals) if intervals else None,
        cpu_count=os.cpu_count(), host_load_average_start=load_start,
        host_load_average_end=os.getloadavg(),
        process_cpu_seconds=time.process_time()-cpu_start, wall_seconds=wall)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
