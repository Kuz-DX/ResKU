# Continuous TCP IK stabilization (stage 4)

These changes target the real-hardware managed control path. No real arm commands,
controller activation or ROS package builds were run during development. The
separate `tcp_ready` movement endpoint is still outstanding; its candidate and
entry gating alone are not a completed hardware movement feature.

## Failure behavior

- `LimitBlocked`: reject this increment, preserve accepted joint/pose target,
  continue TCP_IK heartbeat, do not increment or reset general failure count.
  Reverse input can immediately solve a new increment.
- General IK/convergence/deadline/residual failure: preserve accepted seed/pose,
  record reason, request HOLD after five failures, stop heartbeat. Failure count
  resets only after IK plus collision acceptance.
- Collision/state-invalid response, malformed/nonfinite feedback, invalid solver
  output or corrupted cached seed/pose: block immediately, discard pending checks,
  issue the existing epoch-bound producer FAULT message. No normal target is
  published after blocking. The manager latches FAULT and holds its last valid
  measured position when feedback/controller ownership permits.
- Feedback age >=0.3 s: immediate TCP producer FAULT request, independent of the
  manager's existing 0.5 s feedback watchdog. Controller and hardware fault
  watchdogs remain in the manager. Missing feedback cannot guarantee physical HOLD.
- Excess feedback lead: immediate HOLD request, preserving the previous policy.
- Manager also rejects duplicate/incomplete position/velocity arrays immediately
  instead of waiting for its feedback watchdog. Nonfinite arm values still use
  the existing core validation. Legacy standalone Manual is unchanged.

The collision-service-unavailable and Deadman behaviors are unchanged here;
Deadman release/remaining lead handling belongs to stage 5. Collision checks
remain sampled, not swept-volume proofs. Non-arm joint changes invalidate pending
collision acceptance, and a mismatched epoch can never commit its candidate.

## Transactional calculation

Solver receives copies of the seed and requested transform. The accepted cached
seed must be finite, within limits and match its cached pose by FK. Returned
solutions independently pass shape/finiteness, limit, FK residual, joint speed,
feedback lead and total candidate deadline checks. Residual tolerances remain
10 micrometres translation and 0.0001 rad rotation, matching the shared solver.
Neither general failure nor LimitBlocked commits the proposed target.

## Timing and workload

`/arm/tcp_joy/status.performance` exposes bounded 512-sample windows of IK,
predictor-Jacobian, callback and timer interval durations (mean/max/p95/p99).
Lifetime counters include samples, >20 ms overruns, IK attempts, accepted commands,
LimitBlocked, general failures and safety stops. Failures are counted by reason.
Callback timing includes status publication/logging; its report appears on the
next tick. Runtime `jacobian_ms` measures the predictor Jacobian, not all internal
solver Jacobians. Benchmarks instrument every Jacobian call.

The recorded 2026-10-25 local-host run used source URDF, candidate A, 1200 updates
paced at 50 Hz, alternating all six Base XYZ / Local RPY inputs, and simulated
feedback responding by 30% per step. Collision acceptance was simulated, not
measured. CPU host had 16 logical CPUs and load average about 0.66; process CPU
was 4.716 s over 23.985 s wall time. This is not loaded real-robot ROS latency.

| Metric (ms) | Mean | Maximum | p95 | p99 |
|---|---:|---:|---:|---:|
| Candidate calculation | 3.851 | 6.600 | 5.172 | 5.792 |
| Individual Jacobian (2498 calls) | 0.705 | 1.663 | 1.033 | 1.190 |
| Sum of Jacobians per candidate | 1.468 | 2.792 | 2.090 | 2.431 |
| Scheduled start interval | 20.000 | 20.218 | 20.092 | 20.140 |

All 1200 candidates succeeded; no general failure, LimitBlocked or safety stop.
No candidate computation exceeded 20 ms. Scheduling jitter still exists: this is
not a hard real-time loop. Exact results: `tcp_ik_stage4_benchmark.json`.

The 30 ms budget is retained: this run shows no reason to optimize the solver or
relax speed/safety bounds. Since 30 ms exceeds the 20 ms timer period, it remains
a computation rejection bound rather than a guarantee of fitting one cycle.
Full ROS callback timing, MoveIt latency, CPU contention on the robot and actual
feedback lag must be measured in subsequent Mock/approved hardware validation.
Do not interpret this run as proving the budget on the physical robot.

Reproduce offline, without creating any ROS nodes:

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 src/arm/tool_manipulator_bringup/scripts/benchmark_tcp_ik.py --paced
```
