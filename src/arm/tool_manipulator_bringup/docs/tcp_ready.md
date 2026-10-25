# TCP ready candidate and entry policy

Evaluated 2026-10-25; candidate values are in `config/tcp_ready.yaml`.
`ik_manual.launch.py` loads this file. After startup recovery and an explicit
drive-to-arm focus edge, the TCP producer samples the live joint-space approach,
requires every MoveIt state-validity response to pass, and executes the target
through Mode Manager `tool_auto` ownership. Recovery remains separate.

Candidate A is `[0, 1.2, 1.2, -1.2, 0, 0]` rad in base, shoulder, elbow,
wrist-pitch, wrist-roll, wrist-yaw order. This is a tested candidate, not an
approved hardware pose or a certified workspace.

| Candidate | RMD positions (rad) | Minimum joint margin (rad) | Scaled condition | IK directions passed |
|---|---|---:|---:|---:|
| Recovery reference | 1.4831, 1.5071, -1.5571 | 0.0349 | 8.937 | 7/12 |
| A | 1.2, 1.2, -1.2 | 0.318 | 11.848 | 12/12 |
| B | 0.9, 1.0, -1.0 | 0.542 | 23.088 | 12/12 |
| C | 0.6, 0.8, -0.8 | 0.630 | 242.683 | 8/12 |

Every candidate has base/roll/yaw zero. Margin is distance to the physical soft
limit, before the existing 0.02 rad IK margin. The condition number uses the
existing 0.2 m characteristic-length scaling. B already lies in the existing
singularity slowdown region; C exceeds the existing stop threshold. A has
condition at most 13.24 on evaluated direction paths.

Each direction was solved in 10 increments from the candidate, independently:
base XYZ +/-10 mm, local Roll/Pitch/Yaw +/-5 degrees, using the shared IK and
0.02 rad margin. These are exploration distances, not a safe workspace claim.
Recovery failed X-, X+, Z-, Roll-, Roll+. Detailed per-direction outcomes,
per-joint margins and approach derivatives are in `tcp_ready_evaluation.json`.

Collision evaluation used an isolated ROS domain (173), only MoveIt move_group
with trajectory execution disabled, source URDF with `use_mesh=true`, `tool_id=0`,
source SRDF allowed-collision exclusions, and ee_joint=0. No hardware or
controller node ran. Whole-robot validity passed for all four candidate states,
all successfully solved direction paths and each reference-to-candidate joint
line, sampled at <=0.005 rad per joint. This includes modelled chassis/cases,
but no sensed environment, attached tools or physical TCP calibration. It does
not establish continuous swept-volume collision freedom. MoveIt returned all
results; its process segfaulted during SIGINT teardown (after evaluation).

The approach start `[0, 1.4831, 1.5071, -1.5571, 0, 0]` is a reference, NOT live
feedback. A 10-second quintic timing of the reference-to-A joint line has maximum
joint speed 0.06696 rad/s (below the existing 0.12 rad/s cap) and acceleration
0.02062 rad/s^2. Hardware acceleration bounds are absent and MoveIt acceleration
limits are disabled, so acceleration compliance and real-current-state access
remain UNVERIFIED. The current gateway also does not track this quintic timing.
The executable path uses a configured 10-second synchronized segment. It retains
fresh-feedback, focus, controller, cancellation and ownership checks, but hardware
acceleration bounds remain unavailable. The live MoveIt scene and actual tool must
therefore be correct; passing sampled checks is not continuous swept-volume proof.

## Entry without moving to tcp_ready

The manager requires a read-only assessment from the TCP producer before accepting
TCP_IK and again before completing the stopped transition. The current pose may
qualify without matching candidate A:

- Fresh controller/joint feedback, arm focus, completed startup recovery.
- Stationary joint feedback (<=0.02 rad/s), physical joint margin >=0.10 rad.
- Scaled Jacobian condition <=20 (the existing full-speed region).
- Whole-robot MoveIt state validity for the current full joint snapshot.

The producer uses 0.101 rad to cover the 0.001 rad snapshot comparison tolerance.
Assessment is only made in HOLD, with feedback/status age <0.3 s and collision
response within 0.15 s. Changes in any full-state joint discard a pending result.
The manager requires the matching epoch, proof age <0.5 s and arm feedback within
0.001 rad of the assessed snapshot. A failed/missing/stale assessment rejects the
request, retains HOLD and reports why. The same checks run at transition completion.
An explicitly accepted transition carries its proof to the new epoch; unrelated
cancels and faults invalidate it. A margin/condition rejection after the explicit
focus edge triggers at most one checked ready move. Collision, feedback, service or
execution failure remains HOLD/FAULT and never retries automatically.

The private `/arm/tcp_joy/entry_status` JSON String contains `epoch`, `safe`,
`reason`, `positions`. It is an internal trusted-producer protocol, not an access
control against arbitrary ROS publishers. Manual requests do not need this proof.
Options still requests the configured TCP mode once; after rejection wait for
assessment and explicitly request the mode again. Failed requests never auto-retry.
Once active, the existing TCP seed reset uses the measured current pose, and the
existing per-increment safety checks remain in place. This gate checks entry;
it does not replace continuous collision checks or later safety improvements.

Offline reproduction (no ROS graph or hardware access):

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 src/arm/tool_manipulator_bringup/scripts/evaluate_tcp_ready.py
```

Offline output leaves collision fields null. To reproduce collision results,
provide `evaluate(..., collision_checker=...)` with a whole-robot checker using
the above scene, tool and sampling assumptions. Never interpret null as safe.
