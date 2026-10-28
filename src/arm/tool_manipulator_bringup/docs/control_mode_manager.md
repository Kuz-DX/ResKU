# Arm Control Mode Manager

`ik_manual.launch.py` starts the mode-managed IK/manual stack, initially in
HOLD, using real hardware and an externally published `/joy` topic by default.
It does not modify the legacy manual launch.
Do not run both stacks together. The preflight refuses an existing arm controller
manager. This check is not a distributed lock against simultaneous launches.

## Run from source without building

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch ~/ResKU/src/arm/tool_manipulator_bringup/launch/ik_manual.launch.py
```

The source launch invokes Python scripts directly. Existing installed dependencies
and controller plugins are required. For offline testing, explicitly set
`real_hardware:=false`; startup has been verified with isolated mock hardware.
Startup uses the legacy manual RMD recovery policy: shoulder, elbow and wrist
pitch may start at most 5 degrees outside their soft limits. Their measured
positions are captured before controller activation and ramped inward to a
2-degree inset, at at most 0.03 rad/s. Base, wrist roll and wrist yaw must already
be inside their limits and retain their measured targets. This is not a home move.

Recovery is owned by the manager and runs once per startup, even with drive focus
or no joystick. All producer commands and active-mode requests are blocked during
recovery. Status exposes `startup_recovering`, `startup_complete` and
`startup_goal_rad`; mode remains HOLD with `transitioning=true` until completion.
Completion requires measured RMD positions within 0.005 rad of the recovery goal,
all joints inside limits, and measured velocity at most 0.02 rad/s. The manager
then enters HOLD; MANUAL still requires an explicit request and neutral controls.

Stale feedback pauses recovery increments; 15 seconds without completion faults.
Target lead is limited to 0.10 rad. Invalid feedback, excess excursion, controller
loss, or hardware faults block recovery. `/arm/cancel` during recovery latches
FAULT and discards recovery; inspection and restart are required to retry.
The 5-degree startup allowance is removed permanently after successful recovery.

## Ownership

The manager alone publishes `/manual_position_controller/commands` and
`/ee_velocity_controller/commands`. All modes share the position controller:
MANUAL and TCP_IK supply joint targets; TOOL_AUTO supplies joint waypoints.
Targets are ramped at at most min(0.12 rad/s, half hardware speed).
Manual and TCP commands remain velocity-bounded position ramps. TOOL_AUTO adds
a conservative command-acceleration limit, crosses intermediate trajectory
samples without stopping, and brakes at the final goal. Collision planning is
performed by the external MoveIt entry planner; the gateway retains ownership
and following-error enforcement.

`managed_manual.py` reuses joystick mapping and neutral rearming but does not
activate/deactivate controllers. TCP IK solvers and tool planners are external
producers; this manager does not implement an IK solver or tool workflow.

Legacy `move_to_pose.sh`, MoveIt execution, and direct controller publishers must
not be used with this stack. ROS topic names are not access controls: a rogue
publisher can bypass a software gateway. Deployment should restrict raw command
topics/controller services (e.g. SROS2) if enforced isolation is required.
Manager death requires controller/hardware watchdog protection; a Python gateway
cannot guarantee a physical stop after its process or feedback disappears.

## Public interfaces

* `/control/active_target`: independent drive/arm focus. Leaving arm cancels work.
* `/arm/control_mode`: String, reliable/transient-local depth 1, one of
  manual/tcp_ik/tool_auto/hold/fault.
* `/arm/control_status`: latched JSON String containing mode, requested,
  transitioning, epoch, ready, reason, controllers, execution_id,
  remaining_waypoints, last_error.
* `/arm/set_mode/{manual,tcp_ik,tool_auto,hold}`: std_srvs/Trigger.
  Success means the request was accepted, not that transition has completed.
* `/arm/cancel`: Trigger; discard current path and future waypoints, then HOLD.
  FAULT remains latched and already has no executable path.
* `/arm/reset_fault`: Trigger; requires healthy controllers, fresh stationary
  feedback. Hardware faults require inspection and process restart.

```bash
ros2 topic pub --once --qos-durability transient_local /control/active_target std_msgs/msg/String '{data: arm}'
ros2 service call /arm/set_mode/manual std_srvs/srv/Trigger '{}'
ros2 topic echo /arm/control_status
ros2 service call /arm/cancel std_srvs/srv/Trigger '{}'
```

During a transition all producer commands are rejected. The manager keeps sending
bounded hold commands and EE zero until measured velocity and position settle for
five samples. A fresh UUID epoch invalidates commands queued before each request,
cancel, or fault. Producers wait for transitioning=false and ready=true and copy
the new epoch. Manual additionally requires neutral controls.

## TCP joystick

Start the managed stack with its TCP producer and MoveIt planning-scene service:

```bash
ros2 launch ~/ResKU/src/arm/tool_manipulator_bringup/launch/ik_manual.launch.py
# Press Options: align DXL, then hold L1 for the checked TCP-ready approach.
ros2 topic echo /arm/control_status --once --full-length
ros2 topic echo /arm/tcp_joy/status
```

The default uses real hardware, enables the TCP producer, and consumes `/joy` from
the separately launched remote joystick path. The three launch arguments remain
only as explicit test overrides: `real_hardware:=false`, `launch_tcp_joy:=false`,
or `launch_joy:=true`. Stop the old managed/manual stack first. Do not start another
move_group
with this launch. Startup recovery must finish before requesting TCP_IK.
TCP mock startup defaults to `config/tcp_mock_initial_positions.yaml`, a
nonsingular pose near manual home. An explicit `mock_initial_positions_file`
overrides it. Real hardware continues to start from measured feedback.
With `launch_tcp_joy:=true`, an Options drive-to-arm edge first aligns the DXL
axes in HOLD. Alignment waits for position convergence and 0.20 seconds of
stopped feedback. `tcp_joy` alone requests TCP_IK after a fresh entry assessment;
the manager does not race it with a separate focus-triggered request.
If the recovery pose lacks TCP
entry margin, `tcp_joy` collision-checks the configured `tcp_ready` joint line,
requests TOOL_AUTO ownership, executes it once, verifies the stopped endpoint and
then requests TCP_IK. Options back to drive requests HOLD. Collision/check/service/
execution failure, cancel, command timeout, and fault do not automatically retry
while focus stays on arm; inspect first, then toggle drive and back to arm.

L2 (axis 2 below -0.5) plus triangle (`buttons[2]`), square (`buttons[3]`),
or X (`buttons[0]`) selects tool path 0, 1, or 2 respectively. L2+L1 does not
select a tool path. While L2 is held, the TCP producer suppresses L1 and square HOME
handling so one combination cannot start both a path and TCP/HOME motion.
Paths must contain reviewed captured points and have `execution_enabled: true`.
Standalone `manual.launch.py` accepts the same three L2 combinations and uses
the shared path loader, including timing and optional joint speed caps. Keep L2
held during standalone playback; release stops it. Start feedback must be within
0.10 rad of the first point. Segment targets retain joint travel ratios and pause
together at the feedback lead limit; failure to finish within nominal travel
time plus 30 seconds cancels the path. Rejected selections report a reason.
This standalone launch has no MoveIt collision gate or docking continuation;
use `ik_manual.launch.py` for planned entry, collision checking and tool-1 docking.
Its managed manual producer never starts local paths, and discards local path
state on mode ownership/epoch changes. All three managed paths remain owned by
the trajectory producer and mode manager.
Path 2 is enabled for an approach-only phase-1 test: original taught point 8
(TCP_ready) -> 7 -> ... -> 1, then HOLD. Original point numbers refer to the
user's capture order, not YAML list indices. Capture timestamps are discarded;
nominal 0..7 metadata and `timing_mode: joint_limits` derive motion timing from
joint limits. Tool 2 additionally sets `joint_speed_limits_rad_s` to
`[0.08, 0.04, 0.006, 0.006, 0.08, 0.08]` in arm joint order. Each taught segment
gets a minimum duration from the slowest joint's travel/cap, so elbow/pitch
approach at nominal speeds no greater than 0.006 rad/s. The manager may slow
these segments further. Captured coordinates are unchanged by this timing.
The original fastest timing demanded 0.052..0.060 rad/s in wrist-pitch approach
segments, whereas tool 1's near-limit approach is approximately 0.0064 rad/s.
This retiming addresses the separate measured-motion guard: within 0.05 rad of
a joint limit, outward measured speed above 0.02 rad/s still faults. That guard,
position limits, acceleration/jerk limits and feedback-lead checks remain active.
The optional caps apply only to `joint_limits` timing; other paths keep their
previous timing. A resulting segment of 120 s or longer is rejected at load.
Reverse traversal reuses the durations in reverse order; entry planning keeps
its existing separate timing. An offline 80-ms tracking-lag regression reproduces
the original guard fault and completes the retimed approach; actual actuator
stiction, encoder noise and load response still need a hardware trial.
The `after_path` block is commented out, so ID 4 does not close
and the arm does not return in this test. The commented block connects the
existing sequence: stop at original point 1, close independent Dynamixel ID 4
(`ee_joint`) to 1.251728 rad, confirm completion, then reverse the approach
through original points 2..8 to TCP_ready and HOLD. Confirm the close target
with the actual box before enabling that phase. Close failure prevents return.
The four captured values outside the 0.02-rad joint margin are set to its
boundaries: point 1 elbow to 1.522 rad and wrist pitch at points 1/3/7 to
-1.572 rad. All other captured joint values remain exact. The path is enabled
for `tool_trajectory_joy`, which checks collision and manager limits before
execution. Standalone manual L2+X uses the same local playback as tools 0/1,
without that collision check. The old five-point FK/IK evidence in
`config/tool2_ik_reference.yaml` is historical and does not describe this path.
Live collision validation is required by the existing execution pipeline.
Path 0 is enabled. Path 1 is enabled for approach-only testing and retains
original points 1/2/5/7. Its `timing_mode: joint_limits` keeps capture timestamps
as metadata and lets the manager derive segment timing from joint travel and
existing speed limits, with acceleration, jerk and feedback pacing still applied.
The recorded 102-second gap between points 5 and 7 no longer slows that segment.
Measured docking geometry (2026-10-28): the docking origin is 58.4 mm below
the AprilTag center. In the operator's camera convention (+X left, +Y up,
+Z forward), the tag-to-docking offset is [0, -0.0584, 0] m. In a standard
camera optical frame (+X right, +Y down, +Z forward), it is [0, +0.0584, 0] m.
This is a lateral target offset, not the tag-normal advance distance. The current
fixed-normal advance does not yet apply this calibration or seek that origin.
Conversion to the robot base must use the actual camera TF and the tag pose
from the same observation; do not substitute 58.4 mm for the advance distance.
Paths without this setting retain recorded timing. MoveIt entry and docking
correction/advance durations keep their own timing.
Intermediate points 3/4/6/8 were removed at the user's request because they
violated joint limits or margins. The user reports collision simulation of the
intermediate path is complete; runtime collision validation still applies.
Original final point 9 is excluded from execution. Its joint values and FK-derived
`base_actuator` -> `tcp_link` pose are preserved as comments in
`config/tool1_trajectory.yaml`. That original joint pose exceeds the elbow margin;
the Cartesian reference is not an approved descent command and is not read by
the docking coordinator. Tool 1 now uses `camera_normal_advance`: after the path
and HOLD, align the camera optical axis through live tag 1 (`tag36h11`, 20 mm)
at the initial observed normal distance, then translate 75 mm toward the tag
along the latched normal and HOLD without Arduino actuation. This is relative
travel, not a 75 mm final stand-off. The whole advance is IK/collision checked
before execution. Tag visibility is required through alignment; after the
advance is latched, tag loss is allowed but feedback and ownership checks
remain mandatory. Both launch switches `enable_tool_docking` and
`enable_docking_motion` default to true in `ik_manual.launch.py`, with
`docking_camera_frame=arm_camera_color_optical_frame`. Plain IK launch therefore
enables the full tool-1 sequence on L2 + square; launch itself submits no tool
request. Set `enable_tool_docking:=false` for path-only playback or
`enable_docking_motion:=false` for alignment preview after the approach.
Path 2 has its close-and-return settings commented out for approach-only testing; no tag alignment.
Tool 1 uses `require_l2_hold: false`: one L2 + tool-1 selection starts the path
and automatically continues into tag alignment and the checked 75 mm advance.
Releasing L2 or losing subsequent Joy messages does not cancel that sequence.
Explicit cancel, feedback, ownership and collision checks remain active. Other
docking configurations default to continuous L2 hold unless explicitly disabled.
At one sample per 20 ms timer tick, 861 samples take at least about 17 seconds
to validate. A new path request while checking is ignored.

TCP-ready and entry checks allow the same bounded 0.02-rad DXL/EE encoder
variation used by tool-path validation; RMD entry tolerance stays at 0.001 rad.
Collision checks, fresh feedback and joint limits remain required.

Path stall detection watches each joint after command lead has accumulated,
instead of requiring a fixed 0.001-rad reduction of the largest goal error every
two seconds. A slow, correctly followed command or an error already within
completion tolerance is not a motor-current fault. Blocked joints still abort
to HOLD. Direction reversals stop at the preceding waypoint before proceeding.
The path executor derives its total timeout from manager speed limits plus ramp
allowance, since recorded timestamps can be shorter than the bounded motion.

Sony joystick mapping for this launch (ROS Joy indices in parentheses):

| Input while holding L1 (button 4) | Without R1 | With R1 (button 5) |
| --- | --- | --- |
| Left stick up/down (axis 1) | TCP +X / -X | No motion |
| Left stick left/right (axis 0) | TCP +Y / -Y | Wrist roll joint + / - |
| Right stick up/down (axis 4) | TCP +Z / -Z | No motion |
| Right stick left/right (axis 3) | No motion | Wrist yaw joint + / - |

Translation axes are expressed in `base_actuator`, with `tcp_link` as the controlled
point. R1 switches to direct wrist roll/yaw joint jogs; it does not command TCP
rotation or wrist pitch. Release L1 once after entering
the mode, then hold it to move. Releasing an engaged L1 zeros increments, discards
pending checks and requests epoch-bound measured-position HOLD. Joy timeout
(0.25 s) or malformed input does the same. The manager replaces outstanding
command lead with its fresh measured position and invalidates the old epoch.
To resume, explicitly re-enter TCP_IK, release L1, then press it again; reconnecting
with L1 held cannot restart motion. See [tcp_deadman.md](tcp_deadman.md).

Each candidate integrates from the last approved TCP target and solves IK using
the previous accepted joint solution as seed. Defaults: 0.02 m/s, 0.15 rad/s, joint speed capped
at min(0.12 rad/s, half hardware limit), 0.02 rad soft margin, 0.08 rad feedback
lead, 30 IK iterations and a 30 ms deadline checked between iterations. The Python
loop is not a hard real-time deadline. A dimensionally scaled Jacobian condition
number slows motion from 20 to 100; at 100 or a singular Jacobian it rejects motion.
No integration catches up while collision checks are pending. IK failure retains
the last approved goal; five consecutive failures request HOLD.
Joint demand is predicted from the Jacobian to scale the entire TCP increment.
If the solved joint increment still exceeds speed limits, up to four smaller
Cartesian candidates are tried with one shared time budget and a divided total
iteration budget. Joint components are never independently clipped. Diagnostics
report `velocity_scale` separately from `singularity_scale`.

MoveIt's `/check_state_validity` must approve samples along both measured-to-goal
and previous-goal-to-goal joint paths (at most 0.005 rad per joint between samples).
The check includes the whole robot and planning-scene obstacles, subject to SRDF
allowed-collision exclusions. While the service is unavailable the node holds its
last goal, clears pending checks and requires L1 release after service recovery;
waiting does not accumulate IK failures. A 150 ms response timeout rejects the
candidate and counts as a failure. This is sampled collision validation, not continuous swept-volume
validation. Environmental obstacles must be supplied to the planning scene;
unmodelled objects cannot be detected by this node.

TCP and tool-change use the same `ArmKinematics` implementation and the running
URDF description. The legacy manual `ArmChain` remains; regression tests compare
its FK and Jacobian with the shared implementation. Full migration of the legacy
manual solver is not part of this change.

## Producer message format

Use volatile depth-1 String publishers. All joint positions are radians in order:
base, shoulder, elbow, wrist_pitch, wrist_roll, wrist_yaw.

`/arm/input/manual` and `/arm/input/tcp_ik` accept JSON:

```json
{"epoch":"COPY_FROM_STATUS", "positions":[0,0,0,0,0,0], "ee_velocity":0.0}
```

Streaming targets must be within soft limits and 0.10 rad of feedback, and arrive
more frequently than 0.3 seconds. Timeout transitions to HOLD. EE velocity is
bounded to ±0.12 rad/s. Inactive producers are ignored; an active producer may
report `"fault":true` to latch FAULT.

`/arm/input/tool_auto` accepts a complete validated joint waypoint list:

```json
{"epoch":"COPY_FROM_STATUS", "execution_id":"run-001",
 "waypoints":[[0,0,0,0,0,0],[0,0.1,0.1,0,0,0]]}
```

Only one path runs at a time, with 1..1000 waypoints. It advances after measured
feedback settles, with a 120-second per-waypoint timeout. Completion returns to
HOLD. Cancel immediately clears the queue and invalidates the epoch, so late
messages cannot restart the old execution. Tool EE operations are not yet part of
the waypoint schema. Normal joystick movement is ignored during TOOL_AUTO;
focus changes and `/arm/cancel` still work.

## Fault behavior and verification

Controller activation uses STRICT and verifies active state and the six claimed
position interfaces. Active `arm_controller` is rejected as a conflict. Feedback
expires after 0.5 seconds and controller status after 0.75 seconds. FAULT requests
EE zero and a bounded hold when valid feedback/controller ownership is available;
when those are unavailable it cannot certify that physical holding succeeded.

```bash
python3 -m pytest -q ~/ResKU/src/arm/tool_manipulator_bringup/test/test_arm_mode_core.py
```

## TCP entry assessment

TCP_IK now requires a fresh collision-checked, stationary current pose with at
least 0.10 rad joint margin and scaled condition <=20. Requests without a valid
assessment remain HOLD and report a reason; this also applies to Options focus
activation. An explicit drive-to-arm focus edge may arm one collision-checked
tcp_ready move if the current pose fails the entry margin or condition check;
execution waits for L1. Candidate comparisons, limitations
and the assessment protocol are documented in [tcp_ready.md](tcp_ready.md).
Transient entry rejections (snapshot drift, moving feedback or the 0.20-second
stability wait) are rechecked within a single 3-second window starting with the
first TCP mode request, with at least 0.25 seconds between rejection and retry.
Each retry requires a new collision assessment; retries never extend the window
or relax the manager's stopped-feedback and position tolerances. Expiry leaves
HOLD; a fresh neutral L1 press starts a new assessment. Other rejections fail
immediately.

## Timed tool paths

Tool-auto messages may include `durations`, one positive segment duration per
waypoint (each <120 s). The manager synchronizes joint interpolation, extends time
when existing speed caps require it, and still waits for feedback settling at each
point. No acceleration bound is inferred. Untimed paths retain previous behavior.
Status adds `last_execution_id` and `last_execution_result` (`succeeded`/`aborted`).
A tool producer can send `{epoch, execution_id, cancel:true}` on
`/arm/input/tool_auto`; only its matching active execution can be cancelled.
See [managed tool paths](../../tool_change_min/docs/managed_tool_paths.md) for the
adapter, collision validation and required EE alignment blocker.
