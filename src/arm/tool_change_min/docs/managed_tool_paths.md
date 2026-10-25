# Managed tool paths and docking alignment

`tool1_attach.launch.py` now selects `control_backend:=managed` by default.
It still starts no hardware/controller manager. Run against the existing managed
stack and collision scene; do not start a JTC or a second arm stack alongside it.
Explicit `control_backend:=jtc` preserves the legacy transport for separate setups.
Standalone executor/FSM parameter defaults remain jtc for compatibility.

In managed mode the executor creates no arm/EE ActionClient. Named poses and IK
JointTrajectory points go through `ManagedPath` to `/arm/input/tool_auto`; only
the mode manager publishes controller position commands. This adapter requires
fresh, stopped HOLD with arm focus and completed startup recovery. It checks
joint order/limits, increasing times and position-only trajectory points, then
samples the entire joint path at <=0.005 rad per joint using whole-robot MoveIt
state validity. Missing services, collision, changed feedback or ownership reject
the path before publication. Sampling is not continuous collision proof and does
not account for unmodelled obstacles or certify a moving environment.

After validation it requests tool_auto, waits for the stopped mode transition,
checks the feedback snapshot again and publishes once with a fresh execution_id.
Mode Manager accepts optional `durations` (one positive duration per segment,
less than 120 seconds). Each timed segment uses synchronized joint rates; if the
requested time would exceed existing joint speed bounds it is lengthened.
It never executes faster to catch up. Settling at each waypoint can lengthen total
execution further. This is not acceleration-limited trajectory tracking. The
existing per-waypoint timeout, limits, following-error, controller and feedback
watchdogs remain unchanged. Untimed callers keep their previous speed behavior.

Status includes `last_execution_id` and `last_execution_result` (succeeded/aborted).
Success requires the matching ID plus stopped HOLD; generic HOLD or an empty
queue is not treated as success. Focus loss, FAULT, cancel and mode changes abort
an active execution. Adapter timeout or ownership loss latches an executor fault
and sends `{epoch, execution_id, cancel:true}`. The manager accepts cancellation
only for that active epoch/ID, replaces the remaining command with measured HOLD
when fresh feedback exists, and rejects stale cancellation against another mode.
Loss of ROS delivery cannot certify physical stopping. A previously accepted path
is not a producer heartbeat stream; manager watchdogs remain its independent
protection if the producer process disappears.

## Docking alignment is part of the taught path

Docking alignment is the six-axis TCP pose, not a separate `ee_joint` rotation.
Teach the safe approach in `named_poses.tool0_pre` / `tool1_pre`, then store the
full docking position and quaternion in `cartesian_targets.tool0_target` /
`tool1_target`. The managed adapter validates and collision-checks that arm path
before publishing it. There is no `EE_ALIGN` state, `AlignEe` service, or EE
controller action in this sequence.

The default home development stop and Arduino `enabled:false` settings remain
unchanged. A full run remains unavailable until the selected tool's YAML poses,
timeouts, Arduino timing and collision scene have been measured and approved.

The FSM understands managed controller ownership and monitors manager HOLD/focus
while waiting for Arduino. Existing ROS Mock results for the Arduino handshake do
not validate managed motion over ROS or on hardware.

## Verification

The adapter was tested against the actual ModeCore with deterministic feedback,
mocked ROS transports and collision responses: synchronized duration, speed cap,
matching completion ID, collision rejection, malformed trajectory rejection,
timeout cancellation and stale-cancel isolation. Manual/TCP/entry/Deadman regressions
were run as well. No package build, installed custom-service generation or motor
commands were performed. Installed/source deployment and actual managed ROS
integration remain unverified.
