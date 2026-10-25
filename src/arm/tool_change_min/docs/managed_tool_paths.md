# Managed tool paths and required EE alignment

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

## Required EE alignment: blocked, not skipped

The user confirmed that EE_ALIGN is still required. Hardware operating_mode=1
is velocity-only and the current ee_joint is continuous; the legacy position
FollowJointTrajectory alignment and its bounded-position assumptions do not match.
No limits or operating mode were changed and no EE velocity was commanded.

Managed configuration stages from EE_ALIGN onward are rejected before the first
HOME movement. Direct managed align_ee calls also fail explicitly. Independent
home/docking_wait development stops remain possible with validated poses, but
are not a full tool-change run. The default home development stop and Arduino
`enabled:false` settings remain unchanged. Required managed EE alignment needs a
separate bounded velocity-control design, measured references/travel constraints,
feedback termination, cancellation and collision validation before enabling the
whole sequence. It will not be silently bypassed.

Legacy EE model construction is now lazy, so an unused incompatible alignment
model does not prevent unrelated arm-only startup. Legacy alignment requests still
validate/fail under the existing EE model; the old 5 EE model test failures and
raw-converter limit mismatch have not been disguised by changing hardware config.

The FSM understands managed controller ownership and monitors manager HOLD/focus
while waiting for Arduino. Full managed Arduino sequencing remains intentionally
unreachable until required alignment is implemented. Existing ROS Mock results
for the Arduino handshake used the JTC Mock branch and do not validate this new
managed backend over ROS or on hardware.

## Verification

The adapter was tested against the actual ModeCore with deterministic feedback,
mocked ROS transports and collision responses: synchronized duration, speed cap,
matching completion ID, collision rejection, malformed trajectory rejection,
timeout cancellation and stale-cancel isolation. Manual/TCP/entry/Deadman regressions
were run as well. No package build, installed custom-service generation or motor
commands were performed. Installed/source deployment and actual managed ROS
integration remain unverified.
