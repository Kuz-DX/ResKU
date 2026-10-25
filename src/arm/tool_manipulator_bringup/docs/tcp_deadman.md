# TCP Deadman (stage 5)

The existing TCP L1 button 4 and R1 rotation modifier button 5 are retained.
Legacy Manual mapping, controller operating modes and hardware configuration are
unchanged. L1 is a movement permit, not a replacement for physical emergency stop.

| Input/state | Behavior |
|---|---|
| Fresh L1 held after rearming + stick | Existing bounded IK/collision-checked increments |
| L1 held + neutral sticks | Maintain last accepted target; do not calculate new IK |
| Engaged L1 released | Discard pending checks, zero increment, measured HOLD request |
| No Joy for >=0.25 s | Same stop; block reconnect-with-button-held |
| Malformed Joy (short arrays, nonfinite axes, invalid button values) | Same stop; never treated as valid release/rearm |
| HOLD/FAULT, wrong epoch or inactive focus | No new TCP joint commands |
| General IK failure / LimitBlocked | Existing stage-4 policy remains unchanged |

On normal release, stopping is requested in the Joy callback instead of waiting
for the next TCP timer. Timeout detection runs in the 20 ms timer; scheduling or
CPU stalls can add delay. A valid initial release after entering TCP_IK rearms
without immediately leaving the mode. Once L1 is engaged, releasing it requests
HOLD even if collision-service loss has cleared the rearm flag in the meantime.

The producer sends `{"epoch":"CURRENT", "stop":true}` on `/arm/input/tcp_ik`.
It then blocks all normal heartbeat commands for that epoch. The manager accepts
this only in the matching TCP_IK epoch, replaces target AND goal with its fresh,
valid measured position, sets EE velocity zero, enters HOLD and invalidates the
epoch. The replacement is published by the next manager timer. As with existing
FAULT HOLD, this avoids continuing toward an old forward command or ramping back
from that command. It is not a hardware brake or a certified stopping-distance
bound. Missing controller/fresh feedback follows FAULT instead.

A rapid release/repress cannot outrun the stop: the producer remains blocked and
old queued joint commands fail epoch checks. A delayed stop cannot cancel Manual,
FAULT, HOLD or a newly entered TCP epoch. No unscoped cancel service is used for
Deadman stops. If the stop message cannot be delivered, the existing command
watchdog remains a fallback, with its own timeout and stopping behavior.

After stopping, explicitly request TCP_IK (mode service, or drive-to-arm focus
edge), wait for its entry checks, release L1, then press again. There is no
automatic resume or automatic tcp_ready movement. A newly requested epoch still
requires a release observed after entry, so holding L1 across reconnect is blocked.

The Joy message itself has no independent hardware-connected flag. Timeout detects
loss of messages; it cannot detect a driver that continuously republishes frozen
button/axis values after physical disconnection. Actual driver disconnect behavior
and physical stopping must be verified in the later approved hardware stage.

Offline regression covers release with outstanding collision checks, fast repress,
timeout even without collision service, malformed messages, neutral hold, mode
reentry, stale epochs, measured HOLD replacing lead, and Manual/FAULT isolation.
No ROS package build, controller activation or physical motion was performed.
