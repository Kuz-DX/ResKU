# MoveIt scene and sampled-path validation

The robot description supplies collision geometry for the chassis, U-shaped
circuit box, camera bracket/body (`cam_link`), fixed tool cases, supply box,
selected mounted tool, and the gripper racks.  Tool IDs 0 and 2 expand the real
gripper; tool ID 1 expands the drill.  These are robot-description links, so a
state-validity request checks them without separately publishing world objects.

The SRDF collision exclusions are limited to three reviewed categories:

1. directly connected arm housings and output links;
2. fixed rigid assemblies whose meshes overlap at their mounting interface;
3. declared resting/mating contacts (`circuit_box` with its fixed case or
   `supply_box`, and the selected tool's internal gear/mount contacts).

Fixture/work contacts are deliberately not global exclusions.  In particular,
arm links against the circuit box/tool cases and gripper racks against the
supply box remain collision checked.  Tool0 rack removal and tool2 grasp
contacts require stage-specific planning-scene transitions; they must not be
added to the global SRDF matrix.

HOME/BACK and tool entry plans start from the stopped measured joint snapshot,
including non-arm joints in the MoveIt start state.  Named poses are goals, not
assumed straight-line starts.  A returned MoveIt trajectory is converted to
manager waypoints, then the complete route is independently sampled through
`/check_state_validity`.  Taught-path entry, every taught segment, EE motion,
Cartesian retreat and the direct return to BACK use the same validation gate.

Arm paths are sampled so that the maximum change of any arm joint between
samples is at most `0.005 rad`.  EE-only motion uses `0.02 rad`.  A request is
limited to 20,000 arm samples or 1,000 EE samples.  The serial service budget is
`clamp(max(collision_timeout, samples / 50 Hz + 2 s), 120 s)`; the current
minimum collision timeout is 10 s.

This is discrete configuration sampling, not continuous swept-volume collision
detection.  A thin obstacle can be missed if it lies entirely between two
samples, especially far from a joint axis where 0.005 rad produces a larger
Cartesian sweep.  Collision meshes are approximations and do not include
deflection, cable motion, backlash, payload uncertainty, or braking travel.
Reducing the step improves sampling density but does not turn it into continuous
collision checking.  Small obstacles therefore require conservative inflated
geometry or a continuous-collision-capable validator before physical execution.

Every planning and validation stage retains the starting epoch and full joint
snapshot.  An epoch/ownership change, joint-set change, or feedback motion past
the stopped tolerance rejects the pending result.  A successful but stale
MoveIt response is never submitted to the manager.
