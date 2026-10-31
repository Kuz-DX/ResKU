# Tool 0/1 docking target comparison

All measured positions below use the URDF `arm_world` frame and metres unless
marked as mm.
The measured circular-top centres supplied on 2026-10-31 are:

| Tool | Target `tcp_link` arm_world XYZ (mm) | Surface tilt from world XY |
| --- | --- | --- |
| 0 | `[-394.506, 0.000, 212.964]` | `55 deg` |
| 1 | `[-317.806, 90.000, 163.912]` | `45 deg` |

`arm_world` and the MoveIt planning root `base_actuator` have the same origin
and orientation. Therefore the planning targets are the measured values above.
For visualization in `world`, add the fixed `world -> arm_world` translation
`[+192.1, 0, +235]` mm:

| Tool | Target `tcp_link` world XYZ (mm) |
| --- | --- |
| 0 | `[-202.406, 0.000, 447.964]` |
| 1 | `[-125.706, 90.000, 398.912]` |

The old final joint samples currently stored in `tool0_trajectory.yaml` and
`tool1_trajectory.yaml` produce:

| Tool | Existing final `tcp_link` world XYZ (mm) | target - FK (mm) | Distance (mm) |
| --- | --- | --- | --- |
| 0 | `[538.484, -40.154, 467.662]` | `[-740.890, +40.154, -19.698]` | `742.239` |
| 1 | `[-25.933, 39.270, 453.553]` | `[-99.773, +50.730, -54.641]` | `124.554` |

The old final samples are therefore not the new docking targets and must not be
used as the final goals of the new MoveIt route.

## Angle definition

The `wrist_pitch_joint` rotation axis lies in world XY at these poses, so the
literal angle between that axis and world XY is always `0 deg`; it does not
describe the circular surface tilt. The useful docking quantity is the angle
between the `tcp_link` XY plane and world XY, computed from the absolute dot
product of their normals.

| Tool | Target surface tilt | Existing final TCP-plane tilt | Error |
| --- | --- | --- | --- |
| 0 | `55.000 deg` | `52.073 deg` | `-2.927 deg` |
| 1 | `45.000 deg` | `46.798 deg` | `+1.798 deg` |

This comparison constrains only plane tilt, not which side of the surface the
TCP approaches from or the yaw/roll about the surface normal. A final MoveIt
pose requires that approach-normal direction to be selected before an
orientation quaternion and approved IK goal can be stored.

## Plan-only tool 0 topic check

For the first check, define the wrist direction as the `tcp_link` local `+X`
axis.  The user-supplied `55 deg` is the undirected minimum angle between this
line and the parallel `world`/`arm_world` XY planes; it is not a joint encoder
angle.  The reachable side is represented by world-Y pitch `+55 deg`, whose
quaternion is `[0, 0.46174861323503386, 0, 0.8870108331782217]` in XYZW order.
The opposite directed representative (`125 deg`) has the same line-plane angle
but no bounded IK solution for this arm.

Run only one mock bringup; do not run `ik_manual.launch.py` or another
controller stack at the same time:

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch ~/ResKU/src/arm/tool_manipulator_bringup/launch/mock.launch.py \
  tool_id:=0 use_mesh:=false
```

In a second terminal, run the source script directly.  This requires no package
build and never sends a trajectory to a controller:

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
python3 ~/ResKU/src/arm/tool_manipulator_bringup/scripts/plan_pose_goal.py
```

For mock execution after a plan has been visually reviewed, restart this node
with the explicit opt-in below. Do not use this standalone execution mode on
real hardware; it bypasses the managed arm mode owner:

```bash
python3 ~/ResKU/src/arm/tool_manipulator_bringup/scripts/plan_pose_goal.py \
  --ros-args -p execute_plan:=true
```

Publishing one `PoseStamped` then performs
`topic -> axis IK -> MoveIt plan -> ExecuteTrajectory`. If planning or
collision checking fails, `ExecuteTrajectory` is not called. Status progresses
through `planning -> executing -> executed` on success.

Publish the measured `arm_world` target (the MoveIt root `base_actuator` is
coincident with `arm_world`):

```bash
ros2 topic pub --once /arm/plan_pose_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_actuator}, pose: {position: {x: -0.394506, y: 0.0, z: 0.212964}, orientation: {x: 0.0, y: 0.46174861323503386, z: 0.0, w: 0.8870108331782217}}}"
```

RViz receives a successful plan on `/display_planned_path`.  Inspect the exact
result without the repeated `ros2 topic echo` lost-message warning using:

```bash
ros2 topic echo /arm/pose_plan_status std_msgs/msg/String --once \
  --qos-reliability reliable --qos-durability transient_local
```

The planner interprets the quaternion's local `+X` direction, solves position
plus axis IK while leaving axial rotation free, then asks MoveIt for a
collision-checked joint path. Execution remains disabled unless the mock-only
`execute_plan` opt-in is set.

The 2026-10-31 mock check found a bounded axis-IK solution with TCP position
error below `0.0001 mm` and plane angle `55 deg`, but MoveIt correctly rejected
the final state because `rack_right` contacts `circuit_box`.  Do not add an
allowed-collision pair for these physical parts merely to make the plan pass;
the docking TCP target or required tool clearance must be corrected first.

If the full pose returns MoveIt error `99999`, stop the planner node and rerun
the same source script in position-only diagnostic mode, then publish the same
goal again:

```bash
python3 ~/ResKU/src/arm/tool_manipulator_bringup/scripts/plan_pose_goal.py \
  --ros-args -p use_axis_ik:=false -p use_orientation_constraint:=false
```

This raw position-only mode is diagnostic only. Inspect the `move_group`
terminal for IK, start-state collision, or goal collision diagnostics before
changing geometry.

