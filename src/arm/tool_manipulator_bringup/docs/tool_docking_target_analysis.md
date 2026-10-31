# Tool 0/1 docking target comparison

All positions below use the URDF `world` frame and metres unless marked as mm.
The measured circular-top centres supplied on 2026-10-31 are:

| Tool | Target `tcp_link` world XYZ (mm) | Surface tilt from world XY |
| --- | --- | --- |
| 0 | `[-394.506, 0.000, 212.964]` | `55 deg` |
| 1 | `[-317.806, 90.000, 163.912]` | `45 deg` |

`world -> arm_world/base_actuator` is `[+192.1, 0, +235]` mm with no rotation.
The same targets in the MoveIt planning root `base_actuator` are therefore:

| Tool | Target `tcp_link` base XYZ (mm) |
| --- | --- |
| 0 | `[-586.606, 0.000, -22.036]` |
| 1 | `[-509.906, 90.000, -71.088]` |

The old final joint samples currently stored in `tool0_trajectory.yaml` and
`tool1_trajectory.yaml` produce:

| Tool | Existing final `tcp_link` world XYZ (mm) | target - FK (mm) | Distance (mm) |
| --- | --- | --- | --- |
| 0 | `[538.484, -40.154, 467.662]` | `[-932.990, +40.154, -254.698]` | `967.964` |
| 1 | `[-25.933, 39.270, 453.553]` | `[-291.873, +50.730, -289.641]` | `414.313` |

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

