# Arm Control Mode Manager

`managed_control.launch.py` starts a separate mode-managed stack, initially in
HOLD, using mock hardware by default. It does not modify the legacy manual launch.
Do not run both stacks together. The preflight refuses an existing arm controller
manager. This check is not a distributed lock against simultaneous launches.

## Run from source without building

```bash
source /opt/ros/humble/setup.bash
source ~/ResKU/install/setup.bash
ros2 launch ~/ResKU/src/arm/tool_manipulator_bringup/launch/managed_control.launch.py
```

The source launch invokes Python scripts directly. Existing installed dependencies
and controller plugins are required. `real_hardware:=true` selects real hardware;
startup has been verified with isolated mock hardware, not on the physical arm.
Initial joint positions must be inside hardware soft limits. Managed startup does
not automatically recover out-of-limit joints or move to home.

## Ownership

The manager alone publishes `/manual_position_controller/commands` and
`/ee_velocity_controller/commands`. All modes share the position controller:
MANUAL and TCP_IK supply joint targets; TOOL_AUTO supplies joint waypoints.
Targets are ramped at at most min(0.12 rad/s, half hardware speed).
These are velocity-bounded position ramps, not acceleration-limited planned paths.
There is no collision planning in this gateway.

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
ros2 topic pub --once /control/active_target std_msgs/msg/String '{data: arm}'
ros2 service call /arm/set_mode/manual std_srvs/srv/Trigger '{}'
ros2 topic echo /arm/control_status
ros2 service call /arm/cancel std_srvs/srv/Trigger '{}'
```

During a transition all producer commands are rejected. The manager keeps sending
bounded hold commands and EE zero until measured velocity and position settle for
five samples. A fresh UUID epoch invalidates commands queued before each request,
cancel, or fault. Producers wait for transitioning=false and ready=true and copy
the new epoch. Manual additionally requires neutral controls.

## Producer protocol

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
