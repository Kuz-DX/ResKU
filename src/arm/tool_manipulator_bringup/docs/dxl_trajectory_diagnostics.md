# DXL trajectory investigation (offline preparation)

The trajectory observer itself is passive. RMD commands, Servo/JTC configuration,
homing and position limits are unchanged. No new hardware run was performed.

## Current DXL-only profile trial

At the user's explicit request, hardware.yaml now configures IDs 1/2/3 with
Profile Velocity=0, Profile Acceleration=0, Velocity Limit=200. ID4 is unchanged.
This removes the motor profile (step instruction); it is a trial of the profile
hypothesis, not evidence that the old profile caused the stall. It can produce
sharper physical response. Startup verifies mode, profiles and limit by readback;
a mismatch aborts initialization before enabling torque.

`DIAG_WRITE_ENTRY pre_guard=true` captures the first three DXL ROS position
commands at the first executable statement of write(), before torque handling,
conversion and clamps. It reports 0.5s deltas for up to 2.5s after a change and is
silent once commands stay unchanged. Compare with DIAG_TRAJ desired/output and
DIAG_POSITION ros_cmd/sent_raw. The requested 2s joystick test was not performed.

The local code does not reset commands every cycle just because torque is OFF.
Sync is called at startup and explicit torque transitions. Torque failures used
to ignore their return value and still sync; these failures now preserve the
pending request without overwriting commands. This defect was not observed in
the supplied logs and is not claimed as the runtime stall cause.

Feedback uses SetDxlReadItems shared_ptr -> read_data_list_.item_data_ptr_vec ->
hdl_trans_states_ -> CalcTransmissionToJoint -> exported hdl_joint_states_.
Write uses hdl_trans_commands_ shared_ptr -> SetDxlWriteItems ->
write_data_list_.item_data_ptr_vec. Passing vectors by value copies shared_ptr,
not the pointed double. read_item_buf_/read_flag belongs to a separate service
read queue and is not required by the periodic transport read path.

No small-target deadband exists in WriteDxlDataByTransport; all configured IDs
are sent each cycle. Changing global open_loop_control would also change RMD,
so it was not changed.

## What the existing logs establish

- ID 2 receives JointJog +/-0.12 rad/s while ROS targets remain near zero
  (for example 0.004507254 rad). Goal register matches the packet raw value,
  torque is on and Hardware Error Status is zero.
- ID 1 goals move 25 -> 23 -> 19 ticks and feedback 25 -> 24 -> 22; this is not
  complete serial failure. The small movement does not explain the sustained jog.
- There was no nonzero ID 3 yaw jog in the supplied log. Test that input separately.
- Those logs have no Servo trajectory/controller-state samples. They cannot prove
  whether the small target originates in Servo, JTC, or slow hardware response.

## Five hypotheses checked

1. Measured-state anchoring: upstream Humble Servo `updateJoints()` takes the
   current monitored state; `internalServoUpdate()` starts from that state before
   applying the command increment/filter/limits. This supports the proposed
   mechanism, but does not establish that it caused this hardware symptom.
2. Replacement/interpolation: local JTC configuration uses `open_loop_control:
   false` and full six-joint position trajectories. Replacement start behavior
   has not been proved for the installed binary. Do not switch this globally:
   it also controls RMD. Observe `p0`, `plast`, `desired`, `actual`, and times first.
3. Conversion: native C++ model-loader tests for model 1100 and 321, signed
   multi-turn values -8192..8192, ID 1/2/3 calibration zeros and both directions
   roundtrip within one encoder tick. Read is `(model-zero)*direction`, write is
   `zero+direction*joint`; the configured direction is +/-1. No gross scale/sign
   asymmetry was found. Integer casts can produce one-tick quantization.
4. Scaling: with `speed_units`, upstream Servo uses velocity*publish_period;
   it does not apply the default unitless joint_scale. +/-0.12*0.05 is +/-0.006
   rad before smoothing/limits/collision scaling. Status and actual output still
   need to be measured. JointJog does not use the Cartesian IK singularity path.
5. Profiles: configured velocity=10 and limit=20 correspond to about 0.240 and
   0.479 rad/s in velocity-based mode, above 0.12 rad/s. Acceleration=5 and
   repeated profile updates may affect response, but the stall is not proved.
   Drive Mode bit 2 selects time-based profiles on supported firmware. DIAG_INIT
   now reads Drive Mode and Profile Acceleration as well. Profile Velocity=0 is
   unlimited in velocity-based mode; it is not a conservative default fix.

Sources (upstream, not proof of exact installed binary behavior):
- https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/src/servo_calcs.cpp
- https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/#profile-velocity112

## Reading the new log

Enabled through existing `buses.dynamixel_u2d2.position_diagnostics: true`.
Manual coordinated mode subscribes passively to:
- `/servo_node/delta_joint_cmds`
- `/arm_controller/joint_trajectory`
- `/arm_controller/controller_state` and legacy `/arm_controller/state`
- `/joint_states` (sensor QoS, compatible with reliable and best-effort publishers)
- `/servo_node/status`

`DIAG_TRAJ` prints four lines every 0.5s during input and a one-second stop tail:
base, wrist roll, wrist yaw, shoulder (read-only comparison).
No input means no repeated trajectory diagnostic output.

Fields:
- `traj_index`: index resolved by name, `None` means absent
- `jog_v`: published JointJog velocity
- `p0/v0`, `plast`: first/last Servo point; `t0/tlast`: sec:nanosec
- `dp_msg`: first-point position change between the two latest received trajectories
- `expected_dp`: current jog velocity times observed trajectory receipt interval
- `rx_dt`: receipt interval (not a hardware timestamp; use nominal 0.05s as reference)
- `desired/actual/error/controller_output`: controller point fields; schema selected
  from populated reference/feedback or legacy desired/actual arrays
- `js_p/js_v`: latest published joint state
- `age_*`: monotonic sample ages; missing/nonfinite fields print NA
- `servo_status`: raw Servo status code, not interpreted as a root cause

All streams are sampled asynchronously, so compare sustained trends, not exact
instant equality. DIAG_POSITION adds `hardware_state_rad`, the normal read-path
ROS joint state; compare it with `js_p/actual`, allowing sample age differences.

Decision guide:
- Nonzero jog + nearly constant p0 near actual: investigate Servo/profile-feedback
  interaction and status; do not implement blind goal accumulation.
- p0 accumulates but desired does not: investigate JTC replacement/interpolation.
- desired changes but DIAG_POSITION ros_cmd does not: controller/interface handoff.
- ros_cmd/sent_raw/goal_raw change together but present does not: inspect DXL
  profile/drive-mode/load, bounded by existing limits.
- hardware_state and js_p disagree persistently at similar ages: investigate state
  export/conversion. One-tick differences alone do not establish a scale error.

For the next authorized hardware session, separately operate ID1, ID2, ID3; retain
DIAG_JOG, DIAG_TRAJ and DIAG_POSITION lines from the same time window. Do not change
RMD or global Servo/JTC settings to test a DXL-only hypothesis.
