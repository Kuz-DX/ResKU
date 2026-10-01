# External arm bringup

`tool_change_min` intentionally launches exactly five task nodes and does not
create robot-state, controller, or camera-TF nodes.  Start the arm controller
and the single existing `cam_link -> arm_camera_link` TF owner separately, then
launch `tool_change_min/tool1_attach.launch.py`.
