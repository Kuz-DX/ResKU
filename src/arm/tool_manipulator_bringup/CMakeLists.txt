cmake_minimum_required(VERSION 3.8)
project(tool_manipulator_bringup LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
)
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
<<<<<<< HEAD
install(PROGRAMS scripts/design_load_monitor.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/apriltag_tool_docking_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tool_dock_tcp_sim_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/capture_arm_pose.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tcp_joy_teleop.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gripper_gox.py DESTINATION lib/${PROJECT_NAME})
=======
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/design_load_monitor.py
  scripts/apriltag_tool_docking_node.py
  scripts/tool_attachment_manager.py
  scripts/tool_dock_tcp_sim_node.py
  scripts/capture_arm_pose.py
  scripts/capture_rmd_calibration.py
  scripts/hardware_preflight.py
  scripts/config_validator.py
  scripts/tool_change_state_machine.py
  scripts/docking_manager.py
  scripts/tag_pose_filter.py
  scripts/visual_servo_node.py
  scripts/tool_scene_manager.py
  scripts/tcp_joy_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)
>>>>>>> e7440144978d0a83a71e6c865332224d9a504272
install(FILES scripts/design_sizing.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/design/ DESTINATION share/${PROJECT_NAME}/design)
ament_package()
