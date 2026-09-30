cmake_minimum_required(VERSION 3.8)
project(tool_manipulator_bringup LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
)
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/design_load_monitor.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/apriltag_tool_docking_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tag_pose_filter.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/visual_servo_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/docking_manager.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tool_scene_manager.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/config_validator.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tool_dock_tcp_sim_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/capture_arm_pose.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/capture_arm_raw.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/move_to_named_pose.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/tcp_joy_teleop.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gripper_gox.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/design_sizing.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_package()
