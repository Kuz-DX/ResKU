cmake_minimum_required(VERSION 3.8)
project(tool_change_min)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LinearMove.srv"
  "srv/LinearMoveToPose.srv"
  "srv/SolvePoint.srv"
  "srv/MoveNamedPose.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/RotateWristYaw.srv"
  "srv/AlignEe.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/ik_node.py
  scripts/measure_apriltag_docking.py
  scripts/motion_executor.py
  scripts/tool_change_fsm.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
