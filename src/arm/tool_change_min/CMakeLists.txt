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

# rosidl_generator_py already installs the Python package and its __init__.py.
# Install only our implementation modules to avoid duplicate ament targets.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES
  ${PROJECT_NAME}/config.py
  ${PROJECT_NAME}/kinematics.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(PROGRAMS
  scripts/ik_node.py
  scripts/joystick_manipulator.py
  scripts/measure_apriltag_docking.py
  scripts/motion_executor.py
  scripts/tool_change_fsm.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
