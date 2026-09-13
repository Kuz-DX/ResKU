cmake_minimum_required(VERSION 3.8)
project(path_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(path_relay_node src/path_relay_node.cpp)

ament_target_dependencies(path_relay_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS path_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
