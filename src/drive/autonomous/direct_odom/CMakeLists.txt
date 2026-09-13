cmake_minimum_required(VERSION 3.8)
project(direct_odom)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  direct_odom/direct_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME direct_odom_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
