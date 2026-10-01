cmake_minimum_required(VERSION 3.8)
project(rmd_joint_state_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/joint_state_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_bridge_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
