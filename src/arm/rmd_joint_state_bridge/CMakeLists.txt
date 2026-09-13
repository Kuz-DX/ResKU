cmake_minimum_required(VERSION 3.8)
project(rmd_joint_state_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmd_sdk REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joint_state_bridge_node src/joint_state_bridge_node.cpp)
target_compile_features(joint_state_bridge_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joint_state_bridge_node
  rclcpp
  rmd_sdk
  dynamixel_sdk
  sensor_msgs
  std_msgs
)

install(TARGETS joint_state_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
