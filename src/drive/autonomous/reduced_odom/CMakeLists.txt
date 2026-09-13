cmake_minimum_required(VERSION 3.8)
project(reduced_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(reduced_odom_node src/reduced_odom_node.cpp)
target_include_directories(reduced_odom_node PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(reduced_odom_node
  rclcpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS reduced_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reduced_ekf test/test_reduced_ekf.cpp)
  target_include_directories(test_reduced_ekf PRIVATE include ${EIGEN3_INCLUDE_DIRS})
endif()

ament_package()
