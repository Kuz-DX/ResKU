cmake_minimum_required(VERSION 3.8)
project(mppi_repro_harness)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_compile_options(-g -O1)

# Same opt-in-only ASan pattern as nav2_mppi_controller/our_mppi_critics --
# see nav2_mppi_controller/CMakeLists.txt for the rationale. Build this into
# build_asan/install_asan alongside the ASan-instrumented mppi libraries.
option(ENABLE_ASAN "Build with AddressSanitizer instrumentation (debug-only, do not deploy)" OFF)
if(ENABLE_ASAN)
  message(WARNING "mppi_repro_harness: building WITH AddressSanitizer -- debug-only")
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
endif()

add_executable(repro_harness src/repro_harness.cpp)

ament_target_dependencies(repro_harness
  rclcpp
  rclcpp_lifecycle
  nav2_mppi_controller
  nav2_costmap_2d
  nav2_core
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS repro_harness
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
