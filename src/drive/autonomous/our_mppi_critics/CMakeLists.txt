cmake_minimum_required(VERSION 3.8)
project(our_mppi_critics)

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# [하림 수정] 2026-08-08 SIGSEGV 진단용으로 걸어뒀던 -g -O1(디버그 빌드)이
# SkidCritic(실사용, 매 사이클 실행)까지 최적화 없이 돌게 만들고 있었음 -- 조사가
# 끝나고 진단용 Diag critic도 정리됐으니 제거. 필요하면 ENABLE_ASAN으로 opt-in.

# ASan OFF by default -- see nav2_mppi_controller/CMakeLists.txt for why.
# Opt in with: colcon build --cmake-args -DENABLE_ASAN=ON
option(ENABLE_ASAN "Build with AddressSanitizer instrumentation (debug-only, do not deploy)" OFF)
if(ENABLE_ASAN)
  message(WARNING "our_mppi_critics: building WITH AddressSanitizer -- debug-only, never run this on the real robot")
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
endif()

add_library(our_mppi_critics SHARED
  src/skid_critic.cpp
  src/slope_critic.cpp
  src/accel_critic.cpp
)

target_include_directories(our_mppi_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(our_mppi_critics
  nav2_mppi_controller
  pluginlib
  rclcpp
)

target_link_libraries(our_mppi_critics
  Eigen3::Eigen
)

install(TARGETS our_mppi_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES critics.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_package()