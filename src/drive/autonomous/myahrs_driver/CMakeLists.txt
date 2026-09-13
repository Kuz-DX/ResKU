cmake_minimum_required(VERSION 3.8)
project(myahrs_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 드라이버 실행 파일 빌드
add_executable(myahrs_driver_node src/myahrs_driver_node.cpp)
target_include_directories(myahrs_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# target_link_libraries에 임포트 타겟 이름을 직접 하드코딩하면 ROS 배포판
# 패치 버전에 따라 타겟 이름이 바뀔 때(예: sensor_msgs::sensor_msgs ->
# sensor_msgs::sensor_msgs_library) 깨진다 — ament_target_dependencies는
# 패키지 이름만 쓰고 실제 타겟 이름 해석은 알아서 해줘서 이런 변화에 안전하다.
ament_target_dependencies(myahrs_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

# 인스톨 규칙 설정
install(TARGETS myahrs_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()