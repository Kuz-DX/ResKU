cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

# [2026 사용자 결정, 경량화 + 계절 미션 정리] 계절별 미션 전용 C++ 노드
# (slope_traverse_*, dog_follow_node, summer_supply_drive_node) 및
# current_ramp_node/stability_monitor_node(전류 제한/자세 긴급정지)를 전부
# 삭제하면서, 이 패키지에 남는 C++ 실행 파일이 없어졌다. 지금은 순수
# launch/config 패키지다 -- 필요한 실제 노드(rmd_x8_driver, reduced_odom,
# drive_cmd_mux, return_navigation, nav2_controller 등)는 전부 다른
# 패키지에 있다.

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
