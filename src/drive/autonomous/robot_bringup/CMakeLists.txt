cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

# ==============================================================================
# 1. 패키지 의존성 선언
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)   # [하림 수정] stability_monitor_node의 카메라 roll 구독용
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)        # IMU 쿼터니언 변환 계산용
find_package(diagnostic_msgs REQUIRED)  # /wheel/motor_status 구독용
find_package(nav_msgs REQUIRED)  # [하림 수정] slope_traverse_node의 /path 구독용 (탈출 곡률 판정)
find_package(mission_manager_interfaces REQUIRED)  # [하림 수정] summer_supply_drive_node의 traffic 신호 구독용
# [2026-08-28] slope_traverse_node의 SLOPE_EXIT pure pursuit이 /path를
# camera_link -> base_link로 변환하는 데 필요 (purepursuit_node/path_relay_node와
# 동일 패턴, path_relay/src/path_relay_node.cpp 참고)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==============================================================================
# 2. 빌드 타겟 정의 (Executables & Dependencies)
# ==============================================================================

# [노드 2] 긴급정지 전담 안정성 감시 노드 (IMU pitch/roll + 카메라 roll + 전류/고장 -> /cmd_vel_safety)
# [하림 수정] /speed_limit 발행 제거함 (자세 기반 감속은 slope_traverse_node가
# 전담). [2026-08-27] 이 발행을 담당하던 slope_speed_limiter_node 자체를
# 패키지 통째로 삭제 -- 임계값 미설정으로 오래 방치돼 실질 무동작 상태였음.
add_executable(stability_monitor_node
  src/stability_monitor_node.cpp
)
ament_target_dependencies(stability_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  diagnostic_msgs
)

# [하림 수정] joy_mux_node 제거함 -- manual 조종은 이제 robot_bringup이 아니라
# manual_joy_control+can_driver 전담. robot_bringup은 autonomous 전용 패키지.

# [노드 4] 소프트 전류 램프 노드 (autonomous 전용, 45A/3초 하드컷오프와는 별개 레이어)
add_executable(current_ramp_node
  src/current_ramp_node.cpp
)
ament_target_dependencies(current_ramp_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# [2026-08-25] auto_cmd_relay_node 제거 -- joy_mux_node(위 참고, 이미 삭제됨)의
# SHARE 게이팅을 우회하려던 순수 pass-through였는데, 우회할 대상 자체가 없어져서
# 죽은 코드였음. current_ramp_node가 controller_server의 /cmd_vel_auto를 바로
# 받도록 정리(autonomous.launch.py 참고). 필요해지면 git 이력에서 복구.

# [노드 6, 2026-08-26 imu_slope_mode_node 대체] 인지팀 signed left/right 경사
# 신호를 받아 정렬->직진등반->path 곡률 기반 차동조향 탈출까지 직접
# /cmd_vel_safety로 발행하는 노드 (slope_traverse_node.cpp 상단 docstring 참고)
add_executable(slope_traverse_node
  src/slope_traverse_node.cpp
)
ament_target_dependencies(slope_traverse_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

# [노드 6b, 2026-08-28 신규] slope_traverse_node의 PlanB 대안 -- SLOPE_EXIT
# 진입 시 pure pursuit(TF 의존)으로 전환하는 대신, 기존 blend+P제어
# 구조를 유지하되 blend를 강제로 고정한다(slope_traverse_blend_node.cpp
# 상단 docstring 참고). TF/tf2_ros/tf2_geometry_msgs에 의존하지 않는
# 차선책 -- path_control_blend.launch.py가 slope_traverse_node 대신
# 이 노드를 띄운다.
add_executable(slope_traverse_blend_node
  src/slope_traverse_blend_node.cpp
)
ament_target_dependencies(slope_traverse_blend_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# [노드 6c, 2026-09-04 신규] slope_traverse_node의 "PlanC" 대안 -- 커밋
# c5aef4d1b0c374abda2a688b04c020ff34b62e8a 시점(2026-08-27, SLOPE_EXIT/
# RECOVERY 최초 도입 시점)의 slope_traverse_node.cpp를 그대로 되살린 노드
# (slope_traverse_planc_node.cpp 상단 docstring 참고). PlanA(TF 기반 pure
# pursuit)/PlanB(TF 없는 pure pursuit)와 달리 SLOPE_EXIT을 고정 게인
# P제어(kp_exit_)로 처리하던 더 이전 버전 -- path_control_planc.launch.py가
# slope_traverse_node 대신 이 노드를 띄운다.
add_executable(slope_traverse_planc_node
  src/slope_traverse_planc_node.cpp
)
ament_target_dependencies(slope_traverse_planc_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# [2026-09-05 신규] slope_traverse_planc_node(PlanC)를 그대로 복사한 봄 미션
# 전용 노드 -- STARTUP 상태 하나만 추가(노드 시작 즉시 무조건
# initial_straight_drive_sec_초 직진, slope_traverse_planc_spring_node.cpp
# 상단 docstring 참고). mission_spring_autonomous_drive.launch.py가 이 노드를
# 띄운다.
add_executable(slope_traverse_planc_spring_node
  src/slope_traverse_planc_spring_node.cpp
)
ament_target_dependencies(slope_traverse_planc_spring_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# [노드 6d, 2026-09-05 신규] slope_traverse_planc_node(PlanC)의 "PlanD" 변형 --
# side==0이어도 SLOPE_DRIVE에서 곧장 IDLE로 안 빠지고 SLOPE_EXIT/RECOVERY를
# 반드시 거쳐야 IDLE 복귀(slope_traverse_pland_node.cpp 상단 docstring 참고).
# path_control_pland.launch.py가 이 노드를 띄운다.
add_executable(slope_traverse_pland_node
  src/slope_traverse_pland_node.cpp
)
ament_target_dependencies(slope_traverse_pland_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# [노드 6e, 2026-09-05 신규] slope_traverse_pland_node(PlanD)를 그대로 복사한
# 봄 미션 전용 노드 -- STARTUP 상태 하나만 추가(노드 시작 즉시 무조건
# initial_straight_drive_sec_초 직진, slope_traverse_spring_node.cpp 상단
# docstring 참고). mission_spring_drive.launch.py가 이 노드를 띄운다.
add_executable(slope_traverse_spring_node
  src/slope_traverse_spring_node.cpp
)
ament_target_dependencies(slope_traverse_spring_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# [노드 6f, 2026-09-05 신규] slope_traverse_pland_node(PlanD)의 "PlanE" 변형 --
# SLOPE_EXIT의 slope_exit_side_motor_dps만 400->300(그 외 전부 PlanD와 동일,
# slope_traverse_plane_node.cpp 상단 docstring 참고). path_control_plane.launch.py가
# 이 노드를 띄운다.
add_executable(slope_traverse_plane_node
  src/slope_traverse_plane_node.cpp
)
ament_target_dependencies(slope_traverse_plane_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
)

# 여름 미션 주행 제어: 정지 상태머신 + /path pure pursuit.
add_executable(summer_supply_drive_node
  src/summer_supply_drive_node.cpp
)
ament_target_dependencies(summer_supply_drive_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mission_manager_interfaces
)

# [노드 8, 2026-08-29 신규] 로봇개 추종 미션 전용 주행 제어 노드 -- 로봇개
# 상대위치(PLACEHOLDER 인터페이스)를 받아 거리/방향을 유지하며 쫓아가는
# target-following 상태머신 (dog_follow_node.cpp 상단 docstring 참고).
# MPPI/perception 경로 없이 이 노드가 /cmd_vel_auto의 유일한 publisher.
add_executable(dog_follow_node
  src/dog_follow_node.cpp
)
ament_target_dependencies(dog_follow_node
  rclcpp
  std_msgs
  geometry_msgs
)

# ==============================================================================
# 3. 인스톨 및 배포 규칙 설정 (Install Rules)
# ==============================================================================

install(TARGETS
  stability_monitor_node
  current_ramp_node
  slope_traverse_node
  slope_traverse_blend_node
  slope_traverse_planc_node
  slope_traverse_planc_spring_node
  slope_traverse_pland_node
  slope_traverse_spring_node
  slope_traverse_plane_node
  summer_supply_drive_node
  dog_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

# [2026-09-02, 복원] dolbotz/purepursuit.py의 2026-08-27 시점 코드를
# robot_bringup 전용 독립 노드로 되살린 python 스크립트. ament_cmake
# 패키지라 setup.py entry_point가 없으므로 install(PROGRAMS ...)로
# lib/${PROJECT_NAME}/ 아래 실행 파일로 등록한다 (`ros2 run robot_bringup
# autonomous.planz.py`로 바로 실행 가능).
install(PROGRAMS
  scripts/autonomous.planz.py
  scripts/autonomous.planz_spring.py
  scripts/autonomous.planz_winter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
