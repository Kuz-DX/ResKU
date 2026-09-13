# DolbotZ Repository Structure

이 문서는 현재 워크트리를 정적으로 분석해 폴더, ROS 2 패키지, 실행 노드/플러그인의 역할을 정리한 것이다. 패키지 빌드나 실행 검증은 하지 않았고, `package.xml`, `setup.py`, `CMakeLists.txt`, launch 파일, 주요 소스 상단 설명을 기준으로 작성했다.

세부 발행 토픽 목록은 루트의 `topic.md`가 더 자세하다. 이 문서는 "어디에 무엇이 있고 어떤 책임을 갖는가"에 초점을 둔다.

## 1. 전체 구조 요약

| 경로 | 성격 | 역할 |
|---|---|---|
| `src/arm/` | ROS 2 패키지 묶음 | 로봇팔 URDF/MoveIt/ros2_control, RMD 및 Dynamixel 하드웨어 인터페이스, 수동/자동 팔 bringup |
| `src/drive/manual/` | ROS 2 패키지 묶음 | 수동 구동: 조이스틱 입력을 좌우 모터 속도로 변환하고 CAN으로 송신 |
| `src/drive/autonomous/` | ROS 2 패키지 묶음 | 자율 구동: 휠 모터 드라이버, IMU, 오도메트리, Nav2 MPPI, 경사 통과 상태 머신 |
| `src/dolbotz/` | ROS 2 Python 패키지 | 비전/인지, 주행 가능 영역 추정, 계절별 미션 인식, Pure Pursuit, 시각화 유틸 |
| `src/mission_manager_interfaces/` | ROS 2 인터페이스 패키지 | 계절 미션 결과 공통 메시지 `MissionResult` 정의 |
| `src/usb_cam/` | gitlink 상태 | `usb_cam` 외부 패키지로 보이나 현재 `.gitmodules`에는 매핑이 없다. 워크트리에는 내용이 비어 있을 수 있다. |
| `Arduino/LED_Control/` | Arduino 스케치 | 봄 피아식별 결과를 시리얼 명령으로 받아 빨강/초록 LED 제어 |
| `util/` | 독립 유틸 | ROS `CompressedImage`를 MediaMTX RTSP로 보내는 브리지 스크립트와 사용법 |
| `utils/terminal_launcher/` | 독립 유틸 | `commands.csv`의 여러 실행 명령을 tmux pane에 배치하는 터미널 런처 |
| `tmp_test_scripts/` | 임시/오프라인 도구 | MPPI, synthetic path, bag 기반 재현/스트레스 테스트용 스크립트 |
| 루트 `*.md` | 운영/검증 문서 | 실행법, 체크리스트, 토픽 표, UI/검증/주행 관련 메모 |

## 2. 패키지 역할 지도

| 패키지 | 위치 | 종류 | 한 줄 역할 |
|---|---|---|---|
| `dolbotz` | `src/dolbotz` | Python ROS package | 비전 기반 주행 영역/지형 판단과 계절 미션 인식의 중심 패키지 |
| `mission_manager_interfaces` | `src/mission_manager_interfaces` | interface | `MissionResult` 공통 메시지 |
| `manual_joy_control` | `src/drive/manual/manual_joy_control` | Python ROS package | 조이스틱을 `/motor_speed_cmd` 좌우 dps 명령으로 변환 |
| `can_driver` | `src/drive/manual/can_driver` | Python ROS package | `/motor_speed_cmd`를 RMD 구동 CAN 명령으로 송신, 수동 안전 정지 |
| `rmd_x8_driver` | `src/drive/autonomous/rmd_x8_driver` | Python/CMake ROS package | 자율 주행용 RMD-X8 좌우 구동모터 CAN 드라이버 |
| `myahrs_driver` | `src/drive/autonomous/myahrs_driver` | C++ ROS package | WithRobot myAHRS+ 시리얼 IMU 드라이버 |
| `reduced_odom` | `src/drive/autonomous/reduced_odom` | C++ ROS package | 휠 오도메트리와 IMU yaw를 융합한 5상태 오도메트리 |
| `direct_odom` | `src/drive/autonomous/direct_odom` | Python/CMake ROS package | 휠/IMU 기반 직접 오도메트리 대안 노드 |
| `path_relay` | `src/drive/autonomous/path_relay` | C++ ROS package | 인지팀 `/path`를 Nav2 `FollowPath` 액션으로 중계 |
| `robot_bringup` | `src/drive/autonomous/robot_bringup` | C++/Python ROS package | 자율 주행 launch, 경사 통과, 전류 램프, 안정성 감시 |
| `nav2_mppi_controller` | `src/drive/autonomous/nav2_mppi_controller` | C++ plugin package | Nav2 MPPI controller fork/로컬 복사본 |
| `our_mppi_critics` | `src/drive/autonomous/our_mppi_critics` | C++ plugin package | tracked/skid 차량용 MPPI critic 플러그인 |
| `mppi_repro_harness` | `src/drive/autonomous/mppi_repro_harness` | C++ utility package | MPPI 재현용 하네스와 synthetic path 도구 |
| `rmd_sdk` | `src/arm/rmd_sdk` | C++ library + Python binding | MyActuator RMD-X CAN 저수준 SDK |
| `rmd_hardware_interface` | `src/arm/rmd_hardware_interface` | ros2_control plugin | RMD 관절을 ros2_control actuator interface로 연결 |
| `dynamixel_sdk` | `src/arm/dynamixelSDK/ros/dynamixel_sdk` | SDK library | ROBOTIS Dynamixel SDK ROS 패키지 |
| `dynamixel_sdk_custom_interfaces` | `src/arm/dynamixelSDK/ros/dynamixel_sdk_custom_interfaces` | interface examples | ROBOTIS 예제용 `SetPosition`, `GetPosition` 인터페이스 |
| `dynamixel_interfaces` | `src/arm/dynamixel_interfaces` | interface | Dynamixel 상태/서비스 메시지 |
| `dynamixel_hardware_interface` | `src/arm/dynamixel_hardware_interface` | ros2_control plugin | Dynamixel base/gripper를 ros2_control에 연결 |
| `army_manipulator_description` | `src/arm/army_manipulator_description` | description | 현재 MARU/army manipulator URDF, mesh, joint calibration |
| `army_manipulator_moveit_config` | `src/arm/army_manipulator_moveit_config` | MoveIt config | SRDF, kinematics, controllers, OMPL 등 MoveIt 설정 |
| `army_manipulator_bringup` | `src/arm/army_manipulator_bringup` | launch/scripts | 로봇팔 자동 파지, MoveIt, RealSense, perception bringup |
| `robot_arm_description` | `src/arm/robot_arm_description` | legacy description | 3축 RMD 수동 팔 제어용 구형/별도 URDF와 controller 설정 |
| `robot_arm_bringup` | `src/arm/robot_arm_bringup` | launch/scripts | 조이스틱 기반 수동 팔 제어와 안전 모드 관리 |
| `rmd_joint_state_bridge` | `src/arm/rmd_joint_state_bridge` | C++ ROS package | 팔 관절 엔코더를 읽기 전용으로 `/joint_states`에 미러링 |

## 3. Top-Level Non-ROS 폴더

### `Arduino/LED_Control`

`LED_Control.ino`는 `/led_control`을 시리얼로 전달받은 Arduino가 `roka`, `enemy`, `none` 명령을 해석해 초록/빨강 LED를 제어하는 펌웨어다. 봄 피아식별 미션에서 `dolbotz`의 `spring_ifof` 또는 `led_relay`/`led_bridge_node`와 연결된다.

### `util`

`ros_compressed_to_rtsp.py`는 ROS 2 JPEG `sensor_msgs/CompressedImage` 토픽을 구독해 Python에서는 JPEG를 디코딩하지 않고 GStreamer 파이프라인으로 H.264 변환 후 MediaMTX RTSP로 송출한다. `util/README.md`는 `usb_cam`, MediaMTX, 브리지 실행 순서를 설명한다.

### `utils/terminal_launcher`

`commands.csv`에 정의된 명령들을 tmux pane에 "입력만 해둔 상태"로 열어 주는 런처다. 여러 launch를 현장에서 빠르게 준비하되, 각 pane에서 사람이 Enter를 눌러 실행하도록 설계되어 있다.

### `tmp_test_scripts`

실기 운영 패키지라기보다 MPPI/경로추종/오도메트리 문제를 오프라인으로 재현하기 위한 스크립트 묶음이다. synthetic circle/half-circle/sharp-turn path, bag path 재스탬프, cross-track error logger, EKF divergence monitor 등이 있다.

## 4. `dolbotz`: 비전, 지형 판단, 미션

### 주요 하위 폴더

| 경로 | 역할 |
|---|---|
| `dolbotz/drive_area/` | 주행 가능 영역 세그멘테이션, BEV 경로, 고도맵, 경사맵, 평지/경사 경로 선택 |
| `dolbotz/missions/` | 봄/여름/가을/호위 미션별 인식 및 결과 발행 |
| `dolbotz/utils/` | QoS, 카메라 launch helper, 자세 추정, 경로/지형 시각화, 공통 상수 |
| `config/models/` | OpenVINO/YOLO 계열 모델 파일 |
| `config/realsense_cameras.yaml` | 주행용 D455와 팔 D435i 카메라 설정 |
| `config/purepursuit_params.yaml` | Pure Pursuit 주행 파라미터 |
| `launch/` | 미션별 perception/drive/camera/visualization launch 묶음 |

### 실행 노드

| 실행 이름 | 소스 | 역할 |
|---|---|---|
| `segmentation` | `drive_area/segmentation.py` | 주행 RGB 이미지에서 `area` 마스크를 추론해 `/perception/drivable_mask` 발행 |
| `flat_drive` | `drive_area/flat_drive.py` | 마스크와 카메라 자세를 이용해 평지용 `/flatdrive/planned_path`와 BEV 디버그 영상 발행 |
| `elevation_map` | `drive_area/elevation_map.py` | depth, IMU, 마스크를 이용해 `/terrain/elevation_map` 생성 |
| `gradient_map` | `drive_area/gradient_map.py` | 고도맵에서 gradient/slope field와 경사용 `/terrain/planned_path` 계산 |
| `slope_decision` | `drive_area/slope_decision.py` | 좌우 경사와 전방 slope를 보고 평지/경사 경로 중 하나를 `/path`로 릴레이 |
| `purepursuit` | `purepursuit.py` | `/path`를 직접 추종해 `/motor_speed_cmd` 좌우 dps를 발행하는 Nav2 우회 경로 |
| `summer_supply` | `missions/summer_supply.py` | 팔 카메라에서 보급품 3D 위치를 찾아 `/arm/target_point` 발행 |
| `drive_supply_detector` | `missions/drive_supply_detector.py` | 주행 카메라에서 보급품 접근/검출 이벤트를 판단 |
| `summer_traffic` | `missions/summer_traffic.py` | 여름 신호등 상태를 `MissionResult`와 go/stop 신호로 발행 |
| `spring_ifof` | `missions/spring_ifof.py` | 좌/우 카메라 피아식별 결과를 디바운싱 후 `MissionResult`와 LED 명령으로 발행 |
| `fall_marker` | `missions/fall_marker.py` | 가을 비전 마커를 좌/우 카메라에서 순차 인식 |
| `escort_follow` | `missions/escort_follow.py` | 선도 로봇/로봇개 객체를 인식해 상대 위치, debug image, 추종 path를 발행 |
| `led_relay` | `missions/led_relay.py` | `MissionResult`를 `roka/enemy/none` LED 문자열로 매핑 |
| `led_bridge_node` | `missions/led_bridge_node.py` | `/led_control` 문자열을 Arduino 시리얼 포트로 전달 |
| `arm_visualizer` | `arm_visualizer.py` | `summer_supply` debug image와 target point를 OpenCV 창으로 표시 |
| `slope_visualizer` | `utils/slope_visualizer.py` | slope/elevation을 RViz marker로 시각화 |
| `terrain_viz_relay` | `utils/terrain_viz_relay.py` | `32FC1` 지형 이미지를 컬러 visualization image로 변환 |
| `path_camera_overlay_relay` | `utils/path_camera_overlay_relay.py` | `/path`를 카메라 원본 영상 위에 오버레이 |

### 대표 launch

| launch | 묶는 기능 |
|---|---|
| `perception_common.launch.py` | 주행 영역 `segmentation` 공통 실행 |
| `drive_cam.launch.py`, `arm_cam.launch.py`, `side_cameras.launch.py` | RealSense/USB side camera 실행 |
| `mission_spring.launch.py` | 공통 주행 인지 + side camera + 피아식별 + LED bridge |
| `mission_summer.launch.py` | 주행 인지 + 여름 보급품/신호등 미션 |
| `mission_fall.launch.py` | 주행 인지 + 가을 마커 인식 |
| `mission_winter.launch.py` | 눈길 모델 옵션 포함 주행 인지 |
| `mission_escort.launch.py`, `mission_escort_drive.launch.py` | 선도 로봇 추종 인지/주행 묶음 |
| `purepursuit.launch.py` | static TF + `can_driver_node` + `purepursuit` 기반 직접 주행 |
| `visualization.launch.py` | path/terrain/slope debug visualization |

## 5. 수동 구동 패키지

### `manual_joy_control`

| 노드 | 역할 |
|---|---|
| `manual_joy_control_node` | `/joy`를 받아 좌우 모터 속도 배열 `/motor_speed_cmd`를 발행한다. `/control/active_target`이 있으면 주행/팔 제어 대상 분리를 따른다. |

`manual_control.launch.py`는 `joy_node`와 `manual_joy_control_node`를 함께 실행한다.

### `can_driver`

| 노드 | 역할 |
|---|---|
| `can_driver_node` | `/motor_speed_cmd`와 `/motor_speed_cmd_safety`를 받아 RMD 구동모터 CAN speed command로 송신한다. 명령 timeout과 재연결 로직을 가진다. |
| `manual_stability_node` | `/imu`의 pitch/roll 위험 자세를 감지해 수동 주행용 정지 명령 `/motor_speed_cmd_safety`를 발행한다. |
| `motor_status_monitor` | RMD motor status를 폴링/디코딩하는 CLI 성격의 상태 확인 도구다. |

`manual.launch.py`는 `can_driver_node`, 선택적 `myahrs_driver_node`, `manual_stability_node`를 묶는다.

## 6. 자율 구동 패키지

### 하드웨어/센서/오도메트리

| 패키지/노드 | 역할 |
|---|---|
| `rmd_x8_driver_node` | `/cmd_vel` 또는 우선순위 `/cmd_vel_safety`를 좌우 RMD-X8 속도 명령으로 변환하고 `/wheel/odom`, `/wheel/joint_states`, `/wheel/motor_status`를 발행한다. |
| `myahrs_driver_node` | myAHRS+ 시리얼 데이터를 읽어 `/imu`를 발행한다. |
| `reduced_odom_node` | `/wheel/odom`과 `/imu`를 융합해 `/odometry/filtered`와 `odom -> base_link` TF를 발행한다. |
| `direct_odom_node` | `reduced_odom`과 별도로 휠/IMU를 직접 결합하는 Python 오도메트리 대안 노드다. |

### 경로 추종/Nav2

| 패키지/노드 또는 플러그인 | 역할 |
|---|---|
| `path_relay_node` | 인지에서 나온 robot-relative `/path`를 target frame으로 변환해 Nav2 controller server의 `FollowPath` 액션 goal로 전달한다. |
| `nav2_mppi_controller` | `nav2_core::Controller` 플러그인과 표준 MPPI critic 라이브러리. 단독 노드가 아니라 `controller_server`에서 로드된다. |
| `our_mppi_critics` | `SlopeCritic`, `SkidCritic`, `AccelCritic` 등 tracked/skid 차량용 추가 critic 플러그인이다. |
| `mppi_repro_harness/repro_harness` | MPPI 상태 재현/디버깅용 실행 파일이다. 운영 주행 노드라기보다 분석 도구에 가깝다. |

### `robot_bringup` 실행 노드

| 노드 | 역할 |
|---|---|
| `stability_monitor_node` | IMU pitch/roll, 카메라 roll, 모터 진단을 감시해 위험 시 `/cmd_vel_safety`로 정지 명령을 낸다. |
| `current_ramp_node` | `/cmd_vel_auto`를 받아 전류/가속 관점의 ramp를 적용한 `/cmd_vel`을 발행한다. |
| `slope_traverse_node` | 인지 경사 신호와 `/path`를 이용해 경사 진입/등반/탈출을 `/cmd_vel_safety`로 직접 제어하는 기본 Plan A 노드다. |
| `slope_traverse_blend_node` | TF 기반 pure pursuit 전환 대신 blend/P 제어를 유지하는 Plan B 경사 통과 노드다. |
| `slope_traverse_planc_node` | 이전 고정 gain 방식의 Plan C 경사 통과 노드다. |
| `slope_traverse_planc_spring_node` | Plan C에 봄 미션용 startup 직진 상태를 추가한 변형이다. |
| `slope_traverse_pland_node` | side 신호가 0이 되어도 `SLOPE_EXIT -> RECOVERY`를 반드시 거치도록 한 Plan D 변형이다. |
| `slope_traverse_spring_node` | Plan D에 봄 미션용 startup 직진 상태를 추가한 변형이다. |
| `slope_traverse_plane_node` | Plan D 기반이지만 탈출 구간 side motor 속도를 낮춘 Plan E 변형이다. |
| `summer_supply_drive_node` | 여름 보급 미션의 정지 상태머신과 `/path` pure pursuit 기반 `/cmd_vel_auto` 발행을 담당한다. |
| `dog_follow_node` | 선도 로봇 상대 위치를 받아 거리/방향 유지용 `/cmd_vel_auto`를 발행한다. |
| `autonomous.planz.py` | `dolbotz/purepursuit.py`의 특정 시점 코드를 `robot_bringup` 전용 독립 노드로 복원한 경로추종 노드다. |
| `autonomous.planz_spring.py` | Plan Z의 봄 미션 변형이다. |
| `autonomous.planz_winter.py` | Plan Z의 겨울 미션 변형이다. |

### 대표 launch

| launch | 묶는 기능 |
|---|---|
| `reduced_odom_bringup.launch.py` | RMD-X8, myAHRS, static TF, reduced odom bringup |
| `nav2.launch.py` | Nav2 controller server와 MPPI 설정 |
| `path_control*.launch.py` | 경사 통과 Plan A/B/C/D/E 변형을 path control 체인으로 실행 |
| `autonomous*.launch.py` | odom, Nav2, path control, safety/current ramp를 묶은 자율 주행 변형 |
| `mission_*_drive.launch.py` | 계절 미션별 주행 하드웨어/제어 체인 |
| `manual_drive_sensors.launch.py` | 수동 주행 중에도 센서/TF/camera를 띄우기 위한 보조 launch |

## 7. 로봇팔 패키지

### 저수준 하드웨어 계층

| 패키지/노드 또는 플러그인 | 역할 |
|---|---|
| `rmd_sdk` | MyActuator RMD-X CAN 프로토콜을 다루는 저수준 C++ SDK와 Python binding. |
| `rmd_hardware_interface/MyActuatorRmdHardwareInterface` | shoulder/elbow/wrist 등 RMD 관절을 ros2_control position interface로 노출한다. 송신 직전 NaN/Inf, 리밋, feedback timeout 등을 검사한다. |
| `dynamixel_sdk` | ROBOTIS Dynamixel 저수준 SDK. |
| `dynamixel_sdk_custom_interfaces` | ROBOTIS SDK 예제용 `SetPosition.msg`, `GetPosition.srv` 인터페이스다. 현재 주 arm 제어 흐름은 `dynamixel_interfaces`와 hardware interface 쪽을 더 직접 사용한다. |
| `dynamixel_interfaces` | `DynamixelState`, `GetDataFromDxl`, `SetDataToDxl`, `RebootDxl` 정의. |
| `dynamixel_hardware_interface` | base/gripper Dynamixel을 ros2_control에 연결하고 상태 topic/service/fault 처리를 제공한다. |
| `rmd_joint_state_bridge_node` | 실기 팔 RMD/Dynamixel을 읽기 전용으로 폴링해 `/joint_states`와 raw/calibrated 진단 토픽을 발행한다. ros2_control이 같은 장치를 잡고 있을 때는 같이 쓰면 안 되는 성격의 미러링 노드다. |

### Description / MoveIt

| 패키지 | 역할 |
|---|---|
| `army_manipulator_description` | 현재 주 사용 모델로 보이는 MARU 4DoF + gripper URDF/Xacro, mesh, `joint_calibration.yaml`, RViz display launch를 제공한다. |
| `army_manipulator_moveit_config` | MoveIt SRDF, kinematics, joint limits, controller 설정, OMPL/Pilz 설정, RViz 설정과 `move_group`, `demo`, `spawn_controllers` launch를 제공한다. |
| `robot_arm_description` | 3축 RMD 수동 제어용 구형/별도 모델. 현재 자동 파지/MoveIt 흐름의 중심은 `army_manipulator_description` 쪽이다. |

### `army_manipulator_bringup` 자동 팔 제어 노드/도구

| 실행 파일 | 역할 |
|---|---|
| `maru_ik_node.py` | D435 `supplybox` 3D target을 MoveIt IK/계획/ros2_control 실행으로 연결하는 주 자동 파지 노드다. |
| `ik_node.py` | 보급품 파지를 위한 더 작은/대체 IK 기반 controller다. |
| `move_to_named_pose.py` | `arm_controller`/`gripper_controller` FollowJointTrajectory action에 named pose나 gripper goal을 직접 보낸다. |
| `planned_encoder_trajectory.py` | MoveIt display trajectory를 하드웨어 raw encoder radian 기준 경로로 변환해 관측용 토픽에 발행한다. |
| `arm_pose_array_publisher.py` | 각 관절 TF 위치를 `PoseArray`로 발행해 센터/UI 시각화에 사용한다. |
| `detection_status_monitor.py` | `/arm/target_point` 수신 상태를 보고 `/arm/target_detected`를 발행/로그한다. |
| `fake_target_publisher.py` | 고정 3D target을 `/arm/target_point`로 발행하는 시뮬레이션/테스트 노드다. |
| `random_target_publisher.py` | 임의 target 좌표를 발행하고 자동 파지 흐름을 반복 시험한다. |
| `random_joint_publisher.py` | 임의 관절각으로 팔을 반복 이동시키는 테스트/데이터 수집 노드다. |
| `tcp_jog_marker.py` | RViz interactive marker를 이용해 `tcp_link`를 드래그하고 팔을 따라 움직이게 한다. |
| `tcp_trail_publisher.py` | `base_link -> tcp_link` TF를 누적해 TCP 궤적 `Path`를 발행한다. |
| `cycle_monitor.py` | 실제 encoder 값과 이론 reference를 주기적으로 비교/출력한다. |
| `gripper_current_logger.py` | gripper 전류를 `/arm/gripper_current_ma`와 CSV로 남기는 threshold 튜닝 도구다. |
| `gripper_safe_close.py` | gripper를 천천히 닫다가 전류 threshold 초과 시 취소하는 안전 닫기 노드다. |
| `joint_calibration.py` | raw encoder와 actual URDF/IK 각도 사이 변환 및 limit 조회 유틸이다. |
| `grasp_wait_presets.py` | 자동/수동 공통 `GRASP_WAIT` joint preset 정의 모듈이다. |
| `capture_arm_pose.py` | 수동으로 움직인 팔 자세의 encoder 값을 캡처한다. |
| `autonomous_grasp_demo.py` | 고정 grasp sequence를 ros2_control controller에 replay한다. |
| `trigger_captured_grasp.py` | 캡처된 관절값을 FK로 target point화해 파지 트리거로 사용한다. |
| `verify_grasp_pose_fk.py` | 여러 grasp pose를 FK로 일괄 검증하는 도구다. |
| `check_self_collision.py` | MoveIt `/check_state_validity`로 여러 자세의 자기충돌 여부를 확인한다. |
| `check_reachability.py` | 특정 target 좌표가 관절 리밋 안에서 도달 가능한지 대량 seed로 검사한다. |
| `sid_fallback_grasp.py` | 동적 IK 파지가 실패할 때 쓰는 단순 fallback 파지 노드다. |
| `numerical_grasp_planner.py` | ROS 없이 자동 파지 흐름을 수치 계산하는 단일 파일 도구다. |
| `analytic_grasp_planner.py` | 4-DOF 팔의 closed-form IK 계산 도구다. |

소스에는 `theoretical_state_relay.py`, `side_view_visualizer.py`도 존재하지만 현재 `army_manipulator_bringup/CMakeLists.txt`의 install 대상에는 들어 있지 않다. 필요하면 패키지 설치 규칙에 추가해야 `ros2 run army_manipulator_bringup ...` 형태로 바로 실행할 수 있다.

주요 launch는 `mock_bringup.launch.py`, `control_bringup.launch.py`, `realsense_bringup.launch.py`, `perception_bringup.launch.py`, `depth_camera_ik_bringup.launch.py`, `ik_node_bringup.launch.py`, `numerical_control_bringup.launch.py`, `fixed_grasp_demo.launch.py`, `ui_bringup.launch.py`다. `depth_camera_ik_bringup`은 제어, 팔 카메라, 보급품 perception을 한 번에 묶는 상위 launch 역할을 한다.

### `robot_arm_bringup` 수동 팔 제어 노드

| 실행 파일 | 역할 |
|---|---|
| `safety_manager.py` | 조이스틱 버튼으로 manual/auto/EE 모드와 protective stop, active target을 관리한다. |
| `gamepad_position_controller.py` | MANUAL_100 모드에서 joystick 입력을 관절 위치 명령으로 변환한다. |
| `ee_manual_controller.py` | joystick의 TCP 이동 명령을 differential IK로 관절 명령 `/manual_ee_joint_commands`로 변환한다. |
| `manual_gripper_controller.py` | 수동 모드에서 gripper 위치 명령을 발행한다. |
| `gripper_hold_fin.py` | gripper 위치/effort로 holding 완료 여부를 판단한다. |
| `joint_command_mux.py` | manual joint, manual EE, gripper, home/hold, safety 상태를 합쳐 `/position_controller/commands` 최종 명령을 낸다. |
| `manual_total_position_node.py` | 4개 arm-pose joint 수동 target을 생성하는 legacy/대체 노드다. |

`manual_total_control.launch.py`는 robot description, ros2_control, controller spawner, joy, safety/mux/manual 노드를 함께 실행한다. `remote_joy.launch.py`는 joy만 원격으로 띄울 때 쓰는 보조 launch다.

## 8. 인터페이스 패키지

### `mission_manager_interfaces`

`MissionResult.msg`만 정의한다.

| 필드 | 타입 | 의미 |
|---|---|---|
| `header` | `std_msgs/Header` | stamp/frame 정보 |
| `mission_name` | `string` | `spring_ifof`, `summer_traffic`, `fall_marker`, `escort_follow` 등 |
| `state` | `string` | 미션별 상태 문자열 |
| `target_point` | `geometry_msgs/Point` | 필요 시 target 위치, 불필요하면 0 |
| `valid` | `bool` | 해당 프레임/판정의 유효 여부 |

파일 주석상 임시 스키마이며 실제 미션 로직과 안 맞으면 교체될 수 있다고 표시되어 있다.

## 9. 외부/서브모듈 상태

| 경로 | 상태 |
|---|---|
| `src/arm/pymoveit2` | git submodule로 등록되어 있으며 URL은 `https://github.com/AndrejOrsula/pymoveit2.git`다. 현재 워크트리에서는 gitlink만 보일 수 있다. |
| `src/usb_cam` | gitlink는 존재하지만 현재 `.gitmodules`에 항목이 없다. clone/이식 시 별도 확인이 필요하다. |
| `src/arm/dynamixelSDK/c++` | ROS 패키지로 쓰는 경로는 `dynamixelSDK/ros/...`이며, 별도 C++ SDK 디렉터리는 `COLCON_IGNORE`가 있을 수 있다. |

## 10. HW/센서만 이식할 때의 최소 후보

이 문서는 구조 설명이 목적이지만, 이식 관점에서 보면 아래가 하드웨어/센서 계층의 핵심 후보다.

| 목적 | 우선 이식 후보 |
|---|---|
| 구동 모터 직접 제어 | `src/drive/autonomous/rmd_x8_driver`, 또는 수동 CAN만 필요하면 `src/drive/manual/can_driver` |
| 조이스틱 수동 구동 | `src/drive/manual/manual_joy_control` + `src/drive/manual/can_driver` |
| 차체 IMU | `src/drive/autonomous/myahrs_driver` |
| 휠+IMU 오도메트리 | `src/drive/autonomous/reduced_odom` 또는 대안 `direct_odom` |
| 팔 RMD/Dynamixel 제어 | `rmd_sdk`, `rmd_hardware_interface`, `dynamixelSDK/ros/dynamixel_sdk`, `dynamixel_interfaces`, `dynamixel_hardware_interface` |
| 팔 모델/컨트롤러 설정 | `army_manipulator_description`, 필요한 경우 `army_manipulator_moveit_config`, `army_manipulator_bringup/config/ros2_controllers.yaml` |
| 팔 상태 읽기 전용 캡처 | `rmd_joint_state_bridge` |
| RealSense 설정 | `dolbotz/config/realsense_cameras.yaml`, `dolbotz/utils/realsense_camera_launch.py`, `dolbotz/launch/drive_cam.launch.py`, `army_manipulator_bringup/launch/realsense_bringup.launch.py` |
| side USB camera/RTSP | `src/usb_cam` 상태 확인 + `dolbotz/launch/side_cameras.launch.py` + `util/ros_compressed_to_rtsp.py` |

주의할 점은 카메라/TF frame 이름, CAN interface 이름(`can_drive`, `can_arm`, `can0` 등), serial device(`/dev/ttyACM*`, `/dev/ttyUSB*`), 모델 파일 경로가 launch와 코드에 분산되어 있다는 점이다. 하드웨어 계층만 옮겨도 URDF/Xacro의 ros2_control hardware parameter와 launch의 static TF는 같이 따라가야 한다.
