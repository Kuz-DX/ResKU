# 발행 토픽 정리

루트의 `howtorun.md`와 현재 소스/launch 파일을 기준으로 정리했다. 토픽명은 별도
파라미터나 remap을 주지 않았을 때의 기본값이다.

- 프로젝트 기능과 관계없는 ROS 2 공통 토픽(`/rosout`, `/parameter_events`), 서비스,
  액션 내부 토픽은 제외했다.
- **조건부**는 해당 옵션을 켜거나 해당 노드를 별도로 실행할 때만 발행된다.
- 카메라의 압축 영상 토픽은 해당 `image_transport` 플러그인이 설치되어 있어야 한다.
- `src/dolbotz/howtorun.md`의 `/camera/camera/...` 표기는 오래된 값이므로, 주행/미션
  카메라는 루트 문서의 `/drive/camera/...`, `/arm/camera/...` 표기를 우선한다.

## dolbotz (계절 미션 및 유틸리티)

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `spring_ifof` | `/mission/spring_ifof/result` | `mission_manager_interfaces/msg/MissionResult` | 봄 피아식별 결과 |
| `summer_traffic` | `/mission/summer_traffic/result` | `mission_manager_interfaces/msg/MissionResult` | 여름 신호등 인식 결과 |
| `fall_marker` | `/mission/fall_marker/result` | `mission_manager_interfaces/msg/MissionResult` | 가을 비전 마커 인식 결과 |
| `escort_follow` | `/mission/escort_follow/result` | `mission_manager_interfaces/msg/MissionResult` | 선도 로봇 추종 결과 |
| `escort_follow` | `/mission/escort_follow/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` | 추종 인식 디버그 영상 |
| `summer_supply` | `/arm/target_point` | `geometry_msgs/msg/PointStamped` | 보급품의 3차원 위치 |
| `summer_supply` | `/arm/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` | 보급품 검출 디버그 영상 |
| `led_relay` | `/led_control` | `std_msgs/msg/String` | 피아식별 결과를 변환한 LED 명령 |
| `purepursuit` | `/motor_speed_cmd` | `std_msgs/msg/Float32MultiArray` | Pure Pursuit가 계산한 좌/우 모터 속도 명령 |

`arm_visualizer`와 `led_bridge_node`는 각각 화면 표시와 시리얼 전송만 담당하며 ROS
토픽을 새로 발행하지 않는다.

## mission_manager_interfaces (미션 결과 메시지 정의)

노드를 실행하는 패키지가 아니라 `MissionResult` 메시지 타입만 제공하므로 직접
발행하는 토픽은 없다. `dolbotz`의 계절 미션 노드들이 이 타입을 사용한다.

## manual_joy_control (수동 주행 조이스틱 명령 생성)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/motor_speed_cmd` | `std_msgs/msg/Float32MultiArray` | 좌/우 구동 모터 속도 명령 |
| `/max_speed_dps` | `std_msgs/msg/Float32MultiArray` | 조이스틱으로 조절한 좌/우 최대 속도 표시 |

## can_driver (수동 주행 CAN 출력 및 자세 안전 정지)

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `manual_stability_node` | `/motor_speed_cmd_safety` | `std_msgs/msg/Float32MultiArray` | IMU 위험 자세 감지 시 좌/우 정지 명령 |

`can_driver_node`는 `/motor_speed_cmd`와 `/motor_speed_cmd_safety`를 받아 CAN 프레임을
전송하며 ROS 토픽을 새로 발행하지 않는다.

## myahrs_driver (차체 IMU 드라이버)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | myAHRS+ 자세, 각속도 및 선형가속도 |

수동 주행과 자율 주행 launch가 같은 토픽명을 사용한다. 두 launch를 동시에 실행해
드라이버를 중복 기동하지 않는다.

## rmd_x8_driver (자율 주행 구동 모터 CAN 드라이버)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/wheel/odom` | `nav_msgs/msg/Odometry` | 좌/우 모터 피드백 기반 휠 오도메트리 |
| `/wheel/joint_states` | `sensor_msgs/msg/JointState` | 휠 속도와 전류 상태 |
| `/wheel/motor_status` | `diagnostic_msgs/msg/DiagnosticArray` | 모터 통신 및 오류 진단 |
| `/tf` | `tf2_msgs/msg/TFMessage` | **조건부** `publish_odom_tf:=true`일 때 odom TF 발행. 운영 기본값은 `false` |

## reduced_odom (휠 오도메트리와 IMU 융합)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 5상태 축소 오도메트리 추정 결과 |
| `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 센서 timeout과 추정기 상태 진단 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom` -> `base_link` 동적 TF |

## path_relay (인지 경로를 Nav2 FollowPath 액션으로 중계)

운영 기본 설정에서는 토픽을 직접 발행하지 않고 `/path`를 FollowPath 액션 goal로
전달한다.

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/debug/path_raw` | `nav_msgs/msg/Path` | **조건부** `debug_path_pipeline:=true`일 때 수신 원본 경로 |
| `/debug/path_relay_transformed` | `nav_msgs/msg/Path` | **조건부** `debug_path_pipeline:=true`일 때 `odom` 변환 경로 |

## robot_bringup (자율 주행 명령 연결 및 안전 상태 머신)

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `current_ramp_node` | `/cmd_vel` | `geometry_msgs/msg/Twist` | 전류 제한 램프를 적용한 최종 일반 주행 명령 |
| `stability_monitor_node` | `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 모터 통신/하드웨어 오류 시 안전 정지 명령 |
| `slope_traverse_node` | `/cmd_vel_safety` | `geometry_msgs/msg/Twist` | 경사 진입, 등반, 탈출 중 최우선 주행 명령 |
| `slope_traverse_node` | `/drive/slope_traverse_state` | `std_msgs/msg/String` | 경사 통과 상태 머신 상태 |
| `slope_traverse_node` | `/slope_traverse/debug` | `std_msgs/msg/Float64MultiArray` | 경사 통과 제어 디버그 값 |
| `summer_supply_drive_node` | `/cmd_vel_auto` | `geometry_msgs/msg/Twist` | 여름 보급 미션용 직진/정지 명령 |

`stability_monitor_node`와 `slope_traverse_node`가 같은 `/cmd_vel_safety`를 발행할 수
있으며, `rmd_x8_driver`가 이 안전 토픽을 일반 `/cmd_vel`보다 우선 처리한다.

## nav2_controller / nav2_mppi_controller (MPPI 경로 추종)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/cmd_vel_auto` | `geometry_msgs/msg/Twist` | MPPI 속도 명령. launch에서 기본 `/cmd_vel`을 remap한 결과 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | controller_server 내부 local costmap |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | **조건부** MPPI 후보 궤적. 현재 `visualize:false`이므로 기본 미발행 |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | **조건부** MPPI가 변환한 입력 경로. 시각화가 켜지고 구독자가 있을 때 발행 |
| `/debug/mppi_setplan_path` | `nav_msgs/msg/Path` | **조건부** `debug_path_pipeline:=true`일 때 MPPI에 전달된 경로 |

현재 시스템에는 `/global_costmap/*` 발행 노드가 없다. `/speed_limit`도 기존 발행
패키지가 삭제되어 현재 발행자가 없다.

## rmd_joint_state_bridge (실기 팔 엔코더의 읽기 전용 RViz 미러링)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | RMD와 Dynamixel을 합친 팔 관절 상태 |
| `/arm/joint_angle_deg` | `std_msgs/msg/Float64MultiArray` | 보정된 RMD 각도(shoulder, elbow, wrist) |
| `/arm/rmd_raw_angle_deg` | `std_msgs/msg/Float64MultiArray` | 보정 전 RMD 원시 각도 |
| `/arm/rmd_encoder` | `std_msgs/msg/Float64MultiArray` | RMD 원시 엔코더 카운트 |
| `/arm/dxl_angle_deg` | `std_msgs/msg/Float64MultiArray` | 보정된 Dynamixel 각도(base, gripper) |
| `/arm/dxl_encoder` | `std_msgs/msg/Float64MultiArray` | Dynamixel 원시 pulse |

## robot_arm_bringup (조이스틱 기반 수동 팔 제어와 안전 관리)

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `safety_manager` | `/control/mode` | `std_msgs/msg/String` | 팔 제어 모드 |
| `safety_manager` | `/control/enabled` | `std_msgs/msg/Bool` | 전체 팔 제어 활성 상태 |
| `safety_manager` | `/control/manual_100_enabled` | `std_msgs/msg/Bool` | 관절 직접 조작 모드 활성 상태 |
| `safety_manager` | `/control/manual_ee_enabled` | `std_msgs/msg/Bool` | 말단 직접 조작 모드 활성 상태 |
| `safety_manager` | `/control/auto_enabled` | `std_msgs/msg/Bool` | 자동 제어 활성 상태 |
| `safety_manager` | `/control/active_target` | `std_msgs/msg/String` | 조이스틱 제어 대상(`drive` 또는 `arm`) |
| `safety_manager` | `/control/protective_stop` | `std_msgs/msg/Bool` | 보호 정지 상태 |
| `safety_manager` | `/emergency_stop` | `std_msgs/msg/Bool` | 비상 정지 상태 |
| `gamepad_position_controller` | `/manual_joint_commands` | `sensor_msgs/msg/JointState` | MANUAL_100 관절 위치 명령 |
| `ee_manual_controller` | `/manual_ee_joint_commands` | `sensor_msgs/msg/JointState` | MANUAL_EE 역기구학 관절 명령 |
| `manual_gripper_controller` | `/manual_gripper_command` | `std_msgs/msg/Float64` | 수동 그리퍼 위치 명령 |
| `joint_command_mux` | `/position_controller/commands` | `std_msgs/msg/Float64MultiArray` | 안전 제한과 모드 선택을 적용한 최종 관절 위치 명령 |

위 제어 상태 토픽은 대부분 transient-local QoS를 사용해 늦게 시작한 구독자도 최신
상태를 받을 수 있다.

## army_manipulator_bringup (카메라 검출, MoveIt IK 및 자동 팔 동작)

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `target_detector_node` 또는 `fake_target_publisher` | `/arm/target_point` | `geometry_msgs/msg/PointStamped` | 검출된 물체 또는 시뮬레이션 목표의 3차원 좌표 |
| `target_detector_node` | `/arm/debug_image/compressed` | `sensor_msgs/msg/CompressedImage` | 물체 검출 디버그 영상 |
| `maru_ik_node` | `/arm/grasp_success` | `std_msgs/msg/Bool` | 집기 성공 여부 |
| `planned_encoder_trajectory` | `/arm/planned_encoder_trajectory` | `trajectory_msgs/msg/JointTrajectory` | MoveIt 계획을 엔코더 기준 radian으로 나타낸 관측용 궤적 |
| `theoretical_state_relay` | `/theoretical_joint_states` | `sensor_msgs/msg/JointState` | **조건부** 별도 스크립트 실행 시 controller reference 기반 이론 관절 상태 |

## controller_manager / ros2_control controllers (팔 하드웨어와 컨트롤러 상태)

| 발행 주체 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `joint_state_broadcaster` | `/joint_states` | `sensor_msgs/msg/JointState` | 실제 또는 mock 팔 관절 상태 |
| `joint_state_broadcaster` | `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | 인터페이스별 상세 관절 상태 |
| `arm_controller` | `/arm_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 팔 궤적 controller의 reference/feedback/error |
| `gripper_controller` | `/gripper_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 그리퍼 controller의 reference/feedback/error |

## move_group (MoveIt 팔 경로 계획)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | 계획된 팔 궤적의 RViz 표시 및 `planned_encoder_trajectory` 입력 |

## rmd_hardware_interface (팔 RMD 모터 하드웨어 인터페이스)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/control/hardware_fault` | `std_msgs/msg/String` | RMD 통신, 과전류 또는 하드웨어 오류 상태 |

## dynamixel_hardware_interface (팔 Dynamixel 하드웨어 인터페이스)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/control/hardware_fault` | `std_msgs/msg/String` | Dynamixel 통신 또는 하드웨어 오류 상태 |
| `/dynamixel_hardware_interface/dxl_state` | `dynamixel_interfaces/msg/DynamixelState` | Dynamixel 상세 상태. xacro의 설정값으로 이름이 결정됨 |

## robot_state_publisher / tf2_ros (로봇 좌표계)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | 관절 및 동적 좌표 변환 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 로봇 고정 링크, IMU 및 카메라 장착 위치 변환 |

## joy (조이스틱 입력 드라이버)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | 조이스틱 축과 버튼 입력 |

## realsense2_camera (주행용/팔용 D455 카메라)

아래에서 `{ns}`는 주행 카메라는 `drive`, 팔 카메라는 `arm`이다. 팔 자동 IK
bringup을 namespace override 없이 실행하는 경우에는 해당 launch의 기본값인
`/camera/camera/...`가 사용될 수 있다.

| 발행 토픽 패턴 | 타입 | 용도 |
|---|---|---|
| `/{ns}/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 컬러 카메라 내부 파라미터 |
| `/{ns}/camera/color/image_raw` | `sensor_msgs/msg/Image` | 컬러 원본 영상 |
| `/{ns}/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 압축 컬러 영상 |
| `/{ns}/camera/color/image_raw/theora` | `theora_image_transport/msg/Packet` | **조건부** Theora 전송 영상 |
| `/{ns}/camera/color/metadata` | `realsense2_camera_msgs/msg/Metadata` | 컬러 프레임 메타데이터 |
| `/{ns}/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | depth 카메라 내부 파라미터 |
| `/{ns}/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | 보정된 원본 depth 영상 |
| `/{ns}/camera/depth/image_rect_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 압축 depth 영상 |
| `/{ns}/camera/depth/image_rect_raw/compressed` | `sensor_msgs/msg/CompressedImage` | **조건부** 일반 compressed transport |
| `/{ns}/camera/depth/image_rect_raw/theora` | `theora_image_transport/msg/Packet` | **조건부** Theora 전송 depth 영상 |
| `/{ns}/camera/depth/metadata` | `realsense2_camera_msgs/msg/Metadata` | depth 프레임 메타데이터 |
| `/{ns}/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | 컬러 좌표계에 정렬된 depth 카메라 정보 |
| `/{ns}/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 컬러 좌표계에 정렬된 depth 영상 |
| `/{ns}/camera/aligned_depth_to_color/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 정렬된 압축 depth 영상 |
| `/{ns}/camera/imu` | `sensor_msgs/msg/Imu` | **주행 카메라만** gyro/accel 통합 IMU |
| `/{ns}/camera/extrinsics/depth_to_color` | `realsense2_camera_msgs/msg/Extrinsics` | depth에서 color로의 외부 파라미터 |
| `/{ns}/camera/extrinsics/depth_to_depth` | `realsense2_camera_msgs/msg/Extrinsics` | depth 스트림 외부 파라미터 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | RealSense 프레임 좌표 변환 |

## usb_cam (좌우 사이드 카메라)

아래에서 `{side}`는 `leftview` 또는 `rightview`이다.

| 발행 토픽 패턴 | 타입 | 용도 |
|---|---|---|
| `/{side}/image_raw` | `sensor_msgs/msg/Image` | 사이드 카메라 원본 영상 |
| `/{side}/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 압축 사이드 카메라 영상 |
| `/{side}/camera_info` | `sensor_msgs/msg/CameraInfo` | 사이드 카메라 내부 파라미터 |
