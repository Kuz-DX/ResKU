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

[manual+return 통합 이후] 토픽명만 `/motor_speed_cmd` → `/motor_speed_cmd_manual`로
바뀌었고 타입(dps)은 그대로다 — 2026 사용자 결정으로 dps 유지, Twist 변환은
`drive_cmd_mux_node`로 옮김(디버깅 시 dps 숫자가 더 직관적). 모터 마운팅
방향 보정(`left_motor_sign`/`right_motor_sign`)은 이 노드에서 제거됐고
`rmd_x8_driver`에서 한 번만 적용된다 — 이 노드가 만드는 값은 부호 없는
물리 좌표 기준(양수 = 그 바퀴가 전진) dps다.

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/motor_speed_cmd_manual` | `std_msgs/msg/Float32MultiArray` | 좌/우 구동 모터 속도 명령(dps, 부호 없음) |
| `/max_speed_dps` | `std_msgs/msg/Float32MultiArray` | 조이스틱으로 조절한 좌/우 최대 속도 표시 |
| `/mission/return/trigger` | `std_msgs/msg/Bool` | RETURN 버튼 rising edge — return_state_machine_node가 구독 |

## can_driver (구 수동 주행 CAN 드라이버 — deprecated, 미사용)

[manual+return 통합 이후] `can_driver_node`는 CAN feedback을 안 읽어서
manual 주행 중 wheel odometry가 구조적으로 불가능했던 문제 때문에 더 이상
launch하지 않는다. CAN 소유는 이제 `rmd_x8_driver` 하나뿐이다.

[2026 사용자 결정, 경량화] `manual_stability_node`(IMU pitch/roll 긴급정지)도
`manual_return_bringup.launch.py`에서 제외됐다 — 실기 검증 결과 전복 위험
자세가 나타나지 않는 운용 환경으로 판단, 안전 여유보다 구성 단순화를
우선한 명시적 선택. 소스는 남아있고 `/cmd_vel_safety`(Twist)로 발행하도록
재배선까지 끝나있으므로, 필요해지면 launch에 다시 추가하면 된다. 이
패키지 전체가 현재 실제로는 어떤 launch에서도 안 쓰인다(구
`can_driver/launch/manual.launch.py`만 참고용으로 남아있음).

> **주의**: 혹시 구 `can_driver/launch/manual.launch.py`를 그대로 실행하면
> `can_driver_node`가 여전히 옛 `/motor_speed_cmd_safety`(Float32MultiArray)를
> 기다리는데 `manual_stability_node`는 이제 다른 타입/토픽으로 발행하므로
> **IMU 긴급정지가 조용히 동작하지 않는다.** 이 launch는 쓰지 말 것.

## myahrs_driver (차체 IMU 드라이버)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | myAHRS+ 자세, 각속도 및 선형가속도 |

수동 주행과 자율 주행 launch가 같은 토픽명을 사용한다. 두 launch를 동시에 실행해
드라이버를 중복 기동하지 않는다.

## rmd_x8_driver (manual+return 공용 구동 모터 CAN 드라이버)

[manual+return 통합 이후] 이 노드가 CAN(`can_drive`)/motor ID를 소유하는
**유일한** 저수준 드라이버다. manual 조종(`drive_cmd_mux_node`를 거쳐
`/cmd_vel_manual` → `/cmd_vel`)과 return 자동복귀(`/cmd_vel_return` →
`/cmd_vel`) 모두 이 노드를 공용으로 쓰므로, manual 주행 중에도
`/wheel/odom`이 정상 발행된다. CAN 재연결(consecutive-failure 감지 +
쿨다운 재오픈) 로직도 구 `can_driver_node`에서 이관해 추가됨.

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/wheel/odom` | `nav_msgs/msg/Odometry` | 좌/우 모터 피드백 기반 휠 오도메트리 (manual 주행 중에도 발행) |
| `/wheel/joint_states` | `sensor_msgs/msg/JointState` | 휠 속도와 전류 상태 |
| `/wheel/motor_status` | `diagnostic_msgs/msg/DiagnosticArray` | 모터 통신 및 오류 진단 |
| `/tf` | `tf2_msgs/msg/TFMessage` | **조건부** `publish_odom_tf:=true`일 때 odom TF 발행. 운영 기본값은 `false` |

## reduced_odom (휠 오도메트리와 IMU 융합)

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 5상태 축소 오도메트리 추정 결과 |
| `/odometry/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 센서 timeout과 추정기 상태 진단 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom` -> `base_link` 동적 TF |

## drive_cmd_mux (manual/return 명령 중재 + dps→Twist 변환)

[신규, manual+return 통합] `/motor_speed_cmd_manual`(dps)을 Twist로 변환한
뒤 `/mission/return/state`에 따라 그 값과 `/cmd_vel_return` 중 하나만 골라
`/cmd_vel`로 통과시킨다(`STOP_BEFORE_TURN` 상태에서는 둘 다 막고 0).
`/cmd_vel_safety`는 이 mux를 거치지 않고 `rmd_x8_driver`에 직결되어 항상
최우선. dps→Twist 변환에 쓰는 `wheel_radius_m`/`effective_track_width_m`
파라미터는 `rmd_x8_driver`의 실제 운영값과 반드시 같이 맞출 것.

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 최종 주행 명령 (`rmd_x8_driver` 구독) |

## return_navigation (manual 경로 기록 + 자동 복귀)

[신규, manual+return 통합] 3개 노드로 구성. `manual_path_recorder_node`가
`/odometry/filtered`를 mission 기준 좌표(기록 시작 시점 pose = (0,0,0))로
변환해 경로를 쌓고, `return_state_machine_node`가 RETURN 트리거 이후
정지 확인 → 폐루프 180도 회전 → 복귀 경로 발행 → 완료 판정까지 상태
머신을 총괄하며 `/cmd_vel_return`을 단독 발행하고,
`return_path_follower_node`가 고정된 `/return_path`와 실시간 오도메트리를
비교하는 pure pursuit로 조향값을 계산한다.

| 노드 | 발행 토픽 | 타입 | 용도 |
|---|---|---|---|
| `manual_path_recorder_node` | `/mission/origin_pose` | `geometry_msgs/msg/Pose` | mission 좌표계 원점 T0 (recording 시작 시 1회, TRANSIENT_LOCAL) |
| `manual_path_recorder_node` | `/recorded_path_raw` | `nav_msgs/msg/Path` | 임계값(거리/회전각/시간) 게이팅된 원본 기록 경로, mission frame |
| `manual_path_recorder_node` | `/recorded_path` | `nav_msgs/msg/Path` | 중복점 제거 등 가공된 기록 경로, mission frame |
| `return_state_machine_node` | `/mission/return/state` | `std_msgs/msg/String` | `IDLE`/`MANUAL_RECORDING`/`WAIT_RETURN_COMMAND`/`STOP_BEFORE_TURN`/`TURN_180`/`FOLLOW_RETURN_PATH`/`FINISHED` |
| `return_state_machine_node` | `/cmd_vel_return` | `geometry_msgs/msg/Twist` | 복귀 주행 명령 (정지 대기/180도 회전/경로 추종 중계, 단독 발행자) |
| `return_state_machine_node` | `/manual_path_recorder/command` | `std_msgs/msg/String` | 레코더 제어: `START`/`STOP`/`CLEAR` |
| `return_state_machine_node` | `/return_path` | `nav_msgs/msg/Path` | `/recorded_path`를 역순+재계산 yaw로 뒤집은 고정 복귀 경로, TURN_180 완료 시 1회(TRANSIENT_LOCAL) |
| `return_path_follower_node` | `/cmd_vel_return_path` | `geometry_msgs/msg/Twist` | `/return_path` 기준 pure pursuit 조향 명령 |

## path_relay (인지 경로를 Nav2 FollowPath 액션으로 중계)

운영 기본 설정에서는 토픽을 직접 발행하지 않고 `/path`를 FollowPath 액션 goal로
전달한다.

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/debug/path_raw` | `nav_msgs/msg/Path` | **조건부** `debug_path_pipeline:=true`일 때 수신 원본 경로 |
| `/debug/path_relay_transformed` | `nav_msgs/msg/Path` | **조건부** `debug_path_pipeline:=true`일 때 `odom` 변환 경로 |

## robot_bringup (2026 사용자 결정, 계절 미션 정리 이후)

계절 미션 전용 노드(`slope_traverse_*`, `dog_follow_node`,
`summer_supply_drive_node`)와 전류 램프/자세·통신 긴급정지 노드
(`current_ramp_node`, `stability_monitor_node`)는 소스 자체가 삭제됐다 —
manual+return 미션에서 쓰지 않고, 전복 위험이 없는 운용 환경으로 판단해
안전 여유보다 구성 단순화를 우선한 명시적 선택. 이 패키지는 이제 C++
실행 파일이 없는 순수 launch/config 패키지다. `manual_return_bringup.launch.py`
(manual+return 미션의 실제 진입점)에는 원래도 포함된 적이 없다.

## nav2_controller / nav2_mppi_controller (MPPI 경로 추종 — 2단계 평가용 보존)

[2026 사용자 결정] MPPI 자체는 나중에 recorded return path와 비교하는
2단계 평가용으로 소스를 보존한다(`autonomous.launch.py`로 계속 실행
가능). `current_ramp_node` 삭제로 더 이상 `/cmd_vel_auto`로 우회하지 않고
`controller_server`가 곧장 `/cmd_vel`을 발행한다 — manual+return 미션의
`drive_cmd_mux_node`/`rmd_x8_driver_node`와 이 launch를 절대 동시에
띄우지 말 것(같은 `/cmd_vel`·CAN 버스 충돌).

| 발행 토픽 | 타입 | 용도 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | MPPI 속도 명령 (더 이상 `/cmd_vel_auto`로 우회하지 않음) |
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
