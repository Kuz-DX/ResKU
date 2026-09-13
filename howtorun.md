

## 3. 노드 실행

### segmentation (RGB → 세그멘테이션 마스크 발행 — flat_drive/elevation_map/slope_decision 공용)

세 노드가 각자 YOLO를 돌리지 않도록 세그멘테이션 추론을 이 노드 하나에만 둔다.
이 노드가 죽으면 세 소비자가 다 같이 영향받으므로 항상 먼저(또는 함께) 띄워야 한다.

```bash
ros2 run dolbotz segmentation --ros-args \
  -p conf_threshold:=0.5
```


구독: `/drive/camera/color/image_raw/compressed`, `.../camera_info`
발행: `/perception/drivable_mask` (`sensor_msgs/Image` mono8, undistort된 원근
이미지 좌표계 — BEV 아님)

### flat_drive (평지 주행가능영역 추종 — 마스크 → BEV투영 → centerline 경로 발행)

`gradient_map`(경사 구간)과 역할이 대칭이다: 이 노드도 실제 속도/조향 명령이
아니라 좌표(경로)만 발행하고 끝난다. 평지/경사 전환 판단은 `slope_decision`
노드가 담당한다 (아래 참고, Phase F 1차 버전).

```bash
ros2 run dolbotz flat_drive --ros-args \
  -p bev_meters_per_pixel:=0.03 \
  -p bev_img_width:=200 \
  -p bev_img_height:=200 \
  -p min_row_pixels:=5
```

카메라 마운트 파라미터(`camera_height_m` 등)는 기본값이 있으므로 위 예시에는
생략했다 — 아래 참고. 세그멘테이션은 이 노드가 직접 하지 않고
`/perception/drivable_mask`를 구독만 하므로, `segmentation` 노드를 먼저(또는
함께) 띄워야 한다.

구독: `/drive/camera/color/image_raw/compressed`, `.../camera_info`,
`/drive/camera/imu`, `/perception/drivable_mask` (`segmentation` 노드 발행)
발행: `/flatdrive/planned_path`(`nav_msgs/Path`, x=전방/y=좌측 m — 실제 출력),
`/planning/target_point`, `/bev/image`, `/bev/mask`, `/bev/debug_overlay`,
`/bev/centerline_overlay`, `/bev/H` (디버그용 중간 결과)



>
> **주의 — 카메라 마운트 파라미터**: `camera_height_m`, `camera_pitch_offset_deg`,
> `camera_roll_offset_deg`, `complementary_filter_alpha`의 기본값은
> `src/dolbotz/utils/attitude.py`의 `MOUNT_*_PLACEHOLDER` /
> `COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER` 상수다 (실측 전 임시값 — 대회장에서
> 자주 바뀔 수 있어 의도적으로 config/*.yaml이 아니라 코드 상수로 둔다).
> `elevation_map`과 파라미터 이름이 같다(동일 카메라). 값을 바꾸는 세 가지 방법:
>   (a) 한 번만 다르게 실행 — `--ros-args -p camera_height_m:=X`로 즉석 오버라이드
>   (b) 이후 계속 이 값을 쓰기 — `attitude.py`의 상수 자체를 수정
>   (c) 카메라별로 실측값을 자동 적용 — `config/calibration/`에 해당 카메라
>       `camera_serial_no`의 캘리브레이션 피클을 넣어두고 `-p
>       camera_serial_no:=117222251401`처럼 지정하면, 그 피클에 있는 값이
>       `attitude.py` 상수보다 우선 적용된다 (피클 스키마는
>       `config/calibration/README.md` 참고 — 아직 피클을 만드는 스크립트는
>       없고 스키마만 정해져 있다).

### slope_decision (좌우 기울기(roll) 추정 + 평지/경사 경로 릴레이)

뎁스 카메라로 좌우 기울기를 추정하는 기존 역할에 더해, `gradient_map`의
경사 경로와 `flat_drive`의 평지 경로 중 하나를 임계각 기준으로 골라
`/path`로 릴레이하는 역할을 겸한다 (Phase F 평지/경사 전환 판단의 1차
버전 — TF 기반 odom 변환은 아직 없음, 릴레이 노드 쪽 후속 작업).

```bash
ros2 run dolbotz slope_decision --ros-args \
  -p track_width_m:=0.45 \
  -p slope_threshold_deg:=10.0
```

구독: `/drive/camera/aligned_depth_to_color/camera_info`,
`/drive/camera/aligned_depth_to_color/image_raw/compressedDepth`,
`/perception/drivable_mask` (`segmentation` 노드 — 좌우 ROI를 트랙 폭 중심으로
나누는 데 씀),
`/terrain/planned_path` (`nav_msgs/Path` — `gradient_map`, 경사 경로),
`/flatdrive/planned_path` (`nav_msgs/Path` — `flat_drive`, 평지 경로)
발행: `/terrain/side_slope_angle_deg` (`std_msgs/Float32`),
`/path` (`nav_msgs/Path` — 임계각 기준으로 고른 최종 경로, 제어부가 구독할 토픽),
`/drive/status` (`std_msgs/String`, `'slope'` 또는 `'flat'`)
OpenCV 창(`SlopeVisualizer`)으로 깊이 ROI/기울기 값을 표시하므로 헤드리스 환경에서는 X 디스플레이 필요.

> **주의 — slope_threshold_deg**: 실측 전 임시값(10°)입니다
> (`src/dolbotz/slope_decision.py`의 `SLOPE_THRESHOLD_DEG_PLACEHOLDER` 참고).
> 실제 하드웨어 기울기 테스트 후 조정이 필요합니다. `gradient_map`/`flat_drive`를
> 먼저(또는 함께) 띄워야 `/path`가 정상 발행됩니다 — 선택된 쪽 경로가 아직
> 한 번도 안 들어왔으면 경고 로그만 찍고 `/path`는 발행하지 않습니다.
>
> **주의 — 마스크 의존**: `segmentation` 노드가 안 떠 있거나 ROI 안에 트랙
> 마스크 픽셀이 하나도 없으면 그 프레임의 roll 계산을 스킵합니다 (안전
> 우선 — 마스크 없이 순수 depth로 도는 폴백 없음).

### elevation_map (depth+IMU → 고도맵 게시, gradient_map의 입력을 만듦)

```bash
ros2 run dolbotz elevation_map --ros-args \
  -p depth_topic:=/drive/camera/aligned_depth_to_color/image_raw/compressedDepth \
  -p camera_info_topic:=/drive/camera/aligned_depth_to_color/camera_info \
  -p imu_topic:=/drive/camera/imu \
  -p resolution_m:=0.15 \
  -p min_depth_m:=0.5 \
  -p max_depth_m:=4.0 \
  -p blind_fill_forward_m:=0.6
```

구독: `/drive/camera/aligned_depth_to_color/image_raw/compressedDepth`,
`/drive/camera/aligned_depth_to_color/camera_info`, `/drive/camera/imu`,
`/perception/drivable_mask` (`segmentation` 노드 — 트랙 아닌 depth 포인트를
그리드 집계에서 제외하는 데 씀)
발행: `/terrain/elevation_map` (32FC1, m 단위; NaN=미관측)

> **주의 — min_depth_m/blind_fill_forward_m**: 실측 전 임시값입니다
> (`src/dolbotz/elevation_map.py`의 PLACEHOLDER 주석 참고). 실제 하드웨어
> (D455 최소 인식거리 등)에 맞춰 조정이 필요합니다.
>
> **주의 — 마스크 의존**: `segmentation` 노드를 먼저(또는 함께) 띄워야 합니다.
> 마스크가 한 번도 안 들어왔으면 안전 우선으로 프레임을 스킵합니다 (폴백 없음).
>
> **주의 — 카메라 마운트 파라미터**: `camera_height_m`, `camera_pitch_offset_deg`,
> `camera_roll_offset_deg`, `complementary_filter_alpha`의 기본값/오버라이드
> 방법은 `flat_drive` 섹션의 안내와 동일합니다 (같은 상수, 같은 카메라 —
> `src/dolbotz/utils/attitude.py` 참고).

### gradient_map (고도맵 → 경사 필드 → 슬로프 제한 경로 계획)

```bash
ros2 run dolbotz gradient_map --ros-args \
  -p resolution_m:=0.15 \
  -p max_slope_deg:=30.0
```

구독: `/terrain/elevation_map` (32FC1, m 단위)
발행: `/terrain/gradient_x`, `/terrain/gradient_y`, `/terrain/gradient_magnitude`,
`/terrain/gradient_direction`, `/terrain/slope_deg`, `/terrain/planned_path` (`nav_msgs/Path`)

> **주의**: `max_slope_deg`는 실측 전 임시값(30°)입니다
> (`src/dolbotz/gradient_map.py`의 `MAX_SLOPE_DEG_PLACEHOLDER` 참고).
> `/terrain/elevation_map`을 게시하는 `elevation_map` 노드를 먼저(또는 함께) 띄워야 합니다.

### 전체 노드 한 번에 띄우기 (rqt_graph 확인용)

터미널을 나눠서 아래 순서대로 띄운다 (카메라 → 입력 만드는 노드 →
그걸 구독하는 노드 순서). 각 터미널에서 먼저 `source /opt/ros/humble/setup.bash`
(그리고 워크스페이스라면 `source install/setup.bash`)부터 해야 한다.

```bash
# 터미널 1 — 주행 D455 카메라 (color/depth/imu 전부 필요)
# -r __ns:=/drive로 네임스페이스를 줘야 아래 터미널들이 기대하는
# /drive/camera/... 토픽으로 나온다 (안 주면 기본값 /camera/camera/...로 나감).
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/drive \
  -p serial_no:="'117222251401'" \
  -p enable_color:=true -p enable_depth:=true \
  -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
  -p enable_infra1:=false -p enable_infra2:=false \
  -p align_depth.enable:=true

# 터미널 2 — segmentation (마스크 발행 — flat_drive/elevation_map/slope_decision이
# 구독하므로 먼저/함께 띄워야 함)
ros2 run dolbotz segmentation --ros-args \
  -p conf_threshold:=0.5

# 터미널 3 — elevation_map (gradient_map의 입력을 만듦)
ros2 run dolbotz elevation_map --ros-args \
  -p depth_topic:=/drive/camera/aligned_depth_to_color/image_raw/compressedDepth \
  -p camera_info_topic:=/drive/camera/aligned_depth_to_color/camera_info \
  -p imu_topic:=/drive/camera/imu \
  -p resolution_m:=0.15 -p min_depth_m:=0.5 -p max_depth_m:=4.0 \
  -p blind_fill_forward_m:=0.6

# 터미널 4 — gradient_map (경사 경로)
ros2 run dolbotz gradient_map --ros-args \
  -p resolution_m:=0.15 -p max_slope_deg:=30.0

# 터미널 5 — flat_drive (평지 경로)
ros2 run dolbotz flat_drive --ros-args \
  -p bev_meters_per_pixel:=0.03 -p bev_img_width:=200 -p bev_img_height:=200 \
  -p min_row_pixels:=5

# 터미널 6 — slope_decision (좌우 기울기 추정 + /path, /drive/status 릴레이)
ros2 run dolbotz slope_decision --ros-args \
  -p track_width_m:=0.45 -p slope_threshold_deg:=10.0

# 터미널 7 — rqt_graph
rqt_graph
```

전부 뜬 상태에서 `rqt_graph`를 열면 카메라 → `segmentation` →
`elevation_map`/`flat_drive`/`slope_decision` → `gradient_map` →
`slope_decision`(릴레이) → `/path`까지 전체 흐름이 한 그래프에 보인다.
노드/토픽이 안 보이면 좌상단 새로고침 버튼을 누르거나 "Dead sinks"/"Leaf
topics" 표시 옵션을 꺼서 확인한다. `slope_decision`은 `SlopeVisualizer`
OpenCV 창을 띄우므로 헤드리스 환경에서는 X 디스플레이가 필요하다
(SSH라면 `ssh -X`).


## 4. 테스트 실행

`gradient_map.py`/`elevation_map.py`/`flat_drive.py`가 최상단에서
`rclpy`/`nav_msgs`/`geometry_msgs`/`cv_bridge` 등을 import하므로, ROS 환경을
source한 뒤 실행해야 합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/j/dolbotZ
python3 -m pytest test/ -v
```

- `test_gradient_field.py` — `compute_gradient_field`, `plan_path_on_slope_field`
- `test_elevation_map.py` — depth 역투영, IMU 상보필터, `camera_body_to_level_matrix`,
  고도 그리드 투영 (blind-fill 포함)
- `test_flat_drive.py` — 지면 호모그래피 재유도(`ground_to_image_homography` 등,
  마운트/섀시 기울기 조합에 대한 독립 물리 검증 포함), 세그멘테이션 마스크 →
  BEV → centerline 경로 추출
- `test_paths.py` — `dolbotz.utils.paths`의 리포/패키지 경로 해석(`get_repo_root`,
  `get_package_share_dir`의 ament_index/폴백 하이브리드, `get_models_dir`,
  `load_calibration`)
- `test_attitude.py` — `dolbotz.utils.attitude`의 마운트 파라미터 상수,
  `resolve_mount_defaults`의 캘리브레이션 피클 오버라이드 우선순위, 그리고
  `ElevationMapNode`/`FlatDriveNode`의 `declare_parameter` 기본값이 실제로 이
  상수를 참조하는지에 대한 회귀 테스트(두 노드를 실제로 생성함)

순수 함수 위주의 단위/벤치마크 테스트이며, `test_attitude.py`의 노드 생성
테스트를 제외하면 ROS 노드(`GradientMapNode`/`ElevationMapNode`/`FlatDriveNode`)
자체는 테스트 대상이 아닙니다. `dolbotz.utils.attitude`(`roll_pitch_from_accel_body`,
`update_complementary_filter`, `R_BODY_TO_OPTICAL`, `resolve_mount_defaults`,
`MOUNT_*_PLACEHOLDER` 등)는 `elevation_map.py`와 `flat_drive.py`가 공유하는
IMU 자세 추정 + 마운트 파라미터 공용 모듈입니다.

## 5. config/ 디렉토리

리포 루트의 `config/`에는 모델 파일과 카메라 캘리브레이션 자리만 둔다 (마운트
파라미터는 위에서 설명한 대로 `attitude.py` 코드 상수로 관리하며, 이번
정리로 `config/camera_extrinsics.yaml`은 제거했다):

- `config/models/` — 학습된 모델 가중치 (`supplybest.pt`, `dolbotz_seg_v1/`).
  `dolbotz.utils.paths.get_models_dir()`로 코드가 실행 환경과 무관하게 찾는다.
  자세한 이동 내역은 `config/models/README.md` 참고. 새로 재학습하려면
  `train_drive_area.py` 실행 전에 `export ROBOFLOW_API_KEY=...`로 환경변수를
  설정해야 한다(코드에 키를 하드코딩하지 말 것).
- `config/model_paths.yaml` — 문서화 + override 템플릿 (실제 로딩에는 쓰이지 않음).
- `config/calibration/` — 카메라별 실측 캘리브레이션 피클이 들어갈 자리
  (아직 생성 스크립트 없음). 스키마/네이밍은 `config/calibration/README.md` 참고.


## 6. 워크스페이스 구조 리팩터링 (drive_area/missions 서브패키지 + 계절별 미션 + launch)

`src/dolbotz/src/dolbotz/` 안이 두 서브패키지로 정리됐다. **`ros2 run dolbotz <이름>`
실행 커맨드 자체는 안 바뀌었다** — entry_point 이름은 그대로 두고 가리키는
모듈 경로만 바뀐 거라, 위 3절의 `flat_drive`/`elevation_map`/`gradient_map`/
`slope_decision`/`segmentation` 실행법은 그대로 유효하다.

- `drive_area/` — 지형 인식·경로계획 (`segmentation`, `flat_drive`,
  `elevation_map`, `gradient_map`, `slope_decision`)
- `missions/` — 계절별 미션 인식 (`summer_supply`(구 `arm_pickup`),
  `spring_ifof`, `summer_traffic`, `fall_marker`, `escort_follow`)

### mission_manager_interfaces (신규 패키지, 임시)

`MissionResult.msg` 하나만 있는 msg 전용 패키지 (`ament_cmake`, 구 이름
`dolbotz_interfaces`). 미션 노드들이 공용으로 쓰는 결과 메시지 타입:
```
std_msgs/Header header
string mission_name
string state
geometry_msgs/Point target_point
bool valid
```

> **주의(임시 패키지)**: 미션 노드들이 아직 전부 스텁이라 이 스키마가
> 실제로 맞는지 검증된 적이 없다. 구현 진행하면서 안 맞으면 표준 메시지
> 타입(`std_msgs/String`, `geometry_msgs/PointStamped` 등)으로 대체하고
> 이 패키지 자체를 없애는 것도 고려할 것.

### 계절별 미션 스텁 노드 4개 (실제 인식 로직 미구현, 뼈대만 있음)

```bash
ros2 run dolbotz spring_ifof       # 봄 — 피아식별, /mission/spring_ifof/result
ros2 run dolbotz summer_traffic    # 여름 — 신호등 인식, /mission/summer_traffic/result
ros2 run dolbotz fall_marker       # 가을 — 비전마커 순차 인식, /mission/fall_marker/result
ros2 run dolbotz escort_follow     # 5구간 — 선도 정찰로봇 추종, /mission/escort_follow/result
```

넷 다 컬러 이미지(`/drive/camera/color/image_raw/compressed`, `camera_info`)를
구독하고(`escort_follow`만 뎁스도 message_filters로 동기화 구독), 각자
`/mission/<이름>/result`(`mission_manager_interfaces/MissionResult`)를 발행한다.
지금은 매 프레임 `valid=False`인 더미 결과만 나간다 — 각 파일 안
`# TODO: 실제 인식 로직 구현` 위치에 실제 모델/로직을 채워야 한다.

> **주의**: 각 노드의 배점/판정 기준은 아직 대회 규정 문서가 반영 안 됨 —
> docstring에 TODO로 남겨뒀으니 규정 확정되면 채울 것.

### launch 파일

```bash
ros2 launch dolbotz perception_common.launch.py   # segmentation만 (공용, 상시 필요)
ros2 launch dolbotz mission_spring.launch.py       # + spring_ifof
ros2 launch dolbotz mission_summer.launch.py       # + summer_traffic + summer_supply
ros2 launch dolbotz mission_fall.launch.py         # + fall_marker
ros2 launch dolbotz mission_winter.launch.py       # + flat_drive + elevation_map + gradient_map + slope_decision
```

`mission_*.launch.py`는 전부 `perception_common.launch.py`를 include해서
`segmentation`을 같이 띄운다 — 따로 안 띄워도 됨.

### 자율 제어

**[하림 수정] drive 쪽은 더 이상 스텁이 아님** — `drive_auto` 패키지는 삭제됐고,
doldrive_ws에서 이관한 실제 파이프라인(`src/drive/autonomous/`)으로 완전히
대체됨. 조이스틱 없이 터미널만으로 실행 가능:

```bash
ros2 launch robot_bringup autonomous.launch.py
```

`slope_decision`이 발행하는 `/path`(경로), `/terrain/side_slope_angle_deg`(카메라
roll)를 받아서 `path_relay`+`slope_traverse_node`+MPPI(`nav2_mppi_controller`)로
실제 CAN 모터 명령까지 이어짐. 패키지 구성/상세 흐름은 이 문서의 "3. 노드 실행"
섹션과 `src/drive/autonomous/robot_bringup/launch/` 안 각 launch 파일 docstring
참고.

# 메뉴얼 주행

### CAN_drive 인터페이스 활성화
sudo ip link set can_drive type can bitrate 1000000
sudo ip link set up can_drive
ip -details link show can_drive

### [하림 수정] 실행 — 한 줄로 끝남 (joy_node+manual_joy_control_node+can_driver_node+
### myahrs_driver_node+manual_stability_node 전부 포함, IMU pitch/roll 25°/20° 넘으면
### 자동 긴급정지)
source install/setup.bash
ros2 launch can_driver manual.launch.py

### 예전 방식(터미널 2개로 따로 띄우기, 참고용 — 위 한 줄로 대체됨, IMU 안전장치 없음)
# source install/setup.bash
# ros2 run can_driver can_driver_node --ros-args -p can_channel:=can_drive
# source install/setup.bash
# ros2 launch manual_joy_control manual_control.launch.py

## 확인용 (선택)

### 터미널 3 - 조이스틱 raw 입력 확인
source ~/dolbotZ/install/setup.bash
ros2 topic echo /joy
 

### 터미널 4 - 최종 모터 속도 명령 확인
source ~/dolbotZ/install/setup.bash
ros2 topic echo /motor_speed_cmd

### 터미널 5 - drive/arm 중 조이스틱이 지금 어디에 반응 중인지 확인
source ~/dolbotZ/install/setup.bash
ros2 topic echo /control/active_target
# Options(9번) 버튼으로 drive <-> arm 토글. Arm 진입 시 기본은 MANUAL.
# Arm 활성 중 Share(8번) 버튼으로 MANUAL <-> AUTO 전환.
# active_target은 data: drive 또는 data: arm 으로 표시됨


cansend can_drive 141#7600000000000000
보호상태 해제


cansend can_drive 141#9C00000000000000
DATA[0]   0x9C          명령 코드
DATA[1]   int8_t        모터 온도, °C
DATA[2:3] int16_t       토크 전류
DATA[4:5] int16_t       출력축 속도, deg/s
DATA[6:7] uint16_t      출력축 엔코더 위치

모터상태 확인 명령

회전이 안 될 때, 
cansend can_drive 141#9C00000000000000
candump can_drive
이거 실행하고 로그 찍어두기


### CAN 인터페이스 활성화 can_arm
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm
ip -details link show can_arm
#
ros2 launch robot_arm_bringup robot_arm_bringup.launch.py launch_joy:=false
# drive 쪽 manual_control.launch.py에서 joy_node를 이미 띄웠으므로 launch_joy:=false로
# 중복 실행 방지 (drive를 안 띄우고 arm만 단독으로 쓸 거면 이 인자 빼면 됨)
-> 선주야 ros2 launch robot_arm_bringup robot_arm_bringup.launch.py까지만 치면 됨.

### 실기 팔 RViz 미러링 (실측 각도/엔코더, 읽기 전용 - army_manipulator_description 내장)
# ~/arm_config/rmd_joint_state_bridge.py(외부 스크립트)와 동일한 역할을 하는
# 내장 노드(rmd_joint_state_bridge 패키지) - ros2_control 없이 RMD(CAN)/
# Dynamixel(TTL) 엔코더를 getter로만 읽어서 /joint_states + 진단 토픽으로
# 발행한다. 포지션 커맨드를 절대 안 보내므로 팔을 손으로 자유롭게 움직이며
# 자세(홈/스탠바이 등)를 캡처할 때 안전하게 쓸 수 있다.

# 터미널 1 - RViz (URDF만, MoveIt 없이 가볍게)
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
ros2 launch army_manipulator_description display.launch.py

# 터미널 2 - CAN 인터페이스 켜기 (위 "CAN 인터페이스 활성화 can_arm" 참고, 이미 켰으면 생략)
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set up can_arm

# 터미널 3 - 브릿지 실행 (기본 파라미터 = army_manipulator_ros2_control.xacro 기본값과 동일.
# 실기 캘리브레이션(sign/q_offset, position_direction/position_zero_offset)이 다르면
# 아래처럼 launch argument로 덮어쓸 것 - 값은 army_manipulator_ros2_control.xacro의
# shoulder_sign/elbow_sign/wrist_sign, shoulder_q_offset/elbow_q_offset/wrist_q_offset과
# 그대로 맞추면 됨)
source /opt/ros/humble/setup.bash
source ~/dolbotZ/install/setup.bash
ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py
# 예: ros2 launch rmd_joint_state_bridge joint_state_bridge.launch.py wrist_sign:=-1.0 wrist_q_offset:=0.0523599

# 확인용 - 새 터미널에서
ros2 topic echo /arm/joint_angle_deg      # RMD 보정된 각도(deg), [shoulder, elbow, wrist] 순서
ros2 topic echo /arm/rmd_raw_angle_deg    # RMD 원본(보정 전) 각도(deg), 같은 순서
ros2 topic echo /arm/rmd_encoder          # RMD 원본 엔코더 카운트, 같은 순서
ros2 topic echo /arm/dxl_angle_deg        # Dynamixel 보정된 각도(deg), [base, gripper] 순서
ros2 topic echo /arm/dxl_encoder          # Dynamixel 원본 pulse, 같은 순서
# 실제 팔을 손으로 움직이면 RViz에서 그대로 따라 움직이고 위 토픽 값도 실시간으로 바뀜.

# === 카메라 4대 네이밍 정리 ===
# 주행용 depth D455  -> 네임스페이스 drive   (/drive/camera/...)
# 로봇팔용 depth D435i -> 네임스페이스 arm    (/arm/camera/...)
# 사이드캠 Logitech C920 -> 네임스페이스 leftview  (/leftview/...)
# 사이드캠 Logitech C922 -> 네임스페이스 rightview (/rightview/...)

# 로봇팔 D435i — summer_supply(구 arm_pickup) 전용
# -r __ns:=/arm 필수 — 안 주면 기본 네임스페이스(/camera/camera/...)로 나가서
# summer_supply의 기본 구독 토픽(/arm/camera/...)과 안 맞아 아무 프레임도 안 들어옴
# (주행 D455의 -r __ns:=/drive 패턴과 동일, 위 "카메라 4대 네이밍 정리" 참고)
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/arm -p serial_no:="'213622251385'" -p enable_color:=true -p enable_depth:=true -p align_depth.enable:=true -p enable_infra1:=false -p enable_infra2:=false -p enable_gyro:=false -p enable_accel:=false

ros2 run dolbotz summer_supply
# [2026-08-30] RF-DETR/YOLO 비교 끝나고 yolo(supplyboxv3.pt, 구 supplybest.pt에서
# 교체)로 확정 - detector_backend 파라미터/RF-DETR 코드는 제거됨(더 이상 존재하지 않음).
ros2 run dolbotz arm_visualizer

# 로봇팔 rosbag
ros2 bag record -o summer_supply_bag \
  /arm/camera/color/image_raw/compressed \
  /arm/camera/color/camera_info \
  /arm/camera/aligned_depth_to_color/image_raw/compressedDepth \
  /arm/camera/aligned_depth_to_color/camera_info

ros2 bag record -o summer_supply_bag -a


# 주행 D455 (align_depth: RGB 마스크와 depth를 같은 좌표계로 쓰기 위해 켬)
ros2 run realsense2_camera realsense2_camera_node --ros-args -p serial_no:="'117222251401'" -p enable_color:=true -p enable_depth:=true -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 -p enable_infra1:=false -p enable_infra2:=false -p align_depth.enable:=true


ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=drive \
  camera_name:=camera \
  serial_no:=_117222251401 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15



# 로봇팔 D435i — 시리얼은 위 "summer_supply 전용" 절과 동일 카메라(213622251385)
  ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=arm \
  camera_name:=camera \
  serial_no:=_213622251385 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  pointcloud.enable:=false \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15


# 사이드캠 (Logitech C920 -> leftview, C922 -> rightview, usb_cam 패키지)
# video_device 경로는 `ls /dev/video*` 또는 `v4l2-ctl --list-devices`로
# 실제 장치를 확인해서 채울 것 — 재부팅/재연결 시 번호가 바뀔 수 있으므로
# /dev/v4l/by-id/... 의 고정 심볼릭 경로를 쓰는 게 더 안전함.
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/leftview \
  -p video_device:=<TODO: C920 장치 경로, 예: /dev/video0> \
  -p camera_name:=leftview

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r __ns:=/rightview \
  -p video_device:=<TODO: C922 장치 경로, 예: /dev/video2> \
  -p camera_name:=rightview


  ros2 bag record -o ~/arm_pickup_bag_all \
  /leftview/image_raw/compressed \
  /rightview/image_raw/compressed \
  /arm/camera/color/camera_info \
  /arm/camera/color/image_raw \
  /arm/camera/color/image_raw/compressed \
  /arm/camera/color/image_raw/compressedDepth \
  /arm/camera/color/image_raw/theora \
  /arm/camera/color/metadata \
  /arm/camera/depth/camera_info \
  /arm/camera/depth/image_rect_raw \
  /arm/camera/depth/image_rect_raw/compressed \
  /arm/camera/depth/image_rect_raw/compressedDepth \
  /arm/camera/depth/image_rect_raw/theora \
  /arm/camera/depth/metadata \
  /arm/camera/extrinsics/depth_to_color \
  /arm/camera/extrinsics/depth_to_depth \
  /drive/camera/color/camera_info \
  /drive/camera/color/image_raw \
  /drive/camera/color/image_raw/compressed \
  /drive/camera/color/image_raw/compressedDepth \
  /drive/camera/color/image_raw/theora \
  /drive/camera/color/metadata \
  /drive/camera/depth/camera_info \
  /drive/camera/depth/image_rect_raw \
  /drive/camera/depth/image_rect_raw/compressed \
  /drive/camera/depth/image_rect_raw/compressedDepth \
  /drive/camera/depth/image_rect_raw/theora \
  /drive/camera/depth/metadata \
  /drive/camera/extrinsics/depth_to_color \
  /drive/camera/extrinsics/depth_to_depth

## 7. 통합 RViz 시각화 (경로 주행 스택)

2026-08-20 작업. slope_decision의 OpenCV 창, path_visualizer.py(노트북 단독
실행), arm_visualizer.py처럼 따로따로 흩어져 있던 디버그 뷰어를 RViz 하나로
통합했다. 이번 스코프는 "경로 주행" 관련 스택만 — 사이드캠/암캠 이미지,
`/mission/*/result`는 포함 안 함. 산출물:
  - `src/dolbotz/rviz/dolbotz_visualization.rviz` — 통합 RViz 설정
  - `src/dolbotz/launch/visualization.launch.py` — 위 설정으로 RViz2 + 전제조건
    릴레이 노드 2개(terrain_viz_relay, slope_visualizer)를 띄움
  - `src/dolbotz/dolbotz/utils/terrain_viz_relay.py` — 신규 노드(아래 설명)
  - `src/dolbotz/dolbotz/utils/path_visualizer.py` — 기존 HUD에 SPEED LIMIT
    줄 추가(/speed_limit, nav2_msgs/SpeedLimit)

### 실행

perception 스택(계절 미션 중 하나)과, MPPI 궤적/costmap/odom까지 보려면
autonomous.launch.py(또는 최소 ekf.launch.py+nav2.launch.py)도 따로 떠
있어야 한다 — 이 프로젝트 기존 컨벤션대로(3번 항목 "여러 터미널로 따로
띄우기") visualization.launch.py는 그 전제조건들을 스스로 띄우지 않는다.

```bash
# 터미널 1~N — 평소처럼 perception + (선택) autonomous 스택 띄우기
ros2 launch dolbotz mission_winter.launch.py    # 계절은 spring/summer/fall도 동일 토픽 구조라 무관
ros2 launch robot_bringup autonomous.launch.py  # MPPI 궤적/costmap/odom 보려면 필요

# 터미널 N+1 — 통합 RViz
ros2 launch dolbotz visualization.launch.py

# 헤드리스(SSH, X 디스플레이 없음)에서는 GUI만 끄고 릴레이 노드만:
ros2 launch dolbotz visualization.launch.py start_rviz:=false
```

`ros2 launch dolbotz visualization.launch.py`(RViz 포함)를 로컬에서
`colcon build --packages-select dolbotz --symlink-install` 후 실제로 띄워
확인함 — rviz2 프로세스가 OpenGL 초기화 후 에러/경고 없이 모든 Display를
로드하는 것까지 검증(단, 실제 로봇/카메라가 안 붙어있어 데이터가 흐르는
것까지는 이 로컬 환경에서 확인 못 함 — 실기에서 재확인 필요).

spring/summer/fall 세 계절도 winter와 동일한 노드 이름/토픽 구조를 그대로
쓴다(flat_drive_node, elevation_map_node, gradient_map_node,
side_slope_trigger_node — mission_spring/summer/fall.launch.py 파일 상단
docstring에 명시) — 이번 스코프에서 뺀 계절별 미션 노드(spring_ifof 등)만
다르므로, 이 RViz 설정은 계절 전환 시 수정 없이 그대로 재사용된다.

### Display 목록 및 실제 확인한 사실 (프롬프트 가정과 다른 부분)

작업 시작 프롬프트에 나열된 토픽/타입은 실제 코드로 하나하나 재확인했고,
대부분 일치했다. 달랐던 부분:

  - **`/local_costmap/costmap`는 별도 costmap 노드가 아니라 controller_server에
    내장**돼 있다(nav2.launch.py 주석: "controller_server(및 내부
    local_costmap)"). 그리고 **2026-08-22 커밋으로 planner_server/
    global_costmap/bt_navigator가 통째로 제거**됐다(실제 주행 경로가 그
    셋을 전혀 거치지 않는 걸 확인하고, 안 쓰는 기능 때문에 controller_server
    activate가 막힐 리스크만 남아 정리함 — nav2.launch.py 상단 주석 참고).
    즉 `/global_costmap/...`류는 이제 없고 `/local_costmap/costmap`만 있다.
  - **`/terrain/elevation_map` 외에 `/terrain/slope_deg`, `/terrain/gradient_magnitude`도
    32FC1**이다(gradient_map.py 소스 확인 — gx/gy/magnitude/direction/slope_deg
    다섯 개가 같은 `cv2_to_imgmsg(..., encoding='32FC1')` 호출을 공유). 프롬프트는
    elevation_map만 32FC1 처리가 필요하다고 가정했지만 셋 다 필요해서
    terrain_viz_relay.py가 셋 다 처리한다.
  - **`/debug/slope_markers`는 이미 존재하는 토픽**이었다 — `utils/slope_visualizer.py`가
    이미 발행 중(신규 아님). visualization.launch.py가 이 기존 노드를
    같이 띄우기만 한다.
  - **`transformed_global_plan`은 path_relay_node가 만드는 토픽이 아니라
    nav2_mppi_controller 자신이 만드는 디버그 재발행분**이다
    (trajectory_visualizer.cpp) — path_relay_node는 자기가 변환한 odom
    프레임 경로를 어떤 토픽으로도 노출하지 않고 FollowPath 액션 goal로만
    보낸다. `/trajectories`와 마찬가지로 **RViz가 실제로 구독을 걸어야만
    발행되는 최적화**가 돼 있어(trajectory_visualizer.cpp:118,
    구독자 0명이면 통째로 skip), RViz 켜기 전엔 토픽이 비어 보이는 게 정상.
  - **`/speed_limit`은 `nav2_msgs/SpeedLimit`**이다(단순 Float32 아님).
    `speed_limit=0.0`은 "제한 없음"이라는 별도 의미(nav2_msgs/SpeedLimit
    규약) — path_visualizer.py HUD도 그 규약대로 0.0은 "NO LIMIT"으로 따로
    표시. [2026-08-27] 이 토픽을 발행하던 slope_speed_limiter_node 패키지
    삭제됨 -- 현재는 발행자가 없어 항상 N/A.
  - **`src/dolbotz/howtorun.md`의 `/camera/camera/...`는 오래된 값, 이 파일
    (레포 루트)의 `/drive/camera/...`가 현재 기준**이다 — 3번 항목의 "터미널
    1" 주석에 `-r __ns:=/drive`를 명시적으로 줘야 하는 이유가 적혀있고,
    이게 realsense 노드의 실제 기본 네임스페이스(`/camera/camera/...`)를
    덮어쓰는 것이라 확인됨. `src/dolbotz/howtorun.md` 쪽은 이 재네임 이전
    버전으로 보이며 갱신이 필요하다(이번 작업 스코프 밖이라 직접 고치진
    않음 — 확인만 하고 보고).

### `/terrain/*_viz` 컬러맵 릴레이 (terrain_viz_relay.py)

`/terrain/elevation_map`, `/terrain/slope_deg`, `/terrain/gradient_magnitude`
(전부 32FC1, NaN 섞임)를 NaN-마스킹 + min/max 정규화 + `cv2.applyColorMap`으로
`bgr8` 컬러 이미지로 재발행하는 순수 릴레이 노드(OpenCV 창 없음, 헤드리스
안전). 출력 토픽은 입력에 `_viz` 접미사:
  - `/terrain/elevation_map_viz`
  - `/terrain/slope_deg_viz`
  - `/terrain/gradient_magnitude_viz`

```bash
ros2 run dolbotz terrain_viz_relay
ros2 run dolbotz terrain_viz_relay --ros-args -p colormap:=turbo   # jet(기본)/turbo/viridis
```

로컬에서 순수함수(`normalize_and_colorize`) 단위테스트(NaN 픽셀 검정 처리,
전부-NaN 프레임 스킵, 상수값 프레임)와 rclpy 통합테스트(합성 32FC1 이미지
발행 -> `_viz` 토픽 수신 확인)까지 통과 확인함.

### `/path` <-> `odom` TF 연결 (요구사항 5, 미해결 항목 재확인)

`path_relay/config/path_relay_params.yaml`에 "frame_id/TF가 odom까지
연결되는지 아직 미확인"이라는 주석이 있던 부분. 코드 추적으로는 연결될
조건이 다 갖춰져 있다:
  - `/path.header.frame_id` 기본값 `'camera_link'`
    (gradient_map.py/flat_drive.py의 `path_frame_id` 파라미터 기본값,
    slope_decision.py가 그대로 릴레이)
  - `path_relay_node.cpp`가 `lookupTransform(target_frame_="odom",
    "camera_link", ...)` 수행 (202-203행)
  - TF 체인 `odom -> base_link`(EKF) `-> camera_link`(static,
    ekf.launch.py)는 구조적으로 존재함
다만 이건 **코드 추적 결과이지 실기 검증이 아니다** — 로봇을 켜서
`ros2 topic echo /path --field header.frame_id`로 문자열이 정말
`camera_link`인지, RViz의 `transformed_global_plan` Path가 odom 프레임에서
로봇 근처에 말이 되는 위치로 뜨는지 직접 봐야 확정된다. 다른 파이프라인
코드는 이번 작업 범위 밖이라 고치지 않았다.

### 상태 HUD (요구사항 6) — RViz 내장 텍스트 패널 조사 결과

`/drive/status`, `/terrain/side_slope_angle_deg`, `/cmd_vel_auto`,
`/speed_limit`처럼 숫자/문자열 하나짜리 토픽을 RViz 안에 실시간 텍스트로
띄우는 표준 rviz_default_plugins 디스플레이는 없다. `rviz_2d_overlay_plugins`,
`jsk_rviz_plugins`류 서드파티 플러그인이 있어야 하는데, **둘 다 이
환경(`ros2 pkg list`, `dpkg -l`, `apt list --installed`)에 설치돼 있지
않음을 확인**했다 — 요구사항대로 임의로 설치하지 않았다.

그래서 이번엔 **RViz와 별개의 OpenCV 창**으로 유지했다 — 이미 있던
`path_visualizer.py`의 HUD가 `/drive/status`/`/terrain/side_slope_angle_deg`/
`/cmd_vel_auto` 세 개를 이미 표시하고 있어서(새로 만들지 않고) `/speed_limit`
표시만 추가했다:

```bash
python3 src/dolbotz/dolbotz/utils/path_visualizer.py
```

나중에 RViz 안에 직접 넣고 싶으면 `sudo apt install ros-humble-rviz-2d-overlay-plugins`
설치 여부를 먼저 사용자에게 확인하고 진행할 것 (이번 작업에서는 설치
승인을 받지 않았으므로 보류).


### 여름 자율 매니퓰레이터 실행 순서

아래 순서는 실제 로봇 기준이다. 모든 터미널에서 같은 워크스페이스를 source하고
`ROS_DOMAIN_ID=99`, `ROS_LOCALHOST_ONLY=0`, `rmw_cyclonedds_cpp`를 동일하게 써야
한다. 소스 수정 후 최초 1회는 관련 패키지를 빌드한 다음 새 터미널에서 실행한다.

```bash
cd ~/dolbotZ
colcon build --packages-select \
  army_manipulator_bringup army_manipulator_description
source install/setup.bash
```

**터미널 런처 권장 순서**

1. `CAN Arm`
2. `Arm Camera`
3. `summer_arm` — 팔 controller, MoveIt, `ik_node.py`

`summer_arm` 실행 후 `grasp_wait` 실제 도달 로그를 확인한다. 이 절에서는
`CAN Drive`, `Drive Camera`, `vision_summer`, `summer_drive` 실행 순서를 다루지
않는다.

런처를 사용하지 않을 때의 핵심 수동 명령은 다음과 같다.

```bash
# 1. 로봇팔 RealSense
#    [2026-09-05] 토픽은 /arm/camera/... 그대로, TF 프레임만 arm_camera_link /
#    arm_camera_color_optical_frame 로 발행된다(구동부 D455의 camera_* 프레임과
#    충돌 방지). 팔 URDF의 base_link도 arm_base_link로 바뀜(구동부 odom->base_link와
#    충돌 방지). 상세는 realsense_bringup.launch.py 상단 주석 참고.
ros2 launch army_manipulator_bringup realsense_bringup.launch.py \
  serial_no:=_243322074693 \
  camera_namespace:=arm \
  camera_name:=camera

# 2. 로봇팔 제어: 시작 즉시 grasp_wait로 이동 + 그리퍼 open (2026-09-05부터)
ros2 launch army_manipulator_bringup ik_node_bringup.launch.py \
  use_mock_hardware:=false
```

매니퓰레이터 확인용 명령:

```bash
ros2 control list_controllers
ros2 topic echo /arm/target_point
ros2 topic echo /arm/target_point_base
ros2 topic echo /joint_states
ros2 topic echo /arm/calculation_failed
ros2 topic echo /arm/picking_command
```

`/arm/target_point`는 외부 인지 노드가 발행하는 입력이고,
`/arm/picking_command`와 `/arm/calculation_failed`는 `ik_node.py`의 출력이다.
정지 완료 handshake 및 구동부 실행 방법은 아래 여름 구간 주행 노드 절에서
별도로 다룬다.


### 여름 구간 주행 노드 (summer_supply_drive_node) — 로봇팔 연동 정리

박스는 시작부터 팔 가동범위 안에 배치한다. 전진 신호와 2cm 접근 동작은 제거했다.

1. `summer_arm`의 IK 노드는 컨트롤러 준비 후 그리퍼를 열고 `grasp_wait`로 이동한다.
   준비자세 도착 전에는 검출 목표를 수락하지 않는다.
2. `vision_summer`는 유효 depth와 `base_actuator` 기준 `|X| <= 0.32m`를
   3프레임 연속 확인하면 `/arm/target_point`를 발행한다. 범위 밖이면 정지 상태로 대기한다.
3. IK 목표 Z는 -0.3080m(보정 -5.5mm)이다. 목표 이동 후 2초 대기하고 그리퍼를 닫는다.
4. 파지 성공 후 `hold` → `home`으로 이동하고 `/arm/picking_command`를 발행한 뒤 `bed`로 이동한다.
5. `summer_drive`는 파지 완료 전 `WAIT_PICK`에서 정지한다. 완료 신호 후 `cruise_vx=0.3927m/s`로
   주행하며 `/path`로 조향한다. 신호등 정지는 항상 우선한다.

**확인하면서 눈에 띈 점**

- 현재 파지 성공 판정은 그리퍼 닫기 trajectory 완료 기준이다. 실제 물체 접촉이나
  팔의 목표 도달 오차를 확인하지 않으므로 빈 파지도 성공 처리될 수 있다.
- 파지 완료 후 인지 모델은 종료되고 IK 노드는 새 좌표를 무시한다.
- `state_entered_time_`는 (아마 이전 재시도 루프 삭제 후) 갱신만 되고 어디서도
  읽히지 않는 죽은 변수다(`summer_supply_drive_node.cpp:137,233,263`).
- `utils/terminal_launcher/README.md`는 `Summer Drive`/`Mission Summer
  Perception`이라는 이름을 쓰는데, 실제 `commands.csv` 행 이름은
  `summer_drive`/`vision_summer`다.
- (해결) `commands.csv`의 `summer_arm` 행이 `control_bringup.launch.py`
  (`maru_ik_node.py` 경로)를 실행하던 걸 `ik_node_bringup.launch.py`
  (`ik_node.py`)로 맞췄다 — 위 "여름 자율 구동 매니퓰레이터" 절과 이제 일치.
  `ik_node.py`는 시작 시 자체적으로 GRASP_WAIT 자세로 들어가므로
  `/drive/supplybox_detected` 기반 프리포지셔닝이 원래 필요 없다.

**팔 치수 및 좌표 기준**

- 실제 핑거 부품 길이는 80mm다.
- 피니언 기어 원점에서 핑거 끝 TCP까지의 거리는 90mm이며, URDF의
  `pinion_gear -> tcp_link`도 `0.090m`다. IK의 wrist→TCP 보정은 링크 150mm와
  이 90mm를 합친 240mm를 사용하므로 핑거 80mm를 목표 Z에 다시 더하지 않는다.
- `base_actuator` 원점은 world 지면보다 350mm 높고, 95mm 박스 측면 중앙은
  지면보다 47.5mm 높다. 따라서 `base_actuator` 기준 TCP Z는 `-302.5mm`이고,
  여름 launch 기본 보정 `-5.5mm`(경사 아래쪽 박스를 위해 기존보다 2mm 하강)를 더한 최종
  목표는 `-308.0mm`다.
  새 높이는 반경 160~315mm, Y=-30/0/+30mm의 468개 조합에서 수치 IK 해를 확인했다.
  MoveIt 충돌검사와 실기 도달은 아직 확인하지 않았다.
- 이전 검증 높이 Z=-306.0mm에서
  중심선 기준 IK 가능 수평 반경은 약 157~436mm이며, 운용
  범위 반경 158~315mm, Y=±30mm는 거리별 seed로 모두 1회 시도에 풀린다
  (mock MoveIt, 충돌검사 on, timeout 0.5초 기준).
- 검출 Z는 IK 입력으로 사용하지 않지만 고정 Z와 50mm 이상 다르면 카메라
  TF/캘리브레이션 점검 경고를 출력한다.
