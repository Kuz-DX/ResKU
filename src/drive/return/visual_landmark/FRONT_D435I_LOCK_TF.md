# 전방 D435i LOCK-only 차량 TF (2026-10-27)

현재 장치: Intel RealSense **D435i**, serial `339222071362`, ROS 인자 `_339222071362`.
사용자 실기 확인: color CameraInfo `arm_camera_color_optical_frame`, 640×480,
RGB/aligned depth 약 30Hz, root→color optical 내부 TF 정상. 후방 TF도 정상이다.
이 문서의 **새 차량→전방 root TF는 로컬 구현·검사만 완료**했으며 실기 PASS를 의미하지 않는다.

## 측정 기준과 공식 근거

사용자가 확인한 측정점은 **카메라 후면 중심(사용자 표현: 렌즈 바로 뒤)**이다.
차량 `base_link` 기준 (0.152853, 0, 0.650674)m를 그대로 보존한다.
이 점이 공식 기구도상의 depth/root 축과 횡·수직으로 정확히 일치한다는 것은
입증되지 않았다. 후면 중심과 RGB/depth imager 중심은 동일 개념이 아니다.
따라서 y=0은 사용자 기준을 유지한 **시험 가정**이며 CAD 정밀 보정값이 아니다.

공식 자료(2026-10-27 확인):

- [D400 datasheet, March 2023, revision 015](https://www.realsenseai.com/wp-content/uploads/2023/03/Intel-RealSense-D400-Series-Datasheet-March-2023.pdf):
  §3.8.1 Table 3-52의 D435/D435i 케이스 깊이 nominal **25mm** (24.85–25.15mm),
  §4.8 Table 4-16의 depth zero는 front glass보다 **4.2mm 뒤**.
  §4.8.1의 XY depth 기준은 left imager 중심이다. RGB 원점과 구분한다.
- [공식 D435i description](https://github.com/realsenseai/realsense-ros/blob/7a87097c499e3b0b69518d54a8e7a9ac1924e87c/realsense2_description/urdf/_d435i.urdf.xacro):
  `sensor_d435`를 재사용하고 IMU 모델을 추가한다. D435 기구 모델을 D435i에
  적용하는 근거이며 D455에는 적용하지 않는다.
- [공식 D435 description](https://github.com/realsenseai/realsense-ros/blob/7a87097c499e3b0b69518d54a8e7a9ac1924e87c/realsense2_description/urdf/_d435.urdf.xacro):
  기구 모델 깊이 **25.05mm**, front plate→glass **0.1mm**, glass→depth zero **4.2mm**.
  tripod mount→front 방향 치수 14.9mm는 **tripod 기준**이므로 후면 좌표에 더하지 않는다.
  root는 depth origin과 같은 위치의 ROS body-axis frame이며 optical axes와 다르다.
  nominal 내부 extrinsic 대신 실제 driver calibration을 쓰도록 명시돼 있다.
- [공식 librealsense D435i 좌표 설명](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md#sensor-origin-and-coordinate-system):
  depth optical 좌표는 오른쪽/아래/전방이다. 차량/root body 축인 전방/왼쪽/위와 구분한다.

계산은 공식 description의 nominal 기구값으로 통일한다.

```text
back -> root, forward = 25.05 - 0.1 - 4.2 = 20.75 mm
x_root_trial = 0.152853 + 0.020750 = 0.173603 m
y_root_trial = 0                         # 별도 lateral offset 추가 없음
z_root_trial = 0.650674 m
```

datasheet의 25.00mm를 쓰면 20.70mm로, 모델 계산과 0.05mm 차이가 있다.
이를 장치별 실측 정확도로 해석하지 않는다. front glass는 RGB optical centre가 아니다.
**뒷면→RGB optical의 정확한 전방 거리를 케이스 두께만으로 확정하지 않는다.**
최종 RGB pose는 이 mechanical root 후보와 실제 calibrated root→color TF를 합성한다.

```text
T_vehicle_color = T_vehicle_root_trial * T_root_color_runtime
```

따라서 RGB optical 최종 y/z는 runtime TF로 달라질 수 있다. 이는 임의의 32.5mm를
추가하는 것과 다르다. root trial y=0을 유지하며 runtime calibration도 수정하지 않는다.

회전은 사용자 확인(정면·상하/좌우 수평·body 축 정렬)을 근거로 **시험용 RPY=(0,0,0)**.
실측 외부 보정은 아니며 config에 provisional/unverified로 기록한다.
다른 회전을 입력하면 back→root 벡터도 그 회전으로 차량 좌표에 변환한다.
회전뿐 아니라 final root xyz를 측정했다면 x/y/z를 모두 지정해 계산값을 대체할 수 있다.

## TF 소유권과 팔 LOCK

변경 전(각각 분리된 tree):

```text
odom -> base_link -> rear_camera_optical_frame
arm_camera_cam_link -> driver depth/color frames -> optical frames
```

변경 후 기대 구조(실기 검증 필요):

```text
odom -> base_link
        |-- rear_camera_optical_frame                   # 기존 후방 launch, 변경 없음
        `-- arm_camera_cam_link                         # 신규 LOCK static 하나
            |-- arm_camera_depth_frame -> arm_camera_depth_optical_frame
            `-- arm_camera_color_frame -> arm_camera_color_optical_frame
                                                        # 내부는 기존 driver
```

신규 launcher는 `base_link -> arm_camera_cam_link` **하나만** 발행한다.
optical child를 직접 발행하거나 팔 URDF/FK를 가져오지 않는다.
`arm_camera_cam_link`가 `cam_link`의 자식으로도 발행되면 이 모드를 함께 쓰면 안 된다.

팔 제어가 필요할 때는 기존 IK launch 대신 `vpr_lock_arm_control.launch.py`를 쓴다.
이 wrapper는 기존 IK를 그대로 포함하되 그 launch의 `/tf`, `/tf_static`만
`/arm_model/tf`, `/arm_model/tf_static`로 remap한다. 팔 내부 TF 소비자에도 같은 remap이
적용되며, 지연 시작되는 노드에도 유지된다. joint_states/제어 토픽과 URDF/관절값은 바꾸지 않는다.
차량 VPR TF에는 팔 URDF의 동명 base_link가 참여하지 않는다.
다른 팔 launch/TF publisher는 동시에 실행하지 않는다. 이 분리는 실기 제어 회귀 확인이 필요하다.

**이 TF는 지정 LOCK 자세에서만 유효하다.** 팔이 움직이거나 장착이 바뀌면 VPR 검사를
중단하고 다시 LOCK/측정한다. static TF는 실제 팔 움직임을 반영하지 않으므로,
기존 TF 비교 LOCK guard만으로 실제 팔 이동을 감지할 수 없다. 사람이 LOCK 유지 또는
독립적인 관절 상태 감시를 보장해야 한다. 이 변경은 관절 감시를 새로 구현하지 않는다.

## 빌드와 실행

명령은 사용자가 로봇 PC에서 실행한다. 원격 적용이나 모터 구동 시험은 수행하지 않았다.
코드를 평소 방법으로 반영한 뒤, 기존 구동 프로세스가 안전하게 정지한 상태에서 빌드한다.

```bash
cd ~/ResKU
source /opt/ros/humble/setup.bash
cbr --packages-select vision
si
```

각 새 터미널은 정상 DDS 환경과 ROS_DOMAIN_ID=99를 적용한다. `si`만으로 DDS 설정을
보장하지 않는다. 카메라/TF는 장치·child당 한 번만 실행한다.

팔을 조작해야 하면 로봇 PC 별도 터미널에서 아래를 기존 IK launch **대신** 실행한다.
이 명령은 실제 팔 제어를 시작하므로 기존 팔 launch를 먼저 종료한다.

```bash
ros2 launch vision vpr_lock_arm_control.launch.py real_hardware:=true launch_joy:=false
```

노트북의 기존 `remote_joy` 하나로 팔을 LOCK 위치에 둔다. Options로 drive 포커스로
복귀한 뒤 자세를 유지한다. wrapper 자체가 자동 LOCK 이동을 수행하지는 않는다.

로봇 PC의 각각 별도 터미널에서 (이미 켠 driver는 중복 실행하지 않는다):

```bash
ros2 launch vision arm_camera.launch.py serial_no:=_339222071362
```

```bash
ros2 launch vision rear_vpr_trial.launch.py
```

```bash
ros2 launch vision front_d435i_vpr_tf.launch.py
```

front TF launch는 팔/카메라/모터를 시작하지 않는다. 기존 `arm_d435i.yaml`의
publish_tf=true, color/depth/align_depth=true, 640×480×30을 그대로 사용한다.
신규 YAML은 측정 기록이며 `visual_expected_lock_pose_file` 4×4 행렬이 아니다.

Back-to-Base는 기존 확인된 IMU 포트로 실행하며 계속 다음을 유지한다.

```text
visual_apply_correction:=false
enable_landmark_recovery:=false
```

TF가 바뀐 뒤에는 새 mission/Record로 참조를 생성한다. 이전 TF에서 저장한 참조를
재사용하지 않는다. 신규 TF만으로 recognition, yaw 부호, 외부 보정 정확도가 검증되지는 않는다.

## 실기 검증 (한 명령씩, 별도 터미널)

hz/echo는 확인 후 Ctrl+C로 종료한다. device calibrated extrinsic 수치는 아직 수집하지 않았다.

```bash
ros2 topic echo /arm/camera/color/camera_info --once --qos-reliability best_effort
ros2 topic hz /rear/camera/image_raw/compressed
ros2 topic hz /arm/camera/color/image_raw/compressed
ros2 topic hz /arm/camera/aligned_depth_to_color/image_raw
```

```bash
ros2 run tf2_ros tf2_echo base_link rear_camera_optical_frame
ros2 run tf2_ros tf2_echo arm_camera_cam_link arm_camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link arm_camera_cam_link
ros2 run tf2_ros tf2_echo base_link arm_camera_color_optical_frame
```

runtime root→color 결과를 기록해 nominal 내부 extrinsic으로 대체하지 않는다.
root 후보 출력은 aligned 기본값에서 (0.173603,0,0.650674)m, RPY=(0,0,0)이어야 한다.
후방은 기존 (-0.343627,0,0.650674)m, 약(-93°,0°,90°)를 유지한다.

```bash
ros2 run tf2_tools view_frames
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 run visual_landmark vpr_preflight --seconds 5 --output /tmp/vpr_lock_preflight.json
```

PASS: 후방 유지, driver 내부 TF 유지, 차량→root/optical 조회 성공, 모든 카메라가
차량 tree에 연결, child당 parent/authority 하나, 팔 URDF TF 없이 조회 성공,
LOCK 유지, preflight 통과. topic info는 발행자 목록만 보여주므로 이것만으로
child별 유일성을 입증하지 않는다. view_frames 및 /tf_static 메시지의 child/parent도 확인한다.

RViz를 새로 실행하고 Fixed Frame=odom, Add→TF와 Image/Path를 사용한다.
팔 모델을 별도로 보려면 별도 RViz를 `/arm_model/tf*`로 remap하며 차량 tree와 섞지 않는다.

## 아직 실측/실기 확인이 필요한 것

- 후면 중심 측정점과 root/left-imager/RGB 축의 실제 XY 대응, 장착 치수 오차.
- 보드 기반 실제 6DOF 외부 보정과 LOCK 반복 재현성. RPY=0은 사용자 정렬 확인 기반 trial.
- 해당 장치 runtime root→color translation/rotation, 최종 RGB optical 좌표.
- 팔 wrapper의 실제 제어·HOLD 유지·TF 분리 회귀 확인 및 child authority 중복 여부.
- 기존 visual LOCK guard의 한계를 보완할 독립 관절/자세 감시.
- 새 mission에서 특징 매칭/두 프레임 검증, known-offset 방향·크기·반복성.

검사 통과 전 correction/recovery는 계속 비활성화한다.

## 변경 파일과 로컬 검사 결과

- `vision/camera/launch/front_d435i_vpr_tf.launch.py`: 차량→전방 root 한 개.
- `vision/camera/config/front_d435i_vpr_lock_tf.yaml`: 원 실측·기구 offset·확인된 trial 회전·출처.
- `vision/camera/launch/vpr_lock_arm_control.launch.py`: 기존 팔 제어의 TF topic 분리 wrapper.
- `vision/camera/launch/vpr_observation.launch.py`: 전방 기본 config를 D435i로 수정.
- `vision/camera/config/arm_d435i.yaml`: TF 소유권 주석만 수정, 센서 parameter는 동일.
- `vision/camera/config/vpr_mount_measurements.yaml`: 원 측정값 보존, 사용자 확인·trial config 연결.
- `vision/package.xml`: 선택 arm wrapper가 사용하는 bringup 실행 의존성.
- `vision/test/test_front_vpr_tf_launch.py`: 기구 기준/회전/유한값/단일 root/지연노드 TF remap 검사.
- 이 문서, `CAMERA_VPR_TODO.md`, `MOUNTED_CAMERA_SETUP.md`, `VPR_RUN_COMMANDS.md`,
  루트 `MANUAL_RETURN_RUNBOOK.md`: 현재 D435i LOCK 방식과 실행 절차 연결.

로컬 확인: 전방 신규 검사 + 기존 후방 launch 검사 **15개 통과**,
`vision`을 `/tmp` 별도 build/install 경로에서 colcon 빌드 성공,
설치된 두 새 launch의 `--show-args` 성공(하드웨어 시작 없음), Python 문법·diff 공백 검사 통과.
후방 launch/기구 TF, RealSense 내부 TF, 팔 URDF/SRDF/제어 코드, 모터 watchdog은 변경하지 않았다.
실기 입력 Hz·TF·제어 회귀는 사용자 PC에서 위 명령으로 확인해야 한다.
