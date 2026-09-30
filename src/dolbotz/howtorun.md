# dolbotz 실행 방법

## Supply box 인식 (vision 패키지)

팔 카메라 RGB·정렬 Depth·CameraInfo와 팔 TF를 실행한 후:

```bash
ros2 launch vision supply.launch.py
```

구현은 `src/vision/vision/supply.py`이며, 모델은 vision 패키지의
`models/supplyboxv3_int8_openvino_model`을 사용한다.
카메라 드라이버와 팔 제어기는 이 launch가 실행하지 않는다.
기존 미션 launch가 supply 노드를 실행 중이면 중복 실행하지 않는다.

- 입력: `/arm/camera/color/image_raw/compressed`,
  `/arm/camera/aligned_depth_to_color/image_raw/compressedDepth`,
  `/arm/camera/color/camera_info`
- UI bbox: `/arm/supply/detections` (`vision_msgs/msg/Detection2DArray`)
- 3D 목표: `/arm/target_point`, 카메라 거리: `/arm/target_depth_m`
- 디버그 JPEG: `/arm/debug_image/compressed`

DolBot_Center의 로봇팔 카메라 패널은 원본 영상 위에 supply bbox를 표시한다.
MediaMTX와 compressed 모드 모두 rosbridge 연결이 필요하다.
Bbox는 Depth 값/TF/파지 범위 필터 적용 전에 발행한다. 다만 기존 처리 흐름대로
RGB와 Depth가 동기화되고 CameraInfo가 수신되어야 추론이 시작된다.
목표 발행은 중심 11×11 Depth 중앙값, 최대 1m, base X 절댓값 0.32m 이하
3프레임 확인 조건을 유지한다. `/arm/picking_command` 수신 후 추론은 종료된다.


https://github.com/sw-works-log/manual100_combined.git
https://github.com/harim-54/only_manual.git



## 2. 카메라 드라이버 실행

실행할 미션 노드가 구독하는 토픽에 맞게 필요한 스트림만 켜서 실행합니다.

```bash
# 컬러만 필요할 때
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=false \
  -p rgb_camera.profile:=640x480x30

# 뎁스만 필요할 때
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=false -p enable_depth:=true
```

## 3. 노드 실행

### arm_pickup (YOLO로 박스 탐지 → 3D 좌표 퍼블리시)

```bash
ros2 run dolbotz arm_pickup --ros-args \
  -p target_class:=supply_box
```

구독: `/camera/camera/color/image_raw/compressed`,
`/camera/camera/aligned_depth_to_color/image_raw/compressedDepth`,
`/camera/camera/color/camera_info`

카메라 이미지 입력은 RGB의 `compressed`와 Depth의 `compressedDepth`만 사용한다.
프로젝트 노드가 raw 카메라 이미지 토픽을 구독하지 않으므로, 연산 노드가 원격
컴퓨터에 있어도 비압축 카메라 프레임이 해당 노드로 전송되지 않는다.

`model_path` 기본값은 `dolbotz.utils.paths.get_models_dir()` 기준
`config/models/supplybest.pt`다(cwd/사용자 홈 경로와 무관하게 해석됨).
다른 가중치를 쓰려면 `-p model_path:=/abs/path`로
오버라이드하세요. `ultralytics`가 설치되어 있지 않으면 탐지 기능이 비활성화됩니다.

## 4. 테스트 실행

```bash
source /opt/ros/humble/setup.bash
cd /home/shu/ResKU/src/dolbotz
python3 -m pytest test/ -v
```

## 5. config/ 디렉토리

`config/models/`에는 계절별 미션에서 사용하는 모델 가중치를 보관한다. 경로는 `dolbotz.utils.paths.get_models_dir()`로 해석하며, 모델별 상세 내용은 `config/models/README.md`를 참고한다.
