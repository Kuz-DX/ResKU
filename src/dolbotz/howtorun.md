# dolbotz 실행 방법


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
cd /home/j/dolbotZ/src/dolbotz
python3 -m pytest test/ -v
```

## 5. config/ 디렉토리

`config/models/`에는 계절별 미션에서 사용하는 모델 가중치를 보관한다. 경로는 `dolbotz.utils.paths.get_models_dir()`로 해석하며, 모델별 상세 내용은 `config/models/README.md`를 참고한다.
