# 사람 탐지 OpenVINO 실시간 카메라 시험

이 절차는 현재 실행 중인 카메라의 `sensor_msgs/msg/CompressedImage` 토픽을
직접 구독한다. 기존 `person_detection` 노드는 CPU 경쟁과 중복 발행을 피하기
위해 동시에 실행하지 않는다.

## 0. 빌드와 모델 준비

```bash
cd ~/ResKU
colcon build --packages-select vision --symlink-install
source install/setup.bash
```

동일한 원본에서 나온 PT 모델과 OpenVINO 모델을 준비한다. OpenVINO 모델은 정적
640×640, end-to-end NMS 출력 `[1,N,6]`이어야 하며 폴더에는 XML/BIN과 가능하면
`metadata.yaml`이 있어야 한다.

```text
/absolute/model/person.pt
/absolute/model/person_openvino_model/
```

## 1. 실시간 카메라 실행과 토픽 확인

프로젝트에서 평소 사용하는 Drive_CAM, Arm_CAM, Side_CAM을 먼저 실행한다. 기본
탐지 입력은 다음 네 토픽이다.

```text
/drive/camera/color/image_raw/compressed
/arm/camera/color/image_raw/compressed
/side/left/image_raw/compressed
/side/right/image_raw/compressed
```

네 토픽이 실제로 발행되는지 확인한다.

```bash
ros2 topic hz /drive/camera/color/image_raw/compressed
ros2 topic hz /arm/camera/color/image_raw/compressed
ros2 topic hz /side/left/image_raw/compressed
ros2 topic hz /side/right/image_raw/compressed
```

`vision cameras.launch.py`를 사용하는 경우 side camera 토픽은
`/left/camera/image_raw/compressed`, `/right/camera/image_raw/compressed`일 수 있다.
그 경우 아래 모든 실행 명령에 다음 두 인자를 추가한다.

```bash
left_input_topic:=/left/camera/image_raw/compressed \
right_input_topic:=/right/camera/image_raw/compressed
```

현재 토픽 이름은 다음 명령으로 찾을 수 있다.

```bash
ros2 topic list | grep compressed
```

## 2. 실제 라이브 프레임으로 결과 일치 확인

카메라를 켠 상태에서 `--live`를 사용하면 압축 프레임을 메모리에 먼저 모은 뒤,
각각의 동일 프레임을 PT와 OpenVINO에 순차 입력한다. 따라서 두 backend가 서로
다른 시각의 영상을 비교하지 않는다.

```bash
cd ~/ResKU
source install/setup.bash
python3 src/vision/cputest/compare_backends.py \
  --live \
  --pt /absolute/model/person.pt \
  --ov /absolute/model/person_openvino_model \
  --conf 0.5 --imgsz 640 \
  --max-frames 400 --capture-timeout 60 \
  --out ~/perf_logs/backend_check_live
```

side 토픽이 `/left/camera`, `/right/camera` 계열이면 다음을 추가한다.

```bash
--left-topic /left/camera/image_raw/compressed \
--right-topic /right/camera/image_raw/compressed
```

박스 개수 일치율 99% 이상, 매칭 IoU 평균 0.99 이상일 때만 `PASS`다. 불일치
이미지는 최대 50장, 전체 결과는 `summary.json`에 저장된다. INT8 모델은 양자화
오차가 더 클 수 있으므로 결과의 `openvino_precision`도 함께 확인한다. PASS 전에
성능 시험으로 넘어가지 않는다.

## 3. 실시간 성능 측정 공통 규칙

카메라 네 대를 계속 실행한 상태에서 아래 조건을 하나씩 3~5분 실행하고
`Ctrl+C`로 끝낸다. 움직임과 조명이 조건마다 크게 달라지지 않도록 같은 장면에서
측정한다. CSV와 meta JSON은 기본적으로 `~/perf_logs`에 저장된다.

라이브 측정에서는 촬영부터 발행까지의 지연을 보기 위해 우선
`latency_mode:=stamp`를 사용한다. 카메라 stamp와 호스트 시스템 시계가 같은
기준이 아니어서 latency가 비정상적으로 크면 `latency_mode:=receive`로 바꾼다.

## 4. Backend 및 OpenVINO hint 비교

V0 — Ultralytics 기준선:

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=ultralytics model_path:=/absolute/model/person.pt \
  num_requests:=1 scheduler:=fifo async_infer:=false \
  latency_mode:=stamp run_tag:=live_V0
```

V1 — OpenVINO LATENCY:

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=LATENCY num_requests:=1 scheduler:=fifo async_infer:=false \
  latency_mode:=stamp run_tag:=live_V1
```

V2 — OpenVINO THROUGHPUT:

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT num_requests:=1 scheduler:=fifo async_infer:=false \
  latency_mode:=stamp run_tag:=live_V2
```

## 5. 동시 요청 수 비교

4단계에서 더 나았던 hint를 사용하고 요청 수 1, 2, 4를 각각 측정한다.

```bash
ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=1 scheduler:=roundrobin async_infer:=true latency_mode:=stamp run_tag:=live_R1

ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=2 scheduler:=roundrobin async_infer:=true latency_mode:=stamp run_tag:=live_R2

ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=4 scheduler:=roundrobin async_infer:=true latency_mode:=stamp run_tag:=live_R4
```

`async_infer:=false`는 분리 워커에서 요청을 한 개씩 동기 처리한다. 실제 동시 요청
수를 비교할 때는 `async_infer:=true`를 사용한다.

## 6. 스케줄링 정책 비교

5단계에서 선택한 요청 수를 아래 `2` 자리에 넣는다.

```bash
ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=2 scheduler:=roundrobin async_infer:=false latency_mode:=stamp run_tag:=live_rr_sync

ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=2 scheduler:=roundrobin async_infer:=true latency_mode:=stamp run_tag:=live_rr_async

ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=2 scheduler:=priority async_infer:=true latency_mode:=stamp \
  priority_pattern:="['drive','arm','drive','left','right']" \
  max_wait_ms:=300 stale_input_ms:=1000 run_tag:=live_priority_async
```

## 7. 카메라 일부만 사용할 때

Drive만 사용:

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=LATENCY num_requests:=1 scheduler:=fifo async_infer:=false \
  enable_arm_camera:=false enable_side_cameras:=false \
  latency_mode:=stamp run_tag:=live_drive_only
```

Drive와 arm만 사용:

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=LATENCY num_requests:=1 scheduler:=roundrobin async_infer:=false \
  enable_arm_camera:=true enable_side_cameras:=false \
  latency_mode:=stamp run_tag:=live_drive_arm
```

## 8. RTSP를 함께 사용하는 실제 부하 시험

카메라와 RTSP 브리지를 모두 켠 상태에서 최종 후보 조건을 10분 실행한다.

```bash
ros2 launch vision person_detection_ov.launch.py backend:=openvino \
  model_path:=/absolute/model/person_openvino_model perf_hint:=THROUGHPUT \
  num_requests:=2 scheduler:=priority async_infer:=true \
  latency_mode:=stamp run_tag:=live_rtsp_10m
```

RTSP를 끈 상태와 켠 상태를 각각 측정해 CPU, `proc_fps`, latency p95,
`draw_enc`, drop 및 max gap 변화를 비교한다.

## 9. 그래프 생성

```bash
cd ~/ResKU
python3 src/vision/cputest/plot_perf.py '~/perf_logs/*live*.csv' \
  --output ~/perf_logs/person_detection_live_comparison.png
```

PNG에는 카메라별 처리 FPS, latency p95, 단계별 누적 시간, CPU/RSS 추이,
카메라별 max gap이 들어간다. 종료 직전 `period_s`가 아주 짧은 마지막 CSV 행은
평균을 왜곡할 수 있으므로 필요하면 제외한다.

## 10. 결과 해석 핵심

- `proc_fps`가 `in_fps`에 가까울수록 입력을 잘 따라간다.
- `drop`은 최신 슬롯 덮어쓰기 또는 오래된 입력 폐기 횟수다.
- `stale_drop`은 비동기 완료 순서가 뒤집혀 폐기된 과거 결과다.
- `queue_wait`가 크면 추론 이전부터 처리 용량이 부족한 상태다.
- `infer` p95와 latency p95가 평균보다 크게 튀면 순간 지연이 불안정하다.
- `max_gap`이 큰 카메라는 화면 갱신이 간헐적으로 끊긴다.
- `err`는 정상 시험에서 0이어야 한다.
- 행 앞의 `!`는 `proc_fps`가 `in_fps`의 절반보다 낮다는 의미다.
- 요청 수를 늘렸는데 FPS가 거의 그대로이고 CPU/RSS와 latency만 증가하면 이전
  요청 수가 더 좋은 설정이다.
- priority에서는 drive 지연 개선뿐 아니라 arm/left/right의 drop과 max gap이
  과도하게 증가하지 않는지도 확인한다.
