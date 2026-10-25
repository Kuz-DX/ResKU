# jecs: 실시간 카메라 실행 및 로그 생성

모든 명령은 로봇 PC `jecs@192.168.0.100`에서 실행한다. 입력은 rosbag이 아니라
실시간 카메라 네 대다. 각 성능 조건은 한 번에 하나만 실행한다.

## 0. 준비

노트북에서 jecs에 접속한다.

```bash
ssh jecs@192.168.0.100
cd ~/ResKU
git pull --ff-only
colcon build --packages-select vision --symlink-install
source install/setup.bash
```

터미널 런처에서 기존 `Person_Detect`를 끄고 Drive_CAM, Arm_CAM, Side_CAM을 켠다.
테스트 노드와 기존 사람 탐지 노드를 동시에 실행하면 CPU 측정이 왜곡된다.

기본 입력 토픽을 확인한다.

```bash
ros2 topic hz /drive/camera/color/image_raw/compressed
ros2 topic hz /arm/camera/color/image_raw/compressed
ros2 topic hz /side/left/image_raw/compressed
ros2 topic hz /side/right/image_raw/compressed
```

토픽이 다르면 먼저 다음 명령으로 실제 이름을 찾는다.

```bash
ros2 topic list | grep compressed
```

`vision cameras.launch.py`를 쓴 경우 side 토픽이 `/left/camera/...`와
`/right/camera/...`일 수 있다. 이때 모든 노드 실행 명령에는 다음을 추가한다.

```bash
left_input_topic:=/left/camera/image_raw/compressed \
right_input_topic:=/right/camera/image_raw/compressed
```

## 1. 선택: 480×640 OpenVINO 모델 export

640×480 카메라와 같은 비율의 정적 입력을 사용하면 정사각 640×640 padding과
불필요한 연산을 줄일 수 있다.

FP32 기본 export:

```bash
python3 src/vision/cputest/export_person_ov.py \
  --pt /absolute/model/person.pt \
  --imgsz 480,640 \
  --out /absolute/model/person_480x640_openvino_model
```

FP16 export:

```bash
python3 src/vision/cputest/export_person_ov.py \
  --pt /absolute/model/person.pt \
  --imgsz 480,640 --half \
  --out /absolute/model/person_480x640_fp16_openvino_model
```

INT8 export는 calibration dataset YAML이 필수다.

```bash
python3 src/vision/cputest/export_person_ov.py \
  --pt /absolute/model/person.pt \
  --imgsz 480,640 --int8 \
  --data /absolute/data/person.yaml \
  --out /absolute/model/person_480x640_int8_openvino_model
```

완료 로그에서 입력 shape `1x3x480x640`, 출력 shape `[1,N,6]`, precision을
확인한다. 기존 폴더는 자동으로 덮어쓰지 않는다.

## 2. 라이브 결과 일치 확인

사람이 네 카메라 화면 안에서 위치와 거리를 바꿔가며 움직이는 동안 실행한다.
기본 reference는 같은 OpenVINO 모델을 Ultralytics로 실행한 결과다. 따라서 export
수치 차이가 아니라 직접 구현한 전처리와 후처리가 맞는지를 확인한다.

```bash
python3 src/vision/cputest/compare_backends.py \
  --live --ref ov_ultralytics \
  --ov /absolute/model/person_openvino_model \
  --conf 0.5 --imgsz 640 \
  --max-frames 200 --sample-interval-ms 500 --capture-timeout 120 \
  --out ~/perf_logs/backend_check_live
```

`PASS`를 확인한 뒤 성능 측정으로 넘어간다. 불일치 이미지는 결과 폴더에 최대
50장 저장된다.

PT를 reference로 비교해야 할 때만 다음처럼 실행한다.

```bash
python3 src/vision/cputest/compare_backends.py \
  --live --ref pt \
  --pt /absolute/model/person.pt \
  --ov /absolute/model/person_openvino_model \
  --max-frames 200 --sample-interval-ms 500 --capture-timeout 120 \
  --out ~/perf_logs/backend_check_pt
```

PT의 640×480 직사각 처리와 정사각 640×640 OpenVINO padding 조건이 다르면 결과가
달라질 수 있다. 정확한 직접 구현 검증에는 `--ref ov_ultralytics`를 사용한다.

## 3. 성능 측정

각 실행을 3~5분 유지하고 `Ctrl+C`로 끝낸다. 모든 실행에서 카메라와 장면 조건을
최대한 동일하게 유지하고 `latency_mode:=stamp`를 사용한다. 촬영 stamp가 호스트
시계와 맞지 않아 latency가 비정상적일 때만 `receive`로 바꾼다.

아래 `/absolute/model/...`은 실제 경로로 바꾼다. V1과 V2를 비교한 후 더 나은
`perf_hint`를 S1~S8에 동일하게 사용한다.

### V0 — Ultralytics 기준선

변경점: 비교 기준인 기존 callback 내부 PT 추론.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=ultralytics model_path:=/absolute/model/person.pt \
  separated:=false scheduler:=fifo async_infer:=false num_requests:=1 \
  latency_mode:=stamp run_tag:=V0
```

V0를 30초 이상 실행한 뒤 표의 `drive`, `arm`, `left`, `right` 행을 확인한다.
워밍업이 끝난 각 5초 구간에서 네 카메라의 `proc_fps`가 모두 0보다 커야 하며,
특정 카메라의 `max_gap`만 구간마다 계속 증가하면 안 된다. 기존 노드의 동일 조건
측정값은 다음과 같다.

| camera | 기존 `proc_fps` | 전체 처리량 중 비율 | drive 대비 비율 |
|---|---:|---:|---:|
| drive | 3.7 | 30.1% | 1.00 |
| arm | 2.9 | 23.6% | 0.78 |
| left | 2.8 | 22.8% | 0.76 |
| right | 2.9 | 23.6% | 0.78 |

절대 FPS는 장면과 CPU 상태에 따라 달라질 수 있으므로, 합계보다는 네 카메라가 모두
처리되는지와 처리량 비율을 비교한다. V0 결과도 대략 drive 30%, 나머지 각 23% 안팎이면
기존 노드와 비슷한 분배다. 한 번의 5초 표보다 여러 구간에서 이 경향이 유지되는지
확인한다. CSV 요약은 측정을 끝낸 뒤 다음 명령으로 만든다.

```bash
LATEST_V0=$(ls -1t ~/perf_logs/*_V0.csv | head -n 1)
python3 src/vision/cputest/plot_perf.py "$LATEST_V0" \
  --out ~/perf_logs/V0_check.png --summary
```

출력된 카메라별 `proc_fps` 평균이 모두 0보다 큰지 확인하고, 위 기준값과 비교한다.

### V1 — OpenVINO LATENCY

V0 대비 변경점: backend를 OpenVINO LATENCY로 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=LATENCY separated:=false scheduler:=fifo \
  async_infer:=false num_requests:=1 latency_mode:=stamp run_tag:=V1
```

### V2 — OpenVINO THROUGHPUT

V1 대비 변경점: `perf_hint`만 THROUGHPUT으로 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=false scheduler:=fifo \
  async_infer:=false num_requests:=1 latency_mode:=stamp run_tag:=V2
```

### S1 — callback/worker 분리

선택한 V1/V2 대비 변경점: `separated:=true`만 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=fifo \
  async_infer:=false num_requests:=1 latency_mode:=stamp run_tag:=S1_sep
```

### S2 — round-robin 동기

S1 대비 변경점: scheduler만 roundrobin으로 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=roundrobin \
  async_infer:=false num_requests:=1 latency_mode:=stamp run_tag:=S2_rr
```

### S3 — round-robin 비동기 요청 1개

S2 대비 변경점: `async_infer:=true`만 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=roundrobin \
  async_infer:=true num_requests:=1 latency_mode:=stamp run_tag:=S3_async
```

### S4 — 비동기 요청 2개

S3 대비 변경점: `num_requests`만 2로 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=roundrobin \
  async_infer:=true num_requests:=2 latency_mode:=stamp run_tag:=S4_r2
```

### S5 — 비동기 요청 4개

S4 대비 변경점: `num_requests`만 4로 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=roundrobin \
  async_infer:=true num_requests:=4 latency_mode:=stamp run_tag:=S5_r4
```

### S6 — priority 비동기

S4/S5 중 더 나은 요청 수 조건 대비 변경점: scheduler만 priority로 변경. 아래
예시는 요청 수 2를 선택한 경우다.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=priority \
  async_infer:=true num_requests:=2 latency_mode:=stamp \
  priority_pattern:="['drive','arm','drive','left','right']" \
  max_wait_ms:=300 stale_input_ms:=1000 run_tag:=S6_prio
```

### S7 — 시각화 끄기

S6 대비 변경점: `publish_visualization:=false`만 변경.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=priority \
  async_infer:=true num_requests:=2 latency_mode:=stamp \
  publish_visualization:=false run_tag:=S7_novis
```

### S8 — 실제 RTSP 부하

최종 후보 대비 변경점: RTSP 브리지를 켠 상태로 10분 실행.

```bash
ros2 launch vision person_detection_ov.launch.py \
  backend:=openvino model_path:=/absolute/model/person_openvino_model \
  perf_hint:=THROUGHPUT separated:=true scheduler:=priority \
  async_infer:=true num_requests:=2 latency_mode:=stamp run_tag:=S8_rtsp
```

480×640 모델을 만들었으면 V1/V2와 같은 조건으로 모델 경로만 바꿔
`run_tag:=V1_480`, `run_tag:=V2_480`을 추가 측정한다.

## 4. 로그 위치

각 실행은 다음 위치에 CSV와 `_meta.json`을 만든다.

```text
~/perf_logs
```

노트북으로 옮기기 전에 파일명에 지정한 `run_tag`가 포함됐는지 확인한다.

## 5. 터미널 표와 판단 기준

- `cam_fps`: 카운트 전용 구독에서 본 실제 카메라 입력 FPS
- `in_fps`: 처리 callback에 실제 도착한 FPS
- `proc_fps`: 추론과 발행까지 완료한 FPS
- `lost`: 같은 구간의 `cam_fps` 프레임 수에서 처리 완료 수를 뺀 값
- `drop`: 최신 입력 슬롯이 덮어써졌거나 너무 오래돼 폐기된 수
- `stale_drop`: 완료 순서 역전 또는 발행 대기로 버린 과거 결과
- `req_wait`: 빈 OpenVINO inference request를 기다린 시간
- `queue_wait`: 입력 슬롯 저장부터 scheduler 선택까지 걸린 시간
- `latency`: 촬영 stamp부터 발행 완료까지 걸린 시간
- `max_gap`: 연속 발행 사이 또는 마지막 발행 이후의 최대 공백
- `err`: decode, inference, encode 또는 publish 실패 수

행 앞의 `!`는 `proc_fps < cam_fps × 0.5`라는 뜻이다.

판단 우선순위:

1. 결과 일치가 PASS가 아닌 설정은 성능이 좋아도 제외한다.
2. 카메라별 `proc_fps`가 `cam_fps`를 얼마나 따라가는지 확인한다.
3. latency p95와 max_gap이 낮고 안정적인 설정을 선택한다.
4. 처리 성능이 같다면 CPU%와 RSS가 낮은 쪽을 선택한다.

CPU%는 이 노드 프로세스 기준이며 100%가 논리 CPU 코어 한 개 사용량이다. 최대는
`논리 코어 수 × 100%`다.
