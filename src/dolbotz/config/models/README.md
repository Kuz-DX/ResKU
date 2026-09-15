# config/models/

리포 전체에서 쓰는 학습된 모델 가중치를 한곳에 모아둔다. 예전에는 리포
루트와 `runs/segment/`에 흩어져 있었고, `arm_pickup.py`의 `model_path`
기본값이 절대경로로 하드코딩되어 있다가 사용자/머신이 바뀌면서 깨진 전례가
있다(`/home/j/dolbotZ/...` -> `/home/jecs/dolbotZ/...`, 커밋 `6056cf8`).
이제 모든 노드가 `dolbotz.utils.paths.get_models_dir()`로 이 디렉토리를
실행 환경과 무관하게 찾는다.

## 모델 최적화 기록

## ifofv1_int8_openvino_model / vision_makerv2_int8_openvino_model (2026-09-02)

`spring_ifof_node`(`ifofv1.pt`)와 `fall_marker_node`(`vision_makerv2.pt`)의
`model_path` 기본값을 각각 OpenVINO INT8 변환본으로 교체했다 —
두 모델 모두 원본이 학습에 쓴
`/home/j/vision_marker/ifof/merged/data.yaml`(987장)로 캘리브레이션했다.

2026-09-03부터 `fall_marker_node`는 후속 가중치
`vision_marker.v3_int8_openvino_model`를 기본으로 사용하며, v2 INT8
모델은 롤백과 비교
검증을 위해 보존한다. `spring_ifof_node`는 여전히 INT8 변환본이 기본이다.

**ONNX INT8도 시도했으나 폐기함** — 처음엔 (.onnx 형식으로) 전 모델
양자화를 요청받아 `ultralytics`의 ONNX INT8 export(정적, 캘리브레이션
포함)로 4개 모델(`ifofv1`, `vision_makerv2`,
`trafficlightv1`, `supplyboxv3`)을 다 만들어봤는데, 이 CPU(i7-13650HX)
기준 ONNX Runtime의 기본 CPU 실행 공급자는 OpenVINO만큼 INT8 커널이
최적화돼 있지 않아서 **OpenVINO INT8보다 5~8배 느렸다**(예:
`ifofv1_int8.onnx` 17.6ms vs `ifofv1_int8_openvino_model` 2.86ms — 심지어
원본 FP32 `.pt`(15.52ms)보다도 느림, ONNX 파일은 삭제함, 재시도 전에 이
결과부터 참고할 것). 그래서 이 프로젝트는 계속 OpenVINO IR을 표준
포맷으로 쓴다.

merged/data.yaml 전체(987장) 검증 결과(`imgsz=320`, `batch=1`, CPU):

| 모델 | 지표 | FP32(.pt) | INT8(OpenVINO) |
|---|---|---|---|
| ifofv1 | mAP50 / mAP50-95 / recall | 0.98735 / 0.85763 / 0.96887 | 0.98253 / 0.83808 / 0.96804 |
| ifofv1 | 추론시간(i7-13650HX) | 15.52ms | 2.86ms(약 5.4배) |
| vision_makerv2 | mAP50 / mAP50-95 / recall | 0.98638 / 0.83968 / 0.96898 | 0.98282 / 0.80252 / 0.97140 |
| vision_makerv2 | 추론시간(i7-13650HX) | 25.31ms | 3.01ms(약 8.4배) |

양자화 모델에서 공통으로 관찰되는 경향처럼 mAP50/recall은 거의 유지되고 mAP50-95가
좀 더 뚜렷하게 떨어진다(IoU 임계값이 빡빡할수록 양자화 영향을 더 받음).
`spring_ifof.py`/`fall_marker.py` 둘 다 좌/우 카메라를 `ThreadPoolExecutor`로
병렬 처리하는 구조라, 프레임당 추론시간이 줄면 좌/우 지연도 같이 준다.

예전 모델로 되돌리려면 `IFOF_MODEL_RELATIVE_PATH`/
`MARKER_MODEL_RELATIVE_PATH`를 보관된 v1/v2 모델 경로로 바꾸거나 launch
인자로 해당 경로를 오버라이드한다.

## vision_marker.v3_int8_openvino_model (2026-09-03~)

`vision_marker.v3.pt`를 다른 CPU 배포 모델과 같은 조건인 OpenVINO INT8,
`imgsz=320`, `batch=1`, 정적 입력으로 변환했다. 캘리브레이션에는
`/home/j/vision_marker/ifof/merged/data.yaml` validation 987장을 사용했다.
`fall_marker_node`의 기본 모델은 이 INT8 변환본이며, 원본 `.pt`는
롤백과 재변환을 위해 보존한다.

merged validation 987장 검증 결과(`imgsz=320`, `batch=1`, CPU):

| 지표 | FP32(.pt) | INT8(OpenVINO) |
|---|---:|---:|
| mAP50 | 0.98712 | 0.98168 |
| mAP50-95 | 0.84951 | 0.82807 |
| recall | 0.98016 | 0.96383 |
| 추론시간 | 21.67ms | 4.54ms(약 4.8배 향상) |

## supplyboxv3_int8_openvino_model (2026-09-03~)

`summer_supply_node`/`drive_supply_detector_node`(`_DEFAULT_MODEL_FILE`,
공유)의 기본값을 `supplyboxv3.pt`에서 OpenVINO INT8로 교체했다 —
spring_ifof.py/fall_marker.py에 적용한 것과 동일 절차. `supplyboxv3.pt`
자체의 학습 데이터셋(`/home/s/trafficlightyolo/datasets/supplybox-combined/`)은
팀원 "s"님 컴퓨터에만 있어서 접근 못 했지만, 같은 도메인의 대체
데이터셋으로 캘리브레이션/검증했다: Roboflow `supplybox-l57zm` project
version 2(전용 다운로드 스크립트 `/home/j/vision_marker/supplybox/
supplybox.py`, 단일 클래스 `{0: 'supplybox'}` — `supplyboxv3.pt`와 일치,
833장 중 valid 158장으로 검증). 원본 학습셋과 완전히 같지는 않다는 점은
감안할 것.

valid 158장 검증(`imgsz=320`, `batch=1`, CPU) 결과:

| 지표 | FP32(.pt) | INT8(OpenVINO) |
|---|---|---|
| mAP50 | 0.99053 | 0.98789 |
| mAP50-95 | 0.97792 | 0.95347 |
| recall | 1.00000 | 0.99363 |
| 추론시간(i7-13650HX) | 15.42ms | 2.43ms(약 6.3배) |

다른 모델들과 같은 경향(mAP50/recall 거의 유지, mAP50-95가 더 뚜렷하게
하락). 이전 FP32로 되돌리려면 `summer_supply.py`/`drive_supply_detector.py`
각각의 `_DEFAULT_MODEL_FILE` 선언에서 주석 처리해둔 `.pt` 줄로 바꾸면 된다.

## robodog_test_int8_openvino_model (2026-09-03~)

`escort_follow_node`(`TARGET_MODEL_RELATIVE_PATH`)의 기본값을
`robodog_test.pt`에서 OpenVINO INT8로 교체했다 — 나머지와 동일 절차.
캘리브레이션/검증 데이터는 `/home/j/vision_marker/unitree/unitree_go2-4`
(Roboflow `unitree_go2` project v4, 단일 클래스 `{0: 'go2'}` —
`robodog_test.pt`와 일치, 1296 train / 151 valid / 32 test).

[ONNX 관련 참고, 2026-09-03] 이 모델도 ONNX(순정 `onnxruntime` CPU EP)로
바꿔보자는 요청이 있었으나, jecs가 GPU 없는 Intel i7 온보드 PC라는 게
확인되면서 순정 ONNX Runtime
대신 Intel 자체 최적화 엔진인 OpenVINO를 계속 쓰기로 했다 — 위 "ONNX
INT8도 시도했으나 폐기함" 절의 실측과 같은 결론. `onnxruntime-openvino`
(ONNX 포맷 + 내부적으로 OpenVINO 엔진 사용) 하이브리드도 검토했으나,
순정 OpenVINO IR 대비 이점이 없어 보류.

valid 151장 검증(`imgsz=320`, `batch=1`, CPU) 결과:

| 지표 | FP32(.pt) | INT8(OpenVINO) |
|---|---|---|
| mAP50 | 0.88201 | 0.87542 |
| mAP50-95 | 0.82646 | 0.80258 |
| precision | 0.97232 | 0.99163(상승) |
| recall | 0.86441 | 0.81921 |
| 추론시간(i7-13650HX) | 12.68ms | 2.97ms(약 4.3배) |

**주의**: 다른 3개(ifofv1/vision_makerv2/supplyboxv3)와 달리 이 모델은
**recall이 약 4.5%p로 더 뚜렷하게 하락**했다(86.4%->81.9%) — precision은
오히려 올라서(놓치는 대신 오탐은 줄어드는 방향) 전체적인 위험도가 아주
크지는 않지만, escort_follow 실기 운용 중 LOST/재포착이 예전보다 잦아지면
이 recall 하락을 원인으로 우선 의심할 것. 이전 FP32로 되돌리려면
`escort_follow.py`의 `TARGET_MODEL_RELATIVE_PATH` 선언에서 주석 처리해둔
`.pt` 줄로 바꾸면 된다.

## trafficlightv1_int8_openvino_model (2026-09-03~)

`summer_traffic_node`(`TRAFFIC_MODEL_RELATIVE_PATH`)의 기본값을
`trafficlightv1.pt`에서 OpenVINO INT8로 교체했다 — 나머지와 동일 절차.
`trafficlightv1.pt` 자체의 학습 데이터셋(`/home/s/trafficlightyolo/
datasets/...`)은 팀원 "s"님 컴퓨터에만 있어서 접근 못 했지만, Roboflow
`traffic-light-detection-hznds` project v1(전용 다운로드 스크립트
`/home/j/vision_marker/trafficlight/trafficlight.py`)을 찾아서 대신
썼다 — project slug가 원본 학습 args(`.../traffic-light-detection-hznds-v1-yolo26/`)와
정확히 일치하고 클래스도 `{0: 'green', 1: 'red', 2: 'yellow'}`로 완전히
같아서, 사실상 원본 그 자체이거나 매우 가까운 데이터셋으로 보인다
(2097 train / 200 valid / 67 test).

valid 200장 검증(`imgsz=320`, `batch=1`, CPU) 결과:

| 지표 | FP32(.pt) | INT8(OpenVINO) |
|---|---|---|
| mAP50 | 0.96648 | 0.94970 |
| mAP50-95 | 0.60717 | 0.48652 |
| precision | 0.89059 | 0.91506(상승) |
| recall | 0.95432 | 0.88255 |
| 추론시간(i7-13650HX) | 22.69ms | 2.85ms(약 8배) |

**다른 모델보다 mAP/recall 하락 폭이 뚜렷하다**(mAP50-95 약 12pt, recall
약 7pt) — 그래서 다른 모델 검토 때처럼 실제 판정 로직
(`pick_best_state`, `conf_threshold=0.8`, red→stop/green→go, yellow는
애초에 무시)을 그대로 재현해서 valid 200장 전부 FP32 vs INT8로 프레임
단위 직접 대조했다: **stop↔go가 실제로 뒤바뀐 케이스는 0건**. 차이 나는
36장 중 32장은 FP32가 확신하던 걸 INT8이 `conf_threshold`(0.8)를 못
넘겨서 `unknown`으로 더 보수적으로 판정한 것뿐(반대 방향 4장 포함) —
색 자체를 잘못 본 적은 없다. 이전 FP32로 되돌리려면
`summer_traffic.py`의 `TRAFFIC_MODEL_RELATIVE_PATH` 선언에서 주석
처리해둔 `.pt` 줄로 바꾸면 된다.

참고: `summer_traffic_node`는 [2026-09-02] `/arm/picking_command` 수신
전까지 이 모델을 아예 로드하지 않는 지연 로드 설계다
(`summer-traffic-deferred-model-load` 메모 참고) — 노드 생성 직후
`self._model is None`인 건 정상이며 버그가 아니다.

## supplyboxv3.pt (2026-08-30~)

`summer_supply.py`(`arm_pickup_node`)의 `detector_backend='yolo'` 기본
가중치를 `supplybest.pt`에서 이걸로 교체했다 — 원본은 `~/Desktop/supplyboxv3.pt`
(YOLO26n, `ultralytics==8.4.101`로 로드 확인, 단일 클래스 `{0: 'supplybox'}` —
`target_class` 기본값 `'supplybox'`와 일치하므로 코드 변경 없이 드롭인
교체됨). `_infer_yolo()`가 클래스 id를 아예 안 보고 conf/bbox만 쓰므로
다중 클래스였어도 동작은 했겠지만, 이 모델은 애초에 단일 클래스로 확인됨.

`supplybest.pt`는 지우지 않고 그대로 뒀다 — `detector_backend='yolo'`
기본값에서는 더 이상 안 쓰이지만, `army_manipulator_bringup`(별도 패키지)의
`target_detector_node.py`가 참조하는 `supplybest_openvino_model/` 변환본의
원본 가중치라 여기서 지우면 그쪽 재현이 안 된다(`config/models/README.md`
위쪽 이동 내역 표 참고 — 이 파일은 dolbotz 패키지 것이고
army_manipulator_bringup은 자기 `config/models/`를 따로 갖고 있어 서로
독립이지만, 최초 유래가 같은 가중치라 여기 기록해둔다).

`ultralytics.YOLO()` 로 로드만 확인했고, 실기(D455)에서 검출률/오탐률/FPS
검증은 아직 안 함 — `summer_supply.py`의 TODO(검출기 비교/최종 선택) 참고.
