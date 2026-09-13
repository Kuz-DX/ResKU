"""봄 미션 — 피아식별(IFOF, Identify Friend or Foe) 노드.

[예외 케이스 안내] 팀 전체 컨벤션은 "인식/판정 결과 발행까지만 담당하고
하드웨어 제어는 별도 담당자"이지만, 이 봄 미션(피아식별)만큼은 인식부터
LED 하드웨어 제어(아두이노)까지 전 구간을 본인이 직접 담당하기로 한
예외다. **다른 미션(여름/가을/겨울/정찰동행)은 여전히 좌표·판정 발행까지만
담당하는 경계를 지킨다** — 이 파일의 구조(하드웨어 제어 체인까지 포함)를
다른 미션 코드의 템플릿으로 그대로 가져다 쓰지 말 것.

[2026-08-30, 사용자 결정] 원래는 이 노드 -> led_relay_node(MissionResult ->
/led_control 매핑) -> led_bridge_node(/led_control -> 시리얼 -> 아두이노)
3단 체인이었는데, 판정(디바운싱+락)을 이미 이 노드가 하고 있으므로
led_relay_node를 거치지 않고 이 노드가 /led_control을 직접 발행하도록
단순화함. led_relay.py는 여전히 존재하지만(그 노드는 기본적으로 더 이상
실행 안 함) 매핑 로직(result_to_led_command())은 여기서 그대로 import해서
재사용 — 매핑 기준이 두 군데로 갈라지지 않게 하기 위함. led_bridge_node는
/led_control만 구독하므로 발행 주체가 바뀌어도 코드 변경 없음.

좌/우 사이드캠(side_cameras.launch.py, usb_cam) 듀얼캠 구조 — 카메라로
아군(ROKA)/적군(Enemy) 복장을 YOLO로 판별한다. 프레임마다 즉시 판정하면
오탐 하나에도 결과가 바로 흔들리므로, 좌/우 카메라 각각 독립 디바운서를
둬서, 한 카메라에서 confirm_frames 프레임 연속으로 같은(unknown이 아닌)
판정이 나와야 그 카메라가 "확정"한 것으로 보고, 좌/우 중 먼저 확정된 쪽
결과를 채택해 그 뒤로는 고정 발행한다(다시 흔들리지 않음 — 한 번 피아식별
되면 그 결과가 미션 끝까지 유효하다고 보는 게 맞기 때문) — 이 고정은
/mission/spring_ifof/result의 state/valid와 /led_control 값에만 적용된다.

[2026-08-30, 사용자 결정] YOLO 추론 자체는 확정 후에도 계속 매 프레임
돈다 — 원래는 확정되면 추론을 스킵해서(연산 절약) /spring/ifof/detected와
.../detections도 같이 멈췄는데, bbox/원시판정은 확정 여부와 무관하게
항상 실시간으로 보고 싶다는 요청으로 그 최적화를 뺐다. 프레임당 추론
비용은 YOLO26n nano 모델 기준 20ms 안팎으로 측정됨(summer_supply.py의
supplyboxv3.pt 스모크 테스트 참고) — 15fps 카메라 예산(66ms)에 여유 있어
확정 후에도 매 프레임 계속 돌리는 데 문제 없다고 판단.

구독: /side/left/image_raw/compressed  (sensor_msgs/CompressedImage)
      /side/left/camera_info           (sensor_msgs/CameraInfo)
      /side/right/image_raw/compressed (sensor_msgs/CompressedImage)
      /side/right/camera_info          (sensor_msgs/CameraInfo)
발행: /mission/spring_ifof/result (mission_manager_interfaces/MissionResult)
      state: 'friend' | 'enemy' | 'unknown'
      valid: 좌/우 중 하나라도 확정됐으면 True, 아니면 False
      /led_control (std_msgs/String) — "roka" | "enemy" | "none", 값이
      바뀔 때만 발행(led_relay_node의 기존 동작과 동일). led_bridge_node가
      구독해서 시리얼로 아두이노에 전달.
      /spring/ifof/detected (std_msgs/String) — 디바운싱 이전, 지금 이
      프레임에서 원시로 검출된 상태('friend'/'enemy'/'unknown', 좌/우
      구분 없이 처리되는 즉시 발행). 확정 결과가 아니라 "지금 카메라가
      뭘 보고 있는지" 실시간 확인용 — 확정(락) 이후에도 계속 매 프레임
      갱신된다(위 [2026-08-30, 사용자 결정] 참고, /mission/spring_ifof/result
      의 state와는 달리 이 값은 계속 바뀔 수 있음).
      /mission/spring_ifof/left/detections  (vision_msgs/Detection2DArray)
      /mission/spring_ifof/right/detections (vision_msgs/Detection2DArray)
      확정 여부와 무관하게 매 프레임 발행. 공간 필터 없이 conf_threshold
      이상인 원시 검출 전부(마커 클래스
      포함 — CLASS_NAMES_PLACEHOLDER 전체, friend/enemy로 매핑되는 것만이
      아님). vision_msgs 미설치 시 이 두 토픽만 조용히 미발행(summer_traffic.py
      와 동일한 방어 패턴, 판정/LED 발행에는 영향 없음).

(2026-08-30) 아군/적군 판별용 모델(ifofv1.pt)을 config/models/에 연결함.
      학습에 쓰인 병합 데이터셋(merged/data.yaml)은 이 미션과 무관한
      fall_marker용 문자/하트 마커 클래스(A/E/K/M/O/R/Y/Heart)도 같이
      들어있는 11-클래스 모델이지만 [2026-08-30, 사용자 결정] 이 노드는
      fall_marker와 모델을 공유하지 않고 ROKA/friend/Enemy 세 클래스만
      사용 — 나머지 마커 클래스는 _LABEL_TO_STATE에 없으므로 자동으로
      무시된다(단, 원시 detections 토픽에는 여전히 다 나간다 — 위 발행
      절 참고). ROKA와 friend는 둘 다 'friend'로 취급한다(아래
      _LABEL_TO_STATE 참고). 이 모델은 임시(v1)이며 사용자가 이후 다시
      학습된 모델로 교체할 예정 — 교체 시 CLASS_NAMES_PLACEHOLDER의 순서를
      새 모델의 data.yaml과 다시 맞춰 확인할 것(순서가 틀리면 아군/적군이
      뒤바뀌는, 이 미션에서 제일 위험한 실수가 된다).
"""
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from mission_manager_interfaces.msg import MissionResult

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir
from dolbotz.utils.state_debouncer import ConsecutiveStateDebouncer
from dolbotz.missions.led_relay import result_to_led_command

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    _VISION_MSGS_OK = True
except ImportError:
    # package.xml에는 의존성을 선언해도 실제 apt install(ros-humble-vision-msgs)이
    # 안 된 워크스페이스에서는 여전히 여기로 빠진다 — _YOLO_OK와 같은 취지로
    # 안전하게 no-op(경고만 내고 detections 토픽만 미발행)한다. 판정 결과
    # (/mission/spring_ifof/result, /led_control) 발행은 이 값과 무관하게
    # 항상 정상 동작.
    _VISION_MSGS_OK = False

# config/models/ifofv1_int8_openvino_model — __init__에서 get_models_dir()와
# 조합해 model_path 파라미터 기본값으로 쓴다(summer_traffic.py의
# TRAFFIC_MODEL_RELATIVE_PATH와 동일한 패턴). 모델이 비어있거나 로드
# 실패하면 _load_model()이 None을 반환하고, 이후 모든 프레임에서 'unknown'만
# 반환한다(valid=False 고정 — 모델 없이 아무 판정도 안 내는 게 안전하다는
# 이 프로젝트 전반의 원칙과 동일, elevation_map.py의 min_points_per_cell
# 등과 같은 취지).
# [2026-09-02] ifofv1.pt(FP32, PyTorch)를 merged/data.yaml로 캘리브레이션한
# OpenVINO INT8로 교체 — segmentation.py에 적용한 것과 동일 절차/근거
# (config/models/README.md 참고). ONNX INT8도 시도했으나 OpenVINO INT8보다
# 5배 이상 느려서(이 CPU에서 실측 17.6ms vs 2.9ms) 버리고 OpenVINO로 감.
# merged/data.yaml 전체(987장) 검증: mAP50 0.98735->0.98253,
# mAP50-95 0.85763->0.83808, recall 0.96887->0.96804(거의 유지) —
# 추론시간은 15.52ms->2.86ms(약 5.4배). 이전 FP32로 되돌리려면 아래 주석
# 처리된 줄로 바꿀 것.
IFOF_MODEL_RELATIVE_PATH = Path('ifofv1_int8_openvino_model')
# IFOF_MODEL_RELATIVE_PATH = Path('ifofv1.pt')
CONF_THRESHOLD_PLACEHOLDER = 0.5

# [중요] 이 순서는 ifofv1.pt 학습에 쓰인 merged/data.yaml의 클래스 인덱스
# 순서와 정확히 일치해야 한다(class 0 = CLASS_NAMES[0], ...). 이 모델은
# fall_marker용 문자/하트 마커(A/E/K/M/O/R/Y/Heart)까지 섞인 11-클래스
# 병합 데이터셋으로 학습됐지만, 이 미션에서 실제로 쓰는 건 ROKA/friend/Enemy
# 셋뿐 — 나머지는 _LABEL_TO_STATE(아래)에 없어서 무시된다. 이후 모델을
# 교체하면(사용자가 재학습 예정) 새 data.yaml 순서로 반드시 다시 맞출 것 —
# 틀리면 아군/적군이 뒤바뀐 채로 LED가 켜지는, 이 미션에서 제일 위험한
# 실수가 된다.
CLASS_NAMES_PLACEHOLDER = ['A', 'E', 'Enemy', 'Heart', 'K', 'M', 'O', 'R', 'ROKA', 'Y', 'friend']

# 카메라 한쪽이 이 프레임 수만큼 연속으로 같은 판정을 내야 "확정"으로 본다.
# 너무 작으면 오탐 한두 프레임에도 확정돼버리고, 너무 크면 실제 상황에서도
# 확정이 늦어진다 — 실측 후 조정할 것.
CONFIRM_FRAMES_PLACEHOLDER = 5

# [2026-09-04, 사용자 결정] 확정(락) 후에도 좌/우 카메라 raw_state가 모두
# 이 시간(초) 이상 연속으로 'unknown'이면 락을 해제한다 — 대상이 완전히
# 시야에서 사라진 뒤에도 예전 판정(예: enemy)이 미션 끝까지 눌어붙어 있는
# 걸 막기 위함. 한쪽이라도 여전히 friend/enemy를 보고 있으면(즉 unknown이
# 아니면) 타이머가 리셋되어 락은 유지된다 — 모듈 docstring의 "한 번
# 확정되면 안 바뀐다" 원칙은 "완전히 안 보이는 동안"으로 제한된 것으로
# 해석 변경.
UNLOCK_AFTER_UNKNOWN_SEC = 3.0

# YOLO 클래스 라벨(모델 학습 시 이름) -> MissionResult.state 매핑.
# [2026-08-30, 사용자 결정] ROKA와 friend는 둘 다 'friend'로 동작 — 이
# 병합 모델에 'friend'/'ROKA'가 별개 클래스로 들어있지만 이 미션에서는
# 구분하지 않는다. 나머지 마커 클래스(A/E/Heart/K/M/O/R/Y)는 여기 없어서
# _infer()에서 자동으로 무시된다.
_LABEL_TO_STATE = {'ROKA': 'friend', 'friend': 'friend', 'Enemy': 'enemy'}


class SpringIfofNode(Node):
    """좌/우 카메라 각각 디바운싱한 뒤, 먼저 확정된 쪽을 채택해 피아식별
    결과를 발행하는 노드."""

    def __init__(self):
        super().__init__('spring_ifof_node')

        self.declare_parameter(
            'model_path', str(get_models_dir() / IFOF_MODEL_RELATIVE_PATH))
        self.declare_parameter('conf_threshold', CONF_THRESHOLD_PLACEHOLDER)
        self.declare_parameter('confirm_frames', CONFIRM_FRAMES_PLACEHOLDER)
        self.declare_parameter(
            'color_topic_left', '/side/left/image_raw/compressed')
        self.declare_parameter(
            'camera_info_topic_left', '/side/left/camera_info')
        self.declare_parameter(
            'color_topic_right', '/side/right/image_raw/compressed')
        self.declare_parameter(
            'camera_info_topic_right', '/side/right/camera_info')

        model_path = str(self.get_parameter('model_path').value)
        self._conf_th = float(self.get_parameter('conf_threshold').value)
        confirm_frames = int(self.get_parameter('confirm_frames').value)
        self._confirm_frames = confirm_frames  # _process_image()의 로그 메시지에서 참조
        color_topic_left = str(self.get_parameter('color_topic_left').value)
        info_topic_left = str(self.get_parameter('camera_info_topic_left').value)
        color_topic_right = str(self.get_parameter('color_topic_right').value)
        info_topic_right = str(self.get_parameter('camera_info_topic_right').value)

        self._model = self._load_model(model_path)

        # 좌/우 독립 디바운서 + 워커 스레드 간 프레임 겹침 방지용 per-side lock.
        # 처리 중인 프레임이 있으면 새 프레임은 큐에 쌓지 않고 그냥 드롭한다
        # (밀리는 것보다 최신 프레임 위주로 처리하는 게 실시간성에 낫다는 판단).
        self._debouncers = {
            'left': ConsecutiveStateDebouncer(confirm_frames),
            'right': ConsecutiveStateDebouncer(confirm_frames),
        }
        self._side_locks = {'left': threading.Lock(), 'right': threading.Lock()}
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix='spring_ifof_worker')
        self._is_shutting_down = False

        # 확정 결과 — 좌/우 raw_state가 모두 UNLOCK_AFTER_UNKNOWN_SEC 이상
        # 연속으로 'unknown'이면 풀린다(아래 _unknown_since_monotonic 참고).
        # None이면 미확정.
        self._result_lock = threading.Lock()
        self._locked_state: str | None = None

        # 좌/우 카메라가 각각 raw_state='unknown'을 연속으로 보고하기
        # 시작한 monotonic 시각. 그 side가 unknown이 아닌 걸 보는 순간 None으로
        # 리셋된다 — 두 값이 모두 채워져 있고 각각 UNLOCK_AFTER_UNKNOWN_SEC
        # 이상 지났을 때만 락을 해제한다(위 UNLOCK_AFTER_UNKNOWN_SEC 주석 참고).
        self._unknown_since_monotonic: dict[str, float | None] = {
            'left': None, 'right': None}

        self.create_subscription(
            CameraInfo, info_topic_left, self._on_camera_info_left, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CompressedImage, color_topic_left, self._on_image_left, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CameraInfo, info_topic_right, self._on_camera_info_right, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CompressedImage, color_topic_right, self._on_image_right, SENSOR_DATA_QOS_DEPTH1)
        self._result_pub = self.create_publisher(
            MissionResult, '/mission/spring_ifof/result', 10)
        # led_relay_node를 거치지 않고 이 노드가 직접 /led_control을 발행함
        # (모듈 docstring 상단 참고). 매핑 함수는 led_relay.py에서 그대로
        # 재사용 — 값이 바뀔 때만 발행하는 것도 led_relay_node의 기존 동작과
        # 동일하게 _last_led_cmd로 추적한다.
        self._led_pub = self.create_publisher(String, '/led_control', 10)
        self._last_led_cmd: str | None = None
        # 디바운싱 이전 원시 판정 실시간 확인용(모듈 docstring 발행 절 참고).
        self._detected_pub = self.create_publisher(String, '/spring/ifof/detected', 10)

        self._left_detections_pub = None
        self._right_detections_pub = None
        if _VISION_MSGS_OK:
            self._left_detections_pub = self.create_publisher(
                Detection2DArray, '/mission/spring_ifof/left/detections', 10)
            self._right_detections_pub = self.create_publisher(
                Detection2DArray, '/mission/spring_ifof/right/detections', 10)
        else:
            self.get_logger().warn(
                'vision_msgs 미설치 — /mission/spring_ifof/{left,right}/detections '
                '미발행(판정/LED 발행에는 영향 없음). '
                'ros-humble-vision-msgs 설치 후 재기동하면 발행됨.')

        self.get_logger().info(
            f'SpringIfofNode ready — left={color_topic_left}, right={color_topic_right}, '
            f'confirm_frames={confirm_frames}'
            + ('' if self._model is not None else ' (모델 없음 -- 항상 unknown 발행)'))

    def _load_model(self, path: str):
        if not path or not _YOLO_OK:
            self.get_logger().warn(
                'model_path 미설정 또는 ultralytics 미설치 — 항상 unknown/valid=False만 발행함')
            return None
        try:
            return YOLO(path, task='detect')
        except Exception as e:  # noqa: BLE001 -- 모델 파일 손상 등 다양한 원인 가능
            self.get_logger().error(f'모델 로드 실패({e}) — 항상 unknown/valid=False만 발행함')
            return None

    def _on_camera_info_left(self, msg: CameraInfo) -> None:
        pass

    def _on_camera_info_right(self, msg: CameraInfo) -> None:
        pass

    def _on_image_left(self, msg: CompressedImage) -> None:
        self._submit(msg, 'left')

    def _on_image_right(self, msg: CompressedImage) -> None:
        self._submit(msg, 'right')

    def _submit(self, msg: CompressedImage, side: str) -> None:
        if self._is_shutting_down:
            return
        lock = self._side_locks[side]
        if lock.acquire(blocking=False):
            try:
                self._executor.submit(self._process_image, msg, side)
            finally:
                pass  # 락 해제는 워커 스레드(_process_image)의 finally에서
        else:
            self.get_logger().warn(
                f'{side} 카메라 프레임 드롭 — 이전 프레임 처리 중', throttle_duration_sec=1.0)

    def _process_image(self, msg: CompressedImage, side: str) -> None:
        try:
            # [2026-08-30, 사용자 결정] bbox(/mission/spring_ifof/{left,right}/
            # detections)와 /spring/ifof/detected는 확정(락) 여부와 무관하게
            # 항상 프레임마다 발행한다 — 예전엔 확정 후 YOLO 추론 자체를
            # 스킵해서(연산 절약) 이 두 토픽도 같이 멈췄는데, 그 최적화를
            # 뺐다. 그래서 이제 확정 후에도 매 프레임 추론이 계속 돈다
            # (측정상 프레임당 20ms 안팎이라 15fps 예산에 여유 있음 —
            # summer_supply.py 스모크 테스트에서 같은 계열 모델로 확인).
            # 대신 /mission/spring_ifof/result·/led_control의 state 값
            # 자체는 여전히 self._locked_state로 고정되고 안 바뀐다 —
            # 한 번 피아식별되면 그 판정이 미션 끝까지 유효해야 한다는
            # 원칙은 그대로 유지(모듈 docstring 참고).
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warn(
                    f'{side} 이미지 디코드 실패', throttle_duration_sec=2.0)
                return

            raw_state, raw_detections = self._infer(cv_image)

            # 실시간 원시 판정/검출 — 모듈 docstring 발행 절 참고.
            self._detected_pub.publish(String(data=raw_state))
            side_pub = (
                self._left_detections_pub if side == 'left'
                else self._right_detections_pub)
            self._publish_detections(side_pub, msg.header, raw_detections)

            # 이 side의 raw_state가 unknown으로 계속 이어지는 동안의 시작
            # 시각을 추적 — unknown이 아니면 즉시 리셋(다시 보였으니 그 순간부터
            # 다시 세야 함), unknown이면 처음 그 상태로 들어온 시각을 유지한다.
            now = time.monotonic()
            if raw_state == 'unknown':
                if self._unknown_since_monotonic[side] is None:
                    self._unknown_since_monotonic[side] = now
            else:
                self._unknown_since_monotonic[side] = None

            # 이미 확정됐으면 디바운서로 새로 확정시키진 않지만, 좌/우 모두
            # UNLOCK_AFTER_UNKNOWN_SEC 이상 연속 unknown이면 락을 풀어
            # 재확정 가능한 상태로 되돌린다(모듈 상단 UNLOCK_AFTER_UNKNOWN_SEC
            # 주석 참고) — 그래도 위 bbox/detected 발행은 이 지점과 무관하게
            # 이미 끝났으니 매 프레임 나간다.
            with self._result_lock:
                already_locked = self._locked_state is not None
            if not already_locked:
                confirmed = self._debouncers[side].update(raw_state)
                if confirmed is not None:
                    with self._result_lock:
                        if self._locked_state is None:
                            self._locked_state = confirmed
                            self.get_logger().info(
                                f"{side} 카메라에서 '{confirmed}' {self._confirm_frames}"
                                f"프레임 연속 확정 — 이후 고정 발행")
            else:
                since = self._unknown_since_monotonic
                both_unknown_long_enough = (
                    since['left'] is not None
                    and since['right'] is not None
                    and now - since['left'] >= UNLOCK_AFTER_UNKNOWN_SEC
                    and now - since['right'] >= UNLOCK_AFTER_UNKNOWN_SEC)
                if both_unknown_long_enough:
                    with self._result_lock:
                        if self._locked_state is not None:
                            self._locked_state = None
                            self._debouncers['left'].reset()
                            self._debouncers['right'].reset()
                            self.get_logger().info(
                                f'좌/우 모두 {UNLOCK_AFTER_UNKNOWN_SEC}초 이상 unknown '
                                '— 확정 상태 해제, 재확정 대기')

            self._publish_result(msg.header)
        except Exception as e:  # noqa: BLE001 -- 워커 스레드 예외는 여기서 반드시 삼켜야 함
            self.get_logger().error(
                f'{side} 처리 중 예외: {e}\n{traceback.format_exc()}')
        finally:
            self._side_locks[side].release()

    def _infer(self, cv_image: np.ndarray):
        """cv_image 한 장에 대한 원시(디바운싱 전) 판정 + 원시 검출 전부.

        self._model이 None이면(모델 미확보) 항상 ('unknown', []). friend/enemy
        판정에는 모델이 _LABEL_TO_STATE에 없는 마커 클래스(A/E/Heart/K/M/O/R/Y
        — fall_marker용, 이 미션과 무관)를 검출해도 무시한다. friend/enemy
        둘 다 검출되면 friend(아군)를 우선시한다(오탐으로 적군 표시를 내는
        것보다 안전한 쪽). raw_detections에는 마커 클래스를 포함해 conf_threshold
        이상인 검출 전부가 (bbox_xyxy, label, conf) 튜플로 들어간다 —
        /mission/spring_ifof/{left,right}/detections용(모듈 docstring 발행 절
        참고), friend/enemy 판정과 무관하게 항상 채워진다.

        Returns:
            (state, raw_detections): state는 'friend'|'enemy'|'unknown',
            raw_detections는 [((x1,y1,x2,y2), label, conf), ...].
        """
        if self._model is None:
            return 'unknown', []

        results = self._model(cv_image, conf=self._conf_th, iou=0.45, verbose=False)
        found_friend = False
        found_enemy = False
        raw_detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes.cpu().numpy():
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                bbox = tuple(float(v) for v in box.xyxy[0].tolist())
                label = (
                    CLASS_NAMES_PLACEHOLDER[cls_id]
                    if 0 <= cls_id < len(CLASS_NAMES_PLACEHOLDER) else '')
                raw_detections.append((bbox, label, conf))

                state = _LABEL_TO_STATE.get(label)
                if state == 'friend':
                    found_friend = True
                elif state == 'enemy':
                    found_enemy = True

        if found_friend:
            state = 'friend'
        elif found_enemy:
            state = 'enemy'
        else:
            state = 'unknown'
        return state, raw_detections

    def _publish_detections(self, pub, header, raw_detections) -> None:
        """raw_detections((x1,y1,x2,y2), label, conf) 리스트를
        Detection2DArray로 발행한다. pub이 None이면(vision_msgs 미설치)
        조용히 넘어간다 — summer_traffic.py의 _publish_detections()와 동일
        패턴."""
        if pub is None:
            return
        array = Detection2DArray()
        array.header = header
        for (x1, y1, x2, y2), label, conf in raw_detections:
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = max(0.0, x2 - x1)
            detection.bbox.size_y = max(0.0, y2 - y1)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)
            array.detections.append(detection)
        pub.publish(array)

    def _publish_result(self, header) -> None:
        with self._result_lock:
            locked = self._locked_state

        result = MissionResult()
        result.header = header
        result.mission_name = 'spring_ifof'
        if locked is not None:
            result.state = locked
            result.valid = True
        else:
            result.state = 'unknown'
            result.valid = False
        self._result_pub.publish(result)

        # led_relay_node 없이 이 노드가 직접 /led_control을 발행 — 모듈
        # docstring 상단 참고. 매핑은 led_relay.py의 순수 함수를 그대로
        # 재사용하고, 값이 바뀔 때만 발행하는 것도 그 노드의 기존 동작과
        # 동일(불필요한 시리얼 트래픽/로그 스팸 방지).
        led_cmd = result_to_led_command(result.state, result.valid)
        if led_cmd != self._last_led_cmd:
            self._last_led_cmd = led_cmd
            self._led_pub.publish(String(data=led_cmd))
            self.get_logger().info(
                f"LED 명령 변경: '{led_cmd}' (state='{result.state}', valid={result.valid})")

    def destroy_node(self):
        self.get_logger().info('스레드풀 종료 중...')
        self._is_shutting_down = True
        self._executor.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpringIfofNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
