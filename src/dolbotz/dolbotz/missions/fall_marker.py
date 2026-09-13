"""가을 미션 — 비전마커 순차 인식 노드.

좌/우 사이드캠(side_cameras.launch.py, usb_cam) 듀얼캠 구조 — 카메라로 마커
(문자 A/E/K/M/O/R/Y + Heart, 총 8종)를 YOLO로 인식해 지금 보이는 마커 텍스트를
결과로 발행한다. `expected_sequence` 파라미터로 기대하는 마커 순서를 받아
다음 기대 마커 인덱스를 관리할 자리를 남겨뒀지만, [실제 순서 매칭/배점 로직은
여전히 TODO다] — 대회 규정 문서(순서 이탈 시 감점 여부 등)가 아직 확정 안 돼서
임의로 판단 기준을 만들지 않고 비워둠. 이번에 채운 건 인식 자체(모델 로드 +
추론 + 좌/우 디바운싱 + 결과/bbox 발행)까지다.

[2026-08-30] 비전마커 모델(visionmarkerv1.pt)을 config/models/에 연결함 —
spring_ifof.py가 ifofv1.pt를 붙일 때와 같은 절차. 8개 클래스({0:'A', 1:'E',
2:'Heart', 3:'K', 4:'M', 5:'O', 6:'R', 7:'Y'} — 알파벳 순이 아니라 학습된
모델의 실제 클래스 인덱스 순서, ultralytics로 직접 로드해서 확인함)라 spring_ifof
(friend/enemy 2종)와 달리 순차 인식 특성상 "한 번 확정되면 끝까지 고정"이
아니라, 매 프레임 좌/우 각각 디바운싱만 하고 계속 재평가한다(현재 보이는
마커가 바뀌면 그 마커로 갱신) — summer_traffic.py의 신호등 판정과 같은
패턴, spring_ifof.py의 "한 번 락" 패턴과는 다르다(마커가 여러 개 순서대로
지나가야 하므로 하나로 고정하면 안 됨).

[2026-09-01] visionmarkerv1.pt를 new_vision_marker-2 데이터로 추가
fine-tuning한 vision_makerv2.pt로 교체함(/home/j/vision_marker/ifof/
finetune_v2.py 참고, ifof가 아니라 이 비전마커 미션 전용). 이 파인튜닝은
spring_ifof.py용 11-클래스 통합 데이터셋(merged/data.yaml) 위에서
이어졌기 때문에 모델 자체는 여전히 11클래스({0:'A', 1:'E', 2:'Enemy',
3:'Heart', 4:'K', 5:'M', 6:'O', 7:'R', 8:'ROKA', 9:'Y', 10:'friend'} —
ultralytics로 직접 로드해서 확인함, visionmarkerv1.pt 때와 인덱스 순서가
다름 주의)이지만, 이 노드는 spring_ifof용 클래스(Enemy/ROKA/friend)는
쓰지 않고 원래의 8종 마커만 결과로 인정한다(_VALID_MARKER_LABELS로 필터링
— 아래 참고). raw detections 토픽에는 여전히 11클래스 전부가 그대로
나간다(필터링은 state 확정에만 적용).

[2026-09-03] 후속 가중치 vision_makerv3.pt를 이 워크스페이스의
config/models/vision_marker.v3.pt로 옮기고 기본 모델로 교체함. 클래스 구성은
v2와 동일한 11클래스이며, 아래 8종 마커 필터링도 그대로 적용한다.
이어서 다른 CPU 배포 모델과 같게 merged/data.yaml 987장을
캘리브레이션에 사용한 320x320 OpenVINO INT8 모델으로 변환해
vision_marker.v3_int8_openvino_model을 기본으로 사용한다.

[2026-09-01] 좌/우 추론을 spring_ifof.py와 동일한 패턴(ThreadPoolExecutor
+ side별 threading.Lock)으로 병렬화함 — 이전엔 rclpy.spin()의 기본
SingleThreadedExecutor 콜백 안에서 YOLO 추론을 동기로 돌렸는데, 이 경우
좌측 프레임 추론 중엔 그 시간만큼 우측 카메라 콜백이 아예 실행이 안
됐다(같은 실행자 스레드 공유 — SENSOR_DATA_QOS_DEPTH1 덕에 큐가 쌓이진
않지만, 좌/우가 사실상 직렬화돼서 실질 처리율이 떨어짐). 이제 각 카메라
이미지 콜백은 곧바로 처리하지 않고 _submit()으로 워커 스레드풀에 맡기며,
이전 프레임이 아직 처리 중이면(같은 쪽 lock을 못 잡으면) 새 프레임은
큐에 안 쌓고 그냥 드롭한다(spring_ifof.py의 _submit()/_process_image()와
동일 구현).

디바운싱: 좌/우 카메라 각각 독립 ConsecutiveStateDebouncer를 두고, 한
카메라에서 confirm_frames 프레임 연속으로 같은(unknown이 아닌) 마커가
검출돼야 그 카메라가 "지금 이 마커를 보고 있다"고 인정한다. 좌/우 둘 다
확정 중이면 left를 우선한다(임의 우선순위 — 실기에서 문제되면 조정할 것).
마커가 안 보이게 되면(연속 판정이 끊기면) 다시 unknown/valid=False로
돌아간다.

구독: /side/left/image_raw/compressed  (sensor_msgs/CompressedImage)
      /side/left/camera_info           (sensor_msgs/CameraInfo)
      /side/right/image_raw/compressed (sensor_msgs/CompressedImage)
      /side/right/camera_info          (sensor_msgs/CameraInfo)
발행: /mission/fall_marker/result (mission_manager_interfaces/MissionResult)
      state: 인식한 마커 텍스트('A'/'E'/'Heart'/'K'/'M'/'O'/'R'/'Y') 또는 'unknown'
      valid: 좌/우 중 하나라도 지금 확정 중이면 True
      /mission/fall_marker/left/detections  (vision_msgs/Detection2DArray)
      /mission/fall_marker/right/detections (vision_msgs/Detection2DArray)
      공간 필터 없이 conf_threshold 이상인 원시 검출 전부. vision_msgs
      미설치 시 이 두 토픽만 조용히 미발행(summer_traffic.py/spring_ifof.py와
      동일한 방어 패턴, 판정 결과 발행에는 영향 없음).
"""
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from mission_manager_interfaces.msg import MissionResult

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir
from dolbotz.utils.state_debouncer import ConsecutiveStateDebouncer

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
    # (/mission/fall_marker/result) 발행은 이 값과 무관하게 항상 정상 동작.
    _VISION_MSGS_OK = False

# 실측 전 임시값 — 실제 대회 마커 순서 확정되면 갱신할 것.
EXPECTED_SEQUENCE_PLACEHOLDER = ['']

# config/models/vision_marker_final_int8_openvino_model — __init__에서
# get_models_dir()와 조합해 model_path 파라미터 기본값으로 쓴다
# (spring_ifof.py의 IFOF_MODEL_RELATIVE_PATH와 동일한 패턴). 모델이
# 비어있거나 로드 실패하면 _load_model()이 None을 반환하고, 이후 모든
# 프레임에서 'unknown'만 반환한다(valid=False 고정 — 모델 없이 아무 판정도
# 안 내는 게 안전하다는 이 프로젝트 전반의 원칙과 동일).
# [2026-09-05] vision_makerv4.pt(/home/j/vision_marker/ifof/vision_makerv4.pt,
# 사용자 요청으로 vision_marker_final.pt로 워크스페이스에 반입)로 교체.
# 클래스 구성(11종, 순서 포함)은 이전 vision_marker.v3.pt와 완전히 동일하게
# 확인됨(model.names 비교, 아래 CLASS_NAMES_PLACEHOLDER 그대로 유지 가능) —
# fine-tuning된 가중치만 갱신된 것으로 보임. v3와 동일한 절차(merged/data.yaml
# 캘리브레이션, imgsz=320, batch=1)로 OpenVINO INT8 변환했다(정확도 재검증은
# 아직 안 함 — v3 변환 시의 mAP 드롭 폭(mAP50 -0.5%p 등)이 참고치).
# FP32로 롤백하려면 아래 주석 경로를 쓴다.
MARKER_MODEL_RELATIVE_PATH = Path('vision_marker_final_int8_openvino_model')
# MARKER_MODEL_RELATIVE_PATH = Path('vision_marker_final.pt')
# MARKER_MODEL_RELATIVE_PATH = Path('vision_marker.v3_int8_openvino_model')  # 이전 모델(v3)
# MARKER_MODEL_RELATIVE_PATH = Path('vision_marker.v3.pt')
CONF_THRESHOLD_PLACEHOLDER = 0.5

# [중요] 이 순서는 vision_marker_final.pt 학습 시 클래스 인덱스 순서와 정확히
# 일치해야 한다(class 0 = CLASS_NAMES[0], ...) — ultralytics로 직접 로드해서
# model.names로 확인한 값이다(vision_marker.v3.pt와 완전히 동일한 순서로
# 확인됨, 2026-09-05). visionmarkerv1.pt(8클래스)와 달리 이 모델은
# spring_ifof.py용 11-클래스 통합 데이터셋 위에서 fine-tuning됐기 때문에
# Enemy/ROKA/friend 3종이 섞여 있다 — 틀리면 엉뚱한 마커 텍스트가 나가게
# 되므로 모델을 교체하면 반드시 새 모델의 model.names로 다시 확인할 것.
CLASS_NAMES_PLACEHOLDER = [
    'A', 'E', 'Enemy', 'Heart', 'K', 'M', 'O', 'R', 'ROKA', 'Y', 'friend']

# 이 미션이 실제로 인정하는 마커 텍스트 — vision_marker.v3.pt에 섞여 있는
# spring_ifof.py 전용 클래스(Enemy/ROKA/friend)는 fall_marker 결과로
# 나가면 안 되므로 _infer()에서 이 집합 밖의 라벨은 state 후보에서
# 제외한다(spring_ifof.py의 _LABEL_TO_STATE가 반대 방향으로 걸러내는
# 것과 같은 취지 — 원치 않는 클래스는 자동으로 무시). raw detections
# 토픽에는 필터링 없이 11클래스 전부 그대로 나간다.
_VALID_MARKER_LABELS = frozenset(['A', 'E', 'Heart', 'K', 'M', 'O', 'R', 'Y'])

# 카메라 한쪽이 이 프레임 수만큼 연속으로 같은 판정을 내야 "확정"으로 본다.
CONFIRM_FRAMES_PLACEHOLDER = 5


class FallMarkerNode(Node):
    """좌/우 카메라 각각 디바운싱해서 지금 보이는 마커를 발행하는 노드."""

    def __init__(self):
        super().__init__('fall_marker_node')

        self.declare_parameter('expected_sequence', EXPECTED_SEQUENCE_PLACEHOLDER)
        self.declare_parameter(
            'model_path', str(get_models_dir() / MARKER_MODEL_RELATIVE_PATH))
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

        self._expected_sequence = list(
            self.get_parameter('expected_sequence').value)
        # 다음으로 기대하는 마커의 expected_sequence 인덱스 — TODO: 실제
        # 마커 인식 결과와 비교해서 맞으면 증가시키는 로직 구현(대회 규정
        # 확정 전이라 아직 안 씀, 모듈 docstring 참고).
        self._next_expected_idx = 0

        model_path = str(self.get_parameter('model_path').value)
        self._conf_th = float(self.get_parameter('conf_threshold').value)
        confirm_frames = int(self.get_parameter('confirm_frames').value)
        color_topic_left = str(self.get_parameter('color_topic_left').value)
        info_topic_left = str(self.get_parameter('camera_info_topic_left').value)
        color_topic_right = str(self.get_parameter('color_topic_right').value)
        info_topic_right = str(self.get_parameter('camera_info_topic_right').value)

        self._model = self._load_model(model_path)
        self._bridge = CvBridge()

        # 좌/우 독립 디바운서 — spring_ifof.py와 달리 확정 후 락(고정)하지
        # 않는다(모듈 docstring 참고, 마커가 여러 개 순서대로 지나가야 함).
        self._debouncers = {
            'left': ConsecutiveStateDebouncer(confirm_frames),
            'right': ConsecutiveStateDebouncer(confirm_frames),
        }
        self._current = {'left': None, 'right': None}
        # _current 읽기/쓰기 보호용 — 좌/우 워커 스레드가 각자 자기 쪽
        # 키만 쓰지만(_current['left']는 left 워커만 씀), _publish_result()가
        # 다른 스레드에서 양쪽을 동시에 읽으므로 일관성을 위해 락으로 감싼다.
        self._current_lock = threading.Lock()

        # 좌/우 추론 병렬화 + 처리 중인 프레임이 있으면 새 프레임은 큐에
        # 쌓지 않고 드롭한다 — spring_ifof.py의 _side_locks/_executor와
        # 동일 패턴(모듈 docstring 참고).
        self._side_locks = {'left': threading.Lock(), 'right': threading.Lock()}
        self._executor = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix='fall_marker_worker')
        self._is_shutting_down = False

        self.create_subscription(
            CameraInfo, info_topic_left, self._on_camera_info_left, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CompressedImage, color_topic_left, self._on_image_left, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CameraInfo, info_topic_right, self._on_camera_info_right, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CompressedImage, color_topic_right, self._on_image_right, SENSOR_DATA_QOS_DEPTH1)
        self._result_pub = self.create_publisher(
            MissionResult, '/mission/fall_marker/result', 10)

        self._left_detections_pub = None
        self._right_detections_pub = None
        if _VISION_MSGS_OK:
            self._left_detections_pub = self.create_publisher(
                Detection2DArray, '/mission/fall_marker/left/detections', 10)
            self._right_detections_pub = self.create_publisher(
                Detection2DArray, '/mission/fall_marker/right/detections', 10)
        else:
            self.get_logger().warn(
                'vision_msgs 미설치 — /mission/fall_marker/{left,right}/detections '
                '미발행(판정 결과 발행에는 영향 없음). '
                'ros-humble-vision-msgs 설치 후 재기동하면 발행됨.')

        self.get_logger().info(
            f'FallMarkerNode ready — expected_sequence={self._expected_sequence}, '
            f'left={color_topic_left}, right={color_topic_right}, '
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

    def _infer(self, cv_image: np.ndarray):
        """cv_image 한 장에 대한 원시(디바운싱 전) 판정 + 원시 검출 전부.

        self._model이 None이면(모델 미확보) 항상 ('unknown', []). 여러
        마커가 동시에 잡히면 confidence가 가장 높은 것 하나를 그 프레임의
        원시 상태로 채택한다(summer_traffic.py의 pick_best_state와 같은
        취지 — 한 프레임엔 지금 카메라가 보고 있는 마커 하나만 있다고 가정).

        Returns:
            (state, raw_detections): state는 _VALID_MARKER_LABELS(8종) 중
            하나 또는 'unknown'(Enemy/ROKA/friend가 나와도 state 후보에서
            제외됨 — 위 _VALID_MARKER_LABELS 참고), raw_detections는
            [((x1,y1,x2,y2), label, conf), ...] (11클래스 전부, 최고
            confidence 선택과 무관하게 conf_threshold 이상 전부).
        """
        if self._model is None:
            return 'unknown', []

        results = self._model(cv_image, conf=self._conf_th, iou=0.45, verbose=False)
        raw_detections = []
        best_label = 'unknown'
        best_conf = -1.0
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
                if label in _VALID_MARKER_LABELS and conf > best_conf:
                    best_conf = conf
                    best_label = label

        return best_label, raw_detections

    def _publish_detections(self, pub, header, raw_detections) -> None:
        """raw_detections((x1,y1,x2,y2), label, conf) 리스트를
        Detection2DArray로 발행한다. pub이 None이면(vision_msgs 미설치)
        조용히 넘어간다 — spring_ifof.py/summer_traffic.py와 동일 패턴."""
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

    def _on_image_left(self, msg: CompressedImage) -> None:
        self._submit(msg, 'left')

    def _on_image_right(self, msg: CompressedImage) -> None:
        self._submit(msg, 'right')

    def _submit(self, msg: CompressedImage, side: str) -> None:
        """이전 프레임이 아직 처리 중이면(같은 쪽 lock을 못 잡으면) 새
        프레임은 큐에 쌓지 않고 그냥 드롭한다 — spring_ifof.py의
        _submit()과 동일 구현(모듈 docstring 참고)."""
        if self._is_shutting_down:
            return
        lock = self._side_locks[side]
        if lock.acquire(blocking=False):
            self._executor.submit(self._process_image, msg, side)
        else:
            self.get_logger().warn(
                f'{side} 카메라 프레임 드롭 — 이전 프레임 처리 중', throttle_duration_sec=1.0)

    def _process_image(self, msg: CompressedImage, side: str) -> None:
        try:
            try:
                cv_image = self._bridge.compressed_imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception as e:  # noqa: BLE001 -- 손상된 프레임 등 다양한 원인 가능
                self.get_logger().warn(
                    f'{side} 이미지 디코드 실패: {e}', throttle_duration_sec=2.0)
                return

            raw_state, raw_detections = self._infer(cv_image)

            side_pub = (
                self._left_detections_pub if side == 'left'
                else self._right_detections_pub)
            self._publish_detections(side_pub, msg.header, raw_detections)

            # TODO(순서 매칭): 여기서 raw_state를
            # self._expected_sequence[self._next_expected_idx]와 비교해서 맞으면
            # _next_expected_idx를 증가시키는 로직이 들어갈 자리 — 대회 규정
            # (순서 이탈 시 감점 여부 등) 확정 전이라 아직 비워둠(모듈 docstring
            # 참고). 지금은 "지금 보이는 마커가 뭔지"만 디바운싱해서 발행한다.
            confirmed = self._debouncers[side].update(raw_state)
            with self._current_lock:
                self._current[side] = confirmed

            self._publish_result(msg.header)
        except Exception as e:  # noqa: BLE001 -- 워커 스레드 예외는 여기서 반드시 삼켜야 함
            self.get_logger().error(
                f'{side} 처리 중 예외: {e}\n{traceback.format_exc()}')
        finally:
            self._side_locks[side].release()

    def _publish_result(self, header) -> None:
        # spring_ifof.py와 달리 확정 락이 없다 — 좌/우 중 확정 중인 쪽을
        # 매번 다시 채택한다(left 우선, 모듈 docstring 참고). 마커가 안
        # 보이게 되면 둘 다 None -> unknown/valid=False로 자연히 돌아간다.
        # _current 자체는 좌/우 워커 스레드가 동시에 건드릴 수 있어
        # _current_lock으로 감싼다(__init__ 주석 참고).
        with self._current_lock:
            state = self._current['left'] or self._current['right']

        result = MissionResult()
        result.header = header
        result.mission_name = 'fall_marker'
        if state is not None:
            result.state = state
            result.valid = True
        else:
            result.state = 'unknown'
            result.valid = False
        self._result_pub.publish(result)

    def destroy_node(self):
        self.get_logger().info('스레드풀 종료 중...')
        self._is_shutting_down = True
        self._executor.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FallMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
