import time
import threading
from time import perf_counter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty
from cv_bridge import CvBridge

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir
from vision.ov_supply_detector import OpenVINOSupplyDetector

# [2026-09-03] supplyboxv3.pt(FP32)를 OpenVINO INT8로 교체 -
# summer_supply.py의 _DEFAULT_MODEL_FILE과 동일 절차/근거/검증 결과
# (config/models/README.md 참고). 이전 FP32로 되돌리려면 아래 주석 처리된
# 줄로 바꿀 것.
_DEFAULT_MODEL_FILE = 'supplyboxv3_int8_openvino_model'
# _DEFAULT_MODEL_FILE = 'supplyboxv3.pt'


class DriveSupplyDetectorNode(Node):
    """
    주행용 뎁스카메라(색상 스트림)로 지면의 서플라이박스를 감지해서 "발견했다"
    신호만 발행한다 - summer_supply.py(ArmPickupNode, 팔 카메라)와 같은
    supplyboxv3 OpenVINO INT8 모델을 재사용하되, 좌표 추출은 하지 않는다(그건 팔
    카메라/summer_supply의 역할). 주행 중에는 박스가 지면에 있어 바로 안
    보이다가, 이 노드가 먼저 "근처에 박스가 있다"를 감지해서 팔을
    GRASP_WAIT 자세로 옮기는 트리거로만 쓴다.

    [2026-08-31 신규] 구동부(주행 정지 등) 연동은 이번 스코프 밖 - 팔
    쪽(maru_ik_node.py)만 이 토픽을 구독해서 HOME -> GRASP_WAIT로 이동한다.

    [2026-09-02 신규] 파지 완료("/arm/picking_command", maru_ik_node.py 1회성
    발행) 수신 시 OpenVINO 모델을 영구 종료한다(_on_picking_command) - 이번
    미션은 서플라이박스가 하나뿐이라 이후 재탐지가 불필요해서다. 여러 박스를
    다시 지원해야 하면 이 종료를 없애거나 재시작 트리거를 추가할 것.

    퍼블리시:
      /drive/supplybox_detected (std_msgs/Empty) - conf_threshold 이상 탐지
      시 1회성 신호. detected_min_interval_sec 간격으로 스팸 방지(forward_command/
      picking_command와 동일 패턴).

    파라미터:
      model_path      : OpenVINO IR 경로. 비우면 config/models/supplyboxv3_int8_openvino_model 사용
      target_class     : 결과에 붙일 클래스 이름(로그용) - 단일 클래스 학습이라
                          실제 필터링에는 안 쓰임
      conf_threshold   : 최소 confidence (기본 0.5)
      infer_size       : 추론 해상도(기본 320, 정적 IR 크기가 우선)
      perf_hint        : OpenVINO 성능 힌트(LATENCY 또는 THROUGHPUT)
      stats_period_s   : 처리시간 로그 주기(초, 0이면 비활성)
      color_topic      : 주행 카메라 컬러 토픽
      detected_topic   : 기본 '/drive/supplybox_detected'
      detected_min_interval_sec : 최소 발행 간격(기본 2.0초, 스팸 방지)
    """

    def __init__(self):
        super().__init__('drive_supply_detector_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('target_class', 'supplybox')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('infer_size', 320)
        self.declare_parameter('perf_hint', 'LATENCY')
        self.declare_parameter('stats_period_s', 5.0)
        self.declare_parameter(
            'color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('detected_topic', '/drive/supplybox_detected')
        self.declare_parameter('detected_min_interval_sec', 2.0)

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            model_path = str(get_models_dir() / _DEFAULT_MODEL_FILE)

        self.target = str(self.get_parameter('target_class').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.infer_size = int(self.get_parameter('infer_size').value)
        self.perf_hint = str(
            self.get_parameter('perf_hint').value).strip().upper()
        self.stats_period_s = float(
            self.get_parameter('stats_period_s').value)
        self.detected_topic = str(self.get_parameter('detected_topic').value)
        self.detected_min_interval_sec = float(
            self.get_parameter('detected_min_interval_sec').value)
        self._last_detected_time = 0.0
        color_topic = str(self.get_parameter('color_topic').value)

        self._inference_lock = threading.Lock()
        self.model = self._load_model(model_path)
        self._infer_request = (
            self.model.create_request() if self.model is not None else None)
        self.bridge = CvBridge()
        self._pending_lock = threading.Lock()
        self._pending_event = threading.Event()
        self._stop_event = threading.Event()
        self._pending_color = None
        self._stats_lock = threading.Lock()
        self._stats_started = perf_counter()
        self._stats_processed = 0
        self._stats_errors = 0
        self._stats_overwritten = 0
        self._stats_inference_ms = 0.0
        self._stats_total_ms = 0.0

        self.pub_detected = self.create_publisher(Empty, self.detected_topic, 10)
        self.create_subscription(
            CompressedImage, color_topic, self._on_color, SENSOR_DATA_QOS_DEPTH1)
        # [2026-09-02 신규] maru_ik_node.py가 파지 완료 후 1회성으로 발행하는
        # "/arm/picking_command" - 이번 미션은 서플라이박스가 하나뿐이라 이후
        # 재탐지가 필요 없어 모델을 영구 종료한다(summer_supply.py의
        # _on_picking_command와 동일 패턴, CPU 낭비 방지).
        self.create_subscription(
            Empty, '/arm/picking_command', self._on_picking_command, 10)

        self._stats_timer = None
        if self.stats_period_s > 0.0:
            self._stats_timer = self.create_timer(
                self.stats_period_s, self._report_stats)
        self._worker = threading.Thread(
            target=self._worker_loop,
            name='drive-supply-inference',
        )
        self._worker.start()

        self.get_logger().info(
            f'DriveSupplyDetectorNode ready  |  {color_topic} -> target={self.target} '
            f'conf>={self.conf_th}  (감지 시 {self.detected_topic} 1회성 발행) | '
            f'backend=openvino hint={self.perf_hint} requests=1')

    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn(
                'model_path 미설정 - 학습파일 경로를 파라미터로 전달하세요')
            return None
        if self.perf_hint not in {'LATENCY', 'THROUGHPUT'}:
            raise ValueError("perf_hint must be 'LATENCY' or 'THROUGHPUT'.")
        if self.stats_period_s < 0.0:
            raise ValueError('stats_period_s must not be negative.')
        model = OpenVINOSupplyDetector(
            model_path=path,
            confidence_threshold=self.conf_th,
            infer_size=self.infer_size,
            device='CPU',
            perf_hint=self.perf_hint,
            num_requests=1,
        )
        warmup_ms = model.warmup(3)
        self.get_logger().info(
            f'OpenVINO 모델 로드 완료: {model.model_path} | '
            f'precision={model.precision} input={model.model_input_shape} '
            f'warmup_avg={warmup_ms:.1f}ms')
        return model

    def _on_picking_command(self, _msg: Empty) -> None:
        """[2026-09-02 신규] 파지 완료 신호 수신 시 OpenVINO 모델을 영구
        종료한다. 이번 미션은 서플라이박스가 하나뿐이라 재시작 로직은 없다."""
        with self._inference_lock:
            if self.model is None:
                return
            self.model = None
            self._infer_request = None
        with self._pending_lock:
            self._pending_color = None
            self._pending_event.clear()
        self.get_logger().info(
            'picking_command 수신 - supplybox OpenVINO 모델 종료(추론 중단).')

    def _on_color(self, msg: CompressedImage):
        if self.model is None:
            return
        with self._pending_lock:
            if self._pending_color is not None:
                with self._stats_lock:
                    self._stats_overwritten += 1
            self._pending_color = msg
            self._pending_event.set()

    def _worker_loop(self):
        while not self._stop_event.is_set():
            self._pending_event.wait(0.1)
            if self._stop_event.is_set():
                break
            with self._pending_lock:
                msg = self._pending_color
                self._pending_color = None
                self._pending_event.clear()
            if msg is None:
                continue
            try:
                self._process_color(msg)
            except Exception as exc:
                with self._stats_lock:
                    self._stats_errors += 1
                if not self._stop_event.is_set() and rclpy.ok():
                    self.get_logger().error(
                        f'Drive supply frame processing failed: {exc}',
                        throttle_duration_sec=2.0)

    def _process_color(self, msg: CompressedImage):
        started = perf_counter()
        color = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._inference_lock:
            if self.model is None or self._infer_request is None:
                return
            prediction = self.model.predict_timed(
                color, request=self._infer_request)
        done = perf_counter()
        with self._stats_lock:
            self._stats_processed += 1
            self._stats_inference_ms += prediction.inference_ms
            self._stats_total_ms += (done - started) * 1000.0
        detected = bool(prediction.detections)
        if not detected:
            return
        now = time.monotonic()
        if now - self._last_detected_time < self.detected_min_interval_sec:
            return
        self._last_detected_time = now
        self.get_logger().info(
            f"'{self.target}' 발견 - {self.detected_topic} 발행.",
            throttle_duration_sec=2.0)
        self.pub_detected.publish(Empty())

    def _report_stats(self):
        now = perf_counter()
        with self._stats_lock:
            period = max(now - self._stats_started, 1e-9)
            processed = self._stats_processed
            errors = self._stats_errors
            overwritten = self._stats_overwritten
            inference_ms = self._stats_inference_ms
            total_ms = self._stats_total_ms
            self._stats_started = now
            self._stats_processed = 0
            self._stats_errors = 0
            self._stats_overwritten = 0
            self._stats_inference_ms = 0.0
            self._stats_total_ms = 0.0
        infer_avg = inference_ms / processed if processed else 0.0
        total_avg = total_ms / processed if processed else 0.0
        self.get_logger().info(
            f'drive supply stats ({period:.1f}s): '
            f'proc={processed / period:.1f}fps infer={infer_avg:.1f}ms '
            f'total={total_avg:.1f}ms overwritten={overwritten} errors={errors}')

    def shutdown_worker(self):
        self._stop_event.set()
        self._pending_event.set()
        if self._worker.is_alive():
            self._worker.join()


def main():
    rclpy.init()
    node = DriveSupplyDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_worker()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
