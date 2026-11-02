import json
import threading
from pathlib import Path
from time import perf_counter

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, Float64, String
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from cv_bridge import CvBridge
import message_filters

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from vision.compressed_image import decode_compressed_depth
from vision.ov_supply_detector import OpenVINOSupplyDetector

SENSOR_DATA_QOS_DEPTH1 = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_DEFAULT_MODEL_FILE = 'supplyboxv3_int8_openvino_model'
# _DEFAULT_MODEL_FILE = 'supplyboxv3.pt'

# RealSense에서 유효한 깊이 샘플의 상한.
SENSOR_MAX_DEPTH_M = 6.0


def is_within_base_x_stop_distance(base_x_m: float, threshold_m: float) -> bool:
    """Return whether a finite base-frame X coordinate reached the stop line."""
    return (
        np.isfinite(base_x_m)
        and np.isfinite(threshold_m)
        and threshold_m >= 0.0
        and abs(float(base_x_m)) <= float(threshold_m)
    )


def lookup_transform_with_latest_fallback(
    tf_buffer, target_frame: str, source_frame: str,
    transform_time: Time, timeout: Duration,
):
    """Look up an exact transform, falling back only for future extrapolation.

    Returns ``(transform, used_latest, original_exception)``.  Past
    extrapolation and disconnected/unknown frames remain hard failures.
    """
    try:
        return (
            tf_buffer.lookup_transform(
                target_frame, source_frame, transform_time, timeout=timeout),
            False,
            None,
        )
    except tf2_ros.ExtrapolationException as exc:
        if 'future' not in str(exc).lower():
            raise
        return (
            tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=timeout),
            True,
            exc,
        )


class SupplyNode(Node):
    """
    RealSense 뎁스 카메라로 서플라이 박스를 탐지하고 ``base_actuator`` 기준
    3D 좌표(m)를 로봇팔 제어용으로 퍼블리시한다.

    검출기는 OpenVINO Runtime으로 supplyboxv3 INT8 IR(단일 클래스
    {0: 'supplybox'})을 직접 실행한다. 모델 계열은 RF-DETR와 비교 후
    2026-08-30 YOLO로 확정했다.

    시작부터 박스가 팔 가동범위 안에 있는 정지 파지 흐름이다.
    |X| <= stop_base_x_m인 검출을 연속 확인하면 목표를 발행한다.
    IK 노드는 준비자세 도착 후부터 목표를 수락한다.

    퍼블리시:
      /arm/target_point  (geometry_msgs/PointStamped)  — base_actuator 프레임 XYZ.
      depth 필터와 X축 정지 기준을 통과한 최고 confidence 하나만 발행.
      /arm/target_depth_m (std_msgs/Float64) — 변환 전 camera optical Z.
      그리퍼 close-range 도착 판정은 base Z가 아니라 이 값을 사용한다.
      /arm/debug_image/compressed (sensor_msgs/CompressedImage) — 매 프레임 RGB
      (유효 depth와 TF 변환 성공 시 선택된 bbox 표시 — depth 무효면 오버레이도
      안 그려짐, 아래 /arm/supply/detections와 대조해서 "컬러 검출은
      됐는데 depth가 무효라 못 그려진 건지" 구분 가능. 그런 경우엔 로그로도
      경고를 낸다).
      /arm/supply/detections (std_msgs/String JSON) — person detection과 같은
      스키마로 depth 필터/best 선정 전, conf_threshold 이상 원시 검출 전부
      (target_point/오버레이보다 먼저, 항상 발행). 좌/우 사이드캠 미션들과
      달리 팔 카메라는 하나뿐이라 left/right 구분 없이 하나만 둔다. 이
      노드(supply 미션)만의 검출 결과라는 걸 구분하려고 미션 이름을
      넣은 /arm/<미션이름>/detections 형태로 지었다.

    [2026-09-02 신규] 파지 완료("/arm/picking_command", maru_ik_node.py
    picking_command_pub이 1회성 발행) 수신 시 OpenVINO 모델을 영구 종료한다
    (_on_picking_command) - 이번 미션은 서플라이박스가 하나뿐이라 이후
    재탐지가 불필요해서다. 여러 박스를 다시 지원해야 하면 이 종료를
    없애거나 재시작 트리거를 추가할 것(drive_supply_detector.py도 동일).

    파라미터:
      model_path         : OpenVINO IR 경로. 비우면 models/supplyboxv3_int8_openvino_model 사용
      target_class       : 결과에 붙일 클래스 이름(로그/오버레이 표시용) — 단일 클래스로
                            학습돼 실제 필터링에는 안 쓰임, confidence 임계값을 넘는
                            탐지는 전부 이 이름으로 취급. 기본 'supplybox'
      conf_threshold     : 최소 confidence (기본 0.5)
      infer_size         : 추론 해상도(기본 320, 정적 IR 크기가 우선)
      perf_hint          : OpenVINO 성능 힌트(LATENCY 또는 THROUGHPUT)
      stats_period_s     : 단계별 처리시간 로그 주기(초, 0이면 비활성)
      depth_roi_radius   : 깊이 샘플링 반경 픽셀 (기본 5)
      max_depth_m        : 좌표 계산에 쓸 카메라 depth 상한 m (기본 1.0)
      stop_base_x_m      : base_actuator 기준 |X| 파지 목표 상한(기본 0.32m)
      target_confirm_frames : 범위 내 연속 검출 프레임 수(기본 3)
      color_topic        : 컬러 이미지 토픽 (sensor_msgs/CompressedImage, 젯슨↔원격 네트워크 대역폭 절약용)
      depth_topic        : 컬러 정렬 압축 뎁스 토픽 (aligned_depth_to_color/compressedDepth)
      camera_info_topic  : 컬러 카메라 info 토픽
    """

    def __init__(self):
        super().__init__('supply_node')

        # 비워두면 기본 파일(_DEFAULT_MODEL_FILE)을 씀.
        self.declare_parameter('model_path', '')
        self.declare_parameter('target_class', 'supplybox')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('infer_size', 320)
        self.declare_parameter('perf_hint', 'LATENCY')
        self.declare_parameter('stats_period_s', 5.0)
        self.declare_parameter('depth_roi_radius', 5)
        self.declare_parameter('max_depth_m', 1.0)
        self.declare_parameter('planning_frame', 'base_actuator')
        self.declare_parameter('target_depth_topic', '/arm/target_depth_m')
        self.declare_parameter('stop_base_x_m', 0.32)
        self.declare_parameter('target_confirm_frames', 3)
        self.declare_parameter(
            'color_topic', '/arm/camera/color/image_raw/compressed')
        self.declare_parameter(
            'depth_topic',
            '/arm/camera/aligned_depth_to_color/image_raw/compressedDepth')
        self.declare_parameter(
            'camera_info_topic', '/arm/camera/color/camera_info')

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            model_path = str(Path(get_package_share_directory('vision')) / 'models' / _DEFAULT_MODEL_FILE)

        self.target  = str(self.get_parameter('target_class').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.infer_size = int(self.get_parameter('infer_size').value)
        self.perf_hint = str(
            self.get_parameter('perf_hint').value).strip().upper()
        self.stats_period_s = float(
            self.get_parameter('stats_period_s').value)
        self.depth_r = int(self.get_parameter('depth_roi_radius').value)
        self.max_d   = float(self.get_parameter('max_depth_m').value)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.target_depth_topic = str(
            self.get_parameter('target_depth_topic').value)
        self.stop_base_x_m = float(self.get_parameter('stop_base_x_m').value)
        self.target_confirm_frames = max(
            1, int(self.get_parameter('target_confirm_frames').value))
        self._target_confirm_count = 0
        color_topic  = str(self.get_parameter('color_topic').value)
        depth_topic  = str(self.get_parameter('depth_topic').value)
        info_topic   = str(self.get_parameter('camera_info_topic').value)

        self._inference_lock = threading.Lock()
        self.model = self._load_model(model_path)
        self._infer_request = (
            self.model.create_request() if self.model is not None else None)
        # [2026-09-02 신규] 이번 미션은 서플라이박스가 하나뿐이라, 파지가
        # 끝나면(_on_picking_command) 이후 재탐지가 필요 없다 - 모델을 영구
        # 종료해서 CPU 낭비를 막는다. worker가 model=None을
        # "로드 실패"로 취급해 에러 로그를 5초마다 찍으므로, 의도된 종료와
        # 구분하려고 플래그를 따로 둔다.
        self._model_terminated = False

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.fx = self.fy = self.cx = self.cy = None
        self._diag_frame_count = 0
        self._camera_info_logged = False
        self._pending_lock = threading.Lock()
        self._pending_event = threading.Event()
        self._stop_event = threading.Event()
        self._pending_frames = None
        self._stats_lock = threading.Lock()
        self._stats_started = perf_counter()
        self._stats_processed = 0
        self._stats_errors = 0
        self._stats_overwritten = 0
        self._stats_totals = {
            'decode': 0.0,
            'preprocess': 0.0,
            'inference': 0.0,
            'postprocess': 0.0,
            'spatial': 0.0,
            'jpeg': 0.0,
            'total': 0.0,
        }

        self.create_subscription(
            CameraInfo, info_topic, self._on_info, SENSOR_DATA_QOS_DEPTH1)
        # [2026-09-02 신규] maru_ik_node.py가 파지 시퀀스 완료 후 1회성으로
        # 발행하는 토픽("/arm/picking_command" - 리터럴로 하드코딩된 이유는
        # maru_ik_node.py의 picking_command_pub 발행부 주석 참고, 양쪽 다
        # 파라미터화하지 않고 고정 문자열로 맞춤).
        self.create_subscription(
            Empty, '/arm/picking_command', self._on_picking_command, 10)

        sub_color = message_filters.Subscriber(
            self, CompressedImage, color_topic, qos_profile=SENSOR_DATA_QOS_DEPTH1)
        sub_depth = message_filters.Subscriber(
            self, CompressedImage, depth_topic, qos_profile=SENSOR_DATA_QOS_DEPTH1)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=30, slop=0.20)
        self._sync.registerCallback(self._on_frames)

        self.pub_point = self.create_publisher(
            PointStamped, '/arm/target_point', 10)
        self.pub_target_depth = self.create_publisher(
            Float64, self.target_depth_topic, 10)
        # [수정, 2026-08-31] UI(rqt_image_view/RViz)로 보내는 카메라 화면이라
        # target_point/detections 같은 제어용 데이터와 달리 매 프레임 최신
        # 화면만 중요하고 늦은 프레임은 버려도 된다 - RELIABLE(기본)은 무선
        # 등 불안정한 네트워크에서 재전송이 쌓여 오히려 지연이 커질 수 있어
        # SENSOR_DATA_QOS_DEPTH1(BEST_EFFORT/depth=1)로 바꿈 - 이 파일 위쪽의
        # 다른 카메라 구독들과 동일한 "항상 최신 프레임만" 정책.
        # [갱신, 2026-09-05] debug 이미지는 사람이 보는 용도(rqt_image_view, UI)라
        # RELIABLE/depth=1로 발행한다. RELIABLE 발행자는 BEST_EFFORT 구독자와도
        # 매칭되고(그 쌍은 재전송 없이 전달), RELIABLE로만 구독하는 뷰어(rqt 등)
        # 와도 매칭된다. BEST_EFFORT 발행일 때는 RELIABLE 구독 뷰어에 아예 안 떴다.
        self.pub_debug = self.create_publisher(
            CompressedImage, '/arm/debug_image/compressed',
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.pub_detections = self.create_publisher(
            String, '/arm/supply/detections', qos_profile_sensor_data)

        self._stats_timer = None
        if self.stats_period_s > 0.0:
            self._stats_timer = self.create_timer(
                self.stats_period_s, self._report_stats)

        self._worker = threading.Thread(
            target=self._worker_loop,
            name='supply-inference',
        )
        self._worker.start()

        self.get_logger().info(
            f'SupplyNode ready  |  target={self.target}  '
            f'conf>={self.conf_th}  max_depth={self.max_d}m  '
            f'target when |base X|<={self.stop_base_x_m:.3f}m '
            f'for {self.target_confirm_frames} frames; stationary pickup | '
            f'backend=openvino hint={self.perf_hint} requests=1')

    # ------------------------------------------------------------------
    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn(
                'model_path 미설정 — 학습파일 경로를 파라미터로 전달하세요')
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
            f'streams={model.num_streams} warmup_avg={warmup_ms:.1f}ms')
        return model

    def _on_picking_command(self, _msg: Empty) -> None:
        """[2026-09-02 신규] 파지 완료 신호 수신 시 OpenVINO 모델을 영구
        종료한다. 이번 미션은 서플라이박스가 하나뿐이라 재시작 로직은 없다
        - 다음 미션에서 여러 박스를 다시 지원해야 하면 이 종료 대신
        모델을 유지한 채 추론만 멈췄다 재개하는 방식으로 바꿀 것."""
        with self._inference_lock:
            if self.model is None:
                return
            self.model = None
            self._infer_request = None
        with self._pending_lock:
            self._pending_frames = None
            self._pending_event.clear()
        self._model_terminated = True
        self.get_logger().info(
            'picking_command 수신 - supplybox OpenVINO 모델 종료(추론 중단).')

    def _on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        if not self._camera_info_logged:
            self.get_logger().warn(
                f'CAMERA INFO 수신 | '
                f'fx={self.fx}, fy={self.fy}, '
                f'cx={self.cx}, cy={self.cy}'
            )
            self._camera_info_logged = True

    @staticmethod
    def _to_meters(cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    def _sample_depth(self, depth: np.ndarray, u: int, v: int) -> float:
        """bbox 중심 주변 패치의 유효 깊이 중앙값 (m). 실패시 0.0.

        센서 유효범위(SENSOR_MAX_DEPTH_M)만 걸러낸다 - 팔이 닿는 범위(max_depth_m)
        인지는 호출부(_on_frames)에서 별도로 판단한다(모듈 docstring 참고).
        """
        h, w = depth.shape
        r = self.depth_r
        patch = depth[max(0, v - r):min(h, v + r + 1),
                      max(0, u - r):min(w, u + r + 1)]
        valid = patch[(patch > 0.05) & (patch < SENSOR_MAX_DEPTH_M)]
        return float(np.median(valid)) if valid.size >= 3 else 0.0

    def _publish_detections(self, header, image: np.ndarray, detections) -> None:
        """Publish raw color detections using the person-detection JSON schema."""
        payload = {
            'camera': 'arm',
            'stamp': {
                'sec': int(header.stamp.sec),
                'nanosec': int(header.stamp.nanosec),
            },
            'frame_id': header.frame_id,
            'image_size': {
                'width': int(image.shape[1]),
                'height': int(image.shape[0]),
            },
            'detections': [
                {
                    'class_id': detection.class_id,
                    'class_name': detection.class_name or self.target,
                    'confidence': detection.confidence,
                    'xyxy': list(detection.xyxy),
                }
                for detection in detections
            ],
        }
        output = String()
        output.data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))
        self.pub_detections.publish(output)

    def _infer(self, color_bgr: np.ndarray):
        """Run one direct OpenVINO request and return its timed prediction."""
        with self._inference_lock:
            if self.model is None or self._infer_request is None:
                return None
            return self.model.predict_timed(
                color_bgr, request=self._infer_request)

    def _on_frames(self, color_msg: CompressedImage, depth_msg: CompressedImage):
        """Keep only the newest synchronized color/depth pair for the worker."""
        if self._model_terminated:
            return
        with self._pending_lock:
            if self._pending_frames is not None:
                with self._stats_lock:
                    self._stats_overwritten += 1
            self._pending_frames = (color_msg, depth_msg)
            self._pending_event.set()

    def _worker_loop(self) -> None:
        """Process synchronized pairs without blocking ROS subscription callbacks."""
        while not self._stop_event.is_set():
            self._pending_event.wait(0.1)
            if self._stop_event.is_set():
                break
            with self._pending_lock:
                frames = self._pending_frames
                self._pending_frames = None
                self._pending_event.clear()
            if frames is None:
                continue
            try:
                self._process_frames(*frames)
            except Exception as exc:
                with self._stats_lock:
                    self._stats_errors += 1
                if not self._stop_event.is_set() and rclpy.ok():
                    self.get_logger().error(
                        f'Supply frame processing failed: {exc}',
                        throttle_duration_sec=2.0)

    def _process_frames(
        self, color_msg: CompressedImage, depth_msg: CompressedImage
    ) -> None:
        """Decode, infer, fuse depth/TF, and publish one synchronized pair."""
        total_started = perf_counter()
        self._diag_frame_count += 1
        if self._diag_frame_count == 1:
            color_t = (
                color_msg.header.stamp.sec
                + color_msg.header.stamp.nanosec * 1e-9
            )
            depth_t = (
                depth_msg.header.stamp.sec
                + depth_msg.header.stamp.nanosec * 1e-9
            )
            self.get_logger().warn(
                f'SYNC CALLBACK 진입 | '
                f'dt={abs(color_t - depth_t):.6f}s | '
                f'fx_none={self.fx is None} | '
                f'model_none={self.model is None}'
            )

        if self.fx is None:
            self.get_logger().error(
                'fx가 None이어서 프레임 처리를 중단합니다.',
                throttle_duration_sec=5.0
            )
            return

        if self.model is None:
            if not self._model_terminated:
                self.get_logger().error(
                    'OpenVINO model이 None이어서 프레임 처리를 중단합니다.',
                    throttle_duration_sec=5.0
                )
            return

        try:
            color = self.bridge.compressed_imgmsg_to_cv2(
                color_msg, desired_encoding='bgr8')
            depth = self._to_meters(decode_compressed_depth(depth_msg))
        except Exception as exc:
            self.get_logger().error(
                f'압축 카메라 이미지 디코딩 실패: {exc}',
                throttle_duration_sec=5.0
            )
            return

        if self._diag_frame_count == 1:
            self.get_logger().warn('OpenVINO 추론 시작')
        decode_done = perf_counter()
        prediction = self._infer(color)
        if prediction is None:
            return
        raw_detections = prediction.detections
        detections = [
            (detection.confidence, *detection.xyxy)
            for detection in raw_detections
        ]
        inference_done = perf_counter()
        if self._diag_frame_count == 1:
            self.get_logger().warn('OpenVINO 추론 완료')

        # detections(전체 (conf, x1, y1, x2, y2) 리스트, "채택된 하나(best)"로
        # 좁혀지기 전 원본)를 depth 필터 통과 여부와 무관하게 그대로
        # person 호환 JSON으로 발행 — 컬러 검출 자체가 됐는지 확인하는 용도라,
        # 아래에서 depth 무효로 전부 버려지는 경우와 구분해서 디버깅할 수 있다.
        self._publish_detections(color_msg.header, color, raw_detections)

        best = None  # (conf, X, Y, Z, bbox, cls_name)
        best_too_far = None  # (conf, z, bbox) - 팔 반경 밖(2단계 필터, 모듈 docstring 참고)
        # 단일 클래스로 학습돼 class_id로 이름을 구분할 필요가 없다 -
        # confidence 임계값을 넘는 탐지는 전부 target(기본 'supplybox')로 취급.
        for conf, x1, y1, x2, y2 in detections:
            if best and conf <= best[0]:
                continue

            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
            z = self._sample_depth(depth, u, v)
            if z <= 0.0:
                continue

            if z > self.max_d:
                # depth 범위 밖이면 목표를 발행하지 않는다.
                if best_too_far is None or conf > best_too_far[0]:
                    best_too_far = (conf, z, (x1, y1, x2, y2))
                continue

            X = (u - self.cx) * z / self.fx
            Y = (v - self.cy) * z / self.fy
            best = (conf, X, Y, z, (x1, y1, x2, y2), self.target, u, v)

        if best is not None:
            conf, X, Y, Z, bbox, cls_name, u, v = best

            pt = PointStamped()
            pt.header = color_msg.header
            pt.point.x = X
            pt.point.y = Y
            pt.point.z = Z
            try:
                transform_time = (
                    Time.from_msg(pt.header.stamp)
                    if pt.header.stamp.sec != 0 or pt.header.stamp.nanosec != 0
                    else Time()
                )
                transform, used_latest_transform, extrapolation = (
                    lookup_transform_with_latest_fallback(
                        self.tf_buffer, self.planning_frame,
                        pt.header.frame_id, transform_time,
                        Duration(seconds=0.5))
                )
                if used_latest_transform:
                    # GRASP_WAIT에서 팔이 정지한 뒤 정밀인식을 시작하므로,
                    # RealSense image stamp가 joint TF보다 앞선 경우에 한해
                    # 최신 공통 TF를 써도 공간 오차가 생기지 않는다.
                    self.get_logger().warn(
                        'Image/arm TF timestamps are not synchronized; using the '
                        'latest transform for stationary GRASP_WAIT: '
                        f'{extrapolation}',
                        throttle_duration_sec=5.0)
                base_pt = do_transform_point(pt, transform)
                # latest fallback 좌표에 원래 이미지 시각을 붙이면 downstream이
                # 다시 그 과거/미래 시각으로 TF 조회할 수 있다. 실제 사용한
                # transform 시각을 명시한다.
                base_pt.header.stamp = (
                    transform.header.stamp
                    if used_latest_transform else pt.header.stamp
                )
                base_pt.header.frame_id = self.planning_frame
            except Exception as exc:
                self.get_logger().warn(
                    f'{pt.header.frame_id} -> {self.planning_frame} TF 변환 실패: {exc}',
                    throttle_duration_sec=5.0)
                return

            self.get_logger().info(
                f'[{cls_name} conf={conf:.2f}]  '
                f'base_actuator X={base_pt.point.x:.3f} '
                f'Y={base_pt.point.y:.3f} Z={base_pt.point.z:.3f} m '
                f'(camera depth={Z:.3f}m)')

            if is_within_base_x_stop_distance(base_pt.point.x, self.stop_base_x_m):
                self._target_confirm_count += 1
                if self._target_confirm_count >= self.target_confirm_frames:
                    self.pub_target_depth.publish(Float64(data=float(Z)))
                    self.pub_point.publish(base_pt)
            else:
                self._target_confirm_count = 0
                self.get_logger().info(
                    f'Supplybox outside grasp X range or invalid: {base_pt.point.x}; '
                    'holding position, target withheld.',
                    throttle_duration_sec=2.0)

            # 디버그 이미지 오버레이
            bx1, by1, bx2, by2 = (int(c) for c in bbox)
            cv2.rectangle(color, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
            cv2.circle(color, (u, v), 6, (0, 0, 255), -1)
            cv2.putText(
                color,
                f'{cls_name} {conf:.2f} | Z={Z:.2f}m',
                (bx1, by1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        elif best_too_far is not None:
            conf, z, bbox = best_too_far
            self._target_confirm_count = 0
            self.get_logger().info(
                f"'{self.target}' conf={conf:.2f} z={z:.2f}m - 좌표 계산 depth "
                f"범위(max_depth_m={self.max_d}m) 밖; 정지 유지, 목표 미발행.",
                throttle_duration_sec=2.0,
            )

            # 디버그 이미지 오버레이(주황색 - 탐지는 됐지만 너무 멀다는 표시,
            # 초록색 성공 박스와 구분).
            bx1, by1, bx2, by2 = (int(c) for c in bbox)
            cv2.rectangle(color, (bx1, by1), (bx2, by2), (0, 165, 255), 2)
            cv2.putText(
                color,
                f'TOO FAR {conf:.2f} | Z={z:.2f}m > {self.max_d}m',
                (bx1, by1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        elif detections:
            # [2026-08-30] 컬러 검출은 됐는데(detections 비어있지 않음) 전부
            # depth 무효(z<=0)로 스킵된 경우 — 예전엔 이 케이스가 완전히
            # 무음이라 "왜 bbox가 안 뜨는지" 디버깅이 안 됐다. 컬러 검출
            # 자체는 됐다는 걸(=모델/조명 문제가 아니라 depth 쪽 문제라는 걸)
            # 알 수 있게 로그를 남긴다. best_too_far가 None이라는 건 depth가
            # 유효했던 탐지가 하나도 없었다는 뜻이라 이 분기까지 내려온다.
            self.get_logger().warn(
                f'{len(detections)}개 검출됐지만 전부 depth 무효(z<=0)로 스킵됨 — '
                f'depth 카메라 연결/범위(0.05~{self.max_d}m) 확인할 것',
                throttle_duration_sec=2.0)

        if best is None:
            self._target_confirm_count = 0

        spatial_done = perf_counter()

        # 탐지 성공 여부와 무관하게 매 프레임 RGB를 그대로 퍼블리시
        jpeg_started = perf_counter()
        dbg = self.bridge.cv2_to_compressed_imgmsg(color, dst_format='jpg')
        dbg.header = color_msg.header
        self.pub_debug.publish(dbg)
        done = perf_counter()

        self._record_timings(
            decode_ms=(decode_done - total_started) * 1000.0,
            preprocess_ms=prediction.preprocess_ms,
            inference_ms=prediction.inference_ms,
            postprocess_ms=prediction.postprocess_ms,
            spatial_ms=(spatial_done - inference_done) * 1000.0,
            jpeg_ms=(done - jpeg_started) * 1000.0,
            total_ms=(done - total_started) * 1000.0,
        )

    def _record_timings(self, **timings_ms: float) -> None:
        with self._stats_lock:
            self._stats_processed += 1
            for name, value in timings_ms.items():
                self._stats_totals[name] += float(value)

    def _report_stats(self) -> None:
        now = perf_counter()
        with self._stats_lock:
            period = max(now - self._stats_started, 1e-9)
            processed = self._stats_processed
            errors = self._stats_errors
            overwritten = self._stats_overwritten
            totals = dict(self._stats_totals)
            self._stats_started = now
            self._stats_processed = 0
            self._stats_errors = 0
            self._stats_overwritten = 0
            self._stats_totals = {name: 0.0 for name in self._stats_totals}

        if processed:
            averages = ' '.join(
                f'{name}={total / processed:.1f}ms'
                for name, total in totals.items()
            )
        else:
            averages = 'no completed frames'
        self.get_logger().info(
            f'supply stats ({period:.1f}s): proc={processed / period:.1f}fps '
            f'overwritten={overwritten} errors={errors} | {averages}')

    def shutdown_worker(self) -> None:
        """Stop the inference worker before ROS publishers are destroyed."""
        self._stop_event.set()
        self._pending_event.set()
        if self._worker.is_alive():
            self._worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = SupplyNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_worker()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
