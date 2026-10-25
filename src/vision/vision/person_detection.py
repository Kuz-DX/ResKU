#!/usr/bin/env python3
"""Multi-camera person detection with two weighted inference lanes."""

from __future__ import annotations

import json
import signal
import threading
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vision.camera_lanes import (
    Lane,
    LatestFrameSlots,
    next_job,
    parse_pattern,
    validate_lane_assignments,
)
from vision.ov_person_detector import OpenVINOPersonDetector


@dataclass(frozen=True)
class Detection:
    """One person detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str = 'person'


@dataclass(frozen=True)
class TimedPrediction:
    """Backend-neutral detections and inference time."""

    detections: list[Detection]
    inference_ms: float


@dataclass
class CameraStats:
    """Per-report-interval counters updated by lane workers."""

    processed: int = 0
    errors: int = 0
    stale: int = 0
    inference_total_ms: float = 0.0
    inference_count: int = 0


class YOLOPersonDetector:
    """Ultralytics fallback preserving the existing person output contract."""

    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.5,
        infer_size: int = 640,
        device: str = 'cpu',
    ) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f'YOLO person model not found: {self.model_path}')
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0 and 1.')
        if infer_size <= 0:
            raise ValueError('infer_size must be greater than zero.')
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                'ultralytics is required for person.pt; install the vision package '
                'requirements first.'
            ) from exc

        self.confidence_threshold = float(confidence_threshold)
        self.infer_size = int(infer_size)
        self.device = str(device).strip().lower() or 'cpu'
        self.model = YOLO(str(self.model_path), task='detect')
        self.precision = 'UNKNOWN'
        try:
            self.precision = str(next(self.model.model.parameters()).dtype).replace(
                'torch.', ''
            ).upper()
        except Exception:
            pass
        self.model_input_shape = f'dynamic(imgsz={self.infer_size})'
        self.num_streams = 'n/a'

    def predict_timed(self, image_bgr: np.ndarray) -> TimedPrediction:
        if (
            image_bgr is None
            or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.shape[0] == 0
            or image_bgr.shape[1] == 0
        ):
            raise ValueError('image_bgr must be a non-empty HxWx3 BGR image.')

        started = perf_counter()
        results = self.model.predict(
            source=image_bgr,
            conf=self.confidence_threshold,
            imgsz=self.infer_size,
            device=self.device,
            verbose=False,
        )
        detections: list[Detection] = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                model_name = str(result.names.get(class_id, f'class_{class_id}'))
                if model_name.lower() not in {'person', 'pedestrian'}:
                    continue
                detections.append(
                    Detection(
                        xyxy=tuple(float(value) for value in box.xyxy[0].tolist()),
                        confidence=float(box.conf[0]),
                        class_id=class_id,
                    )
                )
        return TimedPrediction(
            detections=detections,
            inference_ms=(perf_counter() - started) * 1000.0,
        )

    def predict(self, image_bgr: np.ndarray) -> list[Detection]:
        return self.predict_timed(image_bgr).detections


def _default_model_path(backend: str) -> Path:
    """Resolve a backend model from install, source, then local machine storage."""
    relative = (
        Path('person_480_openvino_model')
        if backend == 'openvino'
        else Path('person.pt')
    )
    installed = Path(get_package_share_directory('vision')) / 'models' / relative
    if installed.exists():
        return installed
    source = Path(__file__).resolve().parents[1] / 'models' / relative
    if source.exists():
        return source
    local = Path.home() / 'models' / relative
    if local.exists():
        return local
    return installed


def _stamp_tuple(message: CompressedImage) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


class PersonDetectionNode(Node):
    """Latest-frame dispatcher with two concurrent inference lanes."""

    def __init__(self) -> None:
        super().__init__('person_detection_yolo')
        self._declare_parameters()
        get = lambda name: self.get_parameter(name).value

        self.backend = str(get('backend')).strip().lower()
        self.perf_hint = str(get('perf_hint')).strip().upper()
        self.jpeg_quality = int(get('jpeg_quality'))
        self.publish_visualization = bool(get('publish_visualization'))
        self.work_stealing = bool(get('lane_work_stealing'))
        self.stats_period_s = float(get('stats_period_s'))
        self._validate_parameters()

        self.camera_topics = self._camera_topics(get)
        enabled = {camera for camera, *_topics in self.camera_topics}
        lane_patterns = {
            'lane1': parse_pattern(str(get('lane1_pattern')), enabled),
            'lane2': parse_pattern(str(get('lane2_pattern')), enabled),
        }
        validate_lane_assignments(lane_patterns, enabled)
        self.lanes = [
            Lane('lane1', lane_patterns['lane1'], float(get('lane1_max_fps'))),
            Lane('lane2', lane_patterns['lane2'], float(get('lane2_max_fps'))),
        ]
        self.active_lanes = [lane for lane in self.lanes if lane.pattern]
        for lane in self.lanes:
            if not lane.pattern:
                self.get_logger().info(
                    f'{lane.name} has no active cameras and will not start.'
                )
        if not self.active_lanes:
            raise ValueError('No inference lane has an active camera.')

        requested_model = str(get('model_path')).strip()
        model_path = (
            Path(requested_model).expanduser()
            if requested_model
            else _default_model_path(self.backend)
        )
        confidence = float(get('confidence_threshold'))
        infer_size = int(get('infer_size'))
        device = str(get('device'))
        self._ultralytics_lock = threading.Lock()
        self._lane_requests: dict[str, Any] = {}
        if self.backend == 'openvino':
            self.detector: Any = OpenVINOPersonDetector(
                model_path=model_path,
                confidence_threshold=confidence,
                infer_size=infer_size,
                device=device,
                perf_hint=self.perf_hint,
                num_requests=len(self.active_lanes),
            )
            self.detector.warmup(3)
            self._lane_requests = {
                lane.name: self.detector.create_request() for lane in self.active_lanes
            }
        else:
            self.detector = YOLOPersonDetector(
                model_path=model_path,
                confidence_threshold=confidence,
                infer_size=infer_size,
                device=device,
            )

        self._slots = LatestFrameSlots()
        self._stop_event = threading.Event()
        self._shutdown_lock = threading.Lock()
        self._workers_started = False
        self._workers_stopped = False
        self._stats_lock = threading.Lock()
        self._stats_started = perf_counter()
        self._camera_stats = {camera: CameraStats() for camera in enabled}
        self._lane_processed = {lane.name: 0 for lane in self.active_lanes}
        self._last_published_stamp: dict[str, tuple[int, int]] = {}
        self._publish_locks = {camera: threading.Lock() for camera in enabled}

        subscription_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.detections_pubs: dict[str, Any] = {}
        self.image_pubs: dict[str, Any] = {}
        self._camera_subscriptions: list[Any] = []
        for camera, input_topic, detections_topic, output_topic in self.camera_topics:
            self.detections_pubs[camera] = self.create_publisher(
                String, detections_topic, qos_profile_sensor_data
            )
            self.image_pubs[camera] = self.create_publisher(
                CompressedImage, output_topic, qos_profile_sensor_data
            )
            self._camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    input_topic,
                    lambda message, name=camera: self._slots.put(name, message),
                    subscription_qos,
                )
            )
            self.get_logger().info(
                f'{camera}: input={input_topic}, detections={detections_topic}, '
                f'bbox_image={output_topic}'
            )

        self._stats_timer = None
        if self.stats_period_s > 0.0:
            self._stats_timer = self.create_timer(
                self.stats_period_s, self._report_stats
            )
        self._threads = [
            threading.Thread(
                target=self._lane_loop,
                args=(lane,),
                name=f'person-detection-{lane.name}',
            )
            for lane in self.active_lanes
        ]
        self._log_startup()

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            'backend': 'openvino',
            'model_path': '',
            'perf_hint': 'LATENCY',
            'lane1_pattern': 'left,right',
            'lane2_pattern': 'drive,drive,arm',
            'lane1_max_fps': 0.0,
            'lane2_max_fps': 0.0,
            'lane_work_stealing': False,
            'stats_period_s': 0.0,
            'input_topic': '/drive/camera/color/image_raw/compressed',
            'detections_topic': '/person_detection/detections',
            'output_topic': '/drive/person/detecion',
            'left_input_topic': '/side/left/image_raw/compressed',
            'left_detections_topic': '/left/person/detections',
            'left_output_topic': '/left/person/detection',
            'right_input_topic': '/side/right/image_raw/compressed',
            'right_detections_topic': '/right/person/detections',
            'right_output_topic': '/right/person/detection',
            'arm_input_topic': '/arm/camera/color/image_raw/compressed',
            'arm_detections_topic': '/arm/person/detections',
            'arm_output_topic': '/arm/person/detection',
            'enable_arm_camera': True,
            'enable_side_cameras': True,
            'confidence_threshold': 0.5,
            'infer_size': 640,
            'device': 'cpu',
            'jpeg_quality': 90,
            'publish_visualization': True,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _validate_parameters(self) -> None:
        if self.backend not in {'openvino', 'ultralytics'}:
            raise ValueError("backend must be 'openvino' or 'ultralytics'.")
        if self.perf_hint not in {'LATENCY', 'THROUGHPUT'}:
            raise ValueError("perf_hint must be 'LATENCY' or 'THROUGHPUT'.")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100.')
        if self.stats_period_s < 0.0:
            raise ValueError('stats_period_s must not be negative.')

    @staticmethod
    def _camera_topics(get: Any) -> list[tuple[str, str, str, str]]:
        topics = [
            (
                'drive',
                str(get('input_topic')),
                str(get('detections_topic')),
                str(get('output_topic')),
            )
        ]
        if bool(get('enable_side_cameras')):
            topics.extend(
                [
                    (
                        'left',
                        str(get('left_input_topic')),
                        str(get('left_detections_topic')),
                        str(get('left_output_topic')),
                    ),
                    (
                        'right',
                        str(get('right_input_topic')),
                        str(get('right_detections_topic')),
                        str(get('right_output_topic')),
                    ),
                ]
            )
        if bool(get('enable_arm_camera')):
            topics.append(
                (
                    'arm',
                    str(get('arm_input_topic')),
                    str(get('arm_detections_topic')),
                    str(get('arm_output_topic')),
                )
            )
        return topics

    def _log_startup(self) -> None:
        self.get_logger().info(
            f'Person detector ready: backend={self.backend} '
            f'model={self.detector.model_path} precision={self.detector.precision} '
            f'input_shape={self.detector.model_input_shape} hint={self.perf_hint} '
            f'streams={self.detector.num_streams} work_stealing={self.work_stealing}'
        )
        for lane in self.lanes:
            pattern = ','.join(lane.pattern) if lane.pattern else '(disabled)'
            self.get_logger().info(
                f'{lane.name}: pattern={pattern} max_fps={lane.max_fps:g}'
            )

    def _lane_loop(self, lane: Lane) -> None:
        request = self._lane_requests.get(lane.name)
        while not self._stop_event.is_set():
            job = next_job(
                lane,
                self.active_lanes,
                self._slots,
                self.work_stealing,
                perf_counter(),
            )
            if job is None:
                self._stop_event.wait(0.002)
                continue
            camera, _owner_lane, message = job
            try:
                image = cv2.imdecode(
                    np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is None:
                    raise ValueError('could not decode compressed input image')
                if self.backend == 'openvino':
                    prediction = self.detector.predict_timed(image, request=request)
                else:
                    with self._ultralytics_lock:
                        prediction = self.detector.predict_timed(image)
                if self._stop_event.is_set() or not rclpy.ok():
                    break
                if self._publish_result(
                    camera, message, image, prediction.detections
                ):
                    self._record_processed(
                        lane.name, camera, prediction.inference_ms
                    )
            except Exception as exc:
                self._record_error(camera)
                if not self._stop_event.is_set() and rclpy.ok():
                    self.get_logger().error(f'[{camera}] detection failed: {exc}')

    def _publish_result(
        self,
        camera: str,
        message: CompressedImage,
        image: np.ndarray,
        detections: list[Any],
    ) -> bool:
        stamp = _stamp_tuple(message)
        # A per-camera lock preserves result ordering without serializing JPEG
        # work for cameras assigned to different inference lanes.
        with self._publish_locks[camera]:
            previous = self._last_published_stamp.get(camera)
            if previous is not None and stamp < previous:
                with self._stats_lock:
                    self._camera_stats[camera].stale += 1
                return False
            if self._stop_event.is_set() or not rclpy.ok():
                return False
            self._last_published_stamp[camera] = stamp

            payload = {
                'camera': camera,
                'stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
                'frame_id': message.header.frame_id,
                'image_size': {'width': image.shape[1], 'height': image.shape[0]},
                'detections': [
                    {
                        'class_id': detection.class_id,
                        'class_name': detection.class_name,
                        'confidence': detection.confidence,
                        'xyxy': list(detection.xyxy),
                    }
                    for detection in detections
                ],
            }
            output = String()
            output.data = json.dumps(
                payload, ensure_ascii=False, separators=(',', ':')
            )
            self.detections_pubs[camera].publish(output)

            if self.publish_visualization:
                for detection in detections:
                    x1, y1, x2, y2 = (
                        int(round(value)) for value in detection.xyxy
                    )
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        image,
                        f'{detection.class_name} {detection.confidence:.2f}',
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                success, encoded = cv2.imencode(
                    '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                )
                if not success:
                    raise ValueError('could not encode bbox visualization as JPEG')
                image_message = CompressedImage()
                image_message.header = message.header
                image_message.format = 'jpeg'
                image_message.data = encoded.tobytes()
                self.image_pubs[camera].publish(image_message)
        return True

    def _record_processed(
        self, worker_lane: str, camera: str, inference_ms: float
    ) -> None:
        with self._stats_lock:
            stats = self._camera_stats[camera]
            stats.processed += 1
            stats.inference_total_ms += float(inference_ms)
            stats.inference_count += 1
            self._lane_processed[worker_lane] += 1

    def _record_error(self, camera: str) -> None:
        with self._stats_lock:
            self._camera_stats[camera].errors += 1

    def _report_stats(self) -> None:
        now = perf_counter()
        period = max(now - self._stats_started, 1e-9)
        overwrites = self._slots.take_overwrite_counts()
        with self._stats_lock:
            camera_rows = {
                camera: CameraStats(
                    processed=stats.processed,
                    errors=stats.errors,
                    stale=stats.stale,
                    inference_total_ms=stats.inference_total_ms,
                    inference_count=stats.inference_count,
                )
                for camera, stats in self._camera_stats.items()
            }
            lane_counts = dict(self._lane_processed)
            self._camera_stats = {
                camera: CameraStats() for camera in self._camera_stats
            }
            self._lane_processed = {lane: 0 for lane in self._lane_processed}
            self._stats_started = now

        lane_summary = ' '.join(
            f'{lane}={count / period:.1f}fps'
            for lane, count in lane_counts.items()
        )
        self.get_logger().info(f'lane stats ({period:.1f}s): {lane_summary}')
        for camera in [item[0] for item in self.camera_topics]:
            stats = camera_rows[camera]
            inference_avg = (
                stats.inference_total_ms / stats.inference_count
                if stats.inference_count
                else 0.0
            )
            self.get_logger().info(
                f'  {camera}: proc={stats.processed / period:.1f}fps '
                f'infer_avg={inference_avg:.1f}ms '
                f'overwritten={overwrites.get(camera, 0)} '
                f'stale={stats.stale} errors={stats.errors}'
            )

    def start_workers(self) -> None:
        """Start each configured lane once, after node construction succeeds."""
        with self._shutdown_lock:
            if self._workers_stopped:
                raise RuntimeError('Cannot start workers after shutdown.')
            if self._workers_started:
                return
            self._workers_started = True
        for thread in self._threads:
            thread.start()

    def shutdown_workers(self) -> None:
        """Stop lane work and join every thread before destroying ROS entities."""
        with self._shutdown_lock:
            if self._workers_stopped:
                return
            self._workers_stopped = True
            if self._stats_timer is not None:
                self._stats_timer.cancel()
            self._stop_event.set()
        for thread in self._threads:
            if thread is not threading.current_thread() and thread.is_alive():
                thread.join()


def _raise_keyboard_interrupt(_signum: int, _frame: Any) -> None:
    raise KeyboardInterrupt


def main(args: list[str] | None = None) -> None:
    previous_sigint = signal.signal(signal.SIGINT, signal.default_int_handler)
    previous_sigterm = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    initialized = False
    node: PersonDetectionNode | None = None
    try:
        # Keep the ROS context alive until lane workers have stopped and joined.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = PersonDetectionNode()
        node.start_workers()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward a second signal while cleanup is running.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.shutdown_workers()
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
