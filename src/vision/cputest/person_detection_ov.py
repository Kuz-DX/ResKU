#!/usr/bin/env python3
"""ROS 2 multi-camera person detector for backend and scheduler benchmarks."""

from __future__ import annotations

import csv
import json
import os
import platform
import threading
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import mean
from time import perf_counter, time
from typing import Any, Callable

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

CAMERA_ORDER = ('drive', 'arm', 'left', 'right')
METRIC_NAMES = (
    'decode',
    'pre',
    'infer',
    'post',
    'draw_enc',
    'pub',
    'queue_wait',
    'latency',
)


@dataclass
class Detection:
    """One person detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str = 'person'


@dataclass
class FrameSlot:
    """The newest compressed frame retained for one camera."""

    message: CompressedImage
    received_perf: float
    received_wall: float
    sequence: int


@dataclass
class AsyncContext:
    """State retained until one asynchronous inference request completes."""

    camera: str
    message: CompressedImage
    image: np.ndarray
    tensor: np.ndarray
    letterbox: Any
    received_perf: float
    received_wall: float
    decode_ms: float
    preprocess_ms: float
    queue_wait_ms: float
    infer_started: float


@dataclass
class CameraStats:
    """Fixed-size interval statistics for one camera."""

    input_count: int = 0
    processed_count: int = 0
    drop_count: int = 0
    stale_drop_count: int = 0
    error_count: int = 0
    values: dict[str, deque[float]] = field(
        default_factory=lambda: {
            name: deque(maxlen=4096) for name in METRIC_NAMES
        }
    )
    max_gap_ms: float = 0.0
    last_publish_perf: float | None = None

    def reset_interval(self) -> None:
        """Reset interval data while preserving the previous publish time."""
        self.input_count = 0
        self.processed_count = 0
        self.drop_count = 0
        self.stale_drop_count = 0
        self.error_count = 0
        for values in self.values.values():
            values.clear()
        self.max_gap_ms = 0.0


@dataclass(frozen=True)
class TimedDetections:
    """Backend-neutral result used by the node's synchronous path."""

    detections: list[Detection]
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float


class UltralyticsPersonDetector:
    """Self-contained adapter matching the original node's predict call."""

    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float,
        infer_size: int,
        device: str,
    ) -> None:
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'YOLO person model not found: {path}')
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('ultralytics is required for the ultralytics backend.') from exc
        self.model_path = path
        self.confidence_threshold = confidence_threshold
        self.infer_size = infer_size
        self.device = device.strip().lower() or 'cpu'
        started = perf_counter()
        self.model = YOLO(str(path), task='detect')
        self.load_compile_ms = (perf_counter() - started) * 1000.0
        self.load_ms = self.load_compile_ms
        self.compile_ms = 0.0
        self.precision = 'UNKNOWN'
        try:
            self.precision = str(next(self.model.model.parameters()).dtype).replace('torch.', '').upper()
        except Exception:
            pass
        self.output_shape: str = 'Ultralytics Results[N,6]'
        self.optimal_num_requests: str = 'n/a'
        self.num_streams: str = 'n/a'
        self.warmup_ms = 0.0

    def predict_timed(self, image_bgr: np.ndarray) -> TimedDetections:
        """Run predict and expose Ultralytics' own stage timings."""
        results = self.model.predict(
            source=image_bgr,
            conf=self.confidence_threshold,
            imgsz=self.infer_size,
            device=self.device,
            verbose=False,
        )
        detections: list[Detection] = []
        pre_ms = infer_ms = post_ms = 0.0
        for result in results:
            speed = result.speed or {}
            pre_ms += float(speed.get('preprocess') or 0.0)
            infer_ms += float(speed.get('inference') or 0.0)
            post_ms += float(speed.get('postprocess') or 0.0)
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = str(result.names.get(class_id, f'class_{class_id}'))
                if class_name.lower() not in {'person', 'pedestrian'}:
                    continue
                detections.append(
                    Detection(
                        xyxy=tuple(float(value) for value in box.xyxy[0].tolist()),
                        confidence=float(box.conf[0]),
                        class_id=class_id,
                    )
                )
        return TimedDetections(detections, pre_ms, infer_ms, post_ms)

    def warmup(self, iterations: int = 3) -> float:
        """Warm up with the same black image for stable startup reporting."""
        dummy = np.zeros((self.infer_size, self.infer_size, 3), dtype=np.uint8)
        elapsed: list[float] = []
        for _ in range(iterations):
            started = perf_counter()
            self.predict_timed(dummy)
            elapsed.append((perf_counter() - started) * 1000.0)
        self.warmup_ms = float(mean(elapsed))
        return self.warmup_ms


class ProcessMemory:
    """Low-overhead CPU and memory sampler with a procfs fallback."""

    def __init__(self) -> None:
        self._process = None
        try:
            import psutil

            self._process = psutil.Process()
            self._process.cpu_percent(None)
        except ImportError:
            pass
        self._ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        self._last_cpu = self._proc_cpu_seconds()
        self._last_wall = perf_counter()

    @staticmethod
    def _proc_cpu_seconds_raw() -> tuple[int, int]:
        fields = Path('/proc/self/stat').read_text(encoding='utf-8').split()
        return int(fields[13]), int(fields[14])

    def _proc_cpu_seconds(self) -> float:
        user_ticks, system_ticks = self._proc_cpu_seconds_raw()
        return (user_ticks + system_ticks) / self._ticks

    @staticmethod
    def _pss_mb() -> float | None:
        try:
            for line in Path('/proc/self/smaps_rollup').read_text(encoding='utf-8').splitlines():
                if line.startswith('Pss:'):
                    return float(line.split()[1]) / 1024.0
        except (OSError, ValueError):
            return None
        return None

    def sample(self) -> tuple[float, float, float | None]:
        """Return process CPU percent, RSS MiB and optional PSS MiB."""
        if self._process is not None:
            cpu = float(self._process.cpu_percent(None))
            rss = float(self._process.memory_info().rss) / (1024.0 * 1024.0)
        else:
            now = perf_counter()
            cpu_seconds = self._proc_cpu_seconds()
            wall_delta = max(now - self._last_wall, 1e-9)
            cpu = 100.0 * (cpu_seconds - self._last_cpu) / wall_delta
            self._last_cpu, self._last_wall = cpu_seconds, now
            resident_pages = int(Path('/proc/self/statm').read_text().split()[1])
            rss = resident_pages * os.sysconf('SC_PAGE_SIZE') / (1024.0 * 1024.0)
        return cpu, rss, self._pss_mb()


def _default_model_path() -> Path:
    installed = Path(get_package_share_directory('vision')) / 'models' / 'person.pt'
    if installed.is_file():
        return installed
    return Path(__file__).resolve().parents[1] / 'models' / 'person.pt'


def _stamp_tuple(message: CompressedImage) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def _percentile(values: deque[float] | list[float], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


class PersonDetectionOVNode(Node):
    """Benchmarkable multi-camera detector with latest-frame scheduling."""

    def __init__(self) -> None:
        super().__init__('person_detection_ov')
        self._declare_parameters()
        get = lambda name: self.get_parameter(name).value
        self.backend = str(get('backend')).strip().lower()
        self.perf_hint = str(get('perf_hint')).strip().upper()
        self.num_requests = int(get('num_requests'))
        self.scheduler = str(get('scheduler')).strip().lower()
        self.priority_pattern = [str(item) for item in get('priority_pattern')]
        self.max_wait_s = int(get('max_wait_ms')) / 1000.0
        self.stale_input_s = int(get('stale_input_ms')) / 1000.0
        self.async_infer = bool(get('async_infer'))
        self.jpeg_quality = int(get('jpeg_quality'))
        self.publish_visualization = bool(get('publish_visualization'))
        self.latency_mode = str(get('latency_mode')).strip().lower()
        self.print_table = bool(get('print_table'))
        self.topic_suffix = str(get('topic_suffix'))
        self.log_period_s = float(get('log_period_s'))
        self._validate_parameters()

        if self.backend == 'ultralytics' and (self.num_requests != 1 or self.async_infer):
            self.get_logger().warning(
                'ultralytics does not support num_requests>1 or async_infer here; '
                'forcing num_requests=1 and async_infer=false.'
            )
            self.num_requests = 1
            self.async_infer = False
        self._separated = (
            self.scheduler != 'fifo' or self.num_requests > 1 or self.async_infer
        )

        model_path = str(get('model_path'))
        self._openvino_detector_type: type[Any] | None = None
        self._ov_module: Any = None
        if self.backend == 'openvino':
            import openvino as ov
            from cputest.ov_person_detector import OpenVINOPersonDetector

            self._ov_module = ov
            self._openvino_detector_type = OpenVINOPersonDetector
            self.detector: Any = OpenVINOPersonDetector(
                model_path=model_path,
                confidence_threshold=float(get('confidence_threshold')),
                infer_size=int(get('infer_size')),
                device=str(get('device')),
                perf_hint=self.perf_hint,
                num_requests=self.num_requests,
            )
        else:
            self.detector = UltralyticsPersonDetector(
                model_path=model_path,
                confidence_threshold=float(get('confidence_threshold')),
                infer_size=int(get('infer_size')),
                device=str(get('device')),
            )
        self.detector.warmup(3)

        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._publish_lock = threading.Lock()
        self._stopping = False
        self._sequence = 0
        self._rr_index = 0
        self._priority_index = 0
        self._slots: dict[str, FrameSlot] = {}
        self._last_selected = {name: perf_counter() for name in CAMERA_ORDER}
        self._last_published_stamp: dict[str, tuple[int, int]] = {}
        self._stats = {name: CameraStats() for name in CAMERA_ORDER}
        self._stats_started = perf_counter()
        self._memory = ProcessMemory()

        qos_depth = int(get('qos_depth'))
        self.qos = self._make_qos(qos_depth)
        self.qos_description = 'sensor_data' if qos_depth == 0 else f'BEST_EFFORT/VOLATILE/KEEP_LAST/{qos_depth}'
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self.detections_pubs: dict[str, Any] = {}
        self.image_pubs: dict[str, Any] = {}
        self._camera_subscriptions: list[Any] = []
        self.camera_topics = self._camera_topics(get)
        for camera, input_topic, detections_topic, output_topic in self.camera_topics:
            self.detections_pubs[camera] = self.create_publisher(
                String, detections_topic, self.qos
            )
            self.image_pubs[camera] = self.create_publisher(
                CompressedImage, output_topic, self.qos
            )
            self._camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    input_topic,
                    lambda message, name=camera: self._image_callback(message, name),
                    self.qos,
                    callback_group=self._callback_group,
                )
            )

        self._csv_file, self._csv_writer, self._csv_path, self._meta_path = self._open_logs(
            str(get('csv_dir')), str(get('run_tag'))
        )
        self._async_queue: Any = None
        if self.async_infer:
            assert self._openvino_detector_type is not None
            assert isinstance(self.detector, self._openvino_detector_type)
            self._async_queue = self._ov_module.AsyncInferQueue(
                self.detector.compiled_model, self.num_requests
            )
            self._async_queue.set_callback(self._async_complete)

        self._write_startup_meta(model_path)
        self._worker: threading.Thread | None = None
        if self._separated:
            self._worker = threading.Thread(
                target=self._worker_loop,
                name='person-detection-worker',
                daemon=True,
            )
            self._worker.start()
        self._timer = self.create_timer(
            self.log_period_s,
            self._report_interval,
            callback_group=self._callback_group,
        )

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            'model_path': str(_default_model_path()),
            'backend': 'ultralytics',
            'perf_hint': 'LATENCY',
            'num_requests': 1,
            'scheduler': 'fifo',
            'priority_pattern': ['drive', 'arm', 'drive', 'left', 'right'],
            'max_wait_ms': 300,
            'stale_input_ms': 1000,
            'async_infer': False,
            'log_period_s': 5.0,
            'csv_dir': '~/perf_logs',
            'run_tag': '',
            'latency_mode': 'receive',
            'topic_suffix': '_ov',
            'qos_depth': 0,
            'print_table': True,
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
        if self.backend not in {'ultralytics', 'openvino'}:
            raise ValueError("backend must be 'ultralytics' or 'openvino'.")
        if self.perf_hint not in {'LATENCY', 'THROUGHPUT'}:
            raise ValueError("perf_hint must be 'LATENCY' or 'THROUGHPUT'.")
        if self.scheduler not in {'fifo', 'roundrobin', 'priority'}:
            raise ValueError("scheduler must be 'fifo', 'roundrobin', or 'priority'.")
        if self.num_requests <= 0:
            raise ValueError('num_requests must be greater than zero.')
        if self.max_wait_s < 0.0 or self.stale_input_s < 0.0:
            raise ValueError('max_wait_ms and stale_input_ms must not be negative.')
        if self.log_period_s <= 0.0:
            raise ValueError('log_period_s must be greater than zero.')
        if self.latency_mode not in {'receive', 'stamp'}:
            raise ValueError("latency_mode must be 'receive' or 'stamp'.")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100.')
        unknown = set(self.priority_pattern) - set(CAMERA_ORDER)
        if unknown or not self.priority_pattern:
            raise ValueError(f'priority_pattern has unknown cameras or is empty: {unknown}')

    @staticmethod
    def _make_qos(depth: int) -> QoSProfile:
        if depth == 0:
            return qos_profile_sensor_data
        if depth < 0:
            raise ValueError('qos_depth must be zero or greater.')
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _camera_topics(
        self, get: Callable[[str], Any]
    ) -> list[tuple[str, str, str, str]]:
        topics = [
            (
                'drive',
                str(get('input_topic')),
                str(get('detections_topic')) + self.topic_suffix,
                str(get('output_topic')) + self.topic_suffix,
            )
        ]
        if bool(get('enable_side_cameras')):
            topics.extend(
                [
                    ('left', str(get('left_input_topic')), str(get('left_detections_topic')) + self.topic_suffix, str(get('left_output_topic')) + self.topic_suffix),
                    ('right', str(get('right_input_topic')), str(get('right_detections_topic')) + self.topic_suffix, str(get('right_output_topic')) + self.topic_suffix),
                ]
            )
        if bool(get('enable_arm_camera')):
            topics.append(
                ('arm', str(get('arm_input_topic')), str(get('arm_detections_topic')) + self.topic_suffix, str(get('arm_output_topic')) + self.topic_suffix)
            )
        return topics

    def _open_logs(
        self, csv_dir: str, run_tag: str
    ) -> tuple[Any, csv.DictWriter, Path, Path]:
        directory = Path(csv_dir).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_tag = ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in run_tag.strip())
        suffix = f'_{safe_tag}' if safe_tag else ''
        base = (
            f'{timestamp}_{self.backend}_{self.perf_hint}_r{self.num_requests}_'
            f'{self.scheduler}_{str(self.async_infer).lower()}{suffix}'
        )
        csv_path = directory / f'{base}.csv'
        meta_path = directory / f'{base}_meta.json'
        stream = csv_path.open('w', encoding='utf-8', newline='')
        fields = [
            'timestamp', 'period_s', 'camera', 'backend', 'perf_hint',
            'num_requests', 'scheduler', 'async_infer', 'run_tag', 'latency_mode',
            'cpu_percent', 'rss_mb', 'pss_mb', 'in_fps', 'proc_fps', 'drop',
            'stale_drop', 'err',
        ]
        for metric in METRIC_NAMES:
            fields.extend((f'{metric}_avg_ms', f'{metric}_p95_ms'))
        fields.append('max_gap_ms')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        return stream, writer, csv_path, meta_path

    @staticmethod
    def _cpu_name() -> str:
        try:
            for line in Path('/proc/cpuinfo').read_text(encoding='utf-8').splitlines():
                if line.lower().startswith('model name'):
                    return line.split(':', 1)[1].strip()
        except OSError:
            pass
        return platform.processor() or 'unknown'

    def _write_startup_meta(self, requested_model_path: str) -> None:
        meta = {
            'backend': self.backend,
            'model_path': str(self.detector.model_path),
            'requested_model_path': requested_model_path,
            'precision': self.detector.precision,
            'device': self.detector.device,
            'cpu': self._cpu_name(),
            'cpu_threads': os.cpu_count(),
            'perf_hint': self.perf_hint,
            'num_streams': str(self.detector.num_streams),
            'optimal_num_requests': str(self.detector.optimal_num_requests),
            'num_requests': self.num_requests,
            'scheduler': self.scheduler,
            'priority_pattern': self.priority_pattern,
            'async_infer': self.async_infer,
            'qos': self.qos_description,
            'load_compile_ms': self.detector.load_compile_ms,
            'load_ms': self.detector.load_ms,
            'compile_ms': self.detector.compile_ms,
            'warmup_ms': self.detector.warmup_ms,
            'output_shape': str(self.detector.output_shape),
            'cameras': [
                {'name': camera, 'input': input_topic, 'detections': detections, 'output': output}
                for camera, input_topic, detections, output in self.camera_topics
            ],
            'csv_path': str(self._csv_path),
        }
        self._meta_path.write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8'
        )
        lines = [
            f'backend={self.backend} model={self.detector.model_path}',
            f'precision={self.detector.precision} device={self.detector.device} CPU={meta["cpu"]} threads={meta["cpu_threads"]}',
            f'hint={self.perf_hint} streams={meta["num_streams"]} requests={self.num_requests} optimal={meta["optimal_num_requests"]}',
            f'scheduler={self.scheduler} pattern={self.priority_pattern} async={self.async_infer}',
            f'qos={self.qos_description}',
            f'load={self.detector.load_ms:.1f}ms compile={self.detector.compile_ms:.1f}ms warmup(3 avg)={self.detector.warmup_ms:.1f}ms',
            f'output_shape={self.detector.output_shape}',
        ]
        lines.extend(
            f'{camera}: {input_topic} -> {detections}, {output}'
            for camera, input_topic, detections, output in self.camera_topics
        )
        width = max(len(line) for line in lines) + 2
        print('+' + '-' * width + '+', flush=True)
        for line in lines:
            print(f'| {line:<{width - 1}}|', flush=True)
        print('+' + '-' * width + '+', flush=True)

    def _image_callback(self, message: CompressedImage, camera: str) -> None:
        received_perf = perf_counter()
        received_wall = time()
        with self._lock:
            self._stats[camera].input_count += 1
        if not self._separated:
            slot = FrameSlot(message, received_perf, received_wall, 0)
            self._process_synchronous(camera, slot, 0.0)
            return
        with self._condition:
            if camera in self._slots:
                self._stats[camera].drop_count += 1
            self._sequence += 1
            self._slots[camera] = FrameSlot(
                message, received_perf, received_wall, self._sequence
            )
            self._condition.notify()

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or bool(self._slots))
                if self._stopping and not self._slots:
                    return
                selected = self._select_slot_locked()
            if selected is None:
                continue
            camera, slot = selected
            queue_wait_ms = (perf_counter() - slot.received_perf) * 1000.0
            if self.async_infer:
                self._submit_async(camera, slot, queue_wait_ms)
            else:
                self._process_synchronous(camera, slot, queue_wait_ms)

    def _select_slot_locked(self) -> tuple[str, FrameSlot] | None:
        now = perf_counter()
        stale = [
            camera for camera, slot in self._slots.items()
            if self.stale_input_s > 0.0 and now - slot.received_perf > self.stale_input_s
        ]
        for camera in stale:
            self._slots.pop(camera, None)
            self._stats[camera].drop_count += 1
        if not self._slots:
            return None

        if self.scheduler == 'fifo':
            camera = min(self._slots, key=lambda name: self._slots[name].sequence)
        elif self.scheduler == 'roundrobin':
            enabled = {item[0] for item in self.camera_topics}
            active = [name for name in CAMERA_ORDER if name in enabled]
            camera = self._cycle_pick(active, '_rr_index')
        else:
            overdue = [
                name for name in self._slots
                if now - self._last_selected.get(name, now) >= self.max_wait_s
            ]
            if overdue:
                camera = max(overdue, key=lambda name: now - self._last_selected.get(name, now))
            else:
                camera = self._cycle_pick(self.priority_pattern, '_priority_index')
        slot = self._slots.pop(camera)
        self._last_selected[camera] = now
        return camera, slot

    def _cycle_pick(self, pattern: list[str] | tuple[str, ...], index_name: str) -> str:
        index = int(getattr(self, index_name))
        for offset in range(len(pattern)):
            candidate_index = (index + offset) % len(pattern)
            candidate = pattern[candidate_index]
            if candidate in self._slots:
                setattr(self, index_name, (candidate_index + 1) % len(pattern))
                return candidate
        return next(iter(self._slots))

    @staticmethod
    def _decode(message: CompressedImage) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('could not decode compressed input image')
        return image

    def _process_synchronous(
        self, camera: str, slot: FrameSlot, queue_wait_ms: float
    ) -> None:
        started = perf_counter()
        try:
            image = self._decode(slot.message)
            decoded = perf_counter()
            result = self.detector.predict_timed(image)
            self._publish_result(
                camera=camera,
                message=slot.message,
                image=image,
                detections=result.detections,
                received_perf=slot.received_perf,
                received_wall=slot.received_wall,
                decode_ms=(decoded - started) * 1000.0,
                preprocess_ms=result.preprocess_ms,
                inference_ms=result.inference_ms,
                postprocess_ms=result.postprocess_ms,
                queue_wait_ms=queue_wait_ms,
            )
        except Exception as exc:
            self._frame_error(camera, exc)

    def _submit_async(
        self, camera: str, slot: FrameSlot, queue_wait_ms: float
    ) -> None:
        assert self._openvino_detector_type is not None
        assert isinstance(self.detector, self._openvino_detector_type)
        assert self._async_queue is not None
        try:
            started = perf_counter()
            image = self._decode(slot.message)
            decoded = perf_counter()
            tensor, letterbox = self.detector.preprocess(image)
            prepared = perf_counter()
            context = AsyncContext(
                camera=camera,
                message=slot.message,
                image=image,
                tensor=tensor,
                letterbox=letterbox,
                received_perf=slot.received_perf,
                received_wall=slot.received_wall,
                decode_ms=(decoded - started) * 1000.0,
                preprocess_ms=(prepared - decoded) * 1000.0,
                queue_wait_ms=queue_wait_ms,
                infer_started=perf_counter(),
            )
            self._async_queue.start_async(
                {self.detector.input_port: context.tensor}, userdata=context
            )
        except Exception as exc:
            self._frame_error(camera, exc)

    def _async_complete(self, request: Any, userdata: Any) -> None:
        context = userdata
        if not isinstance(context, AsyncContext):
            return
        infer_done = perf_counter()
        try:
            assert self._openvino_detector_type is not None
            assert isinstance(self.detector, self._openvino_detector_type)
            output = np.array(request.get_output_tensor(0).data, copy=True)
            post_started = perf_counter()
            detections = self.detector.postprocess(output, context.letterbox)
            post_done = perf_counter()
            self._publish_result(
                camera=context.camera,
                message=context.message,
                image=context.image,
                detections=detections,
                received_perf=context.received_perf,
                received_wall=context.received_wall,
                decode_ms=context.decode_ms,
                preprocess_ms=context.preprocess_ms,
                inference_ms=(infer_done - context.infer_started) * 1000.0,
                postprocess_ms=(post_done - post_started) * 1000.0,
                queue_wait_ms=context.queue_wait_ms,
            )
        except Exception as exc:
            self._frame_error(context.camera, exc)

    def _publish_result(
        self,
        *,
        camera: str,
        message: CompressedImage,
        image: np.ndarray,
        detections: list[Detection],
        received_perf: float,
        received_wall: float,
        decode_ms: float,
        preprocess_ms: float,
        inference_ms: float,
        postprocess_ms: float,
        queue_wait_ms: float,
    ) -> None:
        stamp = _stamp_tuple(message)
        with self._publish_lock:
            previous = self._last_published_stamp.get(camera)
            if previous is not None and stamp < previous:
                with self._lock:
                    self._stats[camera].stale_drop_count += 1
                return

            draw_started = perf_counter()
            encoded: bytes | None = None
            if self.publish_visualization:
                for detection in detections:
                    x1, y1, x2, y2 = (int(round(value)) for value in detection.xyxy)
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
                success, jpeg = cv2.imencode(
                    '.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                )
                if not success:
                    raise ValueError('could not encode bbox visualization as JPEG')
                encoded = jpeg.tobytes()
            draw_done = perf_counter()

            payload = {
                'camera': camera,
                'stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
                'frame_id': message.header.frame_id,
                'image_size': {'width': image.shape[1], 'height': image.shape[0]},
                'detections': [
                    {
                        'class_id': item.class_id,
                        'class_name': item.class_name,
                        'confidence': item.confidence,
                        'xyxy': list(item.xyxy),
                    }
                    for item in detections
                ],
            }
            output = String()
            output.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
            pub_started = perf_counter()
            self.detections_pubs[camera].publish(output)
            if encoded is not None:
                image_message = CompressedImage()
                image_message.header = message.header
                image_message.format = 'jpeg'
                image_message.data = encoded
                self.image_pubs[camera].publish(image_message)
            published = perf_counter()
            self._last_published_stamp[camera] = stamp

        latency_ms = self._latency_ms(
            message, received_perf, received_wall, published
        )
        with self._lock:
            stats = self._stats[camera]
            stats.processed_count += 1
            measurements = {
                'decode': decode_ms,
                'pre': preprocess_ms,
                'infer': inference_ms,
                'post': postprocess_ms,
                'draw_enc': (draw_done - draw_started) * 1000.0,
                'pub': (published - pub_started) * 1000.0,
                'queue_wait': queue_wait_ms,
                'latency': latency_ms,
            }
            for name, value in measurements.items():
                stats.values[name].append(max(0.0, value))
            if stats.last_publish_perf is not None:
                stats.max_gap_ms = max(
                    stats.max_gap_ms,
                    (published - stats.last_publish_perf) * 1000.0,
                )
            stats.last_publish_perf = published

    def _latency_ms(
        self,
        message: CompressedImage,
        received_perf: float,
        received_wall: float,
        published_perf: float,
    ) -> float:
        if self.latency_mode == 'receive':
            return (published_perf - received_perf) * 1000.0
        stamp_seconds = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        publish_wall = received_wall + (published_perf - received_perf)
        return max(0.0, (publish_wall - stamp_seconds) * 1000.0)

    def _frame_error(self, camera: str, exc: Exception) -> None:
        with self._lock:
            self._stats[camera].error_count += 1
        self.get_logger().error(f'[{camera}] frame skipped: {exc}')

    @staticmethod
    def _metric_summary(values: deque[float] | list[float]) -> tuple[float, float]:
        return (float(mean(values)), _percentile(values, 95.0)) if values else (0.0, 0.0)

    def _report_interval(self) -> None:
        now = perf_counter()
        period = max(now - self._stats_started, 1e-9)
        cpu, rss, pss = self._memory.sample()
        timestamp = datetime.now().isoformat(timespec='seconds')
        active = [item[0] for item in self.camera_topics]
        with self._lock:
            snapshots: dict[str, dict[str, Any]] = {}
            for camera in active:
                stats = self._stats[camera]
                snapshots[camera] = self._snapshot(stats, period)
            snapshots['total'] = self._total_snapshot(active, period)
            for camera in active:
                self._stats[camera].reset_interval()
            self._stats_started = now

        if self.print_table:
            pss_text = f'{pss:.1f}' if pss is not None else 'n/a'
            print(
                f'[{timestamp}] {period:.1f}s CPU={cpu:.1f}% RSS={rss:.1f}MB PSS={pss_text}MB',
                flush=True,
            )
            print(
                '  cam   | in_fps proc_fps drop stale err | decode avg(p95) | pre avg(p95) | infer avg(p95) | post avg(p95) | draw+enc avg(p95) | pub avg(p95) | queue avg(p95) | latency avg(p95) | max_gap',
                flush=True,
            )
            for camera in active + ['total']:
                row = snapshots[camera]
                warning = '!' if row['in_fps'] > 0.0 and row['proc_fps'] < row['in_fps'] * 0.5 else ' '
                metric = lambda name: f'{row[name + "_avg_ms"]:6.1f}({row[name + "_p95_ms"]:6.1f})'
                print(
                    f'{warning} {camera:<6}| {row["in_fps"]:6.1f} {row["proc_fps"]:8.1f} {row["drop"]:4d} {row["stale_drop"]:5d} {row["err"]:3d} | '
                    f'{metric("decode")} | {metric("pre")} | {metric("infer")} | {metric("post")} | '
                    f'{metric("draw_enc")} | {metric("pub")} | {metric("queue_wait")} | {metric("latency")} | {row["max_gap_ms"]:7.1f}',
                    flush=True,
                )

        for camera, snapshot in snapshots.items():
            row = {
                'timestamp': timestamp,
                'period_s': period,
                'camera': camera,
                'backend': self.backend,
                'perf_hint': self.perf_hint,
                'num_requests': self.num_requests,
                'scheduler': self.scheduler,
                'async_infer': self.async_infer,
                'run_tag': str(self.get_parameter('run_tag').value),
                'latency_mode': self.latency_mode,
                'cpu_percent': cpu,
                'rss_mb': rss,
                'pss_mb': '' if pss is None else pss,
                **snapshot,
            }
            self._csv_writer.writerow(row)
        self._csv_file.flush()

    def _snapshot(self, stats: CameraStats, period: float) -> dict[str, Any]:
        result: dict[str, Any] = {
            'in_fps': stats.input_count / period,
            'proc_fps': stats.processed_count / period,
            'drop': stats.drop_count,
            'stale_drop': stats.stale_drop_count,
            'err': stats.error_count,
            'max_gap_ms': stats.max_gap_ms,
        }
        for name, values in stats.values.items():
            average, p95 = self._metric_summary(values)
            result[f'{name}_avg_ms'] = average
            result[f'{name}_p95_ms'] = p95
        return result

    def _total_snapshot(self, cameras: list[str], period: float) -> dict[str, Any]:
        combined = CameraStats()
        combined.input_count = sum(self._stats[name].input_count for name in cameras)
        combined.processed_count = sum(self._stats[name].processed_count for name in cameras)
        combined.drop_count = sum(self._stats[name].drop_count for name in cameras)
        combined.stale_drop_count = sum(self._stats[name].stale_drop_count for name in cameras)
        combined.error_count = sum(self._stats[name].error_count for name in cameras)
        combined.max_gap_ms = max(
            (self._stats[name].max_gap_ms for name in cameras), default=0.0
        )
        for metric in METRIC_NAMES:
            for camera in cameras:
                combined.values[metric].extend(self._stats[camera].values[metric])
        return self._snapshot(combined, period)

    def shutdown(self) -> None:
        """Stop scheduling, drain OpenVINO requests and flush logs."""
        self._timer.cancel()
        with self._condition:
            self._stopping = True
            self._slots.clear()
            self._condition.notify_all()
        if self._worker is not None:
            self._worker.join()
        if self._async_queue is not None:
            try:
                self._async_queue.wait_all()
            except Exception as exc:
                self.get_logger().error(f'Error while draining inference requests: {exc}')
        try:
            try:
                self._report_interval()
            except KeyboardInterrupt:
                # ros2 launch can deliver SIGINT to the process group and then
                # forward it once more while this final snapshot is collected.
                pass
        finally:
            self._csv_file.flush()
            self._csv_file.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PersonDetectionOVNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = PersonDetectionOVNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f'person_detection_ov startup/runtime failure: {exc}')
        else:
            print(f'person_detection_ov startup failure: {exc}', flush=True)
        raise
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
