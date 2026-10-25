#!/usr/bin/env python3
"""PyTorch-free YOLO person detection with the OpenVINO Runtime."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Any

import cv2
import numpy as np
import openvino as ov
import yaml


@dataclass(frozen=True)
class Detection:
    """One person detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str = 'person'


@dataclass(frozen=True)
class LetterboxInfo:
    """Geometry needed to restore model coordinates to the source image."""

    original_shape: tuple[int, int]
    input_shape: tuple[int, int]
    gain: float
    pad: tuple[int, int]


@dataclass(frozen=True)
class Prediction:
    """Detections and per-stage timings from one synchronous request."""

    detections: list[Detection]
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float


class OpenVINOPersonDetector:
    """Run a static, end-to-end Ultralytics YOLO IR without Torch imports."""

    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.5,
        infer_size: int = 640,
        device: str = 'CPU',
        perf_hint: str = 'LATENCY',
        num_requests: int = 1,
    ) -> None:
        self.model_path = self.resolve_model_path(model_path)
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0 and 1.')
        if infer_size <= 0:
            raise ValueError('infer_size must be greater than zero.')
        perf_hint = perf_hint.strip().upper()
        if perf_hint not in {'LATENCY', 'THROUGHPUT'}:
            raise ValueError("perf_hint must be 'LATENCY' or 'THROUGHPUT'.")
        if num_requests <= 0:
            raise ValueError('num_requests must be greater than zero.')

        self.confidence_threshold = float(confidence_threshold)
        self.device = str(device).strip().upper() or 'CPU'
        self.perf_hint = perf_hint
        self.requested_num_requests = int(num_requests)
        self.metadata = self._load_metadata(self.model_path.parent)
        self.class_names = self._class_names(self.metadata.get('names', {}))
        self.metadata_end2end = self.metadata.get('end2end')

        self.core = ov.Core()
        read_started = perf_counter()
        self.model = self.core.read_model(str(self.model_path))
        self.read_ms = (perf_counter() - read_started) * 1000.0
        self.load_ms = self.read_ms
        self.precision = self._detect_precision(self.model)

        config: dict[str, Any] = {'PERFORMANCE_HINT': self.perf_hint}
        # This is a performance hint rather than a hard queue-size limit. The
        # caller still controls AsyncInferQueue's exact request count.
        config['PERFORMANCE_HINT_NUM_REQUESTS'] = str(self.requested_num_requests)
        compile_started = perf_counter()
        try:
            self.compiled_model = self.core.compile_model(
                self.model, self.device, config
            )
        except RuntimeError as exc:
            if 'PERFORMANCE_HINT_NUM_REQUESTS' not in str(exc):
                raise
            config.pop('PERFORMANCE_HINT_NUM_REQUESTS')
            self.compiled_model = self.core.compile_model(
                self.model, self.device, config
            )
        self.compile_ms = (perf_counter() - compile_started) * 1000.0
        self.load_compile_ms = self.read_ms + self.compile_ms

        if len(self.compiled_model.inputs) != 1:
            raise ValueError(
                f'Expected one image input, got {len(self.compiled_model.inputs)}.'
            )
        self.input_port = self.compiled_model.input(0)
        input_shape = tuple(int(value) for value in self.input_port.shape)
        if len(input_shape) != 4 or input_shape[0] != 1 or input_shape[1] != 3:
            raise ValueError(f'Expected static NCHW input [1,3,H,W], got {input_shape}.')
        self.input_height, self.input_width = input_shape[2], input_shape[3]
        metadata_imgsz = self._metadata_imgsz(self.metadata.get('imgsz'))
        self.infer_size = metadata_imgsz or int(infer_size)
        if (self.input_height, self.input_width) != (self.infer_size, self.infer_size):
            # The graph shape is authoritative for a static exported model.
            self.infer_size = self.input_height

        if len(self.compiled_model.outputs) != 1:
            shapes = [tuple(str(x) for x in port.partial_shape) for port in self.compiled_model.outputs]
            raise ValueError(
                'Only a one-output end-to-end YOLO model is supported; '
                f'got {len(shapes)} outputs with shapes {shapes}.'
            )
        self.output_port = self.compiled_model.output(0)
        self.output_shape = tuple(int(value) for value in self.output_port.shape)
        if self.metadata_end2end is False:
            raise ValueError(
                'metadata.yaml marks this model as end2end=false. Raw YOLO head '
                'decoding/NMS is outside this tool; export an end-to-end model.'
            )
        self._validate_output_shape(self.output_shape)
        self.optimal_num_requests = self._compiled_property(
            'OPTIMAL_NUMBER_OF_INFER_REQUESTS'
        )
        self.num_streams = self._compiled_property('NUM_STREAMS')
        self.warmup_ms = 0.0

    @staticmethod
    def resolve_model_path(model_path: str | Path) -> Path:
        """Resolve a model directory or XML path to exactly one IR XML."""
        resolved = Path(model_path).expanduser().resolve()
        if not resolved.exists():
            raise FileNotFoundError(f'OpenVINO model path not found: {resolved}')
        if resolved.is_dir():
            xml_files = sorted(resolved.glob('*.xml'))
            if len(xml_files) != 1:
                raise FileNotFoundError(
                    f'Expected exactly one .xml in {resolved}, found {len(xml_files)}.'
                )
            resolved = xml_files[0]
        if not resolved.is_file() or resolved.suffix.lower() != '.xml':
            raise ValueError(f'OpenVINO model must be an .xml file or model folder: {resolved}')
        bin_path = resolved.with_suffix('.bin')
        if not bin_path.is_file():
            raise FileNotFoundError(f'OpenVINO weights not found: {bin_path}')
        return resolved

    @staticmethod
    def _load_metadata(folder: Path) -> dict[str, Any]:
        metadata_path = folder / 'metadata.yaml'
        if not metadata_path.is_file():
            return {}
        with metadata_path.open('r', encoding='utf-8') as stream:
            value = yaml.safe_load(stream) or {}
        if not isinstance(value, dict):
            raise ValueError(f'Expected a mapping in {metadata_path}.')
        return value

    @staticmethod
    def _metadata_imgsz(value: Any) -> int | None:
        if isinstance(value, int):
            return value
        if isinstance(value, (list, tuple)) and value:
            dimensions = [int(item) for item in value]
            if len(set(dimensions)) == 1:
                return dimensions[0]
        return None

    @staticmethod
    def _class_names(value: Any) -> dict[int, str]:
        if isinstance(value, list):
            return {index: str(name) for index, name in enumerate(value)}
        if isinstance(value, dict):
            names: dict[int, str] = {}
            for key, name in value.items():
                try:
                    names[int(key)] = str(name)
                except (TypeError, ValueError):
                    continue
            return names
        return {}

    @staticmethod
    def _detect_precision(model: ov.Model) -> str:
        operations = model.get_ops()
        if any(op.get_type_name() == 'FakeQuantize' for op in operations):
            return 'INT8'
        constant_types = {
            str(op.get_output_element_type(0)).lower()
            for op in operations
            if op.get_type_name() == 'Constant'
        }
        if any('f16' in value for value in constant_types):
            return 'FP16'
        if any('f32' in value for value in constant_types):
            return 'FP32'
        return 'UNKNOWN'

    def _compiled_property(self, name: str) -> str | int:
        try:
            return self.compiled_model.get_property(name)
        except Exception:
            return 'n/a'

    @staticmethod
    def _validate_output_shape(shape: tuple[int, ...]) -> None:
        if len(shape) != 3 or shape[0] != 1 or shape[-1] != 6:
            raise ValueError(
                'Unsupported YOLO output. Expected an end-to-end NMS output '
                f'shaped [1,N,6] (xyxy, confidence, class), got {shape}. '
                'Export with end2end/NMS enabled; raw-head decoding is outside this tool.'
            )

    def preprocess(self, image_bgr: np.ndarray) -> tuple[np.ndarray, LetterboxInfo]:
        """Apply Ultralytics-compatible centered static-shape letterboxing."""
        if (
            image_bgr is None
            or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.shape[0] == 0
            or image_bgr.shape[1] == 0
        ):
            raise ValueError('image_bgr must be a non-empty HxWx3 BGR image.')
        height, width = image_bgr.shape[:2]
        gain = min(self.input_height / height, self.input_width / width)
        resized_width = round(width * gain)
        resized_height = round(height * gain)
        dw = (self.input_width - resized_width) / 2.0
        dh = (self.input_height - resized_height) / 2.0
        left, right = round(dw - 0.1), round(dw + 0.1)
        top, bottom = round(dh - 0.1), round(dh + 0.1)

        resized = image_bgr
        if (width, height) != (resized_width, resized_height):
            resized = cv2.resize(
                image_bgr,
                (resized_width, resized_height),
                interpolation=cv2.INTER_LINEAR,
            )
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        rgb_chw = padded[..., ::-1].transpose(2, 0, 1)
        tensor = np.ascontiguousarray(rgb_chw, dtype=np.float32)[None, ...]
        tensor /= 255.0
        info = LetterboxInfo(
            original_shape=(height, width),
            input_shape=(self.input_height, self.input_width),
            gain=gain,
            pad=(left, top),
        )
        return tensor, info

    def postprocess(
        self, raw_output: np.ndarray, info: LetterboxInfo
    ) -> list[Detection]:
        """Filter end-to-end rows and restore coordinates to source pixels."""
        output = np.asarray(raw_output)
        if output.ndim != 3 or output.shape[0] != 1 or output.shape[-1] != 6:
            raise ValueError(
                f'Expected runtime output [1,N,6], got {output.shape}.'
            )
        rows = output[0]
        rows = rows[rows[:, 4] >= self.confidence_threshold]
        detections: list[Detection] = []
        height, width = info.original_shape
        for row in rows:
            class_id = int(row[5])
            model_name = self.class_names.get(class_id, 'person' if class_id == 0 else f'class_{class_id}')
            if model_name.lower() not in {'person', 'pedestrian'}:
                continue
            box = row[:4].astype(np.float32, copy=True)
            box[[0, 2]] -= info.pad[0]
            box[[1, 3]] -= info.pad[1]
            box /= info.gain
            box[[0, 2]] = np.clip(box[[0, 2]], 0, width)
            box[[1, 3]] = np.clip(box[[1, 3]], 0, height)
            detections.append(
                Detection(
                    xyxy=tuple(float(value) for value in box),
                    confidence=float(row[4]),
                    class_id=class_id,
                )
            )
        return detections

    def predict_timed(self, image_bgr: np.ndarray) -> Prediction:
        """Run one synchronous request and return detections plus timings."""
        started = perf_counter()
        tensor, info = self.preprocess(image_bgr)
        pre_done = perf_counter()
        result = self.compiled_model({self.input_port: tensor})
        infer_done = perf_counter()
        detections = self.postprocess(result[self.output_port], info)
        done = perf_counter()
        return Prediction(
            detections=detections,
            preprocess_ms=(pre_done - started) * 1000.0,
            inference_ms=(infer_done - pre_done) * 1000.0,
            postprocess_ms=(done - infer_done) * 1000.0,
        )

    def predict(self, image_bgr: np.ndarray) -> list[Detection]:
        """Return detections using the same contract as the original adapter."""
        return self.predict_timed(image_bgr).detections

    def warmup(self, iterations: int = 3) -> float:
        """Warm up the compiled graph with zero-valued image inputs."""
        if iterations <= 0:
            raise ValueError('iterations must be greater than zero.')
        dummy = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        elapsed: list[float] = []
        for _ in range(iterations):
            started = perf_counter()
            self.predict(dummy)
            elapsed.append((perf_counter() - started) * 1000.0)
        self.warmup_ms = float(np.mean(elapsed))
        return self.warmup_ms


__all__ = [
    'Detection',
    'LetterboxInfo',
    'OpenVINOPersonDetector',
    'Prediction',
]
