"""PyTorch-free RF-DETR inference using OpenVINO Runtime."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np
import openvino as ov


_IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)


@dataclass(frozen=True)
class Detection:
    """One decoded RF-DETR detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str


class RFDETROpenVINO:
    """Run an exported RF-DETR IR without importing PyTorch or RF-DETR."""

    def __init__(
        self,
        model_path: str | Path,
        class_names: Sequence[str] = ('pedestrian',),
        device: str = 'CPU',
        confidence_threshold: float = 0.5,
        background_class_id: Optional[int] = -1,
        cache_dir: str | Path | None = None,
    ) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f'OpenVINO model not found: {self.model_path}')
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0 and 1.')

        self.class_names = tuple(class_names)
        self.confidence_threshold = float(confidence_threshold)
        self.background_class_id = background_class_id

        core = ov.Core()
        config = {'PERFORMANCE_HINT': 'LATENCY'}
        if cache_dir:
            resolved_cache = Path(cache_dir).expanduser().resolve()
            resolved_cache.mkdir(parents=True, exist_ok=True)
            core.set_property({'CACHE_DIR': str(resolved_cache)})

        model = core.read_model(str(self.model_path))
        self.compiled_model = core.compile_model(model, device, config)
        self.input_port = self.compiled_model.input(0)
        shape = tuple(int(value) for value in self.input_port.shape)
        if len(shape) != 4 or shape[0] != 1 or shape[1] != 3:
            raise ValueError(f'Expected static NCHW input [1,3,H,W], got {shape}.')
        self.input_height, self.input_width = shape[2], shape[3]

        self.boxes_port = self._find_output('dets')
        self.logits_port = self._find_output('labels')

    def _find_output(self, expected_name: str):
        for output in self.compiled_model.outputs:
            names = {name.lower() for name in output.get_names()}
            if any(expected_name in name for name in names):
                return output
        available = [sorted(output.get_names()) for output in self.compiled_model.outputs]
        raise ValueError(
            f"RF-DETR output containing {expected_name!r} was not found; "
            f'available outputs: {available}'
        )

    def preprocess(self, image_bgr: np.ndarray) -> np.ndarray:
        if (
            image_bgr is None
            or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.shape[0] == 0
            or image_bgr.shape[1] == 0
        ):
            raise ValueError('image_bgr must be a non-empty HxWx3 BGR image.')
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(
            rgb,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )
        normalized = (resized.astype(np.float32) / 255.0 - _IMAGENET_MEAN) / _IMAGENET_STD
        return np.ascontiguousarray(normalized.transpose(2, 0, 1)[None, ...])

    def predict(self, image_bgr: np.ndarray) -> list[Detection]:
        height, width = image_bgr.shape[:2]
        result = self.compiled_model({self.input_port: self.preprocess(image_bgr)})
        boxes_cxcywh = np.asarray(result[self.boxes_port])[0]
        raw_logits = np.asarray(result[self.logits_port])[0]

        if boxes_cxcywh.ndim != 2 or boxes_cxcywh.shape[-1] != 4:
            raise ValueError(
                f'Expected boxes shaped [queries,4], got {boxes_cxcywh.shape}.'
            )
        if raw_logits.ndim != 2 or raw_logits.shape[0] != boxes_cxcywh.shape[0]:
            raise ValueError(
                'Expected logits shaped [queries,classes] with the same query '
                f'count as boxes, got {raw_logits.shape}.'
            )

        class_slots = np.arange(raw_logits.shape[-1])
        if self.background_class_id is not None:
            background = int(self.background_class_id)
            if not -raw_logits.shape[-1] <= background < raw_logits.shape[-1]:
                raise ValueError('background_class_id is outside the exported class slots.')
            background %= raw_logits.shape[-1]
            foreground = class_slots != background
            raw_logits = raw_logits[:, foreground]
            class_slots = class_slots[foreground]

        if raw_logits.shape[-1] == 0:
            raise ValueError('No foreground class slots remain after background removal.')
        scores_all = 1.0 / (1.0 + np.exp(-np.clip(raw_logits, -88.0, 88.0)))
        # RF-DETR selects the highest-scoring query/class pairs from the
        # flattened score matrix. This preserves multi-label detections and
        # matches its PyTorch/ONNX post-processing.
        flat_scores = scores_all.reshape(-1)
        num_select = min(boxes_cxcywh.shape[0], flat_scores.size)
        if num_select == flat_scores.size:
            top_indices = np.argsort(flat_scores)[::-1]
        else:
            candidates = np.argpartition(flat_scores, -num_select)[-num_select:]
            top_indices = candidates[np.argsort(flat_scores[candidates])[::-1]]
        query_indices, foreground_indices = np.divmod(
            top_indices,
            scores_all.shape[-1],
        )
        scores = flat_scores[top_indices]
        class_ids = class_slots[foreground_indices]
        keep = scores >= self.confidence_threshold

        boxes = boxes_cxcywh[query_indices[keep]]
        scores = scores[keep]
        class_ids = class_ids[keep]
        if boxes.size == 0:
            return []

        cx, cy, box_width, box_height = boxes.T
        xyxy = np.stack(
            (
                cx - box_width / 2.0,
                cy - box_height / 2.0,
                cx + box_width / 2.0,
                cy + box_height / 2.0,
            ),
            axis=1,
        )
        xyxy *= np.asarray([width, height, width, height], dtype=np.float32)
        xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, width - 1)
        xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, height - 1)

        detections = []
        for box, score, class_id in zip(xyxy, scores, class_ids):
            class_id = int(class_id)
            class_name = (
                self.class_names[class_id]
                if 0 <= class_id < len(self.class_names)
                else f'class_{class_id}'
            )
            detections.append(
                Detection(
                    xyxy=tuple(float(value) for value in box),
                    confidence=float(score),
                    class_id=class_id,
                    class_name=class_name,
                )
            )
        return detections

