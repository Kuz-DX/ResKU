#!/usr/bin/env python3
"""PyTorch-free supply-box detection with the OpenVINO Runtime."""

from __future__ import annotations

import numpy as np

from vision.ov_person_detector import (
    Detection,
    LetterboxInfo,
    OpenVINOPersonDetector,
)


class OpenVINOSupplyDetector(OpenVINOPersonDetector):
    """Run the static end-to-end supply-box IR without Ultralytics or Torch."""

    def postprocess(
        self, raw_output: np.ndarray, info: LetterboxInfo
    ) -> list[Detection]:
        """Filter end-to-end rows and restore supply-box coordinates."""
        output = np.asarray(raw_output)
        if output.ndim != 3 or output.shape[0] != 1 or output.shape[-1] != 6:
            raise ValueError(f'Expected runtime output [1,N,6], got {output.shape}.')

        rows = output[0]
        rows = rows[rows[:, 4] >= self.confidence_threshold]
        detections: list[Detection] = []
        height, width = info.original_shape
        for row in rows:
            class_id = int(row[5])
            class_name = self.class_names.get(class_id, 'supplybox')
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
                    class_name=class_name,
                )
            )
        return detections


__all__: list[str] = ['OpenVINOSupplyDetector']
