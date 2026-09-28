import numpy as np
import pytest

from vision.rfdetr_openvino import RFDETROpenVINO


class FakeCompiledModel:

    def __init__(self, boxes, logits):
        self.boxes = boxes
        self.logits = logits

    def __call__(self, _inputs):
        return {'boxes': self.boxes, 'logits': self.logits}


def make_detector(boxes, logits, *, threshold=0.5, background_class_id=-1):
    detector = object.__new__(RFDETROpenVINO)
    detector.class_names = ('pedestrian', 'cyclist')
    detector.confidence_threshold = threshold
    detector.background_class_id = background_class_id
    detector.input_height = 2
    detector.input_width = 2
    detector.input_port = 'input'
    detector.boxes_port = 'boxes'
    detector.logits_port = 'logits'
    detector.compiled_model = FakeCompiledModel(boxes, logits)
    return detector


def test_predict_removes_background_and_scales_boxes():
    boxes = np.asarray([[[0.5, 0.5, 0.5, 0.5], [0.2, 0.2, 0.2, 0.2]]])
    logits = np.asarray([[[4.0, -4.0], [-4.0, 4.0]]])
    detector = make_detector(boxes, logits)

    detections = detector.predict(np.zeros((100, 200, 3), dtype=np.uint8))

    assert len(detections) == 1
    assert detections[0].class_id == 0
    assert detections[0].class_name == 'pedestrian'
    assert detections[0].xyxy == pytest.approx((50.0, 25.0, 150.0, 75.0))


def test_predict_uses_flattened_query_class_topk():
    boxes = np.asarray([[[0.2, 0.2, 0.1, 0.1], [0.8, 0.8, 0.1, 0.1]]])
    logits = np.asarray([[[5.0, 4.0], [3.0, -5.0]]])
    detector = make_detector(boxes, logits, background_class_id=None)

    detections = detector.predict(np.zeros((100, 100, 3), dtype=np.uint8))

    assert [detection.class_id for detection in detections] == [0, 1]
    assert detections[0].xyxy == pytest.approx(detections[1].xyxy)


def test_preprocess_rejects_empty_images():
    detector = make_detector(np.empty((1, 0, 4)), np.empty((1, 0, 2)))

    with pytest.raises(ValueError, match='non-empty'):
        detector.preprocess(np.empty((0, 20, 3), dtype=np.uint8))
