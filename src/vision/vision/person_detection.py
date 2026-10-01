#!/usr/bin/env python3
"""ROS 2 multi-camera person detector backed by an Ultralytics YOLO model."""

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:  # Give a focused startup error instead of hiding the dependency.
    YOLO = None


@dataclass(frozen=True)
class Detection:
    """One person detection in source-image pixel coordinates."""

    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str = 'person'


class YOLOPersonDetector:
    """Small adapter that keeps the node's existing detection output contract."""

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
        if YOLO is None:
            raise RuntimeError(
                'ultralytics is required for person.pt; install the vision package '
                'requirements first.'
            )

        self.confidence_threshold = float(confidence_threshold)
        self.infer_size = int(infer_size)
        self.device = str(device).strip().lower() or 'cpu'
        self.model = YOLO(str(self.model_path), task='detect')

    def predict(self, image_bgr: np.ndarray) -> list[Detection]:
        if (
            image_bgr is None
            or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.shape[0] == 0
            or image_bgr.shape[1] == 0
        ):
            raise ValueError('image_bgr must be a non-empty HxWx3 BGR image.')

        results = self.model.predict(
            source=image_bgr,
            conf=self.confidence_threshold,
            imgsz=self.infer_size,
            device=self.device,
            verbose=False,
        )
        detections = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = str(result.names.get(class_id, f'class_{class_id}'))
                # person.pt is a single-class detector whose training label is
                # "pedestrian". Preserve that model class id while keeping the
                # public person-detection topic's class name stable.
                if class_name.lower() not in {'person', 'pedestrian'}:
                    continue
                detections.append(
                    Detection(
                        xyxy=tuple(float(value) for value in box.xyxy[0].tolist()),
                        confidence=float(box.conf[0]),
                        class_id=class_id,
                    )
                )
        return detections


def _default_model_path() -> Path:
    """Find person.pt in an installed package or a symlink-source workspace."""
    installed = Path(get_package_share_directory('vision')) / 'models' / 'person.pt'
    if installed.is_file():
        return installed

    return Path(__file__).resolve().parents[1] / 'models' / 'person.pt'


class PersonDetectionNode(Node):

    def __init__(self):
        super().__init__('person_detection_yolo')
        default_model = str(_default_model_path())
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('input_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/person_detection/detections')
        self.declare_parameter('output_topic', '/drive/person/detecion')
        self.declare_parameter('left_input_topic', '/side/left/image_raw/compressed')
        self.declare_parameter('left_detections_topic', '/left/person/detections')
        self.declare_parameter('left_output_topic', '/left/person/detection')
        self.declare_parameter('right_input_topic', '/side/right/image_raw/compressed')
        self.declare_parameter('right_detections_topic', '/right/person/detections')
        self.declare_parameter('right_output_topic', '/right/person/detection')
        self.declare_parameter('arm_input_topic', '/arm/camera/color/image_raw/compressed')
        self.declare_parameter('arm_detections_topic', '/arm/person/detections')
        self.declare_parameter('arm_output_topic', '/arm/person/detection')
        self.declare_parameter('enable_arm_camera', True)
        self.declare_parameter('enable_side_cameras', True)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('infer_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('publish_visualization', True)

        get = lambda name: self.get_parameter(name).value
        self.jpeg_quality = int(get('jpeg_quality'))
        self.publish_visualization = bool(get('publish_visualization'))

        self.detector = YOLOPersonDetector(
            model_path=str(get('model_path')),
            confidence_threshold=float(get('confidence_threshold')),
            infer_size=int(get('infer_size')),
            device=str(get('device')),
        )
        camera_topics = [
            (
                'drive',
                str(get('input_topic')),
                str(get('detections_topic')),
                str(get('output_topic')),
            ),
        ]
        if bool(get('enable_side_cameras')):
            camera_topics.extend(
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
            camera_topics.append(
                (
                    'arm',
                    str(get('arm_input_topic')),
                    str(get('arm_detections_topic')),
                    str(get('arm_output_topic')),
                )
            )

        # Keep publisher/subscription references alive for every camera.  The
        # default single-threaded executor also serializes access to the shared
        # YOLO model.
        self.detections_pubs = {}
        self.image_pubs = {}
        self._camera_subscriptions = []
        for camera_name, input_topic, detections_topic, output_topic in camera_topics:
            self.detections_pubs[camera_name] = self.create_publisher(
                String,
                detections_topic,
                qos_profile_sensor_data,
            )
            self.image_pubs[camera_name] = self.create_publisher(
                CompressedImage,
                output_topic,
                qos_profile_sensor_data,
            )
            self._camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    input_topic,
                    lambda msg, name=camera_name: self.image_callback(msg, name),
                    qos_profile_sensor_data,
                )
            )
            self.get_logger().info(
                f'{camera_name}: input={input_topic}, detections={detections_topic}, '
                f'bbox_image={output_topic}'
            )
        self.get_logger().info(
            f'YOLO person detector ready: {get("model_path")} on {get("device")}'
        )

    def image_callback(self, msg: CompressedImage, camera_name: str) -> None:
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning(
                f'[{camera_name}] Could not decode compressed input image.'
            )
            return

        try:
            detections = self.detector.predict(image)
        except Exception as exc:
            self.get_logger().error(f'[{camera_name}] YOLO inference failed: {exc}')
            return

        payload = {
            'camera': camera_name,
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'frame_id': msg.header.frame_id,
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
        output.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.detections_pubs[camera_name].publish(output)

        if self.publish_visualization:
            for detection in detections:
                x1, y1, x2, y2 = (int(round(value)) for value in detection.xyxy)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{detection.class_name} {detection.confidence:.2f}'
                cv2.putText(
                    image,
                    label,
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
                self.get_logger().warning(
                    f'[{camera_name}] Could not encode bbox visualization as JPEG.'
                )
                return

            image_msg = CompressedImage()
            image_msg.header = msg.header
            image_msg.format = 'jpeg'
            image_msg.data = encoded.tobytes()
            self.image_pubs[camera_name].publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
