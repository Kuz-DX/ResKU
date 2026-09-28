#!/usr/bin/env python3
"""ROS 2 compressed-image person detector backed by OpenVINO RF-DETR."""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vision.rfdetr_openvino import RFDETROpenVINO


class PersonDetectionNode(Node):

    def __init__(self):
        super().__init__('person_detection_openvino')
        default_model = str(
            Path(get_package_share_directory('vision'))
            / 'models'
            / 'mando-dummy-v1.xml'
        )
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('detections_topic', '/person_detection/detections')
        self.declare_parameter('output_topic', '/person_detection/image/compressed')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'CPU')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('cache_dir', '')

        get = lambda name: self.get_parameter(name).value
        self.jpeg_quality = int(get('jpeg_quality'))
        self.publish_visualization = bool(get('publish_visualization'))
        cache_dir = str(get('cache_dir')).strip() or None

        self.detector = RFDETROpenVINO(
            model_path=str(get('model_path')),
            class_names=('pedestrian',),
            device=str(get('device')),
            confidence_threshold=float(get('confidence_threshold')),
            background_class_id=-1,
            cache_dir=cache_dir,
        )
        self.detections_pub = self.create_publisher(String, str(get('detections_topic')), 10)
        self.image_pub = self.create_publisher(
            CompressedImage,
            str(get('output_topic')),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            str(get('input_topic')),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'RF-DETR OpenVINO ready: {get("model_path")} on {get("device")}'
        )

    def image_callback(self, msg: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning('Could not decode compressed input image.')
            return

        try:
            detections = self.detector.predict(image)
        except Exception as exc:
            self.get_logger().error(f'OpenVINO inference failed: {exc}')
            return

        payload = {
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
        self.detections_pub.publish(output)

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
            if success:
                image_msg = CompressedImage()
                image_msg.header = msg.header
                image_msg.format = 'jpeg'
                image_msg.data = encoded.tobytes()
                self.image_pub.publish(image_msg)


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

