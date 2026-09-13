"""Live view for summer_supply's debug image + target point overlay."""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1


class ArmVisualizerNode(Node):
    """Subscribes to summer_supply's debug image and target point, and shows them in a window."""

    def __init__(self):
        super().__init__('arm_visualizer_node')

        self.declare_parameter('image_topic', '/arm/debug_image/compressed')
        self.declare_parameter('point_topic', '/arm/target_point')
        self.declare_parameter('window_name', 'arm_debug')
        self.declare_parameter('point_stale_sec', 0.5)

        image_topic = str(self.get_parameter('image_topic').value)
        point_topic = str(self.get_parameter('point_topic').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.point_stale_sec = float(self.get_parameter('point_stale_sec').value)

        self.bridge = CvBridge()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self._last_point = None
        self._last_point_stamp = None

        # [2026-09-02] summer_supply.py의 /arm/debug_image/compressed 발행자는
        # SENSOR_DATA_QOS_DEPTH1(BEST_EFFORT/depth=1)을 쓰는데, 여기서는
        # 정수 10(기본 RELIABLE)으로 구독해서 QoS 비호환 — RELIABLE 구독자는
        # BEST_EFFORT 발행자와 연결이 안 되는 조합이라(qos.py 모듈 docstring
        # 참고) 실기에서 "incompatible QoS, RELIABILITY" 경고와 함께 이미지가
        # 아예 안 들어왔다. 발행자와 맞춰서 고침.
        self.create_subscription(
            CompressedImage, image_topic, self._on_image, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(PointStamped, point_topic, self._on_point, 10)

        self.get_logger().info(
            f'ArmVisualizerNode ready  |  image={image_topic}  point={point_topic}')

    def _on_point(self, msg: PointStamped):
        self._last_point = msg.point
        self._last_point_stamp = self.get_clock().now()

    def _on_image(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if (self._last_point is not None and self._last_point_stamp is not None
                and (self.get_clock().now() - self._last_point_stamp).nanoseconds
                <= self.point_stale_sec * 1e9):
            p = self._last_point
            text = f'X={p.x:.3f} Y={p.y:.3f} Z={p.z:.3f} m'
        else:
            text = 'no target'

        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)

        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
