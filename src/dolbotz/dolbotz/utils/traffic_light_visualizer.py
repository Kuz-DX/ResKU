#!/usr/bin/env python3
"""traffic_light_visualizer.py — summer_traffic_node(신호등 인식)이 젝슨에서
발행하는 카메라 화면 + 검출 결과를 구독해서 로컬 PC 화면에 오버레이로 보여주는
간단한 디버그 뷰어 (독립 실행 스크립트).

dolbotz 패키지 의존성 없음 — rclpy + sensor_msgs + vision_msgs +
mission_manager_interfaces + opencv-python + numpy만 필요(cv_bridge도 안 씀,
CompressedImage 바이트를 cv2.imdecode로 직접 디코드). utils/ 아래 있지만
빌드/설치 없이 `python3 traffic_light_visualizer.py`로 바로 실행된다.

전제: 로컬 PC와 젝슨이 같은 네트워크에서 서로의 토픽을 봐야 한다 —
ROS_DOMAIN_ID를 젝슨과 동일하게 맞추고, ROS_LOCALHOST_ONLY=0이어야 하며,
RMW_IMPLEMENTATION도 맞는지 확인할 것. vision_msgs/mission_manager_interfaces
메시지 타입을 로컬에서 역직렬화해야 하므로 두 패키지 다 로컬에 있어야 한다
(mission_manager_interfaces는 dolbotz 워크스페이스를 최소 한 번 build 후
source해야 함 — 실행법 참고).

구독: image_topic       (sensor_msgs/CompressedImage, BEST_EFFORT/depth=1 —
                          summer_traffic.py의 SENSOR_DATA_QOS_DEPTH1과 동일)
      detections_topic  (vision_msgs/Detection2DArray, RELIABLE/depth=10)
      result_topic      (mission_manager_interfaces/MissionResult, RELIABLE/depth=10)

화면: 최신 카메라 프레임 위에 detections의 bbox를 전부 노랑으로 그리고,
좌상단에 현재 result.state('stop'/'go'/'unknown')를 크게 표시한다
(stop=빨강, go=초록, unknown/미수신=회색).

사용법:
  python3 traffic_light_visualizer.py
  python3 traffic_light_visualizer.py --image-topic ... --detections-topic ... --result-topic ...

키: q  종료 (Ctrl+C로도 종료)
"""
import argparse
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    from vision_msgs.msg import Detection2DArray
    _VISION_MSGS_OK = True
except ImportError:
    _VISION_MSGS_OK = False

try:
    from mission_manager_interfaces.msg import MissionResult
    _MISSION_MSGS_OK = True
except ImportError:
    _MISSION_MSGS_OK = False

STATE_COLOR_BGR = {
    'stop': (0, 0, 255),
    'go': (0, 255, 0),
}
DEFAULT_COLOR_BGR = (160, 160, 160)  # unknown/미수신 -- 회색
BBOX_COLOR_BGR = (0, 255, 255)       # 노랑


def _sensor_data_qos_depth1() -> QoSProfile:
    """카메라 이미지 구독용 — summer_traffic.py의 SENSOR_DATA_QOS_DEPTH1과
    동일 스펙(BEST_EFFORT/VOLATILE/depth=1). 이 파일은 dolbotz 패키지
    무의존 설계라 import 대신 인라인으로 중복 정의."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _reliable_qos() -> QoSProfile:
    """result/detections 발행자 쪽 기본 QoS(RELIABLE, depth 10)와 맞춘 프로필
    (summer_traffic.py가 create_publisher()에 QoS를 안 넘겨 이 기본값을
    그대로 쓰기 때문)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )


class TrafficLightVisualizer(Node):
    def __init__(self, args):
        super().__init__('traffic_light_visualizer')
        self.frame = None
        self.detections: list[tuple[int, int, int, int, str, float]] = []
        self.state = 'unknown'
        self.valid = False
        self._last_image_time = None

        self.create_subscription(
            CompressedImage, args.image_topic, self._on_image, _sensor_data_qos_depth1())

        if _VISION_MSGS_OK:
            self.create_subscription(
                Detection2DArray, args.detections_topic, self._on_detections, _reliable_qos())
        else:
            self.get_logger().warn(
                'vision_msgs 미설치 — bbox 오버레이 없이 카메라 화면만 표시됩니다. '
                "'sudo apt install ros-humble-vision-msgs' 후 재실행할 것.")

        if _MISSION_MSGS_OK:
            self.create_subscription(
                MissionResult, args.result_topic, self._on_result, _reliable_qos())
        else:
            self.get_logger().warn(
                'mission_manager_interfaces 미설치 — STATE 표시 없이 카메라/bbox만 '
                '표시됩니다. dolbotz 워크스페이스를 build/source한 후 재실행할 것.')

        self.get_logger().info(
            f'ready — image={args.image_topic}, detections={args.detections_topic}, '
            f'result={args.result_topic}')

    def _on_image(self, msg: CompressedImage) -> None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('이미지 디코드 실패', throttle_duration_sec=2.0)
            return
        self.frame = frame
        self._last_image_time = time.monotonic()

    def _on_detections(self, msg) -> None:
        dets = []
        for d in msg.detections:
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            w = d.bbox.size_x
            h = d.bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            label, conf = '', 0.0
            if d.results:
                label = d.results[0].hypothesis.class_id
                conf = d.results[0].hypothesis.score
            dets.append((x1, y1, x2, y2, label, conf))
        self.detections = dets

    def _on_result(self, msg) -> None:
        self.state = msg.state
        self.valid = msg.valid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--image-topic', default='/side/left/image_raw/compressed',
                         help='좌측 사이드캠 압축 이미지 토픽')
    parser.add_argument('--detections-topic', default='/mission/summer_traffic/left/detections',
                         help='신호등 bbox 원시 검출 토픽')
    parser.add_argument('--result-topic', default='/mission/summer_traffic/result',
                         help='신호등 판정 결과(state/valid) 토픽')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TrafficLightVisualizer(args)

    window = 'traffic_light_visualizer (q: quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    print('실행 중... 뷰어 창에 포커스 준 상태에서 q 종료, Ctrl+C로도 종료')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            if node.frame is not None:
                display = node.frame.copy()
                for x1, y1, x2, y2, label, conf in node.detections:
                    cv2.rectangle(display, (x1, y1), (x2, y2), BBOX_COLOR_BGR, 2)
                    if label:
                        cv2.putText(
                            display, f'{label} {conf:.2f}', (x1, max(0, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, BBOX_COLOR_BGR, 2, cv2.LINE_AA)

                color = STATE_COLOR_BGR.get(node.state, DEFAULT_COLOR_BGR) if node.valid else DEFAULT_COLOR_BGR
                cv2.putText(
                    display, f'STATE: {node.state}  (valid={node.valid})', (15, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3, cv2.LINE_AA)
            else:
                display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    display, 'waiting for image...', (30, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
