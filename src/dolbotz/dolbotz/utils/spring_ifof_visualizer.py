#!/usr/bin/env python3
"""spring_ifof_visualizer.py — spring_ifof_node(피아식별)이 젝슨에서 발행하는
좌/우 사이드캠 화면 + 검출 결과를 구독해서 로컬 PC 화면에 오버레이로 보여주는
간단한 디버그 뷰어 (독립 실행 스크립트).

traffic_light_visualizer.py와 동일한 설계 — dolbotz 패키지 의존성 없음
(rclpy + sensor_msgs + vision_msgs + mission_manager_interfaces + std_msgs +
opencv-python + numpy만 필요, cv_bridge도 안 씀). utils/ 아래 있지만
빌드/설치 없이 `python3 spring_ifof_visualizer.py`로 바로 실행된다.

spring_ifof는 좌/우 카메라가 각각 독립 판정을 하므로(모듈 docstring
참고), 이 뷰어도 좌/우 화면을 나란히 이어붙여 한 창에 보여준다.

전제: 로컬 PC와 젝슨이 같은 네트워크에서 서로의 토픽을 봐야 한다 —
ROS_DOMAIN_ID를 젝슨과 동일하게 맞추고, ROS_LOCALHOST_ONLY=0이어야 하며,
RMW_IMPLEMENTATION도 맞는지 확인할 것. vision_msgs/mission_manager_interfaces
메시지 타입을 로컬에서 역직렬화해야 하므로 두 패키지 다 로컬에 있어야 한다
(mission_manager_interfaces는 dolbotz 워크스페이스를 최소 한 번 build 후
source해야 함 — 실행법 참고).

구독: left_image_topic/right_image_topic   (sensor_msgs/CompressedImage,
                                              BEST_EFFORT/depth=1 —
                                              spring_ifof.py의
                                              SENSOR_DATA_QOS_DEPTH1과 동일)
      left_detections_topic/right_detections_topic
                                             (vision_msgs/Detection2DArray,
                                              RELIABLE/depth=10)
      result_topic      (mission_manager_interfaces/MissionResult,
                          RELIABLE/depth=10) — state('friend'/'enemy'/'unknown')
      detected_topic    (std_msgs/String, RELIABLE/depth=10) — 디바운싱
                          이전 원시 판정(/spring/ifof/detected)

화면: 좌/우 최신 카메라 프레임을 나란히 이어붙이고, 각각 위에 그
카메라의 detections bbox를 노랑으로 그린다. 좌상단에는 확정 결과
(/mission/spring_ifof/result의 state/valid)를, 그 아래에 원시 판정
(/spring/ifof/detected)을 표시한다.
friend=초록, enemy=빨강, unknown/미수신=회색.

사용법:
  python3 spring_ifof_visualizer.py
  python3 spring_ifof_visualizer.py --left-image-topic ... --right-image-topic ...

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
from std_msgs.msg import String

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
    'friend': (0, 255, 0),
    'enemy': (0, 0, 255),
}
DEFAULT_COLOR_BGR = (160, 160, 160)  # unknown/미수신 -- 회색
BBOX_COLOR_BGR = (0, 255, 255)       # 노랑
PLACEHOLDER_SIZE = (480, 640, 3)     # 프레임 미수신 카메라용 자리표시(H, W, C)


def _sensor_data_qos_depth1() -> QoSProfile:
    """카메라 이미지 구독용 — spring_ifof.py의 SENSOR_DATA_QOS_DEPTH1과
    동일 스펙(BEST_EFFORT/VOLATILE/depth=1). 이 파일은 dolbotz 패키지
    무의존 설계라 import 대신 인라인으로 중복 정의."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _reliable_qos() -> QoSProfile:
    """result/detections/detected 발행자 쪽 기본 QoS(RELIABLE, depth 10)와
    맞춘 프로필(spring_ifof.py가 create_publisher()에 QoS를 안 넘겨 이
    기본값을 그대로 쓰기 때문 — traffic_light_visualizer.py의
    _reliable_qos()와 동일 근거)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )


class _SideState:
    """좌/우 카메라 각각의 최신 프레임 + detections를 담는 그릇."""

    def __init__(self) -> None:
        self.frame = None
        self.detections: list[tuple[int, int, int, int, str, float]] = []
        self.last_image_time = None


class SpringIfofVisualizer(Node):
    def __init__(self, args):
        super().__init__('spring_ifof_visualizer')
        self.sides = {'left': _SideState(), 'right': _SideState()}
        self.state = 'unknown'
        self.valid = False
        self.raw_detected = 'unknown'

        self.create_subscription(
            CompressedImage, args.left_image_topic,
            lambda msg: self._on_image('left', msg), _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, args.right_image_topic,
            lambda msg: self._on_image('right', msg), _sensor_data_qos_depth1())

        if _VISION_MSGS_OK:
            self.create_subscription(
                Detection2DArray, args.left_detections_topic,
                lambda msg: self._on_detections('left', msg), _reliable_qos())
            self.create_subscription(
                Detection2DArray, args.right_detections_topic,
                lambda msg: self._on_detections('right', msg), _reliable_qos())
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

        self.create_subscription(
            String, args.detected_topic, self._on_detected, _reliable_qos())

        self.get_logger().info(
            f'ready — left_image={args.left_image_topic}, '
            f'right_image={args.right_image_topic}, result={args.result_topic}')

    def _on_image(self, side: str, msg: CompressedImage) -> None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn(f'{side} 이미지 디코드 실패', throttle_duration_sec=2.0)
            return
        self.sides[side].frame = frame
        self.sides[side].last_image_time = time.monotonic()

    def _on_detections(self, side: str, msg) -> None:
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
        self.sides[side].detections = dets

    def _on_result(self, msg) -> None:
        self.state = msg.state
        self.valid = msg.valid

    def _on_detected(self, msg: String) -> None:
        self.raw_detected = msg.data


def _render_side(side_state: _SideState, label: str) -> np.ndarray:
    if side_state.frame is not None:
        display = side_state.frame.copy()
        for x1, y1, x2, y2, det_label, conf in side_state.detections:
            cv2.rectangle(display, (x1, y1), (x2, y2), BBOX_COLOR_BGR, 2)
            if det_label:
                cv2.putText(
                    display, f'{det_label} {conf:.2f}', (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BBOX_COLOR_BGR, 2, cv2.LINE_AA)
    else:
        display = np.zeros(PLACEHOLDER_SIZE, dtype=np.uint8)
        cv2.putText(
            display, 'waiting for image...', (30, PLACEHOLDER_SIZE[0] // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(
        display, label, (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return display


def _hconcat_same_height(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """좌/우 프레임 해상도가 다를 수 있어(카메라 개별 설정), 더 작은 쪽
    높이에 맞춰 리사이즈한 뒤 옆으로 붙인다."""
    h = min(left.shape[0], right.shape[0])
    if left.shape[0] != h:
        left = cv2.resize(left, (int(left.shape[1] * h / left.shape[0]), h))
    if right.shape[0] != h:
        right = cv2.resize(right, (int(right.shape[1] * h / right.shape[0]), h))
    return cv2.hconcat([left, right])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--left-image-topic', default='/side/left/image_raw/compressed',
                         help='좌측 사이드캠 압축 이미지 토픽')
    parser.add_argument('--right-image-topic', default='/side/right/image_raw/compressed',
                         help='우측 사이드캠 압축 이미지 토픽')
    parser.add_argument('--left-detections-topic',
                         default='/mission/spring_ifof/left/detections',
                         help='좌측 피아식별 bbox 원시 검출 토픽')
    parser.add_argument('--right-detections-topic',
                         default='/mission/spring_ifof/right/detections',
                         help='우측 피아식별 bbox 원시 검출 토픽')
    parser.add_argument('--result-topic', default='/mission/spring_ifof/result',
                         help='피아식별 확정 결과(state/valid) 토픽')
    parser.add_argument('--detected-topic', default='/spring/ifof/detected',
                         help='디바운싱 이전 원시 판정 토픽')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = SpringIfofVisualizer(args)

    window = 'spring_ifof_visualizer (q: quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    print('실행 중... 뷰어 창에 포커스 준 상태에서 q 종료, Ctrl+C로도 종료')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            left_display = _render_side(node.sides['left'], 'LEFT')
            right_display = _render_side(node.sides['right'], 'RIGHT')
            display = _hconcat_same_height(left_display, right_display)

            color = (
                STATE_COLOR_BGR.get(node.state, DEFAULT_COLOR_BGR)
                if node.valid else DEFAULT_COLOR_BGR)
            cv2.putText(
                display, f'STATE: {node.state}  (valid={node.valid})', (15, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3, cv2.LINE_AA)
            raw_color = STATE_COLOR_BGR.get(node.raw_detected, DEFAULT_COLOR_BGR)
            cv2.putText(
                display, f'raw detected: {node.raw_detected}', (15, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, raw_color, 2, cv2.LINE_AA)

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
