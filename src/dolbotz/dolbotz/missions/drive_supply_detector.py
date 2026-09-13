import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty
from cv_bridge import CvBridge

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

# [2026-09-03] supplyboxv3.pt(FP32)를 OpenVINO INT8로 교체 -
# summer_supply.py의 _DEFAULT_MODEL_FILE과 동일 절차/근거/검증 결과
# (config/models/README.md 참고). 이전 FP32로 되돌리려면 아래 주석 처리된
# 줄로 바꿀 것.
_DEFAULT_MODEL_FILE = 'supplyboxv3_int8_openvino_model'
# _DEFAULT_MODEL_FILE = 'supplyboxv3.pt'


class DriveSupplyDetectorNode(Node):
    """
    주행용 뎁스카메라(색상 스트림)로 지면의 서플라이박스를 감지해서 "발견했다"
    신호만 발행한다 - summer_supply.py(ArmPickupNode, 팔 카메라)와 같은
    YOLO 모델(supplyboxv3.pt)을 재사용하되, 좌표 추출은 하지 않는다(그건 팔
    카메라/summer_supply의 역할). 주행 중에는 박스가 지면에 있어 바로 안
    보이다가, 이 노드가 먼저 "근처에 박스가 있다"를 감지해서 팔을
    GRASP_WAIT 자세로 옮기는 트리거로만 쓴다.

    [2026-08-31 신규] 구동부(주행 정지 등) 연동은 이번 스코프 밖 - 팔
    쪽(maru_ik_node.py)만 이 토픽을 구독해서 HOME -> GRASP_WAIT로 이동한다.

    [2026-09-02 신규] 파지 완료("/arm/picking_command", maru_ik_node.py 1회성
    발행) 수신 시 YOLO 모델을 영구 종료한다(_on_picking_command) - 이번
    미션은 서플라이박스가 하나뿐이라 이후 재탐지가 불필요해서다. 여러 박스를
    다시 지원해야 하면 이 종료를 없애거나 재시작 트리거를 추가할 것.

    퍼블리시:
      /drive/supplybox_detected (std_msgs/Empty) - conf_threshold 이상 탐지
      시 1회성 신호. detected_min_interval_sec 간격으로 스팸 방지(forward_command/
      picking_command와 동일 패턴).

    파라미터:
      model_path      : YOLO 가중치 경로. 비우면 config/models/supplyboxv3.pt 사용
      target_class     : 결과에 붙일 클래스 이름(로그용) - 단일 클래스 학습이라
                          실제 필터링에는 안 쓰임
      conf_threshold   : 최소 confidence (기본 0.5)
      infer_size       : YOLO 추론 해상도(기본 320)
      color_topic      : 주행 카메라 컬러 토픽
      detected_topic   : 기본 '/drive/supplybox_detected'
      detected_min_interval_sec : 최소 발행 간격(기본 2.0초, 스팸 방지)
    """

    def __init__(self):
        super().__init__('drive_supply_detector_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('target_class', 'supplybox')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('infer_size', 320)
        self.declare_parameter(
            'color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('detected_topic', '/drive/supplybox_detected')
        self.declare_parameter('detected_min_interval_sec', 2.0)

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            model_path = str(get_models_dir() / _DEFAULT_MODEL_FILE)

        self.target = str(self.get_parameter('target_class').value)
        self.conf_th = float(self.get_parameter('conf_threshold').value)
        self.infer_size = int(self.get_parameter('infer_size').value)
        self.detected_topic = str(self.get_parameter('detected_topic').value)
        self.detected_min_interval_sec = float(
            self.get_parameter('detected_min_interval_sec').value)
        self._last_detected_time = 0.0
        color_topic = str(self.get_parameter('color_topic').value)

        self.model = self._load_model(model_path)
        self.bridge = CvBridge()

        self.pub_detected = self.create_publisher(Empty, self.detected_topic, 10)
        self.create_subscription(
            CompressedImage, color_topic, self._on_color, SENSOR_DATA_QOS_DEPTH1)
        # [2026-09-02 신규] maru_ik_node.py가 파지 완료 후 1회성으로 발행하는
        # "/arm/picking_command" - 이번 미션은 서플라이박스가 하나뿐이라 이후
        # 재탐지가 필요 없어 모델을 영구 종료한다(summer_supply.py의
        # _on_picking_command와 동일 패턴, CPU 낭비 방지).
        self.create_subscription(
            Empty, '/arm/picking_command', self._on_picking_command, 10)

        self.get_logger().info(
            f'DriveSupplyDetectorNode ready  |  {color_topic} -> target={self.target} '
            f'conf>={self.conf_th}  (감지 시 {self.detected_topic} 1회성 발행)')

    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn(
                'model_path 미설정 - 학습파일 경로를 파라미터로 전달하세요')
            return None
        if not _YOLO_OK:
            self.get_logger().error('ultralytics 미설치 - pip install ultralytics')
            return None
        model = YOLO(path)
        self.get_logger().info(f'YOLO 모델 로드 완료: {path}')
        return model

    def _on_picking_command(self, _msg: Empty) -> None:
        """[2026-09-02 신규] 파지 완료 신호 수신 시 YOLO 모델을 영구
        종료한다. 이번 미션은 서플라이박스가 하나뿐이라 재시작 로직은 없다."""
        if self.model is None:
            return
        self.model = None
        self.get_logger().info(
            'picking_command 수신 - supplybox YOLO 모델 종료(추론 중단).')

    def _on_color(self, msg: CompressedImage):
        if self.model is None:
            return
        color = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(
            color, imgsz=self.infer_size, conf=self.conf_th, verbose=False)
        detected = any(
            r.boxes is not None and len(r.boxes) > 0 for r in results)
        if not detected:
            return
        now = time.monotonic()
        if now - self._last_detected_time < self.detected_min_interval_sec:
            return
        self._last_detected_time = now
        self.get_logger().info(
            f"'{self.target}' 발견 - {self.detected_topic} 발행.",
            throttle_duration_sec=2.0)
        self.pub_detected.publish(Empty())


def main():
    rclpy.init()
    node = DriveSupplyDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
