"""여름 미션 — 신호등(정지/진행) 인식 노드.

좌측 사이드캠(side_cameras.launch.py, usb_cam) 단일캠 구조 — 카메라로 신호등
색상/상태를 인식해 결과만 발행한다. 실제 정지/진행 판단에 따른 주행 제어
(감속/정지)는 이 노드가 아니라 결과를 구독하는 제어부가 담당한다.

판정은 두 단계다. 먼저 각 프레임에서 ROI와 최소 면적 조건을 통과한 박스 중
conf_threshold 이상이고 confidence가 가장 높은 하나를 원시 상태로 고른다
(pick_best_state 참고). 다음으로 같은 non-unknown 상태가 confirm_frames회
연속 나와야 확정한다. 확정 전이나 unknown이 들어온 프레임에는
state='unknown', valid=False를 발행한다.

구독: /side/left/image_raw/compressed (sensor_msgs/CompressedImage)
      /side/left/camera_info (sensor_msgs/CameraInfo)
      /arm/picking_command (std_msgs/Empty) — maru_ik_node.py가 서플라이박스
      파지 시퀀스 완료 후 1회성으로 발행하는 신호(summer_supply.py/
      drive_supply_detector.py가 같은 토픽으로 자기 YOLO 모델을 종료하는 것과
      동일 패턴). [2026-09-02 신규] 이 노드는 반대로 이 신호를 받기 전까지는
      신호등 모델을 아예 로드하지 않는다(_on_picking_command에서 최초 1회
      로드) — 서플라이박스 탐지 구간과 신호등 구간이 겹치지 않는 미션
      구조이므로, 파지 끝나기 전까지 불필요하게 GPU/CPU 메모리를 점유할
      필요가 없다. 로드 전에는 모든 프레임에 state='unknown', valid=False만
      발행한다(model=None 처리 분기, 기존과 동일).
발행: /mission/summer_traffic/result (mission_manager_interfaces/MissionResult)
      state: 'stop' | 'go' | 'unknown' — summer_supply_drive_node.cpp
      (robot_bringup, 구호물자 피킹 구간 주행 상태머신)가 이 토픽을 구독해
      실제 정지/진행 제어를 돌린다.
      /mission/summer_traffic/traffic_go (std_msgs/Bool)
      /mission/summer_traffic/traffic_stop (std_msgs/Bool)
      result를 대체하지 않는 보조 토픽 — traffic_go.data = (valid and
      state=='go'), traffic_stop.data = (valid and state=='stop').
      /mission/summer_traffic/left/detections (vision_msgs/Detection2DArray)
      공간 필터 통과 여부와 무관하게 conf_threshold 이상인 원시 검출 전부
      (result는 필터+확정까지 거친 판정만 반영). vision_msgs 미설치 시 이
      토픽만 발행을 건너뛴다. 좌측 단일캠 구조라 right/detections는 없다.
"""
from pathlib import Path

from std_msgs.msg import Bool, Empty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from mission_manager_interfaces.msg import MissionResult

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir
from dolbotz.utils.state_debouncer import ConsecutiveStateDebouncer

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    _VISION_MSGS_OK = True
except ImportError:
    # package.xml에는 의존성을 선언해도 실제 apt install(ros-humble-vision-msgs)이
    # 안 된 워크스페이스에서는 여전히 여기로 빠진다 — _YOLO_OK와 같은 취지로
    # 안전하게 no-op(경고만 내고 detections 토픽만 미발행)한다. 판정 결과
    # (/mission/summer_traffic/result) 발행은 이 값과 무관하게 항상 정상 동작.
    _VISION_MSGS_OK = False

# 학습된 신호등 모델(trafficlightv1.pt) 상대경로 — config/models/ 바로
# 아래 둔다(절대경로 하드코딩 금지 원칙, config/models/README.md 참고).
# __init__에서 get_models_dir()와 조합해 model_path 파라미터 기본값으로 쓴다.
# [2026-09-03] trafficlightv1.pt(FP32)를 OpenVINO INT8로 교체 - 나머지
# 5개 모델과 동일 절차, 캘리브레이션/검증 데이터는 Roboflow
# traffic-light-detection-hznds v1(원본 학습 args와 project slug까지 일치,
# 단일 세트로 확인됨 - config/models/README.md 참고). valid 200장 mAP는
# 다른 모델들보다 더 뚜렷하게 떨어졌지만(mAP50-95 0.607->0.487), summer_traffic.py
# 의 실제 판정 로직(pick_best_state, conf_threshold=0.8)을 그대로 재현해서
# 200장 전부 직접 대조한 결과 stop<->go가 실제로 뒤바뀐 케이스는 0건이었다
# - 차이 36건은 전부 FP32가 확신하던 걸 INT8이 conf 0.8을 못 넘겨
# unknown으로 더 보수적으로 판정한 것뿐(반대 방향 4건 포함), 색 자체를
# 잘못 본 적은 없음. 추론시간은 22.69ms->2.85ms(약 8배). 이전 FP32로
# 되돌리려면 아래 주석 처리된 줄로 바꿀 것.
TRAFFIC_MODEL_RELATIVE_PATH = Path('trafficlightv1_int8_openvino_model')
# TRAFFIC_MODEL_RELATIVE_PATH = Path('trafficlightv1.pt')
# pick_best_state가 프레임 안에서 무조건 confidence가 가장 높은 박스 하나를
# 채택하므로, 임계값이 낮으면 그 프레임 최고값이 낮아도(사실상 오인식) 그대로
# 채택돼버린다. 0.8로 올려서 애매한 "최고값"까지 detect로 인정하는 걸 막는다.
CONF_THRESHOLD_PLACEHOLDER = 0.8

# [2026-08-27] 신호등은 피아식별보다 빠른 반응이 필요하고, 참고한 기존
# 신호등 코드도 빨간불을 3프레임 연속 확인했다. 단발성 오탐은 막되 지연을
# 줄이기 위해 spring_ifof의 5보다 짧은 3을 기본값으로 사용한다.
CONFIRM_FRAMES_PLACEHOLDER = 3

# [2026-08-27] 기존에 참고한 ROI 기준과 동일하게, 박스 중심이 화면 높이의
# 60%보다 아래면 간판이나 다른 조명일 가능성이 높다고 보고 제외한다.
TRAFFIC_ROI_TOP_RATIO_PLACEHOLDER = 0.6

# [2026-08-27] 기존에 참고했던 red_light_min_area 기본값을 유지한다. 현재
# 노드는 빨강/초록을 대칭적으로 선택하므로 이름은 호환을 위해 그대로 두되
# 두 클래스 모두에 적용한다. 너무 멀리 있어 100px²보다 작은 검출은 판정에서
# 제외한다.
RED_LIGHT_MIN_AREA_PLACEHOLDER = 100

# [중요] 실제 학습된 신호등 모델(trafficlightv1.pt)의 클래스 이름과 반드시
# 일치해야 한다 — 틀리면 빨간불인데 진행 판정이 나가는, 이 미션에서 제일
# 위험한 실수가 된다. [2026-08-30] 모델을 로드해서 실제 클래스 이름을
# 확인함: {0: 'green', 1: 'red', 2: 'yellow'}.
CLASS_NAMES_PLACEHOLDER = ('green', 'red', 'yellow')
# YOLO 클래스 라벨(모델 학습 시 이름) -> MissionResult.state 매핑.
# [2026-08-30, 사용자 결정] yellow는 의도적으로 미포함 — pick_best_state()는
# label_to_state에 없는 클래스를 건너뛰므로, yellow가 프레임 안에서 confidence가
# 가장 높아도 무시되고 다음 후보(red/green)로 넘어가며, red/green 후보가
# 하나도 없으면 그 프레임은 'unknown'으로 빠진다(=go도 stop도 아님).
_LABEL_TO_STATE = {'red': 'stop', 'green': 'go'}


# ---------------------------------------------------------------------------
# 순수 계산 — ROS/ultralytics 의존성 없음
# ---------------------------------------------------------------------------

def pick_best_state(
    detections: list[tuple[str, float]],
    conf_threshold: float,
    label_to_state: dict,
) -> str:
    """(클래스 이름, confidence) 튜플 목록에서 conf_threshold 이상이고
    label_to_state에 등록된 클래스 중 confidence가 가장 높은 것 하나를 골라
    그 상태 문자열을 돌려준다. 해당하는 게 하나도 없으면 'unknown'.

    이전엔(다른 미션들처럼) 임계값 이상인 걸 전부 보고 규칙으로 합쳤는데,
    여긴 "가장 확신하는 것 하나만 믿는다"는 정책으로 단순화했다 — 프레임
    안에 빨간불/초록불이 동시에 잡히는 오탐 상황에서 다수결/우선순위 규칙
    없이 confidence가 곧 우선순위가 된다."""
    best_conf = -1.0
    best_state = 'unknown'
    for cls_name, conf in detections:
        if conf < conf_threshold or conf <= best_conf:
            continue
        state = label_to_state.get(cls_name)
        if state is None:
            continue
        best_conf = conf
        best_state = state
    return best_state


def passes_spatial_filters(
    bbox_xyxy: tuple[float, float, float, float],
    image_height: int,
    roi_top_ratio: float,
    min_area: float,
) -> bool:
    """박스가 상단 ROI 안에 있고 최소 면적 이상인지 검사한다.

    기존에 참고한 기준과 동일하게 중심 y가 image_height * roi_top_ratio보다
    크면 화면 하단 검출로 보고 제외한다. 경계값은 허용한다.
    """
    x1, y1, x2, y2 = bbox_xyxy
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    center_y = (y1 + y2) / 2.0
    return width * height >= min_area and center_y <= image_height * roi_top_ratio


class SummerTrafficNode(Node):
    """신호등 상태 판정 결과를 발행하는 노드."""

    def __init__(self):
        super().__init__('summer_traffic_node')

        self.declare_parameter(
            'model_path', str(get_models_dir() / TRAFFIC_MODEL_RELATIVE_PATH))
        self.declare_parameter('conf_threshold', CONF_THRESHOLD_PLACEHOLDER)
        self.declare_parameter('confirm_frames', CONFIRM_FRAMES_PLACEHOLDER)
        self.declare_parameter(
            'traffic_roi_top_ratio', TRAFFIC_ROI_TOP_RATIO_PLACEHOLDER)
        self.declare_parameter('red_light_min_area', RED_LIGHT_MIN_AREA_PLACEHOLDER)
        self.declare_parameter(
            'color_topic', '/side/left/image_raw/compressed')
        self.declare_parameter(
            'camera_info_topic', '/side/left/camera_info')

        # [2026-09-02 신규] 모델은 여기서 바로 로드하지 않는다 — 경로만
        # 저장해두고 _on_picking_command가 /arm/picking_command 수신 시
        # 최초 1회 로드한다(모듈 docstring 구독 절 참고). 그 전까지 self._model은
        # None으로 남아 _on_image의 기존 model=None 분기(unknown 발행)가 그대로
        # "신호등 모델 비활성" 상태 역할을 한다.
        self._model_path = str(self.get_parameter('model_path').value)
        self._conf_th = float(self.get_parameter('conf_threshold').value)
        confirm_frames = int(self.get_parameter('confirm_frames').value)
        self._traffic_roi_top_ratio = float(
            self.get_parameter('traffic_roi_top_ratio').value)
        self._red_light_min_area = float(
            self.get_parameter('red_light_min_area').value)
        color_topic = str(self.get_parameter('color_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)

        self._model = None
        self._bridge = CvBridge()
        self._debouncer = ConsecutiveStateDebouncer(confirm_frames)

        self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, SENSOR_DATA_QOS_DEPTH1)
        self.create_subscription(
            CompressedImage, color_topic, self._on_image, SENSOR_DATA_QOS_DEPTH1)
        # [2026-09-02 신규] summer_supply.py/drive_supply_detector.py의
        # _on_picking_command(모델 종료)와 대칭되는 활성화 트리거 — 리터럴
        # 하드코딩 이유도 동일(그쪽 _on_picking_command 주석 참고).
        self.create_subscription(
            Empty, '/arm/picking_command', self._on_picking_command, 10)
        self._result_pub = self.create_publisher(
            MissionResult, '/mission/summer_traffic/result', 10)
        # 모듈 docstring 발행 절 참고.
        self._traffic_go_pub = self.create_publisher(
            Bool, '/mission/summer_traffic/traffic_go', 10)
        self._traffic_stop_pub = self.create_publisher(
            Bool, '/mission/summer_traffic/traffic_stop', 10)
        self._left_detections_pub = None
        if _VISION_MSGS_OK:
            self._left_detections_pub = self.create_publisher(
                Detection2DArray, '/mission/summer_traffic/left/detections', 10)
        else:
            self.get_logger().warn(
                'vision_msgs 미설치 — /mission/summer_traffic/left/detections '
                '미발행(판정 결과 발행에는 영향 없음). '
                'ros-humble-vision-msgs 설치 후 재기동하면 발행됨.')

        self.get_logger().info(
            f'SummerTrafficNode ready (모델 대기 중 — /arm/picking_command 수신 '
            f'시 로드) — confirm_frames={confirm_frames}, '
            f'traffic_roi_top_ratio={self._traffic_roi_top_ratio}, '
            f'red_light_min_area={self._red_light_min_area:g}')

    def _load_model(self, path: str):
        if not path or not _YOLO_OK:
            self.get_logger().warn(
                'model_path 미설정 또는 ultralytics 미설치 — 모든 프레임 unknown 발행함')
            return None
        return YOLO(path)

    def _on_picking_command(self, _msg: Empty) -> None:
        """[2026-09-02 신규] 서플라이박스 파지 완료 신호 수신 시 신호등
        YOLO 모델을 최초 1회 로드해 활성화한다. summer_supply.py/
        drive_supply_detector.py의 _on_picking_command(모델 종료)와 반대
        방향의 대칭 동작 — 이번 미션은 서플라이 탐지 구간과 신호등 구간이
        겹치지 않으므로 그 전까지는 로드 자체를 하지 않는다."""
        if self._model is not None:
            return
        self._model = self._load_model(self._model_path)
        self.get_logger().info(
            'picking_command 수신 - 신호등 YOLO 모델 로드 완료(추론 시작).')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        pass

    def _publish_detections(self, header, raw_detections) -> None:
        """raw_detections((x1,y1,x2,y2), cls_name, conf) 리스트를
        Detection2DArray로 발행한다. vision_msgs 미설치면 __init__에서
        _left_detections_pub이 None으로 남아있으므로 조용히 넘어간다."""
        if self._left_detections_pub is None:
            return
        array = Detection2DArray()
        array.header = header
        for (x1, y1, x2, y2), cls_name, conf in raw_detections:
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = max(0.0, x2 - x1)
            detection.bbox.size_y = max(0.0, y2 - y1)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = cls_name
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)
            array.detections.append(detection)
        self._left_detections_pub.publish(array)

    def _publish_result(self, result: MissionResult) -> None:
        """result(MissionResult)와, 거기서 파생한 traffic_go/traffic_stop
        Bool을 항상 같은 프레임에 함께 발행한다 — 모듈 docstring 발행 절
        참고. result가 대표(진실의 원천)고 두 Bool은 그 값을 그대로 반영하는
        파생 뷰라, 이 메서드 하나로 묶어서 셋이 어긋날 일이 없게 한다."""
        self._result_pub.publish(result)
        go = Bool()
        go.data = bool(result.valid and result.state == 'go')
        self._traffic_go_pub.publish(go)
        stop = Bool()
        stop.data = bool(result.valid and result.state == 'stop')
        self._traffic_stop_pub.publish(stop)

    def _on_image(self, msg: CompressedImage) -> None:
        result = MissionResult()
        result.header = msg.header
        result.mission_name = 'summer_traffic'

        if self._model is None:
            self._debouncer.update('unknown')
            result.state = 'unknown'
            result.valid = False
            self._publish_result(result)
            return

        try:
            cv_image = self._bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(
                f'CvBridge decode failed: {exc}', throttle_duration_sec=5.0)
            self._debouncer.update('unknown')
            result.state = 'unknown'
            result.valid = False
            self._publish_result(result)
            return

        results = self._model(cv_image, conf=self._conf_th, verbose=False)
        detections: list[tuple[str, float]] = []
        # raw_detections: bbox 좌표까지 포함한 원시 검출 전부(left/detections용,
        # 모듈 docstring 발행 절 참고) — detections는 그중 공간 필터까지
        # 통과한 것만(판정용).
        raw_detections: list[tuple[tuple[float, float, float, float], str, float]] = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_name = self._model.names.get(int(box.cls[0]), '')
                bbox = tuple(float(value) for value in box.xyxy[0].tolist())
                conf = float(box.conf[0])
                raw_detections.append((bbox, cls_name, conf))
                if not passes_spatial_filters(
                    bbox,
                    cv_image.shape[0],
                    self._traffic_roi_top_ratio,
                    self._red_light_min_area,
                ):
                    continue
                detections.append((cls_name, conf))

        self._publish_detections(msg.header, raw_detections)

        raw_state = pick_best_state(detections, self._conf_th, _LABEL_TO_STATE)
        confirmed_state = self._debouncer.update(raw_state)
        result.state = confirmed_state if confirmed_state is not None else 'unknown'
        result.valid = confirmed_state is not None
        self._publish_result(result)


def main(args=None):
    rclpy.init(args=args)
    node = SummerTrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
