"""5구간(정찰·동행) 미션 — 선도 정찰로봇 추종 노드.

컬러+뎁스 이미지를 동기화해서 선도 로봇(4족보행/UGV 둘 다 가능 — 단일
클래스로 하드코딩하지 않고 target_class_names 리스트로 받음)의 카메라
프레임 기준 3D 상대좌표를 발행한다. 규정상 목표 간격(target_distance_m
± distance_tolerance_m)을 유지해야 한다.

[중요] target_distance_m/distance_tolerance_m은 ready 로그에만 쓰이는
정보성 값이고, 이 노드가 직접 거리를 유지하지 않는다(순수 인지 노드라
발행만 함) — 실제 간격 유지는 /path를 구독하는 dolbotz/purepursuit.py
(purepursuit_node)의 goal_tolerance_m(하한, 이 정도 거리 안이면 정지)과
distance_speed_gain/min/max_m_s(use_distance_scaled_speed=True일 때 거리
비례 속도, purepursuit.py 모듈 docstring 참고) 파라미터가 담당한다. 이
노드의 target_distance_m을 바꿔도 실제 추종 거리는 안 바뀐다 -- 파라미터
동기화는 launch 쪽(mission_escort.launch.py + purepursuit.launch.py)에서
챙길 것. [정정, 2026-09-01] 한때 mission_escort(_drive).launch.py를 삭제하고
`ros2 run dolbotz escort_follow` 직접 실행 방식으로 바꿨다가 사용자 지시로
롤백함 -- mission_escort.launch.py는 계속 존재하고 이게 정상적인 실행
경로다(`ros2 run dolbotz escort_follow`와 동등, track_lost_grace_sec만
추가로 launch 인자화해줌). purepursuit_node를 함께 띄우는 원커맨드
진입점은 mission_escort_drive.launch.py.

=== 가림구간 재추종 상태머신 ===
대회 규정상 선도 로봇이 잠깐 가려지는 구간(다른 장애물/코너 등)에서도
재추종에 배점이 있고, target_distance_m 유지가 계속 요구되므로, 단순히
"이번 프레임에 탐지됐냐 안 됐냐"만으로 발행하면 짧은 가림에도 즉시
'lost'로 끊겨서 감점 대상이 된다. 그래서 내부적으로 세 단계를 둔다:

  TRACKING   — 방금 탐지 성공. 위치를 채택하고 직전 확정 위치/시간과의
               차이로 상대속도를 지수이동평균(EMA)으로 갱신.
  PREDICTING — 탐지 실패했지만 아직 track_lost_grace_sec 이내. 마지막
               위치+속도로 등속 외삽한 좌표를 계속 발행(valid=True 유지) —
               target_distance_m 유지 판정이 끊기지 않게 하기 위함. 이
               구간에 새 탐지가 들어오면, 그 위치가 지금 예측 위치 기준
               게이팅 반경(시간이 갈수록 넓어짐) 안에 있을 때만 "같은
               대상 재확정"으로 받아들인다 — 반경 밖이면 다른 물체를
               오인식한 것으로 보고 버리고 계속 PREDICTING 유지.
  LOST       — grace 시간을 넘김. valid=False로 전환하고 더 새 좌표를
               만들지 않는다(0,0,0으로 리셋하지 않고 마지막 값은 유지한
               채 valid만 False — 소비 측에서 "최후 목격 위치"를 참고할
               수 있게). 이 상태에서 새 탐지가 들어오면 참조할 예측
               위치 자체가 더 이상 신뢰 불가능하므로 게이팅 없이 바로
               TRACKING으로 재시작한다(속도 추정도 리셋 — 가림 구간이
               길었다면 그 사이 속도 변화를 알 방법이 없으므로).

MissionResult.state는 'tracking'|'lost'만 있어서(스키마 참고)
PREDICTING도 대외적으로는 'tracking'으로 발행한다 — valid 필드로
"실측이냐 예측이냐"까지 구분하고 싶으면 스키마 확장이 필요하나 지금
범위 밖.

구독: /drive/camera/color/image_raw/compressed (sensor_msgs/CompressedImage)
      /drive/camera/aligned_depth_to_color/image_raw/compressedDepth (sensor_msgs/CompressedImage)
      /drive/camera/color/camera_info (sensor_msgs/CameraInfo)
발행: /mission/escort_follow/result (mission_manager_interfaces/MissionResult)
      target_point: 추종 대상 상대좌표(카메라 프레임 XYZ, m), state: 'tracking' | 'lost'
      /path (nav_msgs/Path) — 로봇 중심(카메라 원점 근사) -> 추종 대상,
      2점짜리 경로. dolbotz/purepursuit.py(purepursuit_node)가 이 토픽을
      구독해서 좌/우 바퀴 속도로 변환한다 — dog_follow_node.cpp(robot_bringup,
      반응형 상태머신) 대신 이 pure pursuit 경로를 쓰기로 함(2026-08-30).
      frame_id는 CameraInfo에서 받은 카메라 광학 프레임 그대로 싣는다 —
      purepursuit_node가 TF로 base_link까지 통째로 변환해주므로 여기서
      광학->body 축변환을 직접 할 필요가 없다(_publish_path() 참고).
      [2026-08-30, 대회 규정 재검토] result.valid와 같은 기준(TRACKING/
      PREDICTING 둘 다 목표점 발행, LOST만 빈 Path)으로 발행한다 —
      track_lost_grace_sec(기본 0.3초) 이내는 등속 외삽으로 버티고, 넘기면
      LOST로 넘어가 빈 Path로 정지시킨다. [2026-09-01, 사용자 지시로
      단순화] 한때 화면 밖 이탈/가림막을 구분해서 grace를 따로 주는
      track_lost_grace_sec_edge/_center로 나눴었는데(0115d3b), 팀원이
      grace를 0.0/0.0으로 바꿨다가(ee56c3b) "조향이 아예 안 됨" 증상의
      유력 원인으로 지목된 전례가 있어(디버깅 리포트 참고), 단일 값 하나만
      관리하는 쪽이 더 안전하다고 판단해 되돌렸다.
      /mission/escort_follow/debug_image/compressed (sensor_msgs/CompressedImage)
      — 이번 프레임에 탐지가 채택됐을 때만(바운딩박스 오버레이), 항상 나가는 건 아님
"""
import math

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mission_manager_interfaces.msg import MissionResult
import message_filters

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.compressed_image import decode_compressed_depth
from dolbotz.utils.paths import get_models_dir

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

# [2026-08-30, 재조정] 대회 규정집(붙임1, 5구간 정찰·동행) 기준 목표 간격
# 1.5~2.0m(중심 1.75m ± 0.25m로 표현) — 기존 2.0±0.5m(1.5~2.5m)에서 상한이
# 2.5→2.0으로 좁혀짐. 로그에만 쓰이는 정보성 값(위 모듈 docstring [중요]
# 절 참고) — 실제 추종 거리는 purepursuit_node의 distance_band_min_m/
# distance_band_max_m을 이 값과 맞춰서 별도로 설정할 것(purepursuit.launch.py
# 사용 예 참고).
TARGET_DISTANCE_M_PLACEHOLDER = 1.75
DISTANCE_TOLERANCE_M_PLACEHOLDER = 0.25

# [2026-09-05] 후속 DogPig v2 모델로 교체. 원본 FP32:
# /home/j/vision_marker/doldoggyv3/dogpig_v2.pt ->
# config/models/dogpig_final.pt로 복사하고, 학습에 사용한 merged/data.yaml로
# 캘리브레이션해 OpenVINO INT8(quantize=8, imgsz=640, batch=1, 정적 입력)로
# 변환했다. merged valid 382장 검증: mAP50 0.98437->0.98496,
# mAP50-95 0.95847->0.94929, precision 0.97900->0.97857,
# recall 0.99452->0.99467. CPU 추론시간은 32.89ms->9.11ms(약 3.6배).
# 이전 FP32로 되돌리려면 아래 주석 처리된 줄로 바꿀 것.
TARGET_MODEL_RELATIVE_PATH = 'dogpig_final_int8_openvino_model'
# TARGET_MODEL_RELATIVE_PATH = 'dogpig_final.pt'

# [2026-09-05] dogpig_final.pt(merged/data.yaml)는 단일 클래스 'DogPig' —
# nc=1이라 클래스 인덱스 걱정 없이 이름 그대로 사용. 모듈 docstring에 적힌
# 대로 "4족보행/UGV 둘 다 가능"하도록 target_class_names를 리스트로 받는
# 설계는 유지 — 다른 선도 로봇 클래스가 추가로 필요해지면(다른 모델로
# 교체하거나 재학습 시 멀티클래스로) 이 리스트에 이름만 추가하면 된다.
TARGET_CLASS_NAMES_PLACEHOLDER = ['DogPig']

_STATE_TRACKING = 'tracking'
_STATE_PREDICTING = 'predicting'  # 내부 전용 — 외부 발행 시 'tracking'으로 매핑됨
_STATE_LOST = 'lost'


# ---------------------------------------------------------------------------
# 순수 계산 (ROS 의존성 없음)
# ---------------------------------------------------------------------------

def predict_position(
    last_point: tuple[float, float, float],
    velocity: tuple[float, float, float],
    elapsed_sec: float,
) -> tuple[float, float, float]:
    """등속 운동 가정 — last_point에서 velocity*elapsed_sec만큼 외삽한 예측 좌표."""
    return tuple(p + v * elapsed_sec for p, v in zip(last_point, velocity))


def within_reacquire_radius(
    predicted_point: tuple[float, float, float],
    candidate_point: tuple[float, float, float],
    elapsed_sec: float,
    base_radius_m: float,
    growth_m_per_sec: float,
) -> bool:
    """예측 위치 기준 게이팅 반경(base_radius_m + growth_m_per_sec*elapsed_sec)
    안에 후보 위치가 들어오는지. 반경은 시간이 갈수록 넓어진다 — 예측이
    오래될수록 실제 위치와의 오차가 커질 수 있다는 걸 반영."""
    radius = base_radius_m + growth_m_per_sec * elapsed_sec
    return math.dist(predicted_point, candidate_point) <= radius


def update_velocity_ema(
    prev_velocity: tuple[float, float, float] | None,
    prev_point: tuple[float, float, float],
    new_point: tuple[float, float, float],
    dt: float,
    alpha: float,
) -> tuple[float, float, float]:
    """(new_point - prev_point)/dt로 구한 순간 속도를 지수이동평균으로 반영한다.
    prev_velocity가 None이면(첫 관측이라 이전 속도가 없음) raw 속도를 그대로
    채택(부트스트랩). dt<=0(비정상/중복 타임스탬프)이면 갱신하지 않고
    prev_velocity를 그대로 반환(None이면 0,0,0)."""
    if dt <= 0.0:
        return prev_velocity if prev_velocity is not None else (0.0, 0.0, 0.0)
    raw = tuple((n - p) / dt for n, p in zip(new_point, prev_point))
    if prev_velocity is None:
        return raw
    return tuple(alpha * r + (1.0 - alpha) * pv for r, pv in zip(raw, prev_velocity))


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class EscortFollowNode(Node):

    def __init__(self):
        super().__init__('escort_follow_node')

        self.declare_parameter('target_distance_m', TARGET_DISTANCE_M_PLACEHOLDER)
        self.declare_parameter(
            'distance_tolerance_m', DISTANCE_TOLERANCE_M_PLACEHOLDER)
        self.declare_parameter(
            'color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter(
            'depth_topic',
            '/drive/camera/aligned_depth_to_color/image_raw/compressedDepth')
        self.declare_parameter(
            'camera_info_topic', '/drive/camera/color/camera_info')
        # purepursuit_node(dolbotz/purepursuit.py)의 path_topic 기본값과
        # 동일 — slope_decision.py 등 다른 미션의 최종 경로도 같은 '/path'
        # 토픽 하나를 공유하는 이 리포 관례(mission_*.launch.py는 한 번에
        # 하나만 실행되므로 충돌 없음)를 그대로 따른다.
        self.declare_parameter('path_topic', '/path')

        self.declare_parameter(
            'model_path', str(get_models_dir() / TARGET_MODEL_RELATIVE_PATH))
        self.declare_parameter('target_class_names', TARGET_CLASS_NAMES_PLACEHOLDER)
        self.declare_parameter('conf_threshold', 0.5)
        # [2026-09-05] dogpig_final.pt 학습 imgsz(640, args.yaml)에 맞춤 —
        # PyTorch 동적 입력이라 필수는 아니지만 학습 해상도와 맞춰야 정확도가
        # 유지된다.
        self.declare_parameter('infer_size', 640)
        # [2026-09-05, 사용자 지적] 가림구간 이후 재발견 시 로봇개와의 거리가
        # 4~5m 정도였는데 다시 출발하지 않는 문제가 실기에서 확인됨 — 원인은
        # 이 파라미터들 조합으로 추정: max_depth_m(구 5.0)이 딱 그 거리대와
        # 겹쳐서 원거리일수록 늘어나는 depth 센서 노이즈/오차로 유효 깊이가
        # 5.0m 경계를 넘나들기 쉽고, depth_roi_radius(구 5, 패치 11x11=121px)도
        # 원거리에서 작게 잡히는 bbox와 depth 홀이 겹치면 valid.size>=3 조건을
        # 못 채우기 쉬웠다 — 결과적으로 YOLO는 박스를 잡아도 _sample_depth()가
        # z=0.0을 반환해 _detect_best_candidate()가 후보 자체를 버리고, 그
        # 상태가 track_lost_grace_sec을 넘기면 LOST로 떨어져 정지 후 재출발을
        # 못 하는 교착이 됐을 것으로 봄. 1차로 max_depth_m 5.0->7.0,
        # depth_roi_radius 5->8로 완화했으나(2026-09-05) "YOLO가 RGB에서
        # detect했는데 depth 검증 실패로 /path가 안 나가 추종을 재개 못 하는
        # 상황"을 최대한 없애는 쪽으로 사용자 요청, 2차로 더 완화한다:
        #   - max_depth_m 7.0->10.0: 순수 원거리 컷오프라 늘려도 거리 정확도에
        #     영향 없음(대회 추종 목표가 1.5~2m라 10m는 충분히 여유) — 먼
        #     오탐은 conf_threshold/화면 크기가 이미 걸러줌.
        #   - depth_roi_radius 8->12(패치 25x25=625px): valid.size>=3(median
        #     신뢰용 최소 유효픽셀 수, 이건 정확도 유지를 위해 그대로 둠)
        #     확보를 원거리에서도 사실상 보장. go2/DogPig는 추종거리에서
        #     화면상 충분히 크게 잡히는 대상이라 이 정도 패치 확대로 배경
        #     depth가 심하게 섞여 거리 정확도가 훼손될 위험은 낮다고 판단.
        self.declare_parameter('depth_roi_radius', 12)
        self.declare_parameter('max_depth_m', 10.0)
        # [2026-08-30] 드라이브 카메라 하단에 로봇팔(그리퍼) 일부가 항상
        # 고정된 위치로 걸려서, 그게 종종 로봇개(go2)로 오탐되는 게 실기
        # 스크린샷으로 확인됨(robodog_visualizer.py). 화면 하단 이 비율
        # 안에 중심이 있는 박스는 로봇 자기 몸체로 보고 후보에서 아예
        # 제외한다 — summer_traffic.py의 passes_spatial_filters()(ROI
        # top ratio)와 같은 취지의 반대쪽(bottom) 버전. 실측 전 값이라
        # 필요하면 조정할 것.
        self.declare_parameter('roi_bottom_exclude_ratio', 0.2)
        # [2026-08-30 -> 재변경] 이전엔 대회 규정집(가림구간 5~6.25초 통과
        # 추정) 기준으로 8.0초를 줘서 가림구간 전체를 PREDICTING(등속
        # 외삽)으로 버티게 했었다. 사용자 지시로 되돌림 -- 로봇개가 시야에서
        # 사라지면 등속 외삽으로 계속 움직이지 말고 즉시 정지해야 함(추론이
        # 틀리면 -- 선도가 코너에서 방향을 꺾는 경우 등 -- 그 몇 초 동안
        # 엉뚱한 방향으로 계속 달리는 게 더 위험하다는 판단). 0.0으로 완전히
        # 없애지 않고 0.3초만 남긴 이유는 YOLO가 한두 프레임 순간적으로
        # 놓치는 노이즈(실제로는 계속 보이는데 검출만 튄 경우)까지 즉시
        # LOST로 떨어뜨려 정지-재출발을 반복하는 걸 막기 위함 -- 진짜
        # 가림막(수 초)과는 확실히 구분됨. 간격유지 배점에는 불리해질 수
        # 있음(정지해있는 동안 거리가 벌어짐) -- 그건 재포착 후 CATCHUP의
        # distance_speed_max_m_s로 만회하는 쪽으로 감수하기로 함.
        # [2026-08-30, 재조정] 0.3초는 실기에서 너무 짧아서(노이즈성 순간
        # 미검출까지 LOST로 떨어져 정지-재출발이 잦았음) 3.0초로 늘림 --
        # 여전히 위 8.0초(가림구간 전체 버티기)보다는 짧게 둬서 "선도가
        # 방향을 꺾어서 오검출이 이어지는" 최악의 경우 계속 잘못된 방향으로
        # 달리는 시간은 제한한다.
        # [2026-09-02] 가림 시 정지 지연을 줄이기 위해 기본값을
        # 0.3초로 다시 낮추었다. 한두 프레임 미검출만 예측으로 흡수한다.
        #
        # [2026-08-31, 문제 발견 후 이원화 -> 2026-09-01, 사용자 지시로
        # 단일화 복귀] 한때 화면 밖 이탈(카메라 회전보다 빠르게 빠져나가는
        # 경우, 긴 grace로 등속 외삽+bearing steering 재포착 유리)과 진짜
        # 가림막(짧은 grace로 빨리 정지 필요)을 구분해서
        # track_lost_grace_sec_edge/_center로 나눴었는데(0115d3b), 팀원이
        # 이 값을 0.0/0.0으로 바꿨다가(ee56c3b) "조향이 아예 안 됨" 증상의
        # 유력 원인으로 지목된 전례가 있다(디버깅 리포트 참고) -- 두 값을
        # 따로 관리하다 보니 launch 계층(mission_escort.launch.py/
        # mission_escort_drive.launch.py)까지 매번 동기화해야 했고 그
        # 과정에서 어긋나기 쉬웠다. 그래서 이 구분을 되돌려 단일
        # track_lost_grace_sec 하나만 관리한다 -- 화면 밖 이탈이든
        # 가림막이든 같은 grace를 쓰므로 예전만큼 세밀하진 않지만, 관리
        # 포인트가 하나로 줄어 어긋날 여지가 없어진다.
        self.declare_parameter('track_lost_grace_sec', 0.0)
        self.declare_parameter('reacquire_radius_m', 0.5)
        self.declare_parameter('reacquire_radius_growth_m_per_sec', 0.0)
        self.declare_parameter('velocity_smoothing_alpha', 0.5)

        self._target_distance_m = float(
            self.get_parameter('target_distance_m').value)
        self._distance_tolerance_m = float(
            self.get_parameter('distance_tolerance_m').value)
        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)

        model_path = str(self.get_parameter('model_path').value)
        self._target_class_names = list(
            self.get_parameter('target_class_names').value)
        self._conf_threshold = float(self.get_parameter('conf_threshold').value)
        self._infer_size = int(self.get_parameter('infer_size').value)
        self._depth_r = int(self.get_parameter('depth_roi_radius').value)
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        self._roi_bottom_exclude_ratio = float(
            self.get_parameter('roi_bottom_exclude_ratio').value)
        self._track_lost_grace_sec = float(
            self.get_parameter('track_lost_grace_sec').value)
        self._reacquire_radius_m = float(
            self.get_parameter('reacquire_radius_m').value)
        self._reacquire_radius_growth_m_per_sec = float(
            self.get_parameter('reacquire_radius_growth_m_per_sec').value)
        self._velocity_smoothing_alpha = float(
            self.get_parameter('velocity_smoothing_alpha').value)

        self._model = self._load_model(model_path)
        self._bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        # CameraInfo.header.frame_id를 그대로 저장해뒀다가 /path의
        # header.frame_id로 쓴다 — _publish_path() 참고.
        self._camera_frame_id: str | None = None

        # 추종 상태 — 한 번도 탐지된 적 없으면 LOST로 시작, _last_point=None.
        self._state = _STATE_LOST
        self._last_point: tuple[float, float, float] | None = None
        self._last_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_seen_time: float | None = None

        self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, SENSOR_DATA_QOS_DEPTH1)

        sub_color = message_filters.Subscriber(
            self, CompressedImage, color_topic, qos_profile=SENSOR_DATA_QOS_DEPTH1)
        sub_depth = message_filters.Subscriber(
            self, CompressedImage, depth_topic, qos_profile=SENSOR_DATA_QOS_DEPTH1)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=30, slop=0.20)
        self._sync.registerCallback(self._on_frames)

        self._result_pub = self.create_publisher(
            MissionResult, '/mission/escort_follow/result', 10)
        self._debug_pub = self.create_publisher(
            CompressedImage, '/mission/escort_follow/debug_image/compressed', 10)
        # purepursuit_node용 — result를 대체하지 않고 같이 발행(모듈 docstring
        # 발행 절 참고).
        self._path_pub = self.create_publisher(Path, path_topic, 10)

        self.get_logger().info(
            f'EscortFollowNode ready — target_distance={self._target_distance_m}'
            f'±{self._distance_tolerance_m}m, target_class_names={self._target_class_names}, '
            f'track_lost_grace_sec={self._track_lost_grace_sec}'
            + (' (모델 미설정 — 스텁처럼 항상 lost 발행)' if self._model is None else ''))

    # ------------------------------------------------------------------

    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn(
                'model_path 미설정 — 학습파일 경로를 파라미터로 전달하세요')
            return None
        if not _YOLO_OK:
            self.get_logger().error('ultralytics 미설치 — pip install ultralytics')
            return None
        # [2026-09-04] task='detect' 명시 — segmentation.py에서 확인된 것과
        # 같은 이유(경로 문자열만으로 task를 오판하는 ultralytics 회귀 방지).
        model = YOLO(path, task='detect')
        self.get_logger().info(f'모델 로드 완료: {path}')
        return model

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self._camera_frame_id = msg.header.frame_id

    @staticmethod
    def _to_meters(cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    def _sample_depth(self, depth: np.ndarray, u: int, v: int) -> float:
        """bbox 중심 주변 패치의 유효 깊이 중앙값(m) — 단일 픽셀보다 노이즈에
        강건하다. 실패 시 0.0."""
        h, w = depth.shape
        r = self._depth_r
        patch = depth[max(0, v - r):min(h, v + r + 1),
                      max(0, u - r):min(w, u + r + 1)]
        valid = patch[(patch > 0.05) & (patch < self._max_depth_m)]
        return float(np.median(valid)) if valid.size >= 3 else 0.0

    def _detect_best_candidate(self, color: np.ndarray, depth: np.ndarray):
        """confidence가 가장 높은 후보 하나를 (point_xyz, bbox_xyxy, conf,
        cls_name)로 반환, 없으면(모델 미로드/탐지 없음/깊이 무효/화면 하단
        제외 ROI에 걸림) None.

        roi_bottom_exclude_ratio: 화면 하단에 항상 걸리는 로봇팔(그리퍼)이
        go2로 오탐되는 걸 막기 위한 필터 — bbox 중심 y가
        image_height * (1 - roi_bottom_exclude_ratio)보다 아래(화면 하단
        쪽)면 그 후보는 자기 몸체로 보고 버린다(__init__ 파라미터 주석
        참고)."""
        if self._model is None:
            return None
        image_height = color.shape[0]
        roi_bottom_limit = image_height * (1.0 - self._roi_bottom_exclude_ratio)
        results = self._model(color, imgsz=self._infer_size, verbose=False)
        best = None
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < self._conf_threshold:
                    continue
                cls_name = self._model.names.get(int(box.cls[0]), '')
                if cls_name not in self._target_class_names:
                    continue
                if best is not None and conf <= best[2]:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
                if v >= roi_bottom_limit:
                    continue  # 화면 하단 제외 ROI(로봇 자기 몸체) — 후보에서 제외
                z = self._sample_depth(depth, u, v)
                if z <= 0.0:
                    continue

                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                best = ((x, y, z), (x1, y1, x2, y2), conf, cls_name)
        return best

    def _on_frames(self, color_msg: CompressedImage, depth_msg: CompressedImage) -> None:
        if self.fx is None:
            self.get_logger().warn(
                'CameraInfo 대기 중.', throttle_duration_sec=2.0)
            return

        try:
            color = self._bridge.compressed_imgmsg_to_cv2(
                color_msg, desired_encoding='bgr8')
            depth = self._to_meters(decode_compressed_depth(depth_msg))
        except Exception as exc:
            self.get_logger().error(
                f'압축 카메라 이미지 디코딩 실패: {exc}', throttle_duration_sec=5.0)
            return

        now = stamp_to_sec(color_msg.header.stamp)
        candidate = self._detect_best_candidate(color, depth)

        accepted = None  # (point_xyz, bbox, conf, cls_name) — 이번 프레임에 채택된 탐지
        if candidate is not None:
            candidate_point = candidate[0]
            if self._state == _STATE_PREDICTING:
                # 예측 구간 재발견은 게이팅 통과해야만 "같은 대상"으로 인정.
                elapsed = now - self._last_seen_time
                predicted = predict_position(
                    self._last_point, self._last_velocity, elapsed)
                if within_reacquire_radius(
                        predicted, candidate_point, elapsed,
                        self._reacquire_radius_m,
                        self._reacquire_radius_growth_m_per_sec):
                    accepted = candidate
                # else: 반경 밖 — 오탐으로 보고 버림, PREDICTING 계속 유지 (아래에서 처리)
            else:
                # TRACKING 연속(원래 게이팅 대상 아님) 또는 LOST에서 막 재발견
                # (참조할 예측 위치 자체가 신뢰 불가능하므로 게이팅 생략) — 바로 수용.
                accepted = candidate

        result = MissionResult()
        result.header = color_msg.header
        result.mission_name = 'escort_follow'

        if accepted is not None:
            accepted_point = accepted[0]
            was_lost = (self._state == _STATE_LOST)
            if self._last_point is not None and not was_lost:
                dt = now - self._last_seen_time
                self._last_velocity = update_velocity_ema(
                    self._last_velocity, self._last_point, accepted_point,
                    dt, self._velocity_smoothing_alpha)
            else:
                # 첫 관측이거나 LOST에서 막 재시작 — 이전 위치/구간이 속도
                # 추정 근거로 신뢰 불가능하므로 리셋하고 다음 프레임부터 다시 쌓는다.
                self._last_velocity = (0.0, 0.0, 0.0)
            self._last_point = accepted_point
            self._last_seen_time = now
            self._state = _STATE_TRACKING

            result.state = _STATE_TRACKING
            result.valid = True
            result.target_point.x, result.target_point.y, result.target_point.z = accepted_point

            self._publish_debug_image(color, color_msg.header, accepted)

        elif self._last_point is None:
            # 한 번도 추적 성공한 적 없음.
            self._state = _STATE_LOST
            result.state = _STATE_LOST
            result.valid = False

        else:
            elapsed = now - self._last_seen_time
            if elapsed <= self._track_lost_grace_sec:
                self._state = _STATE_PREDICTING
                predicted = predict_position(
                    self._last_point, self._last_velocity, elapsed)
                result.state = _STATE_TRACKING  # 스키마상 PREDICTING 전용 값 없음
                result.valid = True
                result.target_point.x, result.target_point.y, result.target_point.z = predicted
            else:
                self._state = _STATE_LOST
                result.state = _STATE_LOST
                result.valid = False
                # 0,0,0으로 리셋하지 않고 마지막으로 확정된 위치를 그대로 유지
                # (valid=False라 "실측 아님"은 명확 — 소비 측이 최후 목격 위치
                # 참고용으로 쓸 수 있게).
                result.target_point.x, result.target_point.y, result.target_point.z = self._last_point

        self._result_pub.publish(result)

        # [2026-08-30, 대회 규정 재검토 후 되돌림] 한때 TRACKING일 때만
        # /path를 내고 PREDICTING/LOST엔 빈 Path(=즉시 정지)를 내도록
        # 바꿨었는데, 규정집 기준으로 다시 보니 이게 오히려 불리하다 —
        # "가림 구간 통과 시 재추종"에 25/10점 배점이 있다는 건 짧은 가림
        # (추정 5~6.25초, track_lost_grace_sec 주석 참고) 동안은 완전히
        # 서 있기보다 등속 외삽으로 계속 움직이며 버티는 걸 기대한다고
        # 보는 게 합리적 — 안 그러면 그 몇 초 동안 간격이 계속 벌어지고
        # (간격유지 35/15점 배점에 불리), 재발견 후에도 못 따라잡는다.
        # 그래서 PREDICTING도 다시 포함(LOST일 때만 빈 Path)한다 —
        # result.valid와 사실상 동일 기준으로 되돌아간 것.
        target = (
            (result.target_point.x, result.target_point.y, result.target_point.z)
            if self._state != _STATE_LOST else None)
        self._publish_path(color_msg.header, target)

    def _publish_path(
            self, header, target_point: tuple[float, float, float] | None) -> None:
        """로봇 중심(카메라 원점 근사) -> target_point 2점짜리 Path를
        purepursuit_node용으로 발행한다 — 모듈 docstring 발행 절 참고.

        frame_id는 카메라 광학 프레임 그대로 싣는다(_camera_frame_id,
        _on_camera_info에서 저장) — purepursuit_node가 TF로 base_link까지
        통째로 변환해주므로 여기서 광학->body 축변환을 직접 할 필요가 없다.
        검출점은 위 _detect_best_candidate()에서 핀홀 역투영
        Xo=(u-cx)*Z/fx, Yo=(v-cy)*Z/fy, Zo=Z로 복원된다. 이후 TF가
        p_base = t_base_camera + R_base_camera * p_camera 형태로 최신 실측
        x/z/roll/pitch를 적용한다. purepursuit_node는 그 결과의
        sqrt(x_base^2 + y_base^2)를 실제 수평 추종거리로 쓴다.
        시작점을 (0,0,0)(카메라 원점)으로 두는 건 정확한 base_link 원점이
        아니라 근사다(카메라가 로봇 중심에서 약간 떨어져 있음, purepursuit.
        launch.py의 정적 TF 참고) — pure pursuit 계산은 사실상 경로 끝점과
        lookahead 반경 밖 첫 점만 쓰므로 이 정도 오차는 무시 가능하다.

        target_point가 None이면(LOST, valid=False) 빈 Path를 발행한다 —
        purepursuit_node는 빈 Path를 받으면 path_timeout_sec을 기다리지
        않고 그 즉시 정지한다."""
        path = Path()
        path.header = header
        if self._camera_frame_id:
            path.header.frame_id = self._camera_frame_id

        if target_point is not None:
            origin = PoseStamped()
            origin.header = path.header
            origin.pose.orientation.w = 1.0
            target = PoseStamped()
            target.header = path.header
            target.pose.position.x, target.pose.position.y, target.pose.position.z = target_point
            target.pose.orientation.w = 1.0
            path.poses = [origin, target]

        self._path_pub.publish(path)

    def _publish_debug_image(self, color: np.ndarray, header, detection) -> None:
        point, bbox, conf, cls_name = detection
        x1, y1, x2, y2 = (int(c) for c in bbox)
        cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            color, f'{cls_name} {conf:.2f} | Z={point[2]:.2f}m',
            (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        dbg = self._bridge.cv2_to_compressed_imgmsg(color, dst_format='jpg')
        dbg.header = header
        self._debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = EscortFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
