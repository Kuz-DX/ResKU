"""
좌우 기울기(roll) 추정 + 평지/경사 경로 릴레이 노드.

depth ROI로 좌우 기울기를 추정해 /terrain/side_slope_angle_deg로 게시하던
기존 역할에 더해, gradient_map/flat_drive가 각각 내는 경사/평지 경로 중
하나를 임계각 기준으로 골라 /path로 릴레이하는 역할을 겸한다 (Phase F
평지/경사 전환 판단의 1차 버전 — TF 기반 odom 변환은 아직 없음, 릴레이 노드
쪽 후속 작업).

slope 모드 전환 조건 = (좌우 roll 임계각 초과) OR (전방 지형 경사 임계각 초과).
roll만으로는 로봇 정면으로 곧게 솟은 램프/쿠션 같은 장애물(=roll이 아니라
pitch가 바뀌는 지형)을 감지 못 해 flat_drive(평지 전용, 경사 정보 전혀 안 씀)
경로가 계속 선택되는 문제가 있었다 — gradient_map이 이미 계산해 발행하는
/terrain/slope_deg(전방 경사각 필드)를 추가로 구독해서, 그 안에 임계각을
넘는 영역이 일정 픽셀 수 이상 있으면(단일 노이즈 픽셀 오탐 방지) 그것만으로도
slope 모드로 전환한다. 두 조건 다 같은 slope_threshold_deg를 공유한다
(gradient_map의 max_slope_deg와는 별개 — 그쪽은 "경로 자체를 못 지나가게
막는" 임계값이고, 이건 "flat_drive 경로를 못 믿고 gradient_map 경로로
갈아타야 하는" 더 이른 임계값이라 의도적으로 분리해뒀다. 같은 값을 쓰고
싶으면 파라미터로 맞추면 됨).

force_mode 파라미터('auto'|'flat'|'slope', 기본 'auto')로 위 자동 판단을
완전히 우회할 수 있다 — 자갈/모래/눈처럼 depth가 본질적으로 울퉁불퉁한
지형에서는 roll 추정이 노이즈로 오탐(실제론 평평한데 roll_exceeds/
ahead_exceeds가 계속 튐)할 수 있는데, 해당 지형이 실제로는 평평한 구간에
배치된 게 확인되면 'flat'로 강제 고정해서 그 오탐을 원천 차단할 수 있다.

depth ROI 유효 샘플이 부족하면 _decide_and_publish()가 호출되지 않으므로,
depth가 자주 결측되는 상황에서는 /path가 멈출 수 있다.
'slope'는 반대로 항상 gradient_map 경로를 쓰도록 강제. roll_exceeds/
ahead_exceeds 자체는 force_mode와 무관하게 계속 계산·게시된다(디버그/
모니터링용 side_slope_angle_deg가 계속 유효해야 하므로) — force_mode는
그 값들을 최종 경로 선택에 반영할지만 결정한다.

구독: /drive/camera/aligned_depth_to_color/camera_info,
      /drive/camera/aligned_depth_to_color/image_raw/compressedDepth
      /perception/drivable_mask (sensor_msgs/Image mono8 — segmentation.py, 트랙 마스크)
      /terrain/planned_path   (nav_msgs/Path — gradient_map, 경사 경로)
      /flatdrive/planned_path (nav_msgs/Path — flat_drive, 평지 경로)
      /terrain/slope_deg      (sensor_msgs/Image 32FC1, degrees — gradient_map,
                               전방 경사각 필드. compute_gradient_field()가 만드는
                               것과 동일한 그리드/단위)
발행: /terrain/side_slope_angle_deg (std_msgs/Float32, 기존)
      /path         (nav_msgs/Path — 임계각 기준으로 고른 최종 경로)
      /drive/status (std_msgs/String, 'slope' 또는 'flat')
      /terrain/slope_side_signal (std_msgs/Int8, 구동부 연동용 — 아래 참고)

/terrain/slope_side_signal: 구동부 쪽과 맞춘 부호 규약 — +1=right_slope(오른쪽
높음), -1=left_slope(왼쪽 높음), 0=없음(평지 또는 판단 불가). roll_deg 부호가
그대로 이 규약과 일치하므로("+면 오른쪽이 더 높다") 그대로 매핑한다.
|roll_deg| >= slope_side_threshold_deg(파라미터, 기본 2.0deg)일 때만 부호를
싣고, 미만이면 0으로 발행한다. on_depth()가 성공해 roll_deg가 계산될 때만
발행하므로(side_slope_angle_deg와 같은 트리거) depth/마스크가 부실한 프레임엔
이것도 같이 멈춘다.
토픽명 자체(기본값 '/terrain/slope_side_signal')는 slope_side_topic 파라미터로
바꿀 수 있지만, 이 이름/메시지 타입(std_msgs/Int8)은 인지팀 PLACEHOLDER다 —
구동부와는 부호 규약(+오른쪽/-왼쪽)만 확정됐고 토픽명/타입은 아직 확정 아님.
나중에 바뀌면 launch 파라미터로만 갈아끼우면 되게 토픽명을 하드코딩하지
않았다.

좌우 ROI 분리는 이미지 폭 절반이 아니라 ROI 밴드 안 마스크(트랙) 픽셀의
열(column) 범위 중심(center_col)을 기준으로 한다 — 트랙이 화면 중앙에서
벗어나 있어도 정확하고, 트랙 아닌 배경(에지 너머 등)이 좌/우 판단에 안
섞이게 한다.

hL/hR 자체는 center_col 기준 절반 구간 전체가 아니라, track_width_m(로봇
자체 좌우 기준 거리)만큼 좌우로 떨어진 두 지점 근방의 좁은 스트립만 샘플링
한다(edge_sample_strip_bounds() 참고). 예전에는 절반 구간 전체의 median을
썼는데, 지면이 선형으로 기울어 있으면 그 median은 구간 "중간점"(트랙 전체
폭의 1/4, 3/4 지점)의 높이와 같아져서 hL/hR 사이 실제 물리적 거리가
track_width_m이 아니라 "보이는 트랙 폭의 절반"이 돼버렸다 — atan2 분모
(track_width_m)와 분자(dh)가 나타내는 baseline이 안 맞아 roll이 체계적으로
과소추정되는 버그가 있었다(약 10~15%, 세그멘테이션이 잡은 트랙 폭이 좁을수록
더 커짐). z_rep(대표 깊이)으로 핀홀 역산해 목표 지점을 직접 겨냥하고, clamp가
걸리면(트랙이 로봇보다 좁은 등) 분모도 실제 achieved baseline으로 재환산해서
분자·분모가 항상 일치하도록 고쳤다.

dynamic_occlusion_guard는 기본 False이고 mission_summer.launch.py에서만
True다. 매니퓰레이터 그리퍼 등 근접 장애물이 카메라 시야 하단부를 가려서
median_height()의 80개 유효 샘플 문턱을 못 넘기고 on_depth()가 조용히
멈추는 문제 대응용이다. find_occlusion_cutoff() 참고 — ROI 밴드를 행 단위로
훑어 track 마스크/depth 유효율이 동시에 무너지는 지점을 프레임마다 감지해서
그 지점부터 화면 하단까지만 잘라낸다. 장애물이 없는 프레임은 원래 ROI를 쓴다.

lateral_occlusion_guard는 기본 False다. dynamic_occlusion_guard가 화면
"하단"부터의 가림만 다루는 것과 달리, 트랙 "옆"에 붙은 구조물이
세그멘테이션을 좌우로 끊어서 center_col이 틀어지는 문제 대응용이다.
find_lateral_gap()/reconstruct_track_span() 참고 — 끊긴 지점의 depth가 신뢰
세그먼트보다 확연히 가까울 때만 실측 트랙 폭으로 반대쪽 경계를 복원한다.
depth 불연속이 없으면 진짜 트랙 끝일 수 있으므로 원래 값을 유지한다.
flat_drive.py에도 비슷한 처리가 있지만, 거긴 depth가 없어 진짜 트랙 끝과
구조물을 구분할 수 없다.

always_relay_flat은 모든 구간에서 flat_drive 경로를 쓰도록 기본 True이며
force_mode='flat'보다 강한 오버라이드다.
force_mode는 _decide_and_publish()가 호출될 때만 적용되므로, on_depth()가
마스크/depth 부실로 조기 return하면 /path가 나오지 않는다.
always_relay_flat=True면 on_flat_path()가 /flatdrive/planned_path를 받는 즉시
on_depth()/roll 계산과 무관하게 /path·/drive/status를 내보낸다. roll 계산은
모니터링을 위해 계속 수행한다.
"""
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from std_msgs.msg import Float32, Int8, String
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.compressed_image import decode_compressed_depth
from dolbotz.utils.regulations import TRACK_WIDTH_M

# 실측 전 임시값 — 이 각도[deg] 이상이면 경사 경로(/terrain/planned_path)를,
# 미만이면 평지 경로(/flatdrive/planned_path)를 최종 /path로 릴레이한다.
# 하드웨어 트랙션/기울기 테스트 후 조정할 것. ahead_exceeds(전방 경사)가 계속
# 공유해서 쓴다 — roll 쪽은 아래 ROLL_THRESHOLD_DEG_*_PLACEHOLDER로 분리된다.
SLOPE_THRESHOLD_DEG_PLACEHOLDER = 10.0

# roll 임계값은 방향별로 분리한다. roll_exceeds_threshold() 참고. 기본값은 둘 다
# SLOPE_THRESHOLD_DEG_PLACEHOLDER와 같아서, 튜닝 전엔 기존과 동일하게 동작.
ROLL_THRESHOLD_DEG_POSITIVE_PLACEHOLDER = SLOPE_THRESHOLD_DEG_PLACEHOLDER
ROLL_THRESHOLD_DEG_NEGATIVE_PLACEHOLDER = SLOPE_THRESHOLD_DEG_PLACEHOLDER

# /terrain/slope_deg 안에서 이 픽셀 수 이상이 임계각을 넘어야 "전방에 경사가
# 있다"고 인정한다 — depth/gradient 계산 노이즈로 한두 픽셀만 튀는 걸 걸러내기
# 위함(elevation_map/slope_decision 다른 곳의 "최소 샘플 수" 검증과 같은 목적).
FORWARD_SLOPE_MIN_PIXELS = 20

# 모든 계절/구간의 최종 /path는 기본적으로 flat_drive 경로를 사용한다.
# 경사 계산과 /terrain/planned_path 발행은 모니터링용으로 계속 유지하며,
# 실험이 필요할 때만 always_relay_flat 파라미터를 False로 명시해 자동 선택을
# 다시 활성화한다.
ALWAYS_RELAY_FLAT_DEFAULT = True

# dynamic_occlusion_guard(기본 꺼짐, mission_summer에서만 켬)용 기본값 —
# find_occlusion_cutoff() 참고. 매니퓰레이터 그리퍼 등 근접 장애물이 ROI
# 하단부를 가리는 미션(여름)에서만 켜서 쓰도록 설계됨 — 겨울 등 다른 미션은
# 이 값들 자체가 아예 안 쓰인다.
OCCLUSION_FILL_RATIO_THRESHOLD_PLACEHOLDER = 0.3
OCCLUSION_MIN_RUN_ROWS_PLACEHOLDER = 3

# lateral_occlusion_guard(기본 꺼짐)용 기본값 — find_lateral_gap()/
# reconstruct_track_span() 참고. 트랙 옆에 붙은 구조물이 세그멘테이션을
# 옆으로 끊어서 center_col이 틀어지는 문제 대응용(dynamic_occlusion_guard는
# 화면 "하단"부터의 가림만 다루므로 이건 별개).
LATERAL_GAP_MIN_PX_PLACEHOLDER = 12
# 틈 구간 depth가 신뢰 세그먼트보다 이만큼[m] 이상 가까워야 "장애물"로
# 인정 — 그냥 평범한 바닥이면 이 정도 불연속이 안 생긴다는 전제.
LATERAL_DEPTH_JUMP_M_PLACEHOLDER = 0.3
# 신뢰 세그먼트의 대표 depth(중앙값)를 믿으려면 이만큼은 유효 샘플이 있어야
# 함 — median_height()의 80개 문턱과 별개(더 느슨함, 여긴 "장애물이냐 아니냐"
# 판단용 depth 하나만 필요해서).
LATERAL_TRUSTED_MIN_SAMPLES = 20


# ---------------------------------------------------------------------------
# 순수 계산 (ROS 의존성 없음)
# ---------------------------------------------------------------------------

def forward_slope_exceeds(
    slope_deg_field: np.ndarray,
    threshold_deg: float,
    min_pixels: int = FORWARD_SLOPE_MIN_PIXELS,
) -> bool:
    """gradient_map이 발행한 전방 경사각 필드(/terrain/slope_deg, degrees) 안에
    threshold_deg를 넘는 유효(non-NaN) 픽셀이 min_pixels개 이상 있으면 True.

    gradient_map의 Dijkstra 경로 자체는 max_slope_deg를 넘는 셀을 이미 피해서
    돌아가므로, "경로 위에 고경사 구간이 있는지"가 아니라 "필드 안 어딘가에
    고경사 지형이 존재하는지"를 직접 보는 게 맞는 신호다 — 로봇이 아직 그
    지형에 올라타지 않아 roll/pitch로는 안 잡히는 정면 램프/쿠션류 장애물을
    미리 감지하기 위함.
    """
    valid = np.isfinite(slope_deg_field)
    steep = valid & (slope_deg_field > threshold_deg)
    return bool(np.count_nonzero(steep) >= min_pixels)


def roll_exceeds_threshold(
    roll_deg: float, threshold_deg_positive: float, threshold_deg_negative: float,
) -> bool:
    """roll_deg가 양의 임계각(threshold_deg_positive) 이상이거나, 음의
    임계각(-threshold_deg_negative) 이하면 True.

    좌우 트랙션/안전 여유가 다를 수 있어 방향별 임계값을 사용한다.
    threshold_deg_negative는 항상 양수로 받는다(부호는 이 함수 안에서
    붙임) — 호출부에서 음수로 잘못 넘기는 실수를 줄이기 위함. 두 임계값이
    같으면 예전 abs() 버전과 동일하게 동작한다."""
    return roll_deg >= threshold_deg_positive or roll_deg <= -threshold_deg_negative


FORCE_MODE_VALUES = ('auto', 'flat', 'slope')


def resolve_force_mode(value: str) -> str:
    """force_mode 파라미터 값을 검증한다. 'auto'/'flat'/'slope' 중 하나가
    아니면 'auto'로 폴백한다(호출부가 경고 로그를 남기는 건 별개 — 이 함수는
    순수 검증만). path_relay_node.cpp의 tf_failure_behavior 검증과 같은
    패턴(모르는 값이면 조용히 무시하지 않고 안전한 기본값으로 폴백)."""
    return value if value in FORCE_MODE_VALUES else 'auto'


def pixel_span_width_m(col_start: int, col_end_inclusive: int, z_m: float, fx: float) -> float:
    """픽셀 열 범위(col_start~col_end_inclusive, inclusive)가 깊이 z_m
    지점(카메라 광축 방향 거리, m)에서 나타내는 실측 폭(m)을 핀홀 근사로
    추정한다 — (열 개수) * z_m / fx."""
    pixel_span = col_end_inclusive - col_start + 1
    return pixel_span * z_m / fx


def track_width_plausible(
    measured_width_m: float, expected_width_m: float, tolerance_factor: float,
) -> bool:
    """측정된 폭이 expected_width_m * tolerance_factor를 넘으면(세그멘테이션이
    트랙 아닌 영역까지 넓게 잡은 오탐으로 보고) False. 더 좁은 건(가려짐 등)
    정상일 수 있어서 하한은 검사하지 않는다."""
    return measured_width_m <= expected_width_m * tolerance_factor


def edge_sample_strip_bounds(
    track_col_first: int,
    track_col_last: int,
    center_col: int,
    half_span_px: float,
    strip_half_width_px: int,
) -> tuple[int, int, int, int, float] | None:
    """hL/hR 샘플링 스트립의 열 범위를 계산한다 — median_height() 버그 수정
    (roll 과소추정) 핵심 로직.

    예전에는 트랙 절반 구간 전체(track_cols[0]~center_col,
    center_col~track_cols[-1])의 median을 hL/hR로 썼는데, 지면이 선형으로
    기울어 있으면 구간의 median은 그 구간 "중간점"의 높이와 같아진다 —
    즉 hL/hR은 실제로는 트랙 전체 폭의 1/4, 3/4 지점을 대표하게 되고, 두
    대표점 사이의 실제 물리적 거리는 track_width_m(atan2 분모, 로봇 자체
    좌우 기준 거리)이 아니라 "보이는 트랙 폭의 절반"이었다 — 분모가 분자의
    실제 baseline보다 크므로 roll이 체계적으로 과소추정됨.

    이 함수는 절반 전체 대신, center_col에서 좌우로 정확히
    half_span_px(=track_width_m을 핀홀 근사로 픽셀 환산한 half span)만큼
    떨어진 두 지점 근방의 좁은 스트립(폭 2*strip_half_width_px+1)만
    겨냥한다 — 그러면 hL/hR 사이 실제 거리가 track_width_m과 일치해서
    atan2(dh, track_width_m)이 원래 의도대로 맞아떨어진다.

    [track_col_first, track_col_last] 밖으로 스트립 중심이 나가면 안쪽으로
    clamp한다(트랙이 로봇보다 좁은 등 드문 경우). clamp 후 achieved
    baseline(두 스트립 중심 사이 실제 픽셀 거리)도 함께 돌려준다 — 호출부가
    clamp로 줄어든 실제 baseline을 atan2 분모로 다시 환산해 쓸 수 있게 하기
    위함(그래야 clamp가 걸려도 분자·분모가 항상 일치).

    두 스트립이 겹치거나(트랙이 너무 좁음) 어느 한쪽이 비면 계산이 불가능
    하다는 뜻으로 None을 돌려준다 — 호출부는 이 경우 예전 절반-band
    방식으로 폴백해야 한다.

    Returns (left_x0, left_x1_exclusive, right_x0, right_x1_exclusive,
    achieved_baseline_px) 또는 None."""
    left_center = min(max(center_col - half_span_px, track_col_first), track_col_last)
    right_center = min(max(center_col + half_span_px, track_col_first), track_col_last)

    left_x0 = int(round(left_center - strip_half_width_px))
    left_x1 = int(round(left_center + strip_half_width_px)) + 1
    right_x0 = int(round(right_center - strip_half_width_px))
    right_x1 = int(round(right_center + strip_half_width_px)) + 1

    left_x0 = max(left_x0, track_col_first)
    right_x1 = min(right_x1, track_col_last + 1)

    if left_x1 <= left_x0 or right_x1 <= right_x0 or left_x1 > right_x0:
        return None

    return left_x0, left_x1, right_x0, right_x1, right_center - left_center


def find_occlusion_cutoff(
    row_fill_ratios: np.ndarray,
    fill_ratio_threshold: float,
    min_run_rows: int,
) -> int | None:
    """dynamic_occlusion_guard 핵심 로직 (ROS 의존성 없음, 위→아래 순서의
    행별 (트랙 마스크 & 유효 depth) 비율 배열을 받는다).

    그리퍼 같은 근접 장애물은 카메라 기하학상 항상 화면 "하단"부터 위로
    잠식한다(가까운 물체일수록 화면 아래쪽에 나타남). 이 함수는 어느 행부터
    화면 끝까지 track_fill/depth_ok가 동시에 무너지는 패턴, 즉
    fill_ratio_threshold 밑으로 min_run_rows행 이상 연속으로 떨어지는 첫
    구간을 찾아 그 시작 인덱스를 돌려준다 — 호출부는 그 지점부터 화면
    하단까지를 ROI에서 잘라낸다.

    한 번 붕괴가 시작된 뒤 다시 fill이 좋아지는 행(장애물 밑으로 바닥이
    살짝 보이는 경우 등)이 있어도 무시하고 최초 붕괴 시작 지점에서 자른다
    — 애매한 소수 샘플을 억지로 살리는 것보다 안전한 쪽을 택함.

    붕괴 구간을 못 찾으면 None(가릴 게 없다는 뜻, ROI 그대로 사용)."""
    run = 0
    for i, ratio in enumerate(row_fill_ratios):
        if ratio < fill_ratio_threshold:
            run += 1
            if run >= min_run_rows:
                return i - run + 1
        else:
            run = 0
    return None


def find_lateral_gap(col_present: np.ndarray, min_gap_px: int) -> tuple[int, int] | None:
    """lateral_occlusion_guard 1단계 — col_present(ROI 밴드 안 각 열에 트랙
    마스크가 하나라도 있는지, (W,) bool)에서 트랙 전체 열 범위([first, last])
    안쪽에 min_gap_px 이상 연속으로 비어있는 구간을 찾는다.

    구조물이 트랙 옆(왼쪽 또는 오른쪽)에 붙어서 그 구간의 세그멘테이션을
    끊어버리면, track_cols가 그 틈까지 통째로 아우르면서 center_col이
    실제 트랙 중심이 아니라 틈 쪽으로 쏠리게 된다 — 이 함수는 그 틈의
    위치만 찾는다(진짜 장애물인지는 이 함수 밖에서 depth로 확인해야 함,
    reconstruct_track_span 참고). 못 찾으면 None."""
    nz = np.flatnonzero(col_present)
    if nz.size == 0:
        return None
    first, last = int(nz[0]), int(nz[-1])
    run_start = None
    for c in range(first, last + 1):
        if not col_present[c]:
            if run_start is None:
                run_start = c
        else:
            if run_start is not None and c - run_start >= min_gap_px:
                return run_start, c
            run_start = None
    if run_start is not None and (last + 1) - run_start >= min_gap_px:
        return run_start, last + 1
    return None


def reconstruct_track_span(
    track_cols_first: int,
    track_cols_last: int,
    gap: tuple[int, int],
    gap_depth_m: float | None,
    trusted_depth_m: float,
    depth_jump_m: float,
    expected_width_m: float,
    fx: float,
    img_width: int,
) -> tuple[int, int] | None:
    """lateral_occlusion_guard 2단계 — find_lateral_gap()이 찾은 틈이 실제
    구조물(근접 장애물)에 의한 것인지 depth로 확인하고, 맞으면 신뢰할 수
    있는 쪽(더 넓은 세그먼트) 경계 + 실측 트랙 폭(expected_width_m)으로
    반대쪽 경계를 역산해서 새 (col_first, col_last)를 돌려준다
    (pixel_span_width_m의 역함수).

    gap_depth_m(틈 구간 depth 대표값, 유효 depth 없으면 None)이
    trusted_depth_m(신뢰 세그먼트 depth 대표값)보다 depth_jump_m 이상
    가깝지 않으면 "그냥 평범한 바닥"(=여기가 진짜 트랙 끝일 수 있음)으로
    보고 복원하지 않는다(None 반환) — 잘못 복원해서 트랙 아닌 땅의 depth를
    트랙인 것처럼 읽는 것보다, 원래 track_cols를 그대로 쓰는 게 안전하다.

    양쪽 세그먼트가 거의 같은 크기면(어느 쪽이 "가려진 쪽"인지 판단 근거가
    약함) 이 것도 복원하지 않고 None."""
    gap_start, gap_end = gap
    left_len = gap_start - track_cols_first
    right_len = track_cols_last - gap_end + 1
    if left_len <= 0 or right_len <= 0:
        return None
    if gap_depth_m is None or not (gap_depth_m < trusted_depth_m - depth_jump_m):
        return None  # depth 불연속 없음 — 진짜 트랙 끝일 수 있으니 손대지 않음

    expected_span_px = expected_width_m * fx / trusted_depth_m
    if left_len >= right_len:
        # 왼쪽(트랙 시작 ~ 틈 시작)이 더 신뢰할 수 있는 쪽 — 오른쪽 경계를 역산
        new_last = int(round(track_cols_first + expected_span_px))
        return track_cols_first, min(new_last, img_width - 1)
    else:
        new_first = int(round(track_cols_last - expected_span_px))
        return max(new_first, 0), track_cols_last


class _DepthRoiWindow:
    """Owns one OpenCV window showing the depth ROI and the computed slope."""

    def __init__(self, window_name: str):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def show(self, depth_m: np.ndarray, left_box, right_box, slope_deg=None, max_depth_m=3.0):
        """left_box / right_box: (x0, y0, x1, y1) rectangles in image coordinates."""
        vis = np.clip(depth_m, 0, max_depth_m) / max_depth_m
        vis = (vis * 255).astype(np.uint8)
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

        for (x0, y0, x1, y1), color in ((left_box, (0, 255, 0)), (right_box, (0, 0, 255))):
            cv2.rectangle(vis, (x0, y0), (x1, y1), color, 2)

        if slope_deg is not None:
            cv2.putText(vis, f"slope: {slope_deg:+.1f} deg", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)

    def close(self):
        cv2.destroyWindow(self.window_name)


class SideSlopeTriggerNode(Node):
    def __init__(self):
        super().__init__('side_slope_trigger_node')

        # ROI 설정 (바닥 영역)
        self.declare_parameter('roi_y_start_ratio', 0.60)
        self.declare_parameter('roi_y_end_ratio',   0.90)
        self.declare_parameter('sample_step', 8)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter(
            'depth_info_topic', '/drive/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter(
            'depth_image_topic',
            '/drive/camera/aligned_depth_to_color/image_raw/compressedDepth')
        self.declare_parameter('mask_topic', '/perception/drivable_mask')

        # roll 계산(atan2(dh, baseline_m))의 목표 baseline — 로봇의 좌우 기준
        # 거리(차량 폭). 실측값 516mm. hL/hR 샘플링 지점도 이만큼 떨어진
        # 두 지점을 직접 겨냥한다(edge_sample_strip_bounds 참고, roll
        # 과소추정 버그 수정).
        self.declare_parameter('track_width_m', 0.516)

        # edge_sample_strip_bounds()가 center_col ± (track_width_m/2를 픽셀
        # 환산한 값) 지점 근방에 놓는 hL/hR 샘플링 스트립의 반폭(px). 너무
        # 좁으면(<80 유효 샘플) median_height()가 그 프레임을 스킵하고, 너무
        # 넓으면 median-of-half-band 버그가 되살아나 baseline이 흐려진다 —
        # 기본값 30px은 일반적인 해상도/ROI에서 유효 샘플 80개 문턱을
        # 여유있게 넘기면서도 트랙 폭(보통 track_width_m보다 넓음) 안에
        # 충분히 들어가는 절충값.
        self.declare_parameter('edge_strip_half_width_px', 30)

        # 평지/경사 경로 릴레이 임계각 — 전방 slope_deg 조건(ahead_exceeds)이
        # 이 값을 쓴다(_decide_and_publish 참고). TODO: 실측 후 조정
        # (SLOPE_THRESHOLD_DEG_PLACEHOLDER 참고)
        self.declare_parameter('slope_threshold_deg', SLOPE_THRESHOLD_DEG_PLACEHOLDER)

        # roll 쪽은 slope_threshold_deg에서 분리해 좌/우 방향별로
        # 다른 임계각을 쓸 수 있게(roll_exceeds_threshold() 참고). 기본값은
        # 둘 다 SLOPE_THRESHOLD_DEG_PLACEHOLDER와 같아서 튜닝 전엔 기존
        # abs() 버전과 동일하게 동작.
        self.declare_parameter(
            'roll_threshold_deg_positive', ROLL_THRESHOLD_DEG_POSITIVE_PLACEHOLDER)
        self.declare_parameter(
            'roll_threshold_deg_negative', ROLL_THRESHOLD_DEG_NEGATIVE_PLACEHOLDER)

        # /terrain/slope_side_signal (구동부 연동용, 모듈 docstring 참고) —
        # 토픽명은 인지팀 PLACEHOLDER라 파라미터로 뺌. 부호 판단 임계값은
        # roll_threshold_deg_*(경로 선택용, 기본 10deg)와 별개 — 구동부는
        # 훨씬 낮은 각도부터 좌우 정보를 받고 싶어해서 기본 2.0deg로 분리.
        self.declare_parameter('slope_side_topic', '/terrain/slope_side_signal')
        self.declare_parameter('slope_side_threshold_deg', 2.0)

        # OpenCV 디버그 창(ROI+슬로프 표시) — X 디스플레이 없는 헤드리스 환경(SSH,
        # 무헤드 로봇 본체 등)에서는 false로 꺼야 노드가 죽지 않고 뜬다. 꺼도
        # /path, /terrain/side_slope_angle_deg, /drive/status 등 토픽 발행은 그대로다.
        self.declare_parameter('enable_visualizer', False)

        # 'auto'(roll/전방 경사 자동 판단) | 'flat'(항상 평지 경로 강제) |
        # 'slope'(항상 경사 경로 강제) — 모듈 docstring의 force_mode 절 참고.
        # depth가 부실해 on_depth()가 조기 return하면 _decide_and_publish가
        # 호출되지 않으므로, auto 모드에서도 /path가 멈출 수 있다.
        self.declare_parameter('force_mode', 'auto')

        # force_mode='flat'보다 더 강한 오버라이드. force_mode는
        # _decide_and_publish()가 "불렸을 때만" flat을 강제하는데, on_depth()가
        # 마스크/depth 부실로 조기 return하면 그 자체가
        # 안 불려서 /path가 여전히 안 나간다. always_relay_flat=True면
        # on_flat_path()에서 /flatdrive/planned_path가 들어오는 즉시
        # on_depth()/roll 계산과 완전히 무관하게 /path·/drive/status를 바로
        # 내보낸다 — depth/마스크가 아예 죽어도 /path는 flat_drive가 살아있는
        # 한 계속 나간다. roll 계산 자체(및 side_slope_angle_deg 발행)는 계속
        # 돌아간다(모니터링용) — 이 값은 최종 경로 선택에만 영향.
        self.declare_parameter('always_relay_flat', ALWAYS_RELAY_FLAT_DEFAULT)

        # /path 발행 속도는 20Hz로 고정한다. 계산
        # 자체(카메라/segmentation 갱신)는 이보다 느릴 수 있어서, 그 사이엔
        # 같은 경로가 여러 번 재발행된다("새 정보가 20번"이 아니라 "최신
        # 값을 20Hz로 재방송"). _publish_latest_path()/_latest_path_msg 참고.
        self.declare_parameter('path_publish_hz', 20.0)

        # 규정집 트랙 폭(914.4mm) 기준 track_width_plausible() 검사 — ROI
        # 트랙 마스크 열 범위가 이보다 tolerance_factor 배 넘게 넓으면
        # 세그멘테이션 오탐으로 보고 그 프레임 roll 계산을 스킵한다. 0
        # 이하면 검사 비활성화.
        self.declare_parameter('expected_track_width_m', TRACK_WIDTH_M)
        self.declare_parameter('track_width_tolerance_factor', 1.5)

        # 기본 꺼짐 — mission_summer.launch.py에서만 True로 켠다. 매니퓰레이터
        # 그리퍼 등 근접 장애물이 ROI 하단부를 가려서 median_height()의 80개
        # 유효 샘플 문턱을 못 넘기는 문제 대응용.
        # find_occlusion_cutoff() 참고 — 프레임마다 실제로 가림이 감지될 때만
        # ROI를 동적으로 좁히므로, 장애물이 없는 프레임/미션에서는 손해가 없다.
        self.declare_parameter('dynamic_occlusion_guard', False)
        self.declare_parameter(
            'occlusion_fill_ratio_threshold', OCCLUSION_FILL_RATIO_THRESHOLD_PLACEHOLDER)
        self.declare_parameter(
            'occlusion_min_run_rows', OCCLUSION_MIN_RUN_ROWS_PLACEHOLDER)

        # 기본 꺼짐 — 트랙 "옆"에 붙은 구조물이 세그멘테이션을 옆으로 끊어서
        # center_col이 틀어지는 문제 대응용(dynamic_occlusion_guard와 별개,
        # 이쪽은 좌우 방향). find_lateral_gap()/reconstruct_track_span() 참고
        # — depth 불연속으로 "진짜 구조물"인지 확인한 경우에만 복원한다.
        self.declare_parameter('lateral_occlusion_guard', False)
        self.declare_parameter('lateral_gap_min_px', LATERAL_GAP_MIN_PX_PLACEHOLDER)
        self.declare_parameter('lateral_depth_jump_m', LATERAL_DEPTH_JUMP_M_PLACEHOLDER)

        self.roi_y0 = float(self.get_parameter('roi_y_start_ratio').value)
        self.roi_y1 = float(self.get_parameter('roi_y_end_ratio').value)
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        self.depth_image_topic = str(self.get_parameter('depth_image_topic').value)
        self.mask_topic = str(self.get_parameter('mask_topic').value)
        self.track_w = float(self.get_parameter('track_width_m').value)
        self._edge_strip_half_width_px = int(
            self.get_parameter('edge_strip_half_width_px').value)
        self.slope_threshold_deg = float(self.get_parameter('slope_threshold_deg').value)
        self.roll_threshold_deg_positive = float(
            self.get_parameter('roll_threshold_deg_positive').value)
        self.roll_threshold_deg_negative = float(
            self.get_parameter('roll_threshold_deg_negative').value)

        self.slope_side_topic = str(self.get_parameter('slope_side_topic').value)
        self.slope_side_threshold_deg = float(
            self.get_parameter('slope_side_threshold_deg').value)

        force_mode_raw = str(self.get_parameter('force_mode').value)
        self.force_mode = resolve_force_mode(force_mode_raw)
        if self.force_mode != force_mode_raw:
            self.get_logger().warn(
                f"Unknown force_mode '{force_mode_raw}', defaulting to 'auto'")

        self.always_relay_flat = bool(self.get_parameter('always_relay_flat').value)
        if self.always_relay_flat:
            self.get_logger().info(
                'always_relay_flat 활성화 — /flatdrive/planned_path를 depth/roll '
                '계산과 무관하게 항상 /path로 즉시 릴레이합니다.')

        self._path_publish_hz = float(self.get_parameter('path_publish_hz').value)
        if self._path_publish_hz <= 0:
            self.get_logger().warn(
                f'path_publish_hz={self._path_publish_hz}는 유효하지 않음 — 20.0으로 대체.')
            self._path_publish_hz = 20.0

        # ROS 파라미터로는 None을 못 넘기니, 0 이하를 "검사 비활성화"로 취급.
        _expected_track_width_raw = float(self.get_parameter('expected_track_width_m').value)
        self._expected_track_width_m = (
            _expected_track_width_raw if _expected_track_width_raw > 0.0 else None)
        self._track_width_tolerance_factor = float(
            self.get_parameter('track_width_tolerance_factor').value)

        self.dynamic_occlusion_guard = bool(
            self.get_parameter('dynamic_occlusion_guard').value)
        self._occlusion_fill_ratio_threshold = float(
            self.get_parameter('occlusion_fill_ratio_threshold').value)
        self._occlusion_min_run_rows = int(
            self.get_parameter('occlusion_min_run_rows').value)
        if self.dynamic_occlusion_guard:
            self.get_logger().info(
                'dynamic_occlusion_guard 활성화 — '
                f'fill_ratio_threshold={self._occlusion_fill_ratio_threshold}, '
                f'min_run_rows={self._occlusion_min_run_rows}')

        self.lateral_occlusion_guard = bool(
            self.get_parameter('lateral_occlusion_guard').value)
        self._lateral_gap_min_px = int(self.get_parameter('lateral_gap_min_px').value)
        self._lateral_depth_jump_m = float(
            self.get_parameter('lateral_depth_jump_m').value)
        if self.lateral_occlusion_guard:
            self.get_logger().info(
                'lateral_occlusion_guard 활성화 — '
                f'gap_min_px={self._lateral_gap_min_px}, '
                f'depth_jump_m={self._lateral_depth_jump_m}')

        self.fx = self.fy = self.cx = self.cy = None
        self._undistort_map1 = None
        self._undistort_map2 = None
        self._latest_mask: np.ndarray | None = None
        self._bridge = CvBridge()
        self.enable_visualizer = bool(self.get_parameter('enable_visualizer').value)
        self.vis = _DepthRoiWindow('slope_decision') if self.enable_visualizer else None

        # 릴레이 대상 경로 — 아직 한쪽도 안 들어왔으면 None
        self._slope_path = None  # /terrain/planned_path 최신 메시지
        self._flat_path = None   # /flatdrive/planned_path 최신 메시지
        self._forward_slope_field: np.ndarray | None = None  # /terrain/slope_deg 최신 메시지

        # /path 발행 속도를 실제 계산 속도(카메라/segmentation
        # 갱신 주기, 보통 그보다 느림)와 분리하기 위한 캐시 — on_flat_path()/
        # _decide_and_publish()는 여기 캐시만 하고, _publish_latest_path()
        # 타이머가 path_publish_hz 주기로 이걸 그대로 재발행한다. 새 계산이
        # 없는 사이엔 같은 메시지(같은 header.stamp 포함, 일부러 안 고침 —
        # 데이터가 실제로 얼마나 오래됐는지 타임스탬프로 알 수 있어야 하므로)
        # 가 여러 번 나갈 수 있다.
        self._latest_path_msg: Path | None = None

        self.sub_info = self.create_subscription(
            CameraInfo, self.depth_info_topic, self.on_info, SENSOR_DATA_QOS_DEPTH1
        )
        self.sub_depth = self.create_subscription(
            CompressedImage, self.depth_image_topic, self.on_depth,
            SENSOR_DATA_QOS_DEPTH1
        )
        self.sub_mask = self.create_subscription(
            Image, self.mask_topic, self.on_mask, 10
        )
        self.sub_slope_path = self.create_subscription(
            Path, '/terrain/planned_path', self.on_slope_path, 10
        )
        self.sub_flat_path = self.create_subscription(
            Path, '/flatdrive/planned_path', self.on_flat_path, 10
        )
        self.sub_forward_slope = self.create_subscription(
            Image, '/terrain/slope_deg', self.on_forward_slope, 10
        )
        self.pub_slope = self.create_publisher(Float32, '/terrain/side_slope_angle_deg', 10)
        self.pub_path = self.create_publisher(Path, '/path', 10)
        self.pub_status = self.create_publisher(String, '/drive/status', 10)
        self.pub_slope_side = self.create_publisher(Int8, self.slope_side_topic, 10)

        # /path 발행 전용 타이머 — on_flat_path()/_decide_and_publish()와
        # 완전히 분리(path_camera_overlay의 HUD 10Hz 타이머와 같은 패턴).
        self._path_publish_timer = self.create_timer(
            1.0 / self._path_publish_hz, self._publish_latest_path)

        # roi_y_*/sample_step/occlusion_* 등을 ros2 param set으로 노드 재시작
        # 없이 튜닝할 수 있게 함 — 콜백 없이는 declare_parameter()가 딱 한 번
        # 읽어 캐싱한 self.xxx 인스턴스 변수가 파라미터 서버 값과 따로 놀지
        # 않도록 실행 중 변경을 인스턴스 변수에도 반영한다.
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(f'Subscribing camera info: {self.depth_info_topic}')
        self.get_logger().info(f'Subscribing depth image: {self.depth_image_topic}')

    def _on_parameter_update(self, params) -> SetParametersResult:
        """ros2 param set 콜백 — declare_parameter()로 __init__에서 한 번만
        읽어 캐싱해둔 인스턴스 변수들을 실행 중에도 갱신한다. 구독 토픽
        문자열(depth_info_topic 등)처럼 이미 create_subscription()이 걸려버려
        실행 중 바꿔봐야 반영 안 되는 것들과, enable_visualizer처럼 재생성
        비용이 있는 건 명시적으로 거부해서 "성공했다고 나오는데 실제로는
        무시됨" 같은 조용한 혼동을 막는다."""
        for p in params:
            if p.name == 'roi_y_start_ratio':
                self.roi_y0 = float(p.value)
            elif p.name == 'roi_y_end_ratio':
                self.roi_y1 = float(p.value)
            elif p.name == 'sample_step':
                if int(p.value) < 1:
                    return SetParametersResult(
                        successful=False, reason='sample_step은 1 이상이어야 합니다.')
                self.sample_step = int(p.value)
            elif p.name == 'max_depth_m':
                self.max_depth = float(p.value)
            elif p.name == 'track_width_m':
                self.track_w = float(p.value)
            elif p.name == 'edge_strip_half_width_px':
                if int(p.value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='edge_strip_half_width_px는 0 이상이어야 합니다.')
                self._edge_strip_half_width_px = int(p.value)
            elif p.name == 'slope_threshold_deg':
                self.slope_threshold_deg = float(p.value)
            elif p.name == 'roll_threshold_deg_positive':
                if float(p.value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='roll_threshold_deg_positive는 0 이상이어야 합니다.')
                self.roll_threshold_deg_positive = float(p.value)
            elif p.name == 'roll_threshold_deg_negative':
                if float(p.value) < 0:
                    return SetParametersResult(
                        successful=False,
                        reason='roll_threshold_deg_negative는 0 이상이어야 합니다 '
                               '(부호는 내부에서 붙습니다).')
                self.roll_threshold_deg_negative = float(p.value)
            elif p.name == 'force_mode':
                resolved = resolve_force_mode(str(p.value))
                if resolved != str(p.value):
                    self.get_logger().warn(
                        f"Unknown force_mode '{p.value}', defaulting to 'auto'")
                self.force_mode = resolved
            elif p.name == 'always_relay_flat':
                self.always_relay_flat = bool(p.value)
            elif p.name == 'path_publish_hz':
                # create_timer()로 만든 타이머는 실행 중 주기를 안전하게
                # 바꿀 방법이 마땅치 않아서(rclpy 버전별로 API가 다름),
                # enable_visualizer처럼 재시작을 요구하는 쪽을 택함 —
                # "성공했다는데 실제로는 예전 주기 그대로" 같은 조용한
                # 불일치를 만들지 않기 위함.
                return SetParametersResult(
                    successful=False,
                    reason='path_publish_hz는 타이머 주기라 실행 중 변경 불가 — '
                           '노드 재시작 필요.')
            elif p.name == 'expected_track_width_m':
                v = float(p.value)
                self._expected_track_width_m = v if v > 0.0 else None
            elif p.name == 'track_width_tolerance_factor':
                self._track_width_tolerance_factor = float(p.value)
            elif p.name == 'dynamic_occlusion_guard':
                self.dynamic_occlusion_guard = bool(p.value)
            elif p.name == 'occlusion_fill_ratio_threshold':
                self._occlusion_fill_ratio_threshold = float(p.value)
            elif p.name == 'occlusion_min_run_rows':
                if int(p.value) < 1:
                    return SetParametersResult(
                        successful=False,
                        reason='occlusion_min_run_rows는 1 이상이어야 합니다.')
                self._occlusion_min_run_rows = int(p.value)
            elif p.name == 'lateral_occlusion_guard':
                self.lateral_occlusion_guard = bool(p.value)
            elif p.name == 'lateral_gap_min_px':
                if int(p.value) < 1:
                    return SetParametersResult(
                        successful=False, reason='lateral_gap_min_px는 1 이상이어야 합니다.')
                self._lateral_gap_min_px = int(p.value)
            elif p.name == 'lateral_depth_jump_m':
                self._lateral_depth_jump_m = float(p.value)
            elif p.name == 'enable_visualizer':
                return SetParametersResult(
                    successful=False,
                    reason='enable_visualizer는 OpenCV 창 생성/해제가 걸려있어 '
                           '실행 중 변경 불가 — 노드 재시작 필요.')
            elif p.name == 'slope_side_threshold_deg':
                self.slope_side_threshold_deg = float(p.value)
            elif p.name in (
                    'depth_info_topic', 'depth_image_topic', 'mask_topic',
                    'slope_side_topic'):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name}은 구독/퍼블리셔가 이미 걸려있어 실행 중 변경 '
                           '불가 — 노드 재시작 필요.')
        return SetParametersResult(successful=True)

    def on_slope_path(self, msg: Path) -> None:
        self._slope_path = msg

    def on_flat_path(self, msg: Path) -> None:
        self._flat_path = msg
        if self.always_relay_flat:
            # on_depth()/roll 계산이 실패해도(마스크/depth 부실) /drive/status는
            # 계속 나가도록 여기서 바로 발행 — _decide_and_publish()를 거치지
            # 않음. /path 자체는 여기서 바로 발행 안 하고 _latest_path_msg에
            # 캐시만 해두고 _publish_latest_path() 타이머(path_publish_hz, 아래
            # __init__ 참고)가 일정 주기로 재발행한다 — /path 발행 속도를
            # 실제 계산 속도(카메라/segmentation 갱신 주기)와 분리하기 위함.
            status = String()
            status.data = 'flat'
            self.pub_status.publish(status)
            self._latest_path_msg = msg

    def _publish_latest_path(self) -> None:
        """path_publish_hz 타이머 콜백 — _latest_path_msg에 캐시된 최신
        경로를 그대로 재발행한다. 계산이 그보다 느리면(카메라/segmentation
        갱신 주기가 20Hz보다 느린 게 보통) 같은 메시지가 여러 번 나갈 수
        있다 — header.stamp는 원본 계산 시점 그대로 두고 안 고친다
        (재발행한다고 실제로 새 데이터인 척하지 않기 위함, 소비 측이
        stamp로 데이터 나이를 정확히 알 수 있어야 하므로)."""
        if self._latest_path_msg is not None:
            self.pub_path.publish(self._latest_path_msg)

    def on_forward_slope(self, msg: Image) -> None:
        try:
            self._forward_slope_field = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1').astype(np.float32)
        except CvBridgeError as exc:
            self.get_logger().error(f'/terrain/slope_deg CvBridge decode failed: {exc}')

    def on_mask(self, msg: Image) -> None:
        try:
            self._latest_mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self.get_logger().error(f'마스크 CvBridge decode failed: {exc}')

    def on_info(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        if self._undistort_map1 is None:
            # 마스크(undistort된 좌표계)와 픽셀을 맞추기 위해 depth도 undistort.
            camera_matrix = np.array(msg.k).reshape((3, 3))
            dist_coeffs = np.array(msg.d)
            self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None, camera_matrix,
                (msg.width, msg.height), cv2.CV_32FC1)

    def _depth_to_meters(self, cv_img: np.ndarray) -> np.ndarray:
        if cv_img.dtype == np.uint16:
            return cv_img.astype(np.float32) * 0.001
        return cv_img.astype(np.float32)

    def on_depth(self, msg: CompressedImage):
        if self.fx is None:
            return
        if self._latest_mask is None:
            self.get_logger().warn(
                f'{self.mask_topic} 마스크 대기 중.', throttle_duration_sec=2.0)
            return

        try:
            cv_img = decode_compressed_depth(msg)
        except Exception as exc:
            self.get_logger().error(
                f'compressedDepth decode failed: {exc}',
                throttle_duration_sec=5.0
            )
            return
        depth = self._depth_to_meters(cv_img)
        depth = cv2.remap(depth, self._undistort_map1, self._undistort_map2, cv2.INTER_NEAREST)

        if self._latest_mask.shape != depth.shape:
            self.get_logger().warn(
                '마스크/depth 해상도 불일치 — 스킵.', throttle_duration_sec=2.0)
            return

        h = depth.shape[0]
        y_start = int(h * self.roi_y0)
        y_end   = int(h * self.roi_y1)
        y_start = max(0, min(h-1, y_start))
        y_end   = max(0, min(h, y_end))

        # 좌/우는 이미지 중앙이 아니라 ROI 밴드 안 트랙(마스크) 픽셀의 열 범위
        # 중심을 기준으로 나눈다 — 트랙이 화면 중앙을 벗어나 있어도 정확함.
        roi_mask = self._latest_mask[y_start:y_end] > 0
        track_cols = np.flatnonzero(roi_mask.any(axis=0))
        if track_cols.size == 0:
            self.get_logger().warn(
                'ROI 안에 트랙 마스크 픽셀 없음 — roll 계산 스킵.', throttle_duration_sec=2.0)
            return
        center_col = int(round((track_cols[0] + track_cols[-1]) / 2.0))

        if self.dynamic_occlusion_guard:
            band_depth = depth[y_start:y_end, track_cols[0]:track_cols[-1] + 1]
            band_track = roi_mask[:, track_cols[0]:track_cols[-1] + 1]
            band_both = (
                np.isfinite(band_depth) & (band_depth > 0.1) & (band_depth < self.max_depth)
                & band_track)
            row_fill_ratios = band_both.mean(axis=1)
            cutoff = find_occlusion_cutoff(
                row_fill_ratios, self._occlusion_fill_ratio_threshold,
                self._occlusion_min_run_rows)
            if cutoff is not None and y_start + cutoff < y_end:
                new_y_end = y_start + cutoff
                self.get_logger().warn(
                    f'dynamic_occlusion_guard: ROI 하단 가림 감지(행 {new_y_end}부터, '
                    f'원래 y_end={y_end}) — y_end를 {new_y_end}로 축소.',
                    throttle_duration_sec=2.0)
                y_end = new_y_end
                roi_mask = self._latest_mask[y_start:y_end] > 0
                track_cols = np.flatnonzero(roi_mask.any(axis=0))
                if track_cols.size == 0:
                    self.get_logger().warn(
                        'dynamic_occlusion_guard 적용 후 ROI 안에 트랙 마스크 픽셀 '
                        '없음 — roll 계산 스킵.', throttle_duration_sec=2.0)
                    return
                center_col = int(round((track_cols[0] + track_cols[-1]) / 2.0))

        if self.lateral_occlusion_guard and self._expected_track_width_m is not None:
            col_present = roi_mask.any(axis=0)
            gap = find_lateral_gap(col_present, self._lateral_gap_min_px)
            if gap is not None:
                gap_start, gap_end = gap
                left_len = gap_start - int(track_cols[0])
                right_len = int(track_cols[-1]) - gap_end + 1
                if left_len >= right_len:
                    trusted_depth_arr = depth[y_start:y_end, track_cols[0]:gap_start]
                    trusted_mask_arr = roi_mask[:, track_cols[0]:gap_start]
                else:
                    trusted_depth_arr = depth[y_start:y_end, gap_end:track_cols[-1] + 1]
                    trusted_mask_arr = roi_mask[:, gap_end:track_cols[-1] + 1]
                trusted_valid = (
                    np.isfinite(trusted_depth_arr) & (trusted_depth_arr > 0.1)
                    & (trusted_depth_arr < self.max_depth) & trusted_mask_arr)
                if np.count_nonzero(trusted_valid) >= LATERAL_TRUSTED_MIN_SAMPLES:
                    trusted_depth_m = float(np.nanmedian(trusted_depth_arr[trusted_valid]))
                    gap_depth_arr = depth[y_start:y_end, gap_start:gap_end]
                    gap_valid = (
                        np.isfinite(gap_depth_arr) & (gap_depth_arr > 0.1)
                        & (gap_depth_arr < self.max_depth))
                    gap_depth_m = (
                        float(np.nanmedian(gap_depth_arr[gap_valid]))
                        if np.count_nonzero(gap_valid) > 0 else None)
                    new_span = reconstruct_track_span(
                        int(track_cols[0]), int(track_cols[-1]), gap, gap_depth_m,
                        trusted_depth_m, self._lateral_depth_jump_m,
                        self._expected_track_width_m, self.fx, depth.shape[1])
                    if new_span is not None:
                        new_first, new_last = new_span
                        self.get_logger().warn(
                            f'lateral_occlusion_guard: 옆 구조물 감지(열 {gap_start}~'
                            f'{gap_end}, gap_depth={gap_depth_m:.2f}m vs 신뢰 '
                            f'{trusted_depth_m:.2f}m) — 트랙 범위를 '
                            f'[{track_cols[0]},{track_cols[-1]}] -> '
                            f'[{new_first},{new_last}]로 복원.',
                            throttle_duration_sec=2.0)
                        track_cols = np.array([new_first, new_last])
                        center_col = int(round((new_first + new_last) / 2.0))

        # 대표 깊이(z_rep) — 트랙 폭 타당성 검사와 아래 edge strip 위치
        # 계산(핀홀 역산) 둘 다 여기 의존한다. 유효 depth 샘플이 너무
        # 적으면(<80) 신뢰 불가라 None으로 둔다 — median_height()가 뒤에서
        # 어차피 같은 기준으로 hL/hR 중 하나를 None으로 만들어 이번 프레임을
        # 자연스럽게 스킵하게 되므로, 여기선 검사만 건너뛰면 된다.
        span_depth = depth[y_start:y_end, track_cols[0]:track_cols[-1] + 1]
        span_track = roi_mask[:, track_cols[0]:track_cols[-1] + 1]
        span_valid = (
            np.isfinite(span_depth) & (span_depth > 0.1)
            & (span_depth < self.max_depth) & span_track)
        z_rep = (
            float(np.nanmedian(span_depth[span_valid]))
            if np.count_nonzero(span_valid) >= 80 else None)

        if self._expected_track_width_m is not None and z_rep is not None:
            measured_width_m = pixel_span_width_m(
                int(track_cols[0]), int(track_cols[-1]), z_rep, self.fx)
            if not track_width_plausible(
                    measured_width_m, self._expected_track_width_m,
                    self._track_width_tolerance_factor):
                self.get_logger().warn(
                    f'ROI 트랙 폭 추정치 {measured_width_m:.2f}m이 예상 폭 '
                    f'{self._expected_track_width_m:.2f}m보다 너무 넓음 — '
                    '세그멘테이션 오탐으로 보고 이번 프레임 roll 계산 스킵.',
                    throttle_duration_sec=2.0)
                return

        # 좌/우 높이 샘플링 지점 — track_width_m(로봇 자체 좌우 기준 거리)
        # 만큼 center_col에서 떨어진 두 지점 근방의 좁은 스트립만 쓴다
        # (edge_sample_strip_bounds 참고 — 트랙 절반 전체를 median 내면 그
        # median이 실제로는 트랙 폭의 1/4, 3/4 지점을 대표하게 돼서 hL/hR
        # 사이 실제 baseline이 track_width_m이 아니라 "보이는 트랙 폭의
        # 절반"이 돼버리는 게 roll 과소추정의 원인이었다). z_rep이 없거나
        # (depth 부족) 스트립 배치가 불가능하면(트랙이 로봇보다 좁은 극단적
        # 경우) 예전 절반-band 방식으로 폴백한다 — 이 폴백 경로는 위 버그를
        # 다시 갖고 있지만, depth가 부족한 프레임은 어차피 뒤 median_height()
        # 에서 스킵될 가능성이 높다.
        edge_bounds = None
        if z_rep is not None:
            half_span_px = self.track_w * 0.5 * self.fx / z_rep
            edge_bounds = edge_sample_strip_bounds(
                int(track_cols[0]), int(track_cols[-1]), center_col,
                half_span_px, self._edge_strip_half_width_px)

        if edge_bounds is not None:
            left_x0, left_x1, right_x0, right_x1, achieved_baseline_px = edge_bounds
            # clamp로 실제 baseline이 track_width_m보다 줄었을 수 있으니,
            # atan2 분모는 그 실제값으로 다시 환산해서 쓴다 — 항상 분자(dh)와
            # 분모(baseline_m)가 같은 두 지점을 가리키게 하기 위함.
            baseline_m = achieved_baseline_px * z_rep / self.fx
            # 열(column) 방향은 stride를 안 쓴다 — 스트립 자체가 이미
            # edge_strip_half_width_px 하나로 좁혀져 있어서(기본 61px) 여기에
            # 또 sample_step(기본 8)까지 적용하면 열이 7~8개밖에 안 남아
            # median_height()의 80개 유효 샘플 문턱을 실측 환경(마스크
            # fill률이 낮은 프레임)에서 거의 항상 못 넘긴다 — 실측(bagplay)
            # 중 hR 유효 샘플이 66~71개로 문턱을 근소하게 못 넘어 매 프레임
            # on_depth()가 조용히 return, /path·side_slope_angle_deg가 아예
            # 안 나가는 문제로 발견됨. 절반-band였던 예전 코드는 폭이
            # 수백 px라 stride를 걸어도 여유가 있었지만, 좁힌 스트립에는 안
            # 맞는 최적화였다. 스트립 폭 자체가 좁아 열 stride 없이도 계산
            # 비용은 무시할 만하다(행은 여전히 sample_step으로 줄임).
            col_stride = 1
        else:
            if z_rep is not None:
                self.get_logger().warn(
                    'edge strip 배치 불가(트랙이 track_width_m보다 좁음 등) — '
                    '절반-band 방식으로 폴백.', throttle_duration_sec=5.0)
            left_x0, left_x1 = int(track_cols[0]), center_col
            right_x0, right_x1 = center_col, int(track_cols[-1]) + 1
            baseline_m = self.track_w
            # 폴백 경로는 절반-band 전체(수백 px)라 예전처럼 열도 stride.
            col_stride = self.sample_step

        # stride 샘플링 — 행은 항상 sample_step, 열은 위에서 결정된 col_stride.
        left_roi  = depth[y_start:y_end:self.sample_step, left_x0:left_x1:col_stride]
        left_track = roi_mask[::self.sample_step, left_x0:left_x1:col_stride]
        right_roi = depth[y_start:y_end:self.sample_step, right_x0:right_x1:col_stride]
        right_track = roi_mask[::self.sample_step, right_x0:right_x1:col_stride]

        def median_height(roi: np.ndarray, track: np.ndarray):
            # 유효 depth + 트랙 마스크
            valid = np.isfinite(roi) & (roi > 0.1) & (roi < self.max_depth) & track
            if np.count_nonzero(valid) < 80:
                return None

            z = roi[valid]

            # 카메라 좌표에서 Y는 "아래" 방향일 수 있음. height proxy로 -Y를 사용
            # Y ≈ (v - cy) * Z / fy, 여기서 v를 ROI 중앙으로 둔 근사
            v_center = (y_start + y_end) * 0.5
            Y = (v_center - self.cy) * z / self.fy
            height = -Y  # 위쪽이 +가 되도록

            return float(np.nanmedian(height))

        hL = median_height(left_roi, left_track)
        hR = median_height(right_roi, right_track)
        if hL is None or hR is None:
            return

        dh = (hR - hL)  # +면 오른쪽이 더 높다(부호는 실측으로 확인 필요)
        # 분모는 self.track_w가 아니라 baseline_m — 위에서 실제 hL/hR
        # 샘플링 지점 사이의(clamp 반영된) 물리적 거리로 계산해뒀다. 폴백
        # 경로(edge_bounds is None)에서는 baseline_m == self.track_w.
        roll_rad = math.atan2(dh, baseline_m)
        roll_deg = math.degrees(roll_rad)

        if self.vis is not None:
            self.vis.show(
                depth,
                (left_x0, y_start, left_x1, y_end),
                (right_x0, y_start, right_x1, y_end),
                slope_deg=roll_deg,
                max_depth_m=self.max_depth,
            )

        out = Float32()
        out.data = float(roll_deg)
        self.pub_slope.publish(out)

        # 구동부 연동용 부호 신호 — |roll_deg|가 threshold 미만이면 평지(0)로
        # 발행. roll_deg 부호(+면 오른쪽이 더 높다)가 구동부와 맞춘 규약과
        # 그대로 일치해서 부호만 그대로 옮기면 된다(모듈 docstring 참고).
        side = Int8()
        if roll_deg >= self.slope_side_threshold_deg:
            side.data = 1   # right_slope
        elif roll_deg <= -self.slope_side_threshold_deg:
            side.data = -1  # left_slope
        else:
            side.data = 0
        self.pub_slope_side.publish(side)

        self._decide_and_publish(roll_deg)

    def _decide_and_publish(self, roll_deg: float) -> None:
        """임계각 기준으로 경사/평지 경로 중 하나를 골라 /path로,
        선택 결과를 /drive/status로 발행한다.

        slope 모드 전환 = (좌우 roll 임계각 초과) OR (전방 지형 경사 임계각
        초과, /terrain/slope_deg 기준) — roll만 보면 정면으로 곧게 솟은
        램프/쿠션류(roll은 안 바뀌고 pitch만 바뀜)를 못 잡아서 추가함
        (forward_slope_exceeds 참고).

        roll 쪽 임계 판단은 방향별 임계값
        (roll_threshold_deg_positive/negative)을 사용한다.

        force_mode가 'auto'가 아니면 위 자동 판단 결과(roll_exceeds/
        ahead_exceeds)는 계산만 하고(디버그/모니터링용, side_slope_angle_deg는
        이 함수 호출 전에 이미 게시됨) 최종 use_slope에는 반영하지 않는다 —
        'flat'/'slope'로 강제 고정.

        always_relay_flat=True면 force_mode보다도 우선해서 무조건 flat —
        다만 이 함수는 on_depth()가 성공했을 때만 불리므로, 그게 실패하는
        프레임까지 커버하려면 on_flat_path()의 직접 릴레이(같이 참고)가
        필요하다. 여기 있는 always_relay_flat 체크는 on_depth()가 어쩌다
        성공한 프레임에서도 결과가 일관되게 flat이 되도록 하는 것뿐이다.
        """
        roll_exceeds = roll_exceeds_threshold(
            roll_deg, self.roll_threshold_deg_positive, self.roll_threshold_deg_negative)
        ahead_exceeds = (
            self._forward_slope_field is not None
            and forward_slope_exceeds(self._forward_slope_field, self.slope_threshold_deg)
        )
        if self.always_relay_flat:
            use_slope = False
        elif self.force_mode == 'flat':
            use_slope = False
        elif self.force_mode == 'slope':
            use_slope = True
        else:  # 'auto'
            use_slope = roll_exceeds or ahead_exceeds
        status = String()
        status.data = 'slope' if use_slope else 'flat'
        self.pub_status.publish(status)

        path = self._slope_path if use_slope else self._flat_path
        if path is None:
            self.get_logger().warn(
                f'{status.data} 경로 아직 수신 안 됨 — /path 미발행.',
                throttle_duration_sec=2.0)
            return
        # 여기서도 바로 발행 안 하고 캐시만 함 — on_flat_path()와 동일하게
        # _publish_latest_path() 타이머가 path_publish_hz로 재발행.
        self._latest_path_msg = path

    def destroy_node(self):
        if self.vis is not None:
            self.vis.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SideSlopeTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
