"""
Gradient Field Node — step (a) of the terrain path-planning pipeline.

Pure computation (compute_gradient_field) is ROS-free so it can be unit-tested
and benchmarked standalone.

Coordinate convention used throughout this package:
  elevation[r, c]  — height in metres at grid cell (r, c)
  col  increases in  +x direction  (robot forward)
  row  increases in  -y direction  (row 0 = max y = robot left side)

  gx = dH/dx  — positive means uphill going forward
  gy = dH/dy  — positive means uphill going left

Subscribed topics:
  /terrain/elevation_map    sensor_msgs/Image  32FC1  (metres; NaN = unknown)
  /flatdrive/planned_path   nav_msgs/Path  (flat_drive.py의 RGB 세그멘테이션 중심선 —
                             centerline_row_targets()/centerline_cost_weight 유도 비용에만 씀,
                             elevation/slope 계산 자체에는 영향 없음)

Published topics:
  /terrain/gradient_x          sensor_msgs/Image  32FC1
  /terrain/gradient_y          sensor_msgs/Image  32FC1
  /terrain/gradient_magnitude  sensor_msgs/Image  32FC1  (unitless m/m)
  /terrain/gradient_direction  sensor_msgs/Image  32FC1  (radians; 0 = +x/forward)
  /terrain/slope_deg           sensor_msgs/Image  32FC1  (degrees)
  /terrain/planned_path        nav_msgs/Path            (cells with slope <= max_slope_deg)

Parameters:
  resolution_m         float  cell size [m]            default 0.15
  bench_log_hz         float  timing log interval [Hz]  default 1.0
  elevation_median_window_cells  int  default 3  (placeholder — see
                               ELEVATION_MEDIAN_WINDOW_CELLS_PLACEHOLDER; 단일 셀 이상치
                               제거용 중앙값 필터, median_filter_elevation() 참고. smooth_elevation
                               보다 먼저 적용됨. 1 이하면 끔)
  elevation_smoothing_sigma_m  float  default 0.15  (placeholder — see
                               ELEVATION_SMOOTHING_SIGMA_M_PLACEHOLDER; smooth_elevation() 참고.
                               0이면 끔)
  max_slope_deg        float  max traversable slope [deg]  default 35.0  (placeholder — see MAX_SLOPE_DEG_PLACEHOLDER)
  lateral_clearance_m  float  편도(좌우 각각) 확보해야 하는 여유 [m]  default 0.27
                               (로봇 폭 516mm 기준 — apply_lateral_clearance() 참고. 0이면 끔)
  min_valid_neighbors  int    default 2  (placeholder — see MIN_VALID_NEIGHBORS_PLACEHOLDER;
                               고립된 노이즈 셀 마스킹, mask_low_confidence_cells() 참고. 0이면 끔)
  slope_cost_weight    float  default 0.8  (placeholder — see SLOPE_COST_WEIGHT_PLACEHOLDER;
                               완만한 우회로 선호, plan_path_on_slope_field() 참고. 0이면 순수 최단거리)
  centerline_cost_weight  float  default 0.1  (placeholder — see CENTERLINE_COST_WEIGHT_PLACEHOLDER;
                               flat_drive.py RGB 중심선으로 유도, centerline_row_targets() 참고. 0이면 끔)
  flat_drive_path_topic   str  default '/flatdrive/planned_path'
  path_frame_id        str    default 'camera_link' — /terrain/planned_path에 실릴
                               frame_id. 경로 좌표(x=전방,y=좌측)는 body 규약이지
                               입력 /terrain/elevation_map이 물려받은 depth 이미지의
                               optical frame(x=오른쪽,y=아래,z=전방)이 아니므로 그
                               header를 그대로 재사용하면 안 됨.
"""

import heapq
import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from rcl_interfaces.msg import SetParametersResult
from scipy.ndimage import gaussian_filter, generic_filter
from sensor_msgs.msg import Image
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# Pure computation — no ROS dependency
# ---------------------------------------------------------------------------

# PLACEHOLDER — 평지 매트의 몇 cm 잔주름도 0.15m 격자에서 40~60°로
# 증폭될 수 있다. compute_gradient_field() 호출 전에 elevation을 이 반경으로
# 스무딩해서 작은 굴곡을 눌러준다. 값이
# 클수록 더 많이 눌리고(진짜 완만한 경사/단차도 같이 뭉개질 수 있음), 0이면
# 끔. 실측 후 조정할 것.
ELEVATION_SMOOTHING_SIGMA_M_PLACEHOLDER = 0.15

# 평평한 바닥에서도 단일 셀 이상치가 40~60도급 스파이크를 만들 수 있어
# slope_decision.py의 forward_slope_exceeds()가 flat/slope를 계속 오락가락
# 하는 문제를 실기에서 확인 — 위 sigma=0.15 스무딩(가우시안)이 다루던 "매트
# 잔주름"(몇 cm급)과는 다른, 훨씬 큰 단일 셀 이상치(실측: 이웃 대비 0.5m
# 넘게 튐, depth 센서의 순간적 튐값으로 추정)였음. 가우시안 블러는 이런
# 극단적 이상치를 주변으로 "퍼뜨리기"만 해서 오히려 더 넓은 범위가 문턱을
# 넘게 만들 수 있다 — 중앙값(median) 필터는 소수의 극단값에 영향을 거의
# 안 받는 통계량이라 이런 단일 이상치 제거에 훨씬 강건하다. smooth_elevation()
# (가우시안, 원래 목적인 잔주름 완화용)보다 먼저 적용해서 이상치부터 없앤
# 뒤 가우시안으로 넘긴다 — 순서: median_filter_elevation() -> smooth_elevation().
ELEVATION_MEDIAN_WINDOW_CELLS_PLACEHOLDER = 3


def median_filter_elevation(elevation: np.ndarray, window_cells: int) -> np.ndarray:
    """NaN을 무시하는 중앙값 필터로 elevation의 단일 셀 이상치를 제거한다.

    smooth_elevation()(가우시안)과 같은 NaN 처리 원칙을 따른다 — 원래
    NaN이던 셀은 그대로 NaN으로 남기고(inpaint 아님), 원래 값이 있던 셀만
    (window_cells x window_cells) 정사각 이웃의 중앙값(NaN 제외)으로
    바꾼다. 이웃이 전부 NaN인 경우는 없다 — 창(window)이 항상 자기 자신의
    유효값을 포함하므로 nanmedian이 NaN을 낼 일이 없다.

    가우시안과 달리 median은 창 안에 극단값이 하나(혹은 소수) 있어도 그
    값이 결과에 거의 반영되지 않는다 — 그래서 depth 센서의 순간적 튐값
    같은 단일 셀 이상치에 훨씬 강건하다(가우시안은 반대로 그 튐값을 주변
    셀로 퍼뜨려서 문제를 넓히기만 함, 위 ELEVATION_MEDIAN_WINDOW_CELLS_PLACEHOLDER
    주석 참고).

    Args:
        elevation: (H, W) float32, 미터 단위 높이 (NaN=미측정).
        window_cells: 정사각 창 한 변의 셀 수 (홀수 권장: 3, 5, ...).
            1 이하면 그대로 복사본 반환(끔).
        resolution은 이 함수에서 안 쓴다(창 크기가 셀 개수 단위라 물리
        단위 변환이 필요 없음) — smooth_elevation()과 시그니처를 맞추지
        않은 이유.

    Returns:
        필터링된 (H, W) float32 배열 (복사본, NaN 위치는 원본과 동일).
    """
    if window_cells <= 1:
        return elevation.copy()

    valid = np.isfinite(elevation)
    filled = np.where(valid, elevation, np.nan).astype(np.float64)

    with np.errstate(invalid='ignore'):
        filtered = generic_filter(
            filled, np.nanmedian, size=window_cells, mode='constant', cval=np.nan)

    out = elevation.copy()
    out[valid] = filtered[valid]
    return out.astype(np.float32)


def smooth_elevation(elevation: np.ndarray, sigma_m: float, resolution: float) -> np.ndarray:
    """NaN을 무시하는 가우시안 블러로 elevation을 스무딩한다.

    scipy.ndimage.gaussian_filter는 NaN을 그대로 두면 주변 셀까지 다
    오염시키므로, 표준 기법(정규화 컨볼루션)을 쓴다: NaN을 0으로 채운
    값과 유효(non-NaN) 마스크를 각각 같은 커널로 블러링한 뒤 나눠서,
    유효한 이웃들만으로 가중평균한 값을 얻는다.

    원래 NaN이던 셀은 절대 채우지 않고(inpaint 아님) 그대로 NaN으로 남긴다
    — 이 맵은 NaN 비율이 70%대로 높아서, 빈 공간까지 이웃값으로 채우면
    그 경계에서 새로운 가짜 경사가 대량으로 생긴다(실측 확인: 무분별하게
    채웠더니 30° 넘는 셀이 5개에서 100개 이상으로 늘어남). 원래 값이 있던
    셀만 이웃 평균으로 다듬는다.

    Args:
        elevation: (H, W) float32, 미터 단위 높이 (NaN=미측정).
        sigma_m: 가우시안 표준편차 [m]. 0 이하면 그대로 복사본 반환(끔).
        resolution: 셀 크기 [m].

    Returns:
        스무딩된 (H, W) float32 배열 (복사본, NaN 위치는 원본과 동일).
    """
    if sigma_m <= 0.0:
        return elevation.copy()

    sigma_cells = sigma_m / resolution
    valid = np.isfinite(elevation)
    filled = np.where(valid, elevation, 0.0).astype(np.float32)

    weight_sum = gaussian_filter(valid.astype(np.float32), sigma_cells, mode='constant', cval=0.0)
    value_sum = gaussian_filter(filled, sigma_cells, mode='constant', cval=0.0)

    with np.errstate(invalid='ignore', divide='ignore'):
        smoothed = value_sum / weight_sum

    out = elevation.copy()
    out[valid] = smoothed[valid]
    return out.astype(np.float32)


def compute_gradient_field(
    elevation: np.ndarray,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute per-cell gradient vectors from a float32 elevation map.

    Uses numpy central-difference (2nd-order accurate) via np.gradient.
    NaN cells propagate into adjacent gradient cells — caller should mask or
    inpaint unknowns before calling if NaN coverage is high.

    Args:
        elevation:  (H, W) float32 array, heights in metres.
        resolution: isotropic cell size in metres.

    Returns:
        gx        (H, W): dH/dx — positive = uphill going forward (+x)
        gy        (H, W): dH/dy — positive = uphill going left (+y)
        magnitude (H, W): sqrt(gx²+gy²) — steepest-descent slope magnitude
        direction (H, W): atan2(gy, gx) [rad] pointing uphill; 0 = +x
        slope_deg (H, W): arctan(magnitude) in degrees — true slope angle
    """
    grad_row, grad_col = np.gradient(elevation, resolution)
    gx = grad_col          # d(elev)/d(col_coord) == dH/dx
    gy = -grad_row         # row↑ ≡ y↓  →  dH/dy = -dH/d(row_coord)
    magnitude = np.hypot(gx, gy)
    direction = np.arctan2(gy, gx)
    slope_deg = np.degrees(np.arctan(magnitude))
    return gx, gy, magnitude, direction, slope_deg


# 최대 허용 경사각 [deg]. 정면에서 관측된 43~49° 경사를 통과 불가로 처리하고
# SlopeCritic 설정(max_slope_deg=30, critical_slope_deg=25)과 비슷한 기준을
# 적용하도록 35°로 설정했다. 실제 등반 한계는 하드웨어 트랙션 테스트 후
# max_slope_deg 파라미터로 다시 조정할 것.
MAX_SLOPE_DEG_PLACEHOLDER = 35.0

# 그리드 이동: (행 변화량, 열 변화량, 이동 비용)
# 순수 좌우 이동
# (dr=±1, dc=0 — 전방 진행량 0)을 빼서 6-연결로 바꿈. bag 재생 실기
# 디버깅: 시작 지점(row) 코스트가 안 좋으면 Dijkstra가 "같은 열(전방거리
# 고정)에서 여러 칸 옆으로 이동해 더 싼 행을 찾은 뒤 전진 시작"하는 경로를
# 냈음(실측: x가 6스텝 연속 0.0으로 고정된 채 y만 -0.9m까지 휩씀) —
# 그리드 위에서는 "최적"이어도 차동구동 로봇이 실행 불가능한 경로(제자리
# 옆이동)라 카메라 화면에 경로가 전방 방향과 무관하게 가로로 찍히는 걸로
# 나타남. dc=0(제자리 옆이동) 이동만 제거 — 남은 6개 이동은 전부 |dc|=1
# (전진 또는 후진 중 하나)이라, 매 스텝마다 열(전방거리)이 반드시 바뀐다.
# 대각선 이동(dr=±1과 dc=+1)만으로도 여전히 좌우 회피는 가능하고, 순수
# 옆이동만 없어짐. 후진(dc=-1) 자체는 안 막았음 — 이번에 관측된 문제는
# "제자리 옆이동"이었지 후진이 아니었어서 범위를 그것만으로 좁힘.
_SQRT2 = math.sqrt(2.0)
_NEIGHBOR_STEPS = [
    (-1, -1, _SQRT2),                  (-1, 1, _SQRT2),
                       (0, -1, 1.0),    (0, 1, 1.0),
    (1, -1, _SQRT2),                   (1, 1, _SQRT2),
]

# PLACEHOLDER — 도착 셀의 slope_deg 1도당 이동 비용에 더해지는 가중치.
# max_slope_deg 밑이기만 하면 0°든 (threshold 바로
# 밑의) 급경사든 이동 비용이 완전히 동일해서, 옆으로 몇 칸만 돌면 완전
# 평지가 있어도 Dijkstra가 "더 짧다"는 이유만으로 급경사를 그대로 뚫고
# 지나가는 경로를 냈다(실측: 정면 47° 셀 통과 vs 바로 옆 0° — 최단거리라는
# 이유만으로 47° 선택). max_slope_deg는 여전히 하드 컷오프(그 이상은
# 무조건 impassable)로 남기고, 그 밑에서는 가파를수록 비용을 올려 완만한
# 우회로가 있으면 그쪽을 선호하게 만든다. 실측 후 조정할 것 — 너무 크면
# 사소한 경사차에도 불필요하게 지그재그, 너무 작으면 이번 문제가 재현됨.
# bag 재생 실기 디버깅에서 0.3으로는 회피가 약해 0.8로 설정했다
# (centerline_cost_weight의
# 직진 유도와 상쇄돼서 살짝만 비켜감) — max_slope_deg 하드컷(35도)과 같이
# 적용해서 35도 밑에서도 완만한 쪽을 더 확실히 선호하게 올림.
SLOPE_COST_WEIGHT_PLACEHOLDER = 0.8

# PLACEHOLDER — flat_drive.py의 RGB 세그멘테이션 중심선
# (/flatdrive/planned_path)을 향한 유도 비용 가중치, 셀(그리드 칸) 단위
# 거리당. gradient_map 자체 elevation 데이터가 희소해서(NaN 비율 높음)
# 시야가 좁을 때, RGB 쪽이 훨씬 넓게 보는 트랙 범위 안에서 경로가 벗어나지
# 않게 붙잡아준다 — max_slope_deg 하드컷은 그대로 두고, 그 밑에서 어디를
# 고를지에만 영향. 0이면 끔. 실측 후 조정할 것.
CENTERLINE_COST_WEIGHT_PLACEHOLDER = 0.1


def centerline_row_targets(
    centerline_xy: list[tuple[float, float]],
    grid_h: int,
    grid_w: int,
    resolution_m: float,
) -> np.ndarray:
    """RGB 중심선(x_forward, y_left 미터 좌표 리스트 — flat_drive.py의
    /flatdrive/planned_path와 동일 규약, camera_link 프레임)을 gradient_map
    그리드의 열(column)별 "중심선이 위치하는 row" 배열로 변환한다.

    각 열 c의 x_forward=c*resolution_m에 대해 centerline_xy를 x_forward
    기준 선형보간해서 y_left를 구하고 row = grid_h//2 - y_left/resolution_m로
    바꾼다. centerline_xy가 그 x_forward를 커버 못 하는 열(관측 범위 밖)은
    NaN — plan_path_on_slope_field()가 그 열에서는 유도 비용 없이(0) 넘어간다.

    두 노드 다 같은 body 좌표계(x=전방, y=좌측, camera_link)를 쓰므로 별도
    좌표 변환 없이 그대로 비교 가능하다.
    """
    center_row = grid_h // 2
    targets = np.full(grid_w, np.nan, dtype=np.float64)
    if len(centerline_xy) < 2:
        return targets

    xs = np.array([p[0] for p in centerline_xy], dtype=np.float64)
    ys = np.array([p[1] for p in centerline_xy], dtype=np.float64)
    order = np.argsort(xs)
    xs, ys = xs[order], ys[order]

    col_x = np.arange(grid_w, dtype=np.float64) * resolution_m
    in_range = (col_x >= xs[0]) & (col_x <= xs[-1])
    y_interp = np.interp(col_x[in_range], xs, ys)
    targets[in_range] = center_row - y_interp / resolution_m
    return targets


def plan_path_on_slope_field(
    slope_deg: np.ndarray,
    start: tuple[int, int],
    target_col: int,
    max_slope_deg: float = MAX_SLOPE_DEG_PLACEHOLDER,
    slope_cost_weight: float = SLOPE_COST_WEIGHT_PLACEHOLDER,
    centerline_row_by_col: np.ndarray | None = None,
    centerline_cost_weight: float = CENTERLINE_COST_WEIGHT_PLACEHOLDER,
) -> list[tuple[int, int]] | None:
    """Plan the cheapest slope-limited path from `start` toward column `target_col`.

    Cells with slope_deg > max_slope_deg (or NaN) are treated as impassable —
    a hard cutoff, unaffected by slope_cost_weight. Dijkstra search over an
    8-connected grid finds the lowest-cost route to the nearest traversable
    cell in `target_col`.

    Moving into a cell costs `grid_step * (1 + slope_cost_weight * slope_deg)`
    — grid_step is 1.0/√2 for orthogonal/diagonal moves, slope_deg is the
    *destination* cell's slope. slope_cost_weight=0 recovers pure
    shortest-path behaviour (only the max_slope_deg cutoff matters, any two
    passable cells cost the same regardless of steepness). With
    slope_cost_weight>0, a merely-steep-but-passable cell can cost more than
    a longer detour through flatter cells, so the search actually prefers
    gentler routes instead of just the shortest one — see
    SLOPE_COST_WEIGHT_PLACEHOLDER for why this was added: distance-only cost
    can select a 47° patch even when adjacent cells are flat because every
    cell below max_slope_deg is treated as equally "free".

    target_col에 정확히 못 닿더라도 탐색 중 도달한 셀 중 열(전방 거리)이 가장
    먼 셀까지의 경로를 최선책으로 반환한다 — 안전 기준(max_slope_deg 등)은
    그대로 지키면서 "어디까지 갈지"만 유연하게 한 것. start 자체가 막혀서
    한 걸음도 못 뗄 때만 여전히 None.

    Args:
        slope_deg:  (H, W) slope angle in degrees, as returned by
                    compute_gradient_field.
        start:      (row, col) starting cell — must be traversable.
        target_col: column index to reach (forward distance); clamped to grid.
        max_slope_deg: cells steeper than this are impassable (hard cutoff).
        slope_cost_weight: extra cost per degree of destination-cell slope,
                    as a fraction of the base grid-step cost. 0 disables
                    slope-aware routing (pure shortest path).
        centerline_row_by_col: optional (W,) array — target row per column
                    from an independently-sourced (e.g. RGB segmentation)
                    centerline, as returned by centerline_row_targets().
                    NaN columns get no bias. None disables this entirely.
        centerline_cost_weight: extra cost per cell of lateral distance from
                    centerline_row_by_col, per grid step. Only applies where
                    centerline_row_by_col is not NaN for that column.

    Returns:
        List of (row, col) cells from start to the farthest reached cell
        (inclusive) — reaches target_col when possible, otherwise the best
        reachable column short of it. None only if start is impassable.
    """
    h, w = slope_deg.shape
    start_r, start_c = start
    if not (0 <= start_r < h and 0 <= start_c < w):
        raise ValueError(f'start {start} outside grid bounds {(h, w)}')
    target_col = max(0, min(w - 1, target_col))

    traversable = np.isfinite(slope_deg) & (slope_deg <= max_slope_deg)
    if not traversable[start_r, start_c]:
        return None

    dist = np.full((h, w), np.inf)
    dist[start_r, start_c] = 0.0
    visited = np.zeros((h, w), dtype=bool)
    prev: dict[tuple[int, int], tuple[int, int]] = {}
    pq: list[tuple[float, tuple[int, int]]] = [(0.0, (start_r, start_c))]

    goal = None
    # target_col에 못 닿을 경우를 대비한 최선책 — 지금까지 방문한 셀 중
    # 열(전방 거리)이 가장 먼 셀. Dijkstra가 비용 오름차순으로 방문하므로,
    # 같은 열에 처음 도달하는 순간의 노드가 그 열까지의 최저비용 경로다.
    best_node = (start_r, start_c)
    best_col = start_c
    while pq:
        d, node = heapq.heappop(pq)
        if visited[node]:
            continue
        visited[node] = True
        if node[1] > best_col:
            best_col = node[1]
            best_node = node
        if node[1] == target_col:
            goal = node
            break

        r, c = node
        for dr, dc, step_cost in _NEIGHBOR_STEPS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            if visited[nr, nc] or not traversable[nr, nc]:
                continue
            cost_mult = 1.0 + slope_cost_weight * slope_deg[nr, nc]
            if centerline_row_by_col is not None:
                target_row = centerline_row_by_col[nc]
                if not np.isnan(target_row):
                    cost_mult += centerline_cost_weight * abs(nr - target_row)
            nd = d + step_cost * cost_mult
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                prev[(nr, nc)] = node
                heapq.heappush(pq, (nd, (nr, nc)))

    if goal is None:
        goal = best_node  # target_col 도달 실패 — 도달 가능했던 데까지로 대체

    path = [goal]
    while path[-1] != (start_r, start_c):
        path.append(prev[path[-1]])
    path.reverse()
    return path


# 실측값 — 로봇 폭 516mm 기준, 편도(좌우 각각) 이만큼(270mm) 여유가 있어야
# 실제로 그 폭만큼 지나갈 수 있다고 보고 경로를 계획한다(정확한 반폭
# 258mm보다 살짝 더 여유를 둔 값). apply_lateral_clearance() 참고.
LATERAL_CLEARANCE_M = 0.27


def apply_lateral_clearance(
    slope_deg: np.ndarray,
    max_slope_deg: float,
    margin_cells: int,
) -> np.ndarray:
    """로봇 폭만큼 좌우 여유가 없는 셀은 plan_path_on_slope_field()가 알아서
    impassable로 보게 slope_deg를 NaN으로 "오염"시켜서 반환한다.

    plan_path_on_slope_field()는 셀 하나하나를 독립적으로만
    (slope_deg <= max_slope_deg) 통과 가능 여부를 판정한다 — 로봇이 실제로
    그 지점을 지나가려면 옆으로도(좌우, 이 그리드의 row 축) 로봇 폭만큼
    여유가 있어야 한다는 조건이 없다(모듈 상단 좌표 규약: row=좌우,
    col=전방). 그 결과 계단식 지형 등에서 셀 1~2개 폭짜리, 로봇이 실제로는
    못 지나가는 좁은 "통과 가능" 틈을 따라 경로가 날 수 있다.

    이 함수는 plan_path_on_slope_field() 자체(및 기존 테스트)는 안 건드리고,
    호출부에서 그 함수에 넘길 slope_deg를 전처리하는 방식으로 고친다: 셀
    (r, c)가 최종적으로 통과 가능하려면 같은 열(c) 안에서 (r-margin_cells)부터
    (r+margin_cells)까지 전부 원래 기준으로 통과 가능해야 한다 — 그렇지
    않으면 (r, c) 자체도 NaN으로 만들어 막는다. margin_cells가 그리드
    범위를 벗어나는 경우(그리드 가장자리라 여유 확보 여부를 알 수 없는
    경우)는 안전하게 통과 불가로 취급한다.

    slope_deg의 원본(가공 전 실측값)은 그대로 두고 싶을 때를 위해 이 함수는
    복사본을 반환한다 — 예: /terrain/slope_deg로 발행하는 원본 값은 이
    함수를 거치지 않은 원본을 써야 한다(경로 계획용으로만 전처리된 값을
    실측치인 것처럼 발행하면 안 됨).
    """
    traversable = np.isfinite(slope_deg) & (slope_deg <= max_slope_deg)
    if margin_cells <= 0:
        return slope_deg.copy()

    cleared = traversable.copy()
    for offset in range(1, margin_cells + 1):
        # 그리드 밖으로 나가는 경우(위/아래 가장자리 근처)는 여유 확보
        # 여부를 알 수 없으므로 False(통과 불가)로 취급한다.
        up = np.zeros_like(traversable)
        up[offset:] = traversable[:-offset]
        down = np.zeros_like(traversable)
        down[:-offset] = traversable[offset:]
        cleared &= up & down

    out = slope_deg.copy()
    out[~cleared] = np.nan
    return out


# elevation_map.py에서 min_points_per_cell을 적용해도, NaN 이웃이 많은 셀
# 주변 gradient는 신뢰하기 어렵다. 이 조건을 2차 방어선으로 사용한다.
# NaN 커버리지가 원래 높은 맵(현재 77% 수준)이라
# 문턱을 너무 빡빡하게 잡으면 멀쩡한 셀까지 다 막히므로, "완전히 고립된
# (자기 자신 제외 유효 이웃이 0개인) 셀"만 걸러내는 최소한의 기준으로 시작.
# 실측 후 조정할 것.
MIN_VALID_NEIGHBORS_PLACEHOLDER = 2  # 3x3 윈도우(자기 자신 포함) 중 최소 유효 개수


def mask_low_confidence_cells(
    slope_deg: np.ndarray,
    min_valid_neighbors: int = MIN_VALID_NEIGHBORS_PLACEHOLDER,
) -> np.ndarray:
    """3x3 이웃(자기 자신 포함) 중 유효(non-NaN) 셀 수가 min_valid_neighbors
    미만인 셀은 NaN으로 오염시켜 plan_path_on_slope_field()가 impassable로
    보게 한다.

    compute_gradient_field()는 NaN 근처에서 편측/왕복 차분이 왜곡될 수
    있다고 이미 경고한다(모듈 docstring 참고) — 특히 유효 포인트가 희소한
    맵에서 이웃 없이 홀로 존재하는 셀은 진짜 지형보다는 depth 노이즈일
    가능성이 높다. apply_lateral_clearance()와 마찬가지로 원본 slope_deg는
    건드리지 않고 복사본을 반환한다 — 경로 계획 전처리에만 쓸 것.
    """
    valid = np.isfinite(slope_deg)
    h, w = slope_deg.shape
    count = np.zeros((h, w), dtype=np.int32)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            r_src_lo, r_src_hi = max(0, -dr), h - max(0, dr)
            c_src_lo, c_src_hi = max(0, -dc), w - max(0, dc)
            r_dst_lo, r_dst_hi = max(0, dr), h - max(0, -dr)
            c_dst_lo, c_dst_hi = max(0, dc), w - max(0, -dc)
            count[r_dst_lo:r_dst_hi, c_dst_lo:c_dst_hi] += (
                valid[r_src_lo:r_src_hi, c_src_lo:c_src_hi])

    out = slope_deg.copy()
    out[count < min_valid_neighbors] = np.nan
    return out


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class GradientMapNode(Node):
    """ROS2 wrapper: subscribes to elevation map, publishes gradient field."""

    def __init__(self):
        super().__init__('gradient_map_node')

        # 기본 꺼짐. slope_decision.py가
        # always_relay_flat으로 flat_drive 경로만 쓰도록 고정되면서, 이 노드가
        # 만드는 /terrain/planned_path·/terrain/slope_deg 등을 아무도 안 쓰게
        # 됨. 노드는 계속 뜨지만 _on_elevation()이 맨 앞에서 바로 return해서
        # 연산 자체(gradient/slope/Dijkstra 경로탐색)를 안 한다. 코드에서 이
        # 기본값을 바꾸거나 --ros-args -p enabled:=true / ros2 param set으로
        # 켜기 전엔 계속 꺼진 채로 유지됨.
        self.declare_parameter('enabled', False)

        self.declare_parameter('resolution_m', 0.15)
        self.declare_parameter('bench_log_hz', 1.0)
        # 단일 셀 이상치(depth 센서 순간 튐값 등) 제거용 중앙값 필터 창 크기
        # (셀 개수) — median_filter_elevation() 참고. smooth_elevation()
        # (가우시안)보다 먼저 적용된다. 1 이하면 끔.
        self.declare_parameter(
            'elevation_median_window_cells', ELEVATION_MEDIAN_WINDOW_CELLS_PLACEHOLDER)
        # 표면 잔주름 등 작은 굴곡이 grid 해상도에서 큰 각도로 증폭되는 걸
        # 누그러뜨리는 정도 — smooth_elevation() 참고. 0이면 끔(스무딩 없음).
        self.declare_parameter('elevation_smoothing_sigma_m', ELEVATION_SMOOTHING_SIGMA_M_PLACEHOLDER)
        # TODO: 임시 기본값. 하드웨어 등반 한계 실측 후 조정할 것 (MAX_SLOPE_DEG_PLACEHOLDER 참고).
        self.declare_parameter('max_slope_deg', MAX_SLOPE_DEG_PLACEHOLDER)
        # 로봇이 실제로 지나가려면 편도(좌우 각각) 이만큼 여유가 있어야
        # 한다 — apply_lateral_clearance() 참고. 0으로 주면 이 전처리를
        # 끄고 예전처럼 셀 단위로만 판정한다.
        self.declare_parameter('lateral_clearance_m', LATERAL_CLEARANCE_M)
        # 고립된(유효 이웃 없는) 셀을 노이즈로 보고 걸러내는 문턱 —
        # mask_low_confidence_cells() 참고. 0이면 이 전처리를 끈다.
        self.declare_parameter('min_valid_neighbors', MIN_VALID_NEIGHBORS_PLACEHOLDER)
        # 완만한 우회로를 선호하게 만드는 경사 비용 가중치 —
        # plan_path_on_slope_field() 참고. 0이면 예전처럼 순수 최단거리.
        self.declare_parameter('slope_cost_weight', SLOPE_COST_WEIGHT_PLACEHOLDER)
        # flat_drive.py의 RGB 중심선(/flatdrive/planned_path)으로 유도하는
        # 비용 가중치 — centerline_row_targets()/plan_path_on_slope_field()
        # 참고. 0이면 끔(RGB 중심선 무시).
        self.declare_parameter('centerline_cost_weight', CENTERLINE_COST_WEIGHT_PLACEHOLDER)
        self.declare_parameter('flat_drive_path_topic', '/flatdrive/planned_path')
        # /terrain/planned_path에 실릴 frame_id. 발행되는 경로
        # 좌표는 x=전방/y=왼쪽인 body(=camera_link) 규약이지, 입력
        # /terrain/elevation_map이 물려받은 depth 이미지의 optical frame
        # (x=오른쪽/y=아래/z=전방)이 아니다 — msg.header를 그대로 재사용하면
        # 좌표값은 body인데 frame_id만 optical로 잘못 나간다. path_relay_node가
        # 이 frame_id를 그대로 믿고 TF로 odom 변환하므로, 여기서 실제 좌표
        # 규약에 맞는 frame_id를 명시적으로 붙여야 한다.
        self.declare_parameter('path_frame_id', 'camera_link')

        self._enabled = bool(self.get_parameter('enabled').value)
        if not self._enabled:
            self.get_logger().info(
                "enabled=False — gradient/slope/경로탐색 연산 안 함(구독만 걸림). "
                "켜려면 'enabled' 파라미터를 true로.")

        self._res: float = self.get_parameter('resolution_m').value
        self._elevation_median_window_cells: int = int(
            self.get_parameter('elevation_median_window_cells').value)
        self._elevation_smoothing_sigma_m: float = float(
            self.get_parameter('elevation_smoothing_sigma_m').value)
        self._max_slope_deg: float = self.get_parameter('max_slope_deg').value
        self._lateral_clearance_m: float = self.get_parameter('lateral_clearance_m').value
        self._min_valid_neighbors: int = int(self.get_parameter('min_valid_neighbors').value)
        self._slope_cost_weight: float = float(self.get_parameter('slope_cost_weight').value)
        self._centerline_cost_weight: float = float(
            self.get_parameter('centerline_cost_weight').value)
        flat_drive_path_topic = str(self.get_parameter('flat_drive_path_topic').value)
        self._path_frame_id: str = str(self.get_parameter('path_frame_id').value)
        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)

        self._bridge = CvBridge()
        self._timings: list[tuple] = []
        self._latest_centerline_xy: list[tuple[float, float]] = []

        qos = SENSOR_DATA_QOS_DEPTH1

        self._sub = self.create_subscription(
            Image, '/terrain/elevation_map', self._on_elevation, qos)
        self._flat_path_sub = self.create_subscription(
            Path, flat_drive_path_topic, self._on_flat_drive_path, 10)

        self._pubs = {
            'gx':        self.create_publisher(Image, '/terrain/gradient_x', 10),
            'gy':        self.create_publisher(Image, '/terrain/gradient_y', 10),
            'magnitude': self.create_publisher(Image, '/terrain/gradient_magnitude', 10),
            'direction': self.create_publisher(Image, '/terrain/gradient_direction', 10),
            'slope_deg': self.create_publisher(Image, '/terrain/slope_deg', 10),
        }
        self._path_pub = self.create_publisher(Path, '/terrain/planned_path', 10)

        self._bench_timer = self.create_timer(bench_period, self._log_timing)
        self.get_logger().info(
            f'GradientMapNode ready — resolution={self._res} m, '
            f'lateral_clearance={self._lateral_clearance_m} m, '
            f'listening on /terrain/elevation_map'
        )

        # 'enabled'를 재시작 없이 ros2 param set으로 켜고 끌 수 있게 함.
        self.add_on_set_parameters_callback(self._on_parameter_update)

    # ------------------------------------------------------------------

    def _on_parameter_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'enabled':
                self._enabled = bool(p.value)
                self.get_logger().info(
                    f"enabled -> {self._enabled} (ros2 param set)")
        return SetParametersResult(successful=True)

    def _on_flat_drive_path(self, msg: Path) -> None:
        self._latest_centerline_xy = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def _on_elevation(self, msg: Image) -> None:
        if not self._enabled:
            return
        t0 = time.perf_counter()

        try:
            elevation = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='32FC1').astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return

        t1 = time.perf_counter()

        # 순서 중요: median 먼저(단일 셀 이상치 제거) -> gaussian(잔주름 완화).
        # 반대로 하면 가우시안이 이상치를 먼저 주변으로 퍼뜨려서 median이
        # 걸러낼 "단일 셀"이 아니게 돼버린다.
        elevation_median = median_filter_elevation(
            elevation, self._elevation_median_window_cells)
        elevation_smoothed = smooth_elevation(
            elevation_median, self._elevation_smoothing_sigma_m, self._res)
        gx, gy, mag, direction, slope_deg = compute_gradient_field(
            elevation_smoothed, self._res)

        h, w = slope_deg.shape
        start = (h // 2, 0)   # 로봇 현재 위치: 좌우 중앙, 그리드 최근접 열
        target_col = w - 1    # 최대한 전방(+x)으로 이동

        # 경로 계획에만 신뢰도/폭 여유 전처리를 적용한다 — /terrain/slope_deg로
        # 발행하는 slope_deg 원본은 이 전처리 이전 값 그대로 써야 하므로
        # (아래에서 arrays['slope_deg']=slope_deg로 그대로 발행됨) 여기서
        # 별도 변수(slope_deg_for_planning)로만 만들어서 plan_path_on_slope_field
        # 호출에만 쓴다. 순서: 고립 셀(노이즈 의심) 마스킹 → 폭 여유 마스킹.
        slope_deg_confident = mask_low_confidence_cells(
            slope_deg, self._min_valid_neighbors)
        margin_cells = int(math.ceil(self._lateral_clearance_m / self._res))
        slope_deg_for_planning = apply_lateral_clearance(
            slope_deg_confident, self._max_slope_deg, margin_cells)
        centerline_row_by_col = centerline_row_targets(
            self._latest_centerline_xy, h, w, self._res)
        path_cells = plan_path_on_slope_field(
            slope_deg_for_planning, start, target_col, self._max_slope_deg,
            self._slope_cost_weight, centerline_row_by_col, self._centerline_cost_weight)

        t2 = time.perf_counter()

        arrays = {
            'gx': gx,
            'gy': gy,
            'magnitude': mag,
            'direction': direction,
            'slope_deg': slope_deg,
        }
        for key, arr in arrays.items():
            out = self._bridge.cv2_to_imgmsg(
                arr.astype(np.float32), encoding='32FC1')
            out.header = msg.header
            self._pubs[key].publish(out)

        if path_cells is not None:
            self._path_pub.publish(
                self._cells_to_path_msg(path_cells, h, msg.header))
        else:
            # plan_path_on_slope_field()가 이제 target_col을 못 채워도 도달
            # 가능한 데까지의 최선 경로를 반환하므로, 여기 None은 로봇 현재
            # 위치(start) 자체가 막혀 있다는 뜻 — "짧게라도 갈 데가 없다".
            self.get_logger().warn(
                f'Start cell impassable (max_slope_deg={self._max_slope_deg}°) '
                '— no path, not even a short one.',
                throttle_duration_sec=2.0)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode
            (t2 - t1) * 1e3,   # compute
            (t3 - t2) * 1e3,   # encode+publish
            (t3 - t0) * 1e3,   # total
        ))

    def _cells_to_path_msg(self, cells, grid_h: int, source_header) -> Path:
        """Convert (row, col) grid cells to a metric nav_msgs/Path.

        x = col * resolution (forward distance from robot)
        y = (center_row - row) * resolution (left of centerline is +y,
            matching the gy = dH/dy convention documented at module top)

        source_header(입력 elevation map의 Header, optical frame)를 그대로
        재사용하지 않는다 — 위 x/y는 body 좌표라 frame_id가 안 맞는다. 새
        Header를 만들어 stamp만 유지하고(TF 시간 정합용) frame_id는
        path_frame_id 파라미터로 명시한다. source_header 객체 자체는 절대
        수정하지 않는다 (참조 공유 시 구독 메시지까지 오염될 수 있음).
        """
        center_row = grid_h // 2

        path_header = Header()
        path_header.stamp = source_header.stamp
        path_header.frame_id = self._path_frame_id

        path_msg = Path()
        path_msg.header = path_header
        for row, col in cells:
            pose = PoseStamped()
            pose.header = path_header
            pose.pose.position.x = float(col * self._res)
            pose.pose.position.y = float((center_row - row) * self._res)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def _log_timing(self) -> None:
        if not self._timings:
            return
        arr = np.array(self._timings, dtype=np.float64)
        mean = arr.mean(axis=0)
        p95 = np.percentile(arr, 95, axis=0)
        self.get_logger().info(
            f'[bench {len(self._timings)} frames] '
            f'mean  decode={mean[0]:.2f} compute={mean[1]:.2f} '
            f'pub={mean[2]:.2f} total={mean[3]:.2f} ms | '
            f'p95_total={p95[3]:.2f} ms'
        )
        self._timings.clear()


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GradientMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
