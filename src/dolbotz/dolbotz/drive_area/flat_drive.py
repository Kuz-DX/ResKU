"""
평지 주행가능영역 추종 노드 — segmentation.py가 발행하는 "주행가능영역" 마스크의
중심선을 코스 경계를 벗어나지 않는 목표경로로 게시한다.

세그멘테이션 추론은 이 노드가 하지 않는다 — segmentation.py가 한 번만 돌려서
/perception/drivable_mask로 내보내고, flat_drive.py는 그걸 구독만 한다
(flat_drive/elevation_map/slope_decision이 각자 YOLO를 돌리면 임베디드에서
추론이 중복되는 걸 피하기 위함). undistort는 이 노드가 자체적으로 계속 함 —
/bev/image, /bev/debug_overlay 디버그 발행에 undistort된 원본 컬러 이미지가
필요하기 때문. 마스크와 이 노드가 받는 RGB 프레임은 서로 다른 메시지라
카메라 fps만큼의 지연이 있을 수 있다(완전히 동기화되지 않음).

gradient_map.py(경사 구간)와 역할 경계가 동일하다: 이 노드는 좌표(경로)만
계산해서 발행하고, 실제 로봇 제어(속도/조향 명령)나 평지/경사 전환 판단은
하지 않는다 — 항상 "지금 보이는 화면 기준 주행가능영역 경로"만 계산한다.

순수 계산 부분(호모그래피 유도, 세그멘테이션 마스크 -> BEV -> centerline)은
ROS에 의존하지 않으므로 단독으로 단위 테스트 가능하다 (test/test_flat_drive.py).

좌표 규약 (elevation_map.py/gradient_map.py와 일치): world-level(=body-level)
프레임은 카메라 위치를 원점으로 하며 x=전방, y=왼쪽, z=위이다.

IMU 자세보정: 이 노드의 IMU는 elevation_map.py와 동일한 카메라 내장 IMU이다
(별도 섀시 장착 IMU 아님, frame_id 'camera_*_optical_frame'). orientation
필드는 항상 무효값(0,0,0,0)이므로 절대 사용하지 않는다 — linear_acceleration/
angular_velocity만으로 dolbotz.utils.attitude의 상보필터를 돌려 roll/pitch를
추정한다. 이 추정치는 이미 마운트+섀시 결합 총 기울기이므로, 한 번만 되돌리면
world-level에 도달한다 (elevation_map.py의 camera_body_to_level_matrix()에서
발견/수정된 마운트 오프셋 이중 제거 버그와 동일한 원칙 — 별도로 마운트
오프셋을 다시 빼지 않는다).

구독 토픽:
  /drive/camera/color/image_raw/compressed
                                     sensor_msgs/CompressedImage (JPEG)
  /drive/camera/color/camera_info  sensor_msgs/CameraInfo   (fx,fy,cx,cy,왜곡계수)
  /drive/camera/imu                sensor_msgs/Imu          (원시 자이로 + 가속도만)
  /perception/drivable_mask         sensor_msgs/Image (mono8) — segmentation.py 발행

게시 토픽:
  /flatdrive/planned_path   nav_msgs/Path          목표경로 (x=전방,y=좌측, m)
  /planning/target_point    geometry_msgs/PointStamped  디버그용 첫 waypoint
  /bev/image                sensor_msgs/Image      BEV로 투영한 컬러 이미지 (디버그)
  /bev/mask                 sensor_msgs/Image      BEV로 투영한 세그멘테이션 마스크 (디버그)
  /bev/debug_overlay        sensor_msgs/Image      왜곡보정 원본 + 세그멘테이션 컨투어 (디버그)
  /bev/centerline_overlay   sensor_msgs/Image      BEV 마스크 + 추출된 중심선 (디버그)
  /bev/H                    std_msgs/Float64MultiArray  image->ground 호모그래피 (디버그)

파라미터 (마운트 오프셋/상보필터 alpha 기본값은 dolbotz.utils.attitude의
MOUNT_*_PLACEHOLDER / COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER 상수 참고 —
실측 전 임시값이며 대회장에서 자주 바뀔 수 있어 의도적으로 config/*.yaml이
아니라 코드에 상수로 둔다. elevation_map.py와 동일 파라미터명 유지(같은
카메라이므로 launch에서 값 공유 가능). camera_serial_no로 config/calibration/의
캘리브레이션 피클을 지정하면 그 값이 이 상수보다 우선 적용된다):
  camera_serial_no         str    기본값 ''     — config/calibration/{model}_{serial}.pkl 조회용
  camera_height_m          float  attitude.MOUNT_CAMERA_HEIGHT_M_PLACEHOLDER
  camera_pitch_offset_deg  float  attitude.MOUNT_PITCH_OFFSET_DEG_PLACEHOLDER
  camera_roll_offset_deg   float  attitude.MOUNT_ROLL_OFFSET_DEG_PLACEHOLDER
  complementary_filter_alpha float attitude.COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER
  bev_meters_per_pixel      float  기본값 0.03   — PLACEHOLDER, 6m x 6m 커버리지 기준 재계산값
  bev_img_width             int    기본값 200    — PLACEHOLDER (0.03 * 200 = 6.0 m)
  bev_img_height             int    기본값 200    — PLACEHOLDER (0.03 * 200 = 6.0 m)
  mask_topic                str    기본값 /perception/drivable_mask
  min_row_pixels             int    기본값 10     — PLACEHOLDER, 실측 튜닝 대상
  path_frame_id              str    기본값 camera_link — /flatdrive/planned_path에
                                     실릴 frame_id. 경로 좌표(x=전방,y=좌측)는
                                     body 규약이지 입력 컬러 이미지의 optical
                                     frame(x=오른쪽,y=아래,z=전방, 보통
                                     'camera_color_optical_frame')이 아니므로
                                     이미지 header를 그대로 재사용하면 안 됨.
  bench_log_hz               float  기본값 1.0
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Header
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation
from scipy.interpolate import splprep, splev

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.attitude import (
    R_BODY_TO_OPTICAL,
    optical_vector_to_body,
    resolve_mount_defaults,
    update_complementary_filter,
)
from dolbotz.utils.regulations import TRACK_WIDTH_M
# slope_decision.py의 lateral_occlusion_guard와 같은 발상(트랙 옆 구조물이
# 마스크를 끊어놓는 문제)의 mask-only 버전에 재사용 — 순수 함수라 의존성
# 걱정 없음. lateral_gap_correction 절 참고.
from dolbotz.drive_area.slope_decision import find_lateral_gap
# PLACEHOLDER — 실측/튜닝 전 임시값. 세그멘테이션 마스크의 row별 유효 픽셀 수가
# 이보다 적으면 그 row는 경로에서 건너뛴다 (bev_mask_to_centerline_path 참고).
MIN_ROW_PIXELS_PLACEHOLDER = 10
# lateral_gap_correction(기본 꺼짐, mission_summer에서만 켬)용 기본값 — BEV
# 픽셀 단위(원본 이미지 픽셀이 아님, bev_meters_per_pixel 기준). 이보다 넓게
# 끊긴 row는 구조물이 트랙 내부를 가로막은 것으로 보고, 마스크 픽셀 수가
# 많은 쪽으로 쏠리는 mean() 대신 바깥쪽 두 edge의 중점을 센터로 쓴다.
LATERAL_GAP_MIN_PX_PLACEHOLDER = 6


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음 (지면 호모그래피)
# ---------------------------------------------------------------------------
#
# 좌표 규약 (elevation_map.py/gradient_map.py와 일치): world-level(=body-level)
# 프레임은 카메라 위치를 원점으로 하며 x=전방, y=왼쪽, z=위이다. 지면은
# z=-camera_height_m 평면으로 취급한다 (elevation_map.py의
# `elevation = pts_level[:,2] + camera_height_m` 규약과 동일 — 지면
# elevation=0은 z=-camera_height_m에 대응).
#
# 이 노드의 IMU는 elevation_map.py와 동일한 카메라 내장 IMU이므로 동일한
# 원칙이 적용된다: roll_meas/pitch_meas(상보필터 추정치)는 이미 마운트+섀시
# 결합 총 기울기이며, 한 번만 되돌리면 world-level에 도달한다.
# camera_body_to_level_matrix()의 관계를 그대로 재사용한다:
#   R_meas := Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0])
#   accel_camera_frame = R_meas.inv().apply(accel_world_level)
# 즉 R_meas.inv()가 world-level → camera-현재-body 프레임 회전이다.


def ground_to_image_homography(
    camera_matrix: np.ndarray,
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
    camera_height_m: float,
) -> np.ndarray:
    """지면(z=-camera_height_m 평면) 위의 world-level 좌표 (x_forward, y_left)를
    원본(왜곡보정된) 카메라 이미지의 동차 픽셀 좌표로 매핑하는 3x3 호모그래피
    H_g2i를 핀홀 투영식으로부터 처음부터 유도한다.

    유도:
      P_level = [x_forward, y_left, -camera_height_m]  (지면 위 점, 카메라 위치 원점)
      P_body_current = R_meas.inv().apply(P_level)      (world-level → 카메라 현재 body 프레임)
      P_optical = R_BODY_TO_OPTICAL @ P_body_current    (body → optical, 검증된 상수)
      [u,v,w] ~ camera_matrix @ P_optical                (표준 핀홀 투영)

    A := R_BODY_TO_OPTICAL @ R_meas.inv().as_matrix() 로 두면
    P_optical = x_forward*A[:,0] + y_left*A[:,1] - camera_height_m*A[:,2] 이므로
      H_g2i = camera_matrix @ [A[:,0], A[:,1], -camera_height_m*A[:,2]]
    (열벡터 3개를 나열한 3x3 행렬).

    roll_offset/pitch_offset은 camera_body_to_level_matrix()와 동일한 이유로
    이 계산에 쓰이지 않는다 — roll_meas/pitch_meas가 이미 마운트+섀시 결합
    총 기울기이므로 마운트 오프셋을 별도로 다시 반영하면 존재하지 않는
    성분을 중복 제거/추가하는 오차가 생긴다. 호출 시그니처는 마운트 오프셋
    값을 호출부에 명시적으로 남겨두기 위해 유지한다.
    """
    del roll_offset, pitch_offset  # 의도적으로 미사용 — 위 docstring 참고
    r_meas_inv = Rotation.from_euler('xyz', [roll_meas, pitch_meas, 0]).inv()
    a = R_BODY_TO_OPTICAL @ r_meas_inv.as_matrix()
    ground_cols = np.column_stack([a[:, 0], a[:, 1], -camera_height_m * a[:, 2]])
    return camera_matrix @ ground_cols


def bev_ground_projection_matrix(
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> np.ndarray:
    """world-level 지면 좌표 (x_forward, y_left)를 BEV 픽셀 (col, row)로 매핑하는
    3x3 아핀 행렬 M.

    BEV 이미지 레이아웃(일반적인 주행 BEV 관례 — 로봇이 이미지 하단 중앙에
    위치, 전방이 이미지 위쪽으로 멀어짐):
      col(가로) = bev_width_px/2  - y_left / mpp   (y_left>0=왼쪽 → col 감소, 즉 이미지 왼쪽)
      row(세로) = bev_height_px   - x_forward / mpp (x_forward↑ → row 감소, 즉 이미지 위쪽)

    row=bev_height_px-1(하단)이 로봇 바로 앞(x_forward≈0), row=0(상단)이
    가장 먼 전방이다. 이 레이아웃 덕분에 "row별로 훑으며 좌우(col) 중심을
    구해 전방 경로점으로 삼는다"는 이후 단계(bev_mask_to_centerline_path)가
    자연스럽게 성립한다.
    """
    mpp = bev_meters_per_pixel
    return np.array([
        [0.0, -1.0 / mpp, bev_width_px / 2.0],
        [-1.0 / mpp, 0.0, float(bev_height_px)],
        [0.0, 0.0, 1.0],
    ])


def bev_pixel_to_meters(
    row: float,
    col: float,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> tuple[float, float]:
    """bev_ground_projection_matrix()의 역변환: BEV 픽셀 (row, col) -> world-level
    미터 좌표 (x_forward, y_left)."""
    mpp = bev_meters_per_pixel
    x_forward = (bev_height_px - row) * mpp
    y_left = (bev_width_px / 2.0 - col) * mpp
    return float(x_forward), float(y_left)


def image_to_bev_homography(
    camera_matrix: np.ndarray,
    roll_meas: float,
    pitch_meas: float,
    roll_offset: float,
    pitch_offset: float,
    camera_height_m: float,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
) -> np.ndarray:
    """원본 이미지 픽셀 -> BEV 픽셀 순방향 호모그래피 (cv2.warpPerspective(src, H,
    (bev_width_px, bev_height_px))에 바로 사용). ground_to_image_homography()의
    역행렬(image->ground)에 bev_ground_projection_matrix()(ground->bev)를
    합성한다."""
    h_g2i = ground_to_image_homography(
        camera_matrix, roll_meas, pitch_meas, roll_offset, pitch_offset, camera_height_m)
    h_i2g = np.linalg.inv(h_g2i)
    m = bev_ground_projection_matrix(bev_width_px, bev_height_px, bev_meters_per_pixel)
    return m @ h_i2g


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음 (세그멘테이션 마스크 -> BEV -> centerline 경로)
# ---------------------------------------------------------------------------

def mask_to_bev(
    mask: np.ndarray,
    homography: np.ndarray,
    bev_width_px: int,
    bev_height_px: int,
) -> np.ndarray:
    """단일 채널 바이너리(0/255) 마스크를 순방향 호모그래피로 BEV 평면에 투영한다.

    cv2.warpPerspective의 얇은 래퍼. 반드시 원본(왜곡보정된) 원근 이미지
    좌표계의 마스크를 입력으로 받아야 한다 — 세그멘테이션 모델은 일반 원근
    시점 이미지로 학습되었으므로, BEV로 먼저 왜곡한 이미지에 추론을 돌리면
    학습 분포를 벗어나 결과가 나빠진다. 따라서 추론은 항상 원본 이미지에서
    먼저 수행하고, 그 결과 마스크만 이 함수로 BEV에 투영한다.
    마스크는 이진값이므로 보간으로 인한 경계 흐림을 피하기 위해
    INTER_NEAREST를 사용한다.
    """
    return cv2.warpPerspective(
        mask, homography, (bev_width_px, bev_height_px), flags=cv2.INTER_NEAREST)


def mask_row_width_plausible(
    cols: np.ndarray,
    meters_per_pixel: float,
    expected_width_m: float,
    tolerance_factor: float,
) -> bool:
    """켜진 픽셀들의 열 범위(cols[0]~cols[-1], inclusive)가 만드는 실측 폭이
    expected_width_m * tolerance_factor를 넘으면 세그멘테이션이 트랙이
    아닌 배경까지 같이 잡은(오탐) 것으로 보고 False를 반환한다.

    반대로 더 좁은 건(가려짐, 원거리 부분 가시성, 코너 등으로) 정상적일
    수 있으므로 하한은 검사하지 않는다 — 상한만 본다. cols가 비어있으면
    (호출부가 min_row_pixels로 먼저 걸러내는 게 정상이지만) 방어적으로
    False.
    """
    if cols.size == 0:
        return False
    span_m = (float(cols[-1]) - float(cols[0]) + 1.0) * meters_per_pixel
    return span_m <= expected_width_m * tolerance_factor


# expected_track_width_m 검사를 비활성화했을 때만 사용하는 폴백값. 정상
# 설정에서는 한쪽 경계가 잘리면 예상 트랙 폭의 절반을 사용한다.
ROBOT_HALF_WIDTH_M = 0.3


def bev_mask_to_centerline_path(
    bev_mask: np.ndarray,
    min_row_pixels: int,
    bev_width_px: int,
    bev_height_px: int,
    bev_meters_per_pixel: float,
    expected_track_width_m: float | None = None,
    track_width_tolerance_factor: float = 1.5,
    robot_half_width_m: float = ROBOT_HALF_WIDTH_M,
    lateral_gap_correction: bool = False,
    lateral_gap_min_px: int = LATERAL_GAP_MIN_PX_PLACEHOLDER,
) -> list[tuple[float, float]]:
    """BEV 마스크에서 row(전방 거리)별 좌우 중심(centroid) 경로를 추출한다.

    bev_ground_projection_matrix()의 레이아웃(row가 작을수록 더 먼 전방,
    row=bev_height_px-1이 로봇 바로 앞)을 그대로 따라서, 로봇에 가까운
    row부터 먼 row 순서로 훑는다. 각 row에서 마스크가 켜진(>0) 픽셀 개수가
    min_row_pixels 미만이면 그 row는 건너뛴다(구멍이 있는 경로로 남음 —
    보간/채움을 하지 않는다). expected_track_width_m이 주어지면
    (dolbotz.utils.regulations.TRACK_WIDTH_M 등, 규정집 트랙 폭) 그 row의
    폭이 mask_row_width_plausible()을 통과하지 못하는 경우도 같은 방식으로
    건너뛴다 — 세그멘테이션이 트랙 아닌 영역까지 넓게 잡아서 centroid가
    조용히 틀어지는 걸 막기 위함(None이면 이 검사 자체를 끔, 기본값).

    양쪽 경계가 다 보이는 row는 켜진 픽셀들의 열(column) 평균을 좌우 중심으로
    쓴다. 한쪽 경계가 BEV 프레임 밖으로 잘려서(cols[0]==0 또는
    cols[-1]==bev_width_px-1) 안 보이는 row는 그 centroid가 신뢰 불가이므로,
    보이는 쪽 경계에서 로컬 진행방향(같은 쪽 경계의 직전 row 대비 접선벡터)의
    법선방향으로 expected_track_width_m의 절반만큼 오프셋한 점을 쓴다.
    expected_track_width_m=None이면 하위호환용 robot_half_width_m를 사용하며,
    양쪽 다 잘리면(정보 없음) centroid로 폴백한다.

    lateral_gap_correction=True(기본 꺼짐)면, 트랙 "안쪽"에 구조물이 있어서
    row 픽셀이 두 세그먼트로 끊긴 경우(find_lateral_gap 참고)를 추가로
    본다 — 이 경우 mean()을 그대로 쓰면 픽셀 수가 많은 쪽 세그먼트로
    centroid가 쏠려서 경로가 한쪽으로 치우친다.
    양쪽 진짜 트랙 edge(cols[0], cols[-1])는 둘 다 보이는 상태이므로, 그
    둘의 중점을 쓰면 세그먼트 크기 불균형에 안 끌려간다. depth가 없어서
    "진짜 구조물 vs 원래 트랙이 좁아지는 구간"을 구분할 수 없으므로 —
    안전을 위해 요청받은 미션(여름)에서만 켜서 쓸 것.

    Returns:
        (x_forward, y_left) 미터 좌표 튜플의 리스트, 로봇에 가까운 순서부터.
        min_row_pixels(및 폭 검사)를 만족하는 row가 하나도 없으면 빈 리스트.
    """
    center_offset_m = (
        expected_track_width_m / 2.0
        if expected_track_width_m is not None
        else robot_half_width_m)
    half_width_px = center_offset_m / bev_meters_per_pixel
    prev_edge: dict[str, tuple[float, float]] = {}

    path: list[tuple[float, float]] = []
    for row in range(bev_height_px - 1, -1, -1):
        row_on = bev_mask[row] > 0
        if int(np.count_nonzero(row_on)) < min_row_pixels:
            continue
        cols = np.flatnonzero(row_on)
        if expected_track_width_m is not None and not mask_row_width_plausible(
                cols, bev_meters_per_pixel, expected_track_width_m, track_width_tolerance_factor):
            continue

        gap = (
            find_lateral_gap(row_on, lateral_gap_min_px)
            if lateral_gap_correction else None)

        left_truncated = cols[0] == 0
        right_truncated = cols[-1] == bev_width_px - 1

        if gap is not None:
            col, side, inward = float((cols[0] + cols[-1]) / 2.0), None, 0.0
        elif left_truncated and not right_truncated:
            col, side, inward = float(cols[-1]), 'right', -1.0
        elif right_truncated and not left_truncated:
            col, side, inward = float(cols[0]), 'left', 1.0
        else:
            col, side, inward = float(cols.mean()), None, 0.0

        if side is None:
            row_out, col_out = float(row), col
        else:
            prev = prev_edge.get(side)
            t_row, t_col = (row - prev[0], col - prev[1]) if prev is not None else (-1.0, 0.0)
            t_mag = float(np.hypot(t_row, t_col))
            if t_mag < 1e-6:
                t_row, t_col, t_mag = -1.0, 0.0, 1.0
            t_row, t_col = t_row / t_mag, t_col / t_mag
            n_row, n_col = -t_col, t_row
            if n_col * inward < 0:
                n_row, n_col = -n_row, -n_col
            row_out = row + n_row * half_width_px
            col_out = col + n_col * half_width_px
            prev_edge[side] = (row, col)

        path.append(bev_pixel_to_meters(
            row_out, col_out, bev_width_px, bev_height_px, bev_meters_per_pixel))
    return path


# 기본값 5row. B-spline 보간 전에 먼저 이 함수로
# 훑는다. 대부분의 gap은 세그멘테이션 노이즈로 row 한두 개가 빠진 것뿐이라
# 스플라인으로 이어도 안전하지만, 트랙 내부 구조물/시야 밖처럼 "거기 뭐가
# 있는지 실측 근거가 아예 없는" 긴 구간까지 곡선으로 이어버리면 실측 없는
# 곳으로 경로를 지어내는 셈이 된다 — 그래서 짧은 구멍만 보간 대상으로
# 남기고, 긴 구멍을 만나면 그 지점에서 경로 자체를 끊는다(그 이후, 즉 더
# 먼 쪽 점들은 버림 — 안 뻗는 쪽이 안전).
MAX_GAP_ROWS_PLACEHOLDER = 5


def truncate_at_large_gap(
    path_points: list[tuple[float, float]],
    bev_meters_per_pixel: float,
    max_gap_rows: int,
) -> list[tuple[float, float]]:
    """path_points(로봇에 가까운 순서, x_forward 오름차순)를 훑다가 연속된
    두 점 사이의 row 간격이 max_gap_rows를 넘으면 그 지점에서 잘라낸다 —
    그 뒤(더 먼 쪽) 점들은 버린다. bev_mask_to_centerline_path()가 row를
    건너뛸 때 row 인덱스 자체를 남기지 않으므로, 연속 두 점의 x_forward
    차이를 bev_meters_per_pixel로 나눠 몇 row가 비었는지 역산한다."""
    if not path_points:
        return []
    kept = [path_points[0]]
    for prev, cur in zip(path_points, path_points[1:]):
        gap_rows = round((cur[0] - prev[0]) / bev_meters_per_pixel) - 1
        if gap_rows > max_gap_rows:
            break
        kept.append(cur)
    return kept


# bridge_interior_gaps()가 큰 gap의 앞/뒤 구간에서 방향을 추정할 때 쓰는
# 점 개수 — EXTEND_FIT_POINTS와 같은 이유(노이즈 완화)로 여러 점을 쓴다.
BRIDGE_FIT_POINTS = 5


def bridge_interior_gaps(
    path_points: list[tuple[float, float]],
    bev_meters_per_pixel: float,
    max_gap_rows: int,
    fit_points: int = BRIDGE_FIT_POINTS,
) -> list[tuple[float, float]]:
    """truncate_at_large_gap()과 달리, 중간의 큰 구멍(자갈 구간 등)을 만나도
    경로를 거기서 끊지 않고 gap 앞쪽 마지막 fit_points개 점과 gap 뒤쪽
    (더 먼 쪽) 첫 fit_points개 점을 각각 최소제곱 직선으로 피팅해, 그 두
    직선이 만나는 지점(또는 안 만나면 중점)을 지나는 직선 보간으로 이어
    붙인다. 자갈이 실제 주행 가능 구간이라 트랙 전체가 이어져 있다는 걸
    아는 여름 미션에서만 쓴다(사용자 요청, 2026-09-05) — 실측 근거가 아예
    없는 구간까지 지어내는 게 위험한 다른 미션은 여전히
    truncate_at_large_gap()을 쓴다.

    path_points가 비어있으면 빈 리스트, 큰 gap이 없으면 원본 그대로 반환.
    gap이 맨 앞(로봇 바로 앞)에 있어 앞쪽 피팅에 쓸 점이 없으면 그 gap은
    건너뛴다(extend_path_to_robot()이 별도로 처리하는 영역이라 여기서
    중복 처리하지 않음).
    """
    if not path_points:
        return []

    result = [path_points[0]]
    i = 1
    n = len(path_points)
    while i < n:
        prev = path_points[i - 1]
        cur = path_points[i]
        gap_rows = round((cur[0] - prev[0]) / bev_meters_per_pixel) - 1
        if gap_rows <= max_gap_rows:
            result.append(cur)
            i += 1
            continue

        # 큰 gap 발견 — result(현재까지 이어붙인 쪽)의 뒤쪽 fit_points개와
        # path_points의 i부터 이어지는 앞쪽 fit_points개로 각각 직선을
        # 피팅해서 gap을 잇는다.
        before = result[-fit_points:]
        after = path_points[i:i + fit_points]
        if len(before) < 2 or len(after) < 2:
            # 피팅할 점이 부족(예: 로봇 바로 앞 gap) — 여기서는 못 잇고
            # 기존 truncate_at_large_gap()과 동일하게 그 지점에서 끊는다.
            break

        bx = np.array([p[0] for p in before], dtype=float)
        by = np.array([p[1] for p in before], dtype=float)
        ax = np.array([p[0] for p in after], dtype=float)
        ay = np.array([p[1] for p in after], dtype=float)

        if float(np.ptp(bx)) > 1e-9:
            b_slope, b_intercept = np.polyfit(bx, by, 1)
        else:
            b_slope, b_intercept = 0.0, float(by[-1])
        if float(np.ptp(ax)) > 1e-9:
            a_slope, a_intercept = np.polyfit(ax, ay, 1)
        else:
            a_slope, a_intercept = 0.0, float(ay[0])

        # 두 피팅선이 이미 이어붙인 쪽 끝점(prev)에서 gap 건너 첫 점(cur)의
        # x_forward까지 부드럽게 전환하도록, 각 x에서 두 직선의 가중평균을
        # 쓴다(앞쪽 직선 가중치는 gap 시작에서 1 -> gap 끝에서 0으로 선형
        # 감소) — 양쪽 접선 방향을 모두 반영한 완만한 연결선이 된다.
        x0, x1 = prev[0], cur[0]
        span = x1 - x0
        n_extra = max(1, round(span / bev_meters_per_pixel)) - 1
        for k in range(1, n_extra + 1):
            x = x0 + span * k / (n_extra + 1)
            w = 1.0 - k / (n_extra + 1)  # 1(before 쪽) -> 0(after 쪽)
            y_before = b_slope * x + b_intercept
            y_after = a_slope * x + a_intercept
            y = w * y_before + (1.0 - w) * y_after
            result.append((x, y))

        result.append(cur)
        i += 1

    return result


# 로봇 바로 앞(x_forward가 이 값 이하) 구간에 유효 점이 하나도 없으면
# "하단 결측"으로 보고 extend_path_to_robot()을 적용한다. ROBOT_HALF_WIDTH_M과
# 같은 값(로봇 반폭 정도) — 사용자 요청(여름 미션, 2026-09-05)에 따름.
EXTEND_TO_ROBOT_GAP_M = 0.3

# extend_path_to_robot()이 "자갈 너머 경로"의 중심/방향을 추정할 때 쓰는
# 앞쪽(로봇에서 먼 쪽) 점 개수. 1~2개짜리 첫점만 쓰면 세그멘테이션 노이즈로
# 반대쪽으로 튀는 조향이 나올 수 있어(2026-09-05, 사용자 지적) 여러 점을
# 최소제곱 직선 피팅해서 완만하게 추정한다.
EXTEND_FIT_POINTS = 5


def extend_path_to_robot(
    path_points: list[tuple[float, float]],
    point_spacing_m: float,
    min_gap_m: float = EXTEND_TO_ROBOT_GAP_M,
    fit_points: int = EXTEND_FIT_POINTS,
) -> list[tuple[float, float]]:
    """BEV 하단(로봇 근처, x_forward가 작은 쪽)에 세그멘테이션 유효 row가 없어서
    path_points의 첫 점이 로봇에서 min_gap_m 넘게 떨어져 있으면, 로봇(BEV 하단
    중앙, x_forward=0, y_left=0)부터 자갈 너머 첫 번째로 신뢰 가능한 경로
    구간까지 직선으로 연결해 구멍을 메운다.

    방향/목표점은 path_points 맨 앞쪽(로봇에서 먼 쪽 끝, 자갈을 갓 벗어난
    지점) 최대 fit_points개 점에 1차 최소제곱 직선을 피팅해서 구한다 — 점
    1~2개만 보면 세그멘테이션 노이즈 한 프레임에 연장선 전체가 휘둘려
    반대쪽으로 조향하는 사고가 날 수 있어(2026-09-05, 사용자 지적), 여러
    점으로 완만하게 추정한다. 이 피팅선 위의(원본 y_left가 아닌) 값을
    연결 끝점으로 써서 노이즈를 추가로 한 번 더 누른다.

    시작점은 항상 로봇 원점(0, 0)으로 고정한다 — 로봇이 실제로 서 있는
    화면 하단 중앙에서 출발해야 하며, 상단 경로의 접선을 로봇 위치까지
    수학적으로 연장한 값(0이 아닐 수 있음)을 쓰지 않는다(이전 구현의
    문제점 — 2026-09-05 사용자 지적).

    path_points가 비어있거나 이미 로봇 근처(첫 점이 min_gap_m 이내)까지
    덮여 있으면 그대로 반환 — 원래 계산된 경로를 건드리지 않는다.

    시작 각도(로봇 헤딩 대비)에는 별도 상한을 두지 않는다 — 급격한 꺾임은
    호출부의 smooth_path_bspline()이 스무딩으로 완화한다(사용자 결정,
    2026-09-05).
    """
    if not path_points or path_points[0][0] <= min_gap_m:
        return path_points

    n_fit = min(fit_points, len(path_points))
    xs = np.array([p[0] for p in path_points[:n_fit]], dtype=float)
    ys = np.array([p[1] for p in path_points[:n_fit]], dtype=float)

    if n_fit >= 2 and float(np.ptp(xs)) > 1e-9:
        slope, intercept = np.polyfit(xs, ys, 1)
    else:
        # 점이 하나뿐이거나 fit 구간의 x_forward가 사실상 동일한(옆으로만
        # 도는) 퇴화 케이스 — 방향 정보가 없으므로 그 점의 y_left를 유지한
        # 평평한 선으로 폴백.
        slope, intercept = 0.0, float(ys[0])

    x_end = float(path_points[0][0])
    y_end = float(slope * x_end + intercept)

    n_extra = max(1, int(round(x_end / point_spacing_m)))
    extension = []
    for i in range(n_extra):
        t = i / n_extra  # 0(로봇)에서 1(x_end 직전)까지
        x = x_end * t
        y = y_end * t  # 로봇 원점(0,0) -> (x_end, y_end) 직선 보간
        extension.append((x, y))
    return extension + path_points


def smooth_path_bspline(
    path_points: list[tuple[float, float]],
    point_spacing_m: float,
    degree: int = 3,
) -> list[tuple[float, float]]:
    """path_points를 호 길이(arc-length)로 파라미터화한 B-spline으로 약하게
    스무딩(s>0)해서 보간한 뒤, point_spacing_m 간격으로 재샘플링한
    (x_forward, y_left) 리스트를 돌려준다.

    row 단위로 뽑힌 원본 점들은 BEV 그리드 해상도(bev_meters_per_pixel)만큼의
    양자화 노이즈로 계단 형태를 띤다. s=0(원본 점을 정확히 지나는 보간)은
    그 계단 노이즈까지 그대로 따라가 버리므로, s를 point_spacing_m 기반으로
    잡아 눌러준다(아래 s 계산 참고) — 이제는 원본을 정확히 지나가는 보간이
    아니라 약한 스무딩을 적용한 근사다.

    (x, y) 쌍을 파라미터 t의 함수로 각각 피팅하는 parametric spline이라
    (splprep), y=f(x) 형태의 다항식 피팅과 달리 x_forward가 완전히
    단조증가가 아니어도(급커브 등) 깨지지 않는다.

    점 개수가 degree+1 미만이면(스플라인 적합 불가) 원본을 그대로
    돌려준다 — 억지로 무리한 차수로 피팅하지 않고, 데이터 부족한 채로
    스플라인을 "지어내지" 않는다."""
    n = len(path_points)
    if n < degree + 1:
        return list(path_points)

    xs = np.array([p[0] for p in path_points], dtype=np.float64)
    ys = np.array([p[1] for p in path_points], dtype=np.float64)

    try:
        # scipy splprep 공식 권장치: s ~= n * (예상 좌표 노이즈 표준편차)^2.
        # point_spacing_m(재샘플링 후 우리가 신뢰하는 위치 정밀도 스케일)을
        # 그 노이즈 표준편차 근사로 그대로 쓴다 — 임의 상수 아님.
        smoothing = n * (point_spacing_m ** 2)
        tck, _ = splprep([xs, ys], k=degree, s=smoothing)
    except Exception:
        return list(path_points)  # 중복점 등 특이 케이스 — 안전하게 원본 폴백

    arc_length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    n_samples = max(2, int(round(arc_length / point_spacing_m)) + 1)
    u_fine = np.linspace(0.0, 1.0, n_samples)
    x_fine, y_fine = splev(u_fine, tck)
    return list(zip(x_fine.tolist(), y_fine.tolist()))


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class FlatDriveNode(Node):
    """ROS2 wrapper: color+camera_info+imu를 구독해 /flatdrive/planned_path를 게시한다."""

    def __init__(self):
        super().__init__('flat_drive_node')

        self.declare_parameter(
            'color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/drive/camera/color/camera_info')
        self.declare_parameter('imu_topic', '/drive/camera/imu')

        # 마운트 파라미터 기본값: 캘리브레이션 피클 > MOUNT_*_PLACEHOLDER 순
        # (resolve_mount_defaults). --ros-args로 넘기면 항상 최우선.
        self.declare_parameter('camera_serial_no', '')
        serial_no = str(self.get_parameter('camera_serial_no').value)
        mount_defaults = resolve_mount_defaults(serial_no)

        self.declare_parameter('camera_height_m', mount_defaults['camera_height_m'])
        self.declare_parameter('camera_pitch_offset_deg', mount_defaults['camera_pitch_offset_deg'])
        self.declare_parameter('camera_roll_offset_deg', mount_defaults['camera_roll_offset_deg'])
        self.declare_parameter('complementary_filter_alpha', mount_defaults['complementary_filter_alpha'])

        # PLACEHOLDER — gradient_map.py의 grid_forward_m=4.0과 커버리지를 맞추는
        # 방향으로 재계산된 임시값 (0.03 m/px * 200 px = 6.0 m 정사각 커버리지).
        # 실측/튜닝 대상.
        self.declare_parameter('bev_meters_per_pixel', 0.03)
        self.declare_parameter('bev_img_width', 200)
        self.declare_parameter('bev_img_height', 200)

        self.declare_parameter('mask_topic', '/perception/drivable_mask')
        self.declare_parameter('min_row_pixels', MIN_ROW_PIXELS_PLACEHOLDER)
        # 규정집 트랙 폭(914.4mm) 기준 mask_row_width_plausible() 검사 —
        # 0 이하로 주면(또는 이 값을 그대로 두되 tolerance_factor를 아주
        # 크게 주면) 사실상 검사 비활성화. 실측 폭이 이보다 tolerance_factor
        # 배 넘게 넓게 잡히는 row는 세그멘테이션 오탐으로 보고 건너뛴다.
        # 2026-08-27 slope_test_0827 bag 실측: 기본값 1.5(=1.372m 상한)에서는
        # 로봇 약 0.8~0.9m 앞 구간의 실측 폭(최대 1.56m, 슬로프 테스트용 실내
        # 세팅이라 규정 트랙보다 넓은 바닥)이 상한을 넘어 그 지점부터
        # truncate_at_large_gap()이 경로를 통째로 잘라버림(0.86m로 단축,
        # 원래는 최소 1.75m 이상 이어지는 걸 확인) — 2.0으로 올려서 해결.
        self.declare_parameter('expected_track_width_m', TRACK_WIDTH_M)
        self.declare_parameter('track_width_tolerance_factor', 2.0)
        self.declare_parameter('robot_half_width_m', ROBOT_HALF_WIDTH_M)

        # 기본 꺼짐 — mission_summer.launch.py에서만 True로 켠다. 트랙 안쪽에
        # 구조물이 있어서 경로가 한쪽으로 치우치는 문제 대응용
        # (bev_mask_to_centerline_path()의 lateral_gap_correction 절 참고).
        self.declare_parameter('lateral_gap_correction', False)
        self.declare_parameter('lateral_gap_min_px', LATERAL_GAP_MIN_PX_PLACEHOLDER)

        # 기본 꺼짐 — mission_summer.launch.py에서만 True로 켠다. BEV 하단
        # (로봇 근처)에 세그멘테이션 유효 row가 없어 path_points가 로봇
        # 바로 앞부터 시작하지 못하는 경우, 상단(먼 쪽) 경로의 진행방향으로
        # 직선을 연장해 그 구멍을 메운다 (extend_path_to_robot 참고).
        self.declare_parameter('extend_path_to_robot', False)
        self.declare_parameter('extend_to_robot_gap_m', EXTEND_TO_ROBOT_GAP_M)

        # row 단위 계단 경로 대신 B-spline으로
        # 이어서 발행(smooth_path_bspline 참고). max_gap_rows보다 큰 구멍은
        # 그 지점에서 경로를 끊는다(truncate_at_large_gap 참고, 실측 없는
        # 구간까지 곡선으로 지어내지 않기 위함).
        self.declare_parameter('max_gap_rows', MAX_GAP_ROWS_PLACEHOLDER)

        # 기본 꺼짐 — mission_summer.launch.py에서만 True로 켠다. True면
        # max_gap_rows보다 큰 중간 구멍(자갈 구간 등)을 truncate_at_large_gap()
        # 처럼 끊지 않고 bridge_interior_gaps()로 앞뒤를 이어 붙인다 — 자갈이
        # 실제 주행 가능 구간이라는 걸 아는 여름 미션 전용(사용자 요청,
        # 2026-09-05). 다른 미션은 계속 truncate_at_large_gap()만 쓴다.
        self.declare_parameter('bridge_interior_gaps', False)

        # 0.0(기본, 비활성) — mission_summer.launch.py에서만 양수로 켠다.
        # 세그멘테이션이 화면 전체에서 사라졌을 때(_latest_mask가 없거나
        # path_points가 완전히 빈 상태), odom/IMU 연동 없이 마지막으로 발행한
        # path_points를 이 시간(초) 동안만 그대로 재사용한다 — 자갈 구간이
        # 화면 전체를 덮는 짧은 순간 대응용. 그보다 오래 세그멘테이션이
        # 안 돌아오면 빈 Path로 안전 정지한다(사용자 요청, 2026-09-05 —
        # 직전 경로를 무기한 쓰면 위험하므로 시간 상한 필수).
        self.declare_parameter('hold_last_path_sec', 0.0)
        # bev_meters_per_pixel(원본 픽셀 간격)보다 확실히 크게 잡아서 실제로
        # 점 개수를 줄이는 용도 — 원본과 같은 값이면 재샘플링을 해도 출력
        # 점 개수가 원본 row 개수와 거의 같아져 MPPI에 가볍게 넘기려는
        # 목적이 무효화된다.
        self.declare_parameter('path_point_spacing_m', 0.12)

        # /flatdrive/planned_path에 실릴 frame_id. 발행되는 경로
        # 좌표는 x=전방/y=왼쪽인 body(=camera_link) 규약이지, 입력 컬러
        # 이미지의 optical frame(x=오른쪽/y=아래/z=전방, 보통
        # 'camera_color_optical_frame' 등)이 아니다 — msg.header를 그대로
        # 재사용하면 좌표값은 body인데 frame_id만 optical로 잘못 나간다.
        # path_relay_node가 이 frame_id를 그대로 믿고 TF로 odom 변환하므로,
        # 여기서 실제 좌표 규약에 맞는 frame_id를 명시적으로 붙여야 한다.
        self.declare_parameter('path_frame_id', 'camera_link')

        self.declare_parameter('bench_log_hz', 1.0)

        color_topic = str(self.get_parameter('color_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self._mask_topic = str(self.get_parameter('mask_topic').value)
        self._path_frame_id = str(self.get_parameter('path_frame_id').value)

        self._camera_height_m = float(self.get_parameter('camera_height_m').value)
        self._pitch_offset_rad = np.deg2rad(float(self.get_parameter('camera_pitch_offset_deg').value))
        self._roll_offset_rad = np.deg2rad(float(self.get_parameter('camera_roll_offset_deg').value))
        self._alpha = float(self.get_parameter('complementary_filter_alpha').value)

        self._mpp = float(self.get_parameter('bev_meters_per_pixel').value)
        self._bev_w = int(self.get_parameter('bev_img_width').value)
        self._bev_h = int(self.get_parameter('bev_img_height').value)

        self._min_row_pixels = int(self.get_parameter('min_row_pixels').value)

        # ROS 파라미터로는 None을 못 넘기니, 0 이하를 "검사 비활성화"
        # 의미로 취급해서 bev_mask_to_centerline_path()가 기대하는
        # None으로 바꿔둔다.
        _expected_track_width_raw = float(self.get_parameter('expected_track_width_m').value)
        self._expected_track_width_m = (
            _expected_track_width_raw if _expected_track_width_raw > 0.0 else None)
        self._track_width_tolerance_factor = float(
            self.get_parameter('track_width_tolerance_factor').value)
        self._robot_half_width_m = float(self.get_parameter('robot_half_width_m').value)
        self._lateral_gap_correction = bool(
            self.get_parameter('lateral_gap_correction').value)
        self._lateral_gap_min_px = int(self.get_parameter('lateral_gap_min_px').value)
        if self._lateral_gap_correction:
            self.get_logger().info(
                f'lateral_gap_correction 활성화 — gap_min_px={self._lateral_gap_min_px}')
        self._extend_path_to_robot = bool(
            self.get_parameter('extend_path_to_robot').value)
        self._extend_to_robot_gap_m = float(
            self.get_parameter('extend_to_robot_gap_m').value)
        if self._extend_path_to_robot:
            self.get_logger().info(
                f'extend_path_to_robot 활성화 — gap_m={self._extend_to_robot_gap_m}')
        self._max_gap_rows = int(self.get_parameter('max_gap_rows').value)
        self._bridge_interior_gaps = bool(
            self.get_parameter('bridge_interior_gaps').value)
        if self._bridge_interior_gaps:
            self.get_logger().info('bridge_interior_gaps 활성화')
        self._hold_last_path_sec = float(
            self.get_parameter('hold_last_path_sec').value)
        if self._hold_last_path_sec > 0.0:
            self.get_logger().info(
                f'hold_last_path_sec 활성화 — {self._hold_last_path_sec:.2f}초')
        # hold_last_path_sec용 상태 — 마지막으로 성공 계산된 path_points와
        # 그 시각. odom/IMU로 갱신하지 않고 그대로 재사용한다(사용자 결정,
        # 2026-09-05: 짧은 시간만 버티는 용도라 좌표 보정까지는 불필요).
        self._last_good_path_points: list[tuple[float, float]] = []
        self._last_good_path_time: float | None = None
        self._path_point_spacing_m = float(
            self.get_parameter('path_point_spacing_m').value)

        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)

        self._bridge = CvBridge()
        self._timings: list[tuple] = []

        self._camera_matrix = None
        self._dist_coeffs = None
        self._roll_est: float | None = None
        self._pitch_est: float | None = None
        self._last_imu_stamp: float | None = None
        self._latest_mask: np.ndarray | None = None

        qos = SENSOR_DATA_QOS_DEPTH1

        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, qos)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, qos)
        self._image_sub = self.create_subscription(
            CompressedImage, color_topic, self._on_image, qos)
        self._mask_sub = self.create_subscription(
            Image, self._mask_topic, self._on_mask, 10)

        self._path_pub = self.create_publisher(Path, '/flatdrive/planned_path', 10)
        self._target_pub = self.create_publisher(PointStamped, '/planning/target_point', 10)
        self._bev_image_pub = self.create_publisher(Image, '/bev/image', 10)
        self._mask_pub = self.create_publisher(Image, '/bev/mask', 10)
        self._debug_image_pub = self.create_publisher(Image, '/bev/debug_overlay', 10)
        self._centerline_overlay_pub = self.create_publisher(Image, '/bev/centerline_overlay', 10)
        self._homography_pub = self.create_publisher(Float64MultiArray, '/bev/H', 10)

        # lateral_gap_correction/lateral_gap_min_px을 ros2 param set으로
        # 재시작 없이 튜닝할 수 있게 함 — declare_parameter()로 __init__에서
        # 한 번만 읽어 캐싱한 값은 콜백 없이는 파라미터 서버 값과 따로 논다.
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self._bench_timer = self.create_timer(bench_period, self._log_timing)
        self.get_logger().info(
            f'FlatDriveNode ready — bev={self._bev_w}x{self._bev_h}@{self._mpp}m/px, '
            f'listening on {color_topic}, {camera_info_topic}, {imu_topic}, {self._mask_topic}'
        )

    # ------------------------------------------------------------------

    def _on_parameter_update(self, params) -> SetParametersResult:
        """ros2 param set 콜백 — lateral_gap_correction/lateral_gap_min_px/
        extend_path_to_robot/extend_to_robot_gap_m/max_gap_rows/
        bridge_interior_gaps/hold_last_path_sec/path_point_spacing_m만 실행
        중 재조정 가능하게 함(다른 파라미터는 기존과 동일하게 노드 재시작
        필요, 아직 손 안 댐)."""
        for p in params:
            if p.name == 'lateral_gap_correction':
                self._lateral_gap_correction = bool(p.value)
            elif p.name == 'lateral_gap_min_px':
                if int(p.value) < 1:
                    return SetParametersResult(
                        successful=False, reason='lateral_gap_min_px는 1 이상이어야 합니다.')
                self._lateral_gap_min_px = int(p.value)
            elif p.name == 'extend_path_to_robot':
                self._extend_path_to_robot = bool(p.value)
            elif p.name == 'extend_to_robot_gap_m':
                if float(p.value) < 0:
                    return SetParametersResult(
                        successful=False, reason='extend_to_robot_gap_m은 0 이상이어야 합니다.')
                self._extend_to_robot_gap_m = float(p.value)
            elif p.name == 'max_gap_rows':
                if int(p.value) < 0:
                    return SetParametersResult(
                        successful=False, reason='max_gap_rows는 0 이상이어야 합니다.')
                self._max_gap_rows = int(p.value)
            elif p.name == 'bridge_interior_gaps':
                self._bridge_interior_gaps = bool(p.value)
            elif p.name == 'hold_last_path_sec':
                if float(p.value) < 0:
                    return SetParametersResult(
                        successful=False, reason='hold_last_path_sec은 0 이상이어야 합니다.')
                self._hold_last_path_sec = float(p.value)
            elif p.name == 'path_point_spacing_m':
                if float(p.value) <= 0:
                    return SetParametersResult(
                        successful=False, reason='path_point_spacing_m은 0보다 커야 합니다.')
                self._path_point_spacing_m = float(p.value)
        return SetParametersResult(successful=True)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape((3, 3))
            self._dist_coeffs = np.array(msg.d)
            self.destroy_subscription(self._info_sub)
            self.get_logger().info('CameraInfo 수신 완료, 구독 해제.')

    def _on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel_body = optical_vector_to_body(msg.linear_acceleration)
        gyro_body = optical_vector_to_body(msg.angular_velocity)

        dt = 0.0 if self._last_imu_stamp is None else max(0.0, stamp - self._last_imu_stamp)
        self._roll_est, self._pitch_est = update_complementary_filter(
            self._roll_est, self._pitch_est, gyro_body, accel_body, dt, alpha=self._alpha)
        self._last_imu_stamp = stamp

    # ------------------------------------------------------------------

    def _on_mask(self, msg: Image) -> None:
        try:
            self._latest_mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self.get_logger().error(f'마스크 CvBridge decode failed: {exc}')

    def _path_to_msg(self, path_points: list[tuple[float, float]], source_header) -> Path:
        # source_header(입력 컬러 이미지의 Header)를 그대로 msg.header에
        # 대입하지 않는다 — frame_id가 optical frame이라 실제 좌표 규약(body,
        # x=전방/y=왼쪽)과 안 맞는다. 새 Header를 만들어 stamp만 유지하고(TF
        # 시간 정합용) frame_id는 path_frame_id 파라미터로 명시한다. 입력
        # source_header 객체 자체는 절대 수정하지 않는다 (참조 공유 시 구독
        # 메시지까지 오염될 수 있음).
        path_header = Header()
        path_header.stamp = source_header.stamp
        path_header.frame_id = self._path_frame_id

        msg = Path()
        msg.header = path_header
        previous_yaw = 0.0
        for index, (x_forward, y_left) in enumerate(path_points):
            # 각 pose가 다음 경로점을 향하도록 접선 방향 yaw를 넣는다.
            if index + 1 < len(path_points):
                next_x, next_y = path_points[index + 1]
                previous_yaw = float(np.arctan2(
                    next_y - y_left, next_x - x_forward))
            quat_x, quat_y, quat_z, quat_w = Rotation.from_euler(
                'z', previous_yaw).as_quat()

            pose = PoseStamped()
            pose.header = path_header
            pose.pose.position.x = float(x_forward)
            pose.pose.position.y = float(y_left)
            pose.pose.orientation.x = float(quat_x)
            pose.pose.orientation.y = float(quat_y)
            pose.pose.orientation.z = float(quat_z)
            pose.pose.orientation.w = float(quat_w)
            msg.poses.append(pose)
        return msg

    def _draw_centerline_overlay(self, bev_mask: np.ndarray, path_points: list[tuple[float, float]]) -> np.ndarray:
        overlay = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
        pts_px = []
        for x_forward, y_left in path_points:
            col = self._bev_w / 2.0 - y_left / self._mpp
            row = self._bev_h - x_forward / self._mpp
            pts_px.append((int(round(col)), int(round(row))))
        for pt in pts_px:
            cv2.circle(overlay, pt, 2, (0, 0, 255), -1)
        for p1, p2 in zip(pts_px, pts_px[1:]):
            cv2.line(overlay, p1, p2, (0, 255, 0), 1)
        return overlay

    # ------------------------------------------------------------------

    def _get_held_path_points(self) -> list[tuple[float, float]]:
        """hold_last_path_sec 이내면 마지막 정상 경로를, 넘었으면 빈 리스트를
        반환한다. odom/IMU로 좌표를 갱신하지 않고 그대로 재사용한다 — 짧은
        시간만 버티는 용도라 로봇이 그 사이 얼마나 움직였는지는 반영하지
        않는다(사용자 결정, 2026-09-05). 무기한 재사용을 막기 위한 시간
        상한이 핵심이다 — 넘으면 빈 리스트를 돌려줘서 호출부가 빈 Path를
        발행하게 한다(안전 정지)."""
        if not self._last_good_path_points or self._last_good_path_time is None:
            return []
        age = time.monotonic() - self._last_good_path_time
        if age > self._hold_last_path_sec:
            return []
        return self._last_good_path_points

    def _publish_held_or_empty_path(self, header) -> None:
        """카메라 프레임은 왔지만 CameraInfo/IMU/마스크 등 선행 조건이 아직
        안 갖춰져 경로 계산 자체를 못 하는 경우에 쓴다(_on_image() 상단 가드
        참고). hold_last_path_sec이 켜져 있고 시간 이내면 마지막 정상 경로를
        그대로 재발행하고, 아니면 빈 Path를 발행해 명시적으로 정지시킨다 —
        아예 발행을 안 하면 소비 측(path_relay_node 등)이 "아직 새 goal이
        안 왔을 뿐"이라 여겨 옛 목표를 계속 쫓아가는 문제(사용자 지적,
        2026-09-05)가 있어, 이 경로들에서도 매 프레임 뭔가는 발행한다."""
        if self._hold_last_path_sec > 0.0:
            path_points = self._get_held_path_points()
        else:
            path_points = []
        path_points = smooth_path_bspline(path_points, self._path_point_spacing_m)
        self._path_pub.publish(self._path_to_msg(path_points, header))

    def _on_image(self, msg: CompressedImage) -> None:
        if self._camera_matrix is None:
            self.get_logger().warn('CameraInfo 대기 중.', throttle_duration_sec=2.0)
            self._publish_held_or_empty_path(msg.header)
            return
        if self._roll_est is None or self._pitch_est is None:
            self.get_logger().warn('IMU 메시지 대기 중.', throttle_duration_sec=2.0)
            self._publish_held_or_empty_path(msg.header)
            return
        if self._latest_mask is None:
            self.get_logger().warn(
                f'{self._mask_topic} 마스크 대기 중 (segmentation 노드가 떠 있는지 확인).',
                throttle_duration_sec=2.0)
            self._publish_held_or_empty_path(msg.header)
            return

        t0 = time.perf_counter()

        try:
            cv_image = self._bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return
        undistorted = cv2.undistort(cv_image, self._camera_matrix, self._dist_coeffs)

        t1 = time.perf_counter()

        raw_mask = self._latest_mask  # segmentation.py가 발행한 최신 마스크(비동기, 완전 동기화 아님)

        final_homography = image_to_bev_homography(
            self._camera_matrix, self._roll_est, self._pitch_est,
            self._roll_offset_rad, self._pitch_offset_rad, self._camera_height_m,
            self._bev_w, self._bev_h, self._mpp)
        bev_mask = mask_to_bev(raw_mask, final_homography, self._bev_w, self._bev_h)
        path_points = bev_mask_to_centerline_path(
            bev_mask, self._min_row_pixels, self._bev_w, self._bev_h, self._mpp,
            self._expected_track_width_m, self._track_width_tolerance_factor,
            self._robot_half_width_m, self._lateral_gap_correction,
            self._lateral_gap_min_px)

        # bev_mask_to_centerline_path()는 순수 함수라 로그를 안 남긴다 —
        # 실제로 몇 개 row가 보정됐는지는 여기서 같은 기준으로 다시 세서만
        # 보여준다(경로 계산 자체에는 영향 없음, 관측용).
        if self._lateral_gap_correction:
            gap_rows = sum(
                1 for row in range(self._bev_h)
                if find_lateral_gap(bev_mask[row] > 0, self._lateral_gap_min_px) is not None)
            if gap_rows > 0:
                self.get_logger().warn(
                    f'lateral_gap_correction: {gap_rows}개 row에서 옆 구조물 감지 — '
                    '중점으로 보정.', throttle_duration_sec=2.0)

        n_before_truncate = len(path_points)
        if self._bridge_interior_gaps:
            # 여름 미션 전용 — 중간 큰 gap(자갈 구간 등)을 끊지 않고 앞뒤를
            # 이어붙인다. 맨 앞(로봇 바로 앞) gap은 bridge_interior_gaps()가
            # 처리하지 않고 그대로 두며, 그건 아래 extend_path_to_robot()의
            # 역할이다.
            path_points = bridge_interior_gaps(path_points, self._mpp, self._max_gap_rows)
        else:
            path_points = truncate_at_large_gap(path_points, self._mpp, self._max_gap_rows)
        if len(path_points) < n_before_truncate and not self._bridge_interior_gaps:
            self.get_logger().warn(
                f'{self._max_gap_rows}row 넘는 구멍 감지 — 그 지점부터 경로 절단 '
                f'({n_before_truncate}개 -> {len(path_points)}개 점).',
                throttle_duration_sec=2.0)

        if self._extend_path_to_robot and path_points and (
                path_points[0][0] > self._extend_to_robot_gap_m):
            gap_start_m = path_points[0][0]
            path_points = extend_path_to_robot(
                path_points, self._path_point_spacing_m, self._extend_to_robot_gap_m)
            self.get_logger().warn(
                f'extend_path_to_robot: 로봇 앞 0m~{gap_start_m:.2f}m 구간 '
                'segmentation 결측 — 자갈 너머 경로 방향으로 직선 연결.',
                throttle_duration_sec=2.0)

        if path_points:
            # hold_last_path_sec용 캐시 — 완전 미검출 상태로 넘어가기 직전의
            # "정상적으로 계산된" 경로만 남긴다(스무딩 이전 원본 그대로 —
            # 다음에 재사용할 때 다시 스무딩을 거치므로 충분).
            self._last_good_path_points = path_points
            self._last_good_path_time = time.monotonic()
        elif self._hold_last_path_sec > 0.0:
            path_points = self._get_held_path_points()
            if path_points:
                self.get_logger().warn(
                    'segmentation 완전 미검출 — 직전 정상 경로를 '
                    f'hold_last_path_sec={self._hold_last_path_sec:.2f}초 이내라 유지.',
                    throttle_duration_sec=1.0)

        path_points = smooth_path_bspline(path_points, self._path_point_spacing_m)

        t2 = time.perf_counter()

        self._path_pub.publish(self._path_to_msg(path_points, msg.header))

        if path_points:
            x_forward, y_left = path_points[0]
            target = PointStamped()
            target.header = msg.header
            target.point.x = float(x_forward)
            target.point.y = float(y_left)
            self._target_pub.publish(target)
        else:
            self.get_logger().warn('주행가능영역 경로를 찾지 못함.', throttle_duration_sec=2.0)

        bev_color = cv2.warpPerspective(undistorted, final_homography, (self._bev_w, self._bev_h))
        bev_image_msg = self._bridge.cv2_to_imgmsg(bev_color, encoding='bgr8')
        bev_image_msg.header = msg.header
        self._bev_image_pub.publish(bev_image_msg)

        mask_msg = self._bridge.cv2_to_imgmsg(bev_mask, encoding='mono8')
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)

        debug_overlay = undistorted.copy()
        contours, _ = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug_overlay, contours, -1, (0, 255, 0), 2)
        debug_msg = self._bridge.cv2_to_imgmsg(debug_overlay, encoding='bgr8')
        debug_msg.header = msg.header
        self._debug_image_pub.publish(debug_msg)

        centerline_overlay = self._draw_centerline_overlay(bev_mask, path_points)
        centerline_msg = self._bridge.cv2_to_imgmsg(centerline_overlay, encoding='bgr8')
        centerline_msg.header = msg.header
        self._centerline_overlay_pub.publish(centerline_msg)

        h_g2i = ground_to_image_homography(
            self._camera_matrix, self._roll_est, self._pitch_est,
            self._roll_offset_rad, self._pitch_offset_rad, self._camera_height_m)
        h_msg = Float64MultiArray()
        h_msg.data = np.linalg.inv(h_g2i).flatten().tolist()
        self._homography_pub.publish(h_msg)

        t3 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode+undistort
            (t2 - t1) * 1e3,   # segment+bev+centerline
            (t3 - t2) * 1e3,   # publish
            (t3 - t0) * 1e3,   # total
        ))

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
    node = FlatDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
