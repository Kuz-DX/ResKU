#!/usr/bin/env python3
"""
path_visualizer.py — /path를 RGB 카메라 화면에 빨간 선으로 덧씌우는 노트북
전용 로컬 디버그 뷰어 (독립 실행 스크립트).

dolbotz 패키지 의존성 없음 — rclpy + sensor_msgs/nav_msgs/std_msgs +
opencv-python + numpy만 필요(cv_bridge도 안 씀, CompressedImage 바이트를
cv2.imdecode로 직접 디코드). utils/ 아래 있지만 dolbotz 패키지의 일부로
빌드/설치될 필요 없이 `python3 path_visualizer.py`로 바로 실행된다. 젝슨
쪽 노드/launch 파일은 이 작업의 대상이 아니며 이 스크립트는 그쪽 코드를
전혀 import하지 않는다.

=== 필수: flat_drive_node가 항상 같이 떠 있어야 함 ===
/bev/H는 flat_drive_node(src/dolbotz/dolbotz/drive_area/flat_drive.py)만
발행한다(flat_drive.py:349,481-486). gradient_map(경사 경로)만 단독으로
떠 있거나 flat_drive_node가 죽어 있으면 /bev/H가 안 들어와서 이 뷰어는
경로를 투영할 좌표 변환 자체를 못 한다 — 그 경우 카메라 원본(왜곡보정
전)만 보여주고 화면/로그에 경고를 띄운다.

=== 좌표 변환: /bev/H를 왜 두 번 뒤집는가 ===
flat_drive.py 소스(481~486행)를 직접 확인한 결과:
    h_g2i = ground_to_image_homography(...)   # ground(지면 x_forward,y_left) -> image
    h_msg.data = np.linalg.inv(h_g2i).flatten().tolist()   # <- 역행렬을 발행!
    self._homography_pub.publish(h_msg)  # 토픽: /bev/H
즉 /bev/H에 실제로 실리는 값은 h_i2g = inv(h_g2i) (image->ground, BEV 변환
쪽에서 쓰기 편한 방향)이다. 이 뷰어가 원하는 건 반대 방향(ground->image,
경로 점을 화면 픽셀로 옮기는 것)이므로 받은 행렬을 다시 한 번 뒤집는다:
    h_g2i = inv(수신한 /bev/H)
    [u, v, w]^T = h_g2i @ [x_forward, y_left, 1]^T
    pixel = (u/w, v/w)          (w<=0 이면 카메라 뒤쪽/소실점 방향 -> 버림)

/path의 poses[i].pose.position.x/y가 곧 x_forward/y_left(전방/좌측 미터)
라는 것도 flat_drive.py._path_to_msg()와 gradient_map.py._cells_to_path_msg()
양쪽 소스에서 같은 필드 이름(x=전방, y=좌측)으로 쓰고 있음을 직접 확인했다
— slope_decision.py가 이 둘 중 하나를 헤더/필드 변환 없이 그대로 /path로
릴레이하므로(_decide_and_publish 확인) 어느 쪽이 선택돼도 이 규약은 유지된다.

=== 왜곡보정(undistort) 순서 ===
ground_to_image_homography()의 독스트링(flat_drive.py)에 "원본(왜곡보정된)
카메라 이미지 좌표계"라고 명시돼 있다 — 즉 h_g2i가 만들어내는 픽셀 좌표는
cv2.undistort()를 거친 이미지 기준이다. 그래서 이 뷰어도 flat_drive.py와
똑같은 순서(raw 디코드 -> cv2.undistort(raw, K, D) -> 그 결과 위에 그리기)
를 지킨다. 순서를 바꿔서 왜곡보정 안 된 원본 위에 그대로 그리면 화면
중앙에서는 크게 안 티나지만 렌즈 왜곡이 큰 가장자리로 갈수록 경로가
실제 트랙과 어긋나 보인다 — 특히 광각 렌즈일수록 문제가 커진다.

=== 한계: 지면 평탄(flat-ground) 가정 ===
h_g2i(그리고 그 역인 /bev/H)는 flat_drive_node가 "카메라 아래
z=-camera_height_m인 평면"을 가정해 매 프레임 계산한 호모그래피다
(ground_to_image_homography 유도 참고). /path가 지금 /terrain/planned_path
(경사 경로, gradient_map)를 릴레이하고 있는 순간에도 이 뷰어는 항상 같은
평탄-평면 호모그래피로 투영한다 — 실제로는 경사면 위에 있는 점을 평평한
가상 평면에 투영한 것처럼 그리므로, 경사 구간에서는 화면상 위치가 실제
지면과 어긋날 수 있다. 이 뷰어는 "경로가 카메라 쪽에서 대략 어디로
잡히는지" 보는 정성적 디버그용이며, 경사 구간 정량 검증에는 쓰지 말 것.

=== QoS (소스 분석 — 실 하드웨어 연결 없이 코드로 검증함) ===
  color_topic, camera_info_topic : BEST_EFFORT/depth=1
      (flat_drive.py:332,334-339 — self._image_sub/_info_sub 모두
      dolbotz.utils.qos.SENSOR_DATA_QOS_DEPTH1로 생성. 이 뷰어는 dolbotz
      무의존 설계라 같은 스펙을 _sensor_data_qos_depth1()로 인라인 중복)
  /bev/H  (Float64MultiArray)    : create_publisher 기본값 = RELIABLE, depth 10
      (flat_drive.py:349)
  /path   (nav_msgs/Path)        : create_publisher 기본값 = RELIABLE, depth 10
      (slope_decision.py:129)
이 뷰어의 구독 QoS도 토픽별로 위와 똑같이 맞춰서 만든다 — 안 맞으면 그
토픽만 조용히 연결이 안 된다(에러 없이 그냥 콜백이 안 불림. 원인 파악이
어려우므로 이 파일에서 QoS를 하드코딩해 실수를 방지했다).

=== HUD 사이드패널 ===
아래 다섯 토픽을 추가로 구독해서 별도 창(HUD)에 STATUS/ROLL/FPS/PATH LEN/
WHEEL YAW/CMD_VEL_AUTO/SPEED LIMIT을 띄운다:
  /drive/status                (std_msgs/String,  slope_decision.py 발행 — 'flat'/'slope')
  /terrain/side_slope_angle_deg (std_msgs/Float32, slope_decision.py 발행)
  /wheel/odom                  (nav_msgs/Odometry, rmd_x8_driver_node.py 발행,
                                twist.twist.angular.z만 씀 — 이 값은 발행 쪽
                                자체 docstring에 "intentionally NOT trusted"로
                                명시돼 있어 EKF도 안 씀, 여기서도 디버그/비교용일 뿐)
  /cmd_vel_auto                 (geometry_msgs/Twist, controller_server(MPPI)
                                출력, nav2.launch.py에서 /cmd_vel -> /cmd_vel_auto로 remap)
  /speed_limit                  (nav2_msgs/SpeedLimit. 이 토픽을
                                발행하던 slope_speed_limiter_node 패키지가
                                삭제됨(임계값 미설정으로 오래 무동작 상태였음)
                                -- 지금은 발행자가 없어 항상 N/A로 표시된다.
                                HUD 필드/파서는 나중에 다른 발행자가 생길 걸
                                대비해 남겨둠. percentage=true/speed_limit=0.0=
                                "제한 없음" 관례는 nav2_msgs/SpeedLimit 자체
                                규약이라 이 필드 처리 로직도 그대로 유지.)
QoS는 /drive/status, /terrain/side_slope_angle_deg, /cmd_vel_auto, /speed_limit이
/bev/H, /path와 동일하게 RELIABLE(_reliable_qos()), /wheel/odom만 발행
소스(rmd_x8_driver_node.py) 그대로 BEST_EFFORT(_wheel_odom_qos()). HUD 갱신은
카메라 프레임 처리(_on_image)와 완전히 분리된 10Hz 타이머(_update_hud)에서만
하므로, 카메라 프레임 처리 속도가 떨어져도 HUD 자체는 계속 10Hz로 갱신된다.

사용법:
  python3 path_visualizer.py                      # 실제 로봇/네트워크에 붙어서 구독
  python3 path_visualizer.py --image-topic ...     # 토픽명 오버라이드
  python3 path_visualizer.py --dummy               # 로컬 스모크테스트(더미 발행 내장, 실제 로봇 불필요)

키: q  종료
    s  현재 화면 스냅샷 PNG 저장 (path_overlay_snapshots/)

실행 전 확인:
  - flat_drive_node가 반드시 떠 있어야 함(위 "필수" 항목).
  - ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY/RMW_IMPLEMENTATION이 젝슨/노트북 동일한지.
  - `ros2 topic list`에 /bev/H, /path, 컬러 이미지/camera_info 토픽이
    보이는지. 이 뷰어 실행 5초 뒤에도 안 들어오면 터미널에 토픽별로
    개별 경고가 뜬다(어느 토픽이 문제인지 바로 알 수 있게 나눠서 검사함).
"""
import argparse
import collections
import time
from pathlib import Path as FsPath

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Float64MultiArray, String, Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit

SNAPSHOT_DIR = FsPath(__file__).resolve().parent / 'path_overlay_snapshots'

RED_BGR = (0, 0, 255)          # 경로 선 색(OpenCV는 BGR 순서)
LINE_THICKNESS = 3
NO_DATA_WARN_SEC = 5.0
# HUD 사이드패널 창 크기(px) — CMD_VEL_AUTO 줄이 길어서 정사각형 400x400보단
# 가로로 좀 더 넓힘(cv2.getTextSize로 실측 후 결정, 스케일 0.8 기준 폭 ~430px).
# SPEED LIMIT 줄까지 표시할 수 있도록 높이는 460px로 둔다.
HUD_WIDTH = 500
HUD_HEIGHT = 460


def _reliable_qos() -> QoSProfile:
    """/bev/H, /path 발행자 쪽 기본 QoS(RELIABLE, depth 10)와 맞춘 프로필."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )


def _sensor_data_qos_depth1() -> QoSProfile:
    """카메라 이미지/CameraInfo 구독용 — rclpy 기본 qos_profile_sensor_data
    (BEST_EFFORT/VOLATILE/depth=5)와 동일하되 depth만 1로 낮춘 프로필.
    구독 처리가 잠깐 밀려도 오래된 프레임을 쌓아두지 않고 항상 가장 최신
    프레임만 남긴다(dolbotz.utils.qos.SENSOR_DATA_QOS_DEPTH1과 동일 스펙 —
    이 파일은 dolbotz 패키지 무의존 설계라 import 대신 인라인으로 중복 정의)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _wheel_odom_qos() -> QoSProfile:
    """/wheel/odom 발행자(rmd_x8_driver_node.py) 쪽 QoS와 맞춘 프로필 —
    소스 확인: QoSProfile(depth=10, reliability=BEST_EFFORT, history=KEEP_LAST)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


# ---------------------------------------------------------------------------
# 순수 계산 (ROS 의존성 없음) — 좌표 변환/필터링
# ---------------------------------------------------------------------------

def bev_h_to_ground_to_image(bev_h_flat) -> np.ndarray:
    """/bev/H로 받은 flatten(9,) 배열 -> ground-to-image 호모그래피 h_g2i.

    flat_drive.py가 /bev/H에 발행하는 값은 h_i2g = inv(h_g2i)이므로
    (파일 상단 독스트링 "좌표 변환" 절 참고) 여기서 다시 한 번 뒤집는다.
    """
    h_i2g = np.asarray(bev_h_flat, dtype=np.float64).reshape(3, 3)
    return np.linalg.inv(h_i2g)


def project_ground_points(xy_forward_left: np.ndarray, h_g2i: np.ndarray):
    """(N,2) x_forward,y_left 지면 좌표 -> (N,2) 픽셀 좌표, (N,) valid bool.

    valid=False인 점(w<=0, 카메라 뒤쪽/소실점 방향으로 발산)의 픽셀 값은
    의미 없는 값이므로 호출부가 반드시 valid로 걸러서 그려야 한다 — 안
    거르면 화면 밖 극단적인 좌표로 선이 튀어 보인다.
    """
    n = xy_forward_left.shape[0]
    if n == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty((0,), dtype=bool)
    homog = np.hstack([xy_forward_left, np.ones((n, 1), dtype=np.float64)])  # (N,3)
    proj = (h_g2i @ homog.T).T  # (N,3) = [u*w, v*w, w]
    w = proj[:, 2]
    valid = w > 1e-6
    px = np.zeros((n, 2), dtype=np.float64)
    px[valid, 0] = proj[valid, 0] / w[valid]
    px[valid, 1] = proj[valid, 1] / w[valid]
    return px, valid


def path_arc_length_m(xy_forward_left: np.ndarray) -> float:
    """경로 점들을 순서대로 이은 선분 길이의 합(m) — 경로의 근사 길이.
    점이 2개 미만이면 0.0."""
    if xy_forward_left.shape[0] < 2:
        return 0.0
    diffs = np.diff(xy_forward_left, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def approx_steering_deg(xy_forward_left: np.ndarray, lookahead_m: float) -> float | None:
    """x_forward가 lookahead_m에 가장 가까운 경로점을 찾아 atan2(y_left, x_forward)를
    degrees로 반환 — 그 지점으로 가려면 대략 몇 도를 틀어야 하는지의 근사치.
    경로가 비어있으면 None."""
    if xy_forward_left.shape[0] == 0:
        return None
    idx = int(np.argmin(np.abs(xy_forward_left[:, 0] - lookahead_m)))
    x, y = xy_forward_left[idx]
    return float(np.degrees(np.arctan2(y, x)))


def fps_from_timestamps(timestamps) -> float:
    """최근 프레임 처리 시각(예: collections.deque of time.monotonic(), 초 단위)들로
    FPS를 추정한다 — (개수) / (최신-최오래). 샘플이 2개 미만이거나 시간차가
    0 이하면 0.0."""
    n = len(timestamps)
    if n < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    if span <= 0:
        return 0.0
    return n / span


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sync_delay_ms(stamp_a, stamp_b) -> float | None:
    """두 builtin_interfaces/Time 스탬프 차이[ms]. 둘 중 하나라도 0(스탬프
    안 찍힘)이면 비교 의미가 없으니 None."""
    ta, tb = stamp_to_sec(stamp_a), stamp_to_sec(stamp_b)
    if ta == 0.0 or tb == 0.0:
        return None
    return abs(ta - tb) * 1000.0


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------

class PathCameraOverlay(Node):
    def __init__(self, args, dummy: bool = False):
        super().__init__('path_camera_overlay')

        self._camera_matrix = None
        self._dist_coeffs = None
        self._h_g2i = None
        self._h_g2i_wall = None       # /bev/H는 header가 없는 Float64MultiArray라 도착 시각(wall clock)만 기록
        self._latest_path = None
        self._latest_path_wall = None
        self.display_frame = None
        self._start_wall = time.monotonic()
        self._warned = {
            'image': False, 'camera_info': False, 'bev_h': False, 'path': False,
            'drive_status': False, 'side_slope': False,
            'wheel_odom': False, 'cmd_vel_auto': False, 'speed_limit': False,
        }

        # HUD용 상태 — /drive/status, /terrain/side_slope_angle_deg는 카메라
        # 프레임 처리와 무관하게 그 자체로 최신값만 들고 있으면 됨.
        self._drive_status: str | None = None
        self._side_slope_deg: float | None = None
        self._steer_lookahead_m = float(args.steer_lookahead_m)
        self._wheel_yaw_rate: float | None = None            # /wheel/odom twist.twist.angular.z
        self._cmd_vel_auto_linear_x: float | None = None
        self._cmd_vel_auto_angular_z: float | None = None
        self._speed_limit_pct: float | None = None  # percentage=true 고정이므로 %로만 옴
        self._frame_times = collections.deque(maxlen=30)  # _on_image 처리 시각(monotonic) — FPS 계산용
        self.hud_frame = None

        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_camera_info, _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, args.image_topic, self._on_image, _sensor_data_qos_depth1())
        self.create_subscription(
            Float64MultiArray, args.bev_h_topic, self._on_bev_h, _reliable_qos())
        self.create_subscription(
            Path, args.path_topic, self._on_path, _reliable_qos())
        self.create_subscription(
            String, args.drive_status_topic, self._on_drive_status, _reliable_qos())
        self.create_subscription(
            Float32, args.side_slope_topic, self._on_side_slope, _reliable_qos())
        self.create_subscription(
            Odometry, args.wheel_odom_topic, self._on_wheel_odom, _wheel_odom_qos())
        self.create_subscription(
            Twist, args.cmd_vel_auto_topic, self._on_cmd_vel_auto, _reliable_qos())
        self.create_subscription(
            SpeedLimit, args.speed_limit_topic, self._on_speed_limit, _reliable_qos())

        self._topics = {
            'image': args.image_topic, 'camera_info': args.camera_info_topic,
            'bev_h': args.bev_h_topic, 'path': args.path_topic,
            'drive_status': args.drive_status_topic, 'side_slope': args.side_slope_topic,
            'wheel_odom': args.wheel_odom_topic, 'cmd_vel_auto': args.cmd_vel_auto_topic,
            'speed_limit': args.speed_limit_topic,
        }
        self._got = {k: False for k in self._topics}

        if dummy:
            self._setup_dummy(args)

        self.create_timer(1.0, self._connectivity_check)
        # HUD는 _on_image(카메라 프레임 처리)와 완전히 분리된 자체 10Hz 타이머로만
        # 갱신한다 — 카메라 프레임 처리 속도와 무관하게 항상 가벼운 주기로 돈다.
        self.create_timer(0.1, self._update_hud)

    # --- 구독 콜백 --------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._got['camera_info'] = True
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f"CameraInfo 수신 완료 — fx={self._camera_matrix[0,0]:.1f} "
                f"fy={self._camera_matrix[1,1]:.1f} dist_coeffs={self._dist_coeffs.tolist()}")

    def _on_bev_h(self, msg: Float64MultiArray) -> None:
        self._got['bev_h'] = True
        try:
            self._h_g2i = bev_h_to_ground_to_image(msg.data)
        except np.linalg.LinAlgError:
            self.get_logger().warn('/bev/H 역행렬 계산 실패(특이행렬) — 이번 값은 버림.')
            return
        self._h_g2i_wall = time.monotonic()

    def _on_path(self, msg: Path) -> None:
        self._got['path'] = True
        self._latest_path = msg
        self._latest_path_wall = time.monotonic()

    def _on_drive_status(self, msg: String) -> None:
        self._got['drive_status'] = True
        self._drive_status = msg.data

    def _on_side_slope(self, msg: Float32) -> None:
        self._got['side_slope'] = True
        self._side_slope_deg = float(msg.data)

    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._got['wheel_odom'] = True
        self._wheel_yaw_rate = float(msg.twist.twist.angular.z)

    def _on_cmd_vel_auto(self, msg: Twist) -> None:
        self._got['cmd_vel_auto'] = True
        self._cmd_vel_auto_linear_x = float(msg.linear.x)
        self._cmd_vel_auto_angular_z = float(msg.angular.z)

    def _on_speed_limit(self, msg: SpeedLimit) -> None:
        self._got['speed_limit'] = True
        # percentage 필드가 false로 오면(소스상 지금은 안 그러지만, 나중에
        # 바뀔 수도 있으니 방어적으로) 절대값(m/s)일 수 있다는 걸 로그로만
        # 알리고 HUD 표시는 그대로 진행 — 단위 오인 방지용 경고.
        if not msg.percentage and not self._warned.get('speed_limit_unit', False):
            self._warned['speed_limit_unit'] = True
            self.get_logger().warn(
                '/speed_limit이 percentage=false(절대 m/s)로 옴 — HUD는 '
                '%%로 표시하는 코드라 단위가 실제와 다를 수 있음. '
                '현재 발행자가 없으므로 발행 쪽을 확인할 것.')
        self._speed_limit_pct = float(msg.speed_limit)

    def _on_image(self, msg: CompressedImage) -> None:
        self._got['image'] = True
        self._frame_times.append(time.monotonic())  # HUD FPS 계산용 — _update_hud(10Hz)에서 소비
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        raw = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if raw is None:
            self.get_logger().error('이미지 디코드 실패 — CompressedImage 포맷 확인 필요.')
            return

        if self._camera_matrix is None:
            self._warn_once('camera_info', 'CameraInfo 대기 중 — 왜곡보정 전 원본만 표시.')
            self._draw_status(raw, ['WAITING: no CameraInfo -> undistort skipped, raw frame shown'])
            self.display_frame = raw
            return

        # flat_drive.py와 동일 순서: 왜곡보정 먼저, 그 위에 투영된 경로를 그린다.
        undistorted = cv2.undistort(raw, self._camera_matrix, self._dist_coeffs)
        h_img, w_img = undistorted.shape[:2]

        status = []
        if self._h_g2i is None:
            self._warn_once('bev_h', '/bev/H 없음 — flat_drive_node가 떠 있는지 확인할 것.')
            status.append('WAITING: no /bev/H -> check flat_drive_node (path projection unavailable)')
        elif self._latest_path is None or not self._latest_path.poses:
            self._warn_once('path', f"{self._topics['path']}가 비어있거나 아직 안 들어옴.")
            status.append(f"WAITING: {self._topics['path']} empty/not received yet")
        else:
            xy = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in self._latest_path.poses],
                dtype=np.float64)
            px, valid = project_ground_points(xy, self._h_g2i)
            self._draw_path(undistorted, px, valid, w_img, h_img)

            delay_bits = []
            d_path = sync_delay_ms(msg.header.stamp, self._latest_path.header.stamp)
            if d_path is not None:
                delay_bits.append(f'path sync {d_path:.0f}ms')
            if self._h_g2i_wall is not None:
                delay_bits.append(f'bev/H age {(time.monotonic() - self._h_g2i_wall) * 1000:.0f}ms')
            n_valid = int(valid.sum())
            status.append(
                f'projected {n_valid}/{len(valid)} pts' +
                ('  |  ' + '  '.join(delay_bits) if delay_bits else ''))
            if n_valid < len(valid):
                status.append(f'{len(valid) - n_valid} pts dropped (behind camera / w<=0)')

        self._draw_status(undistorted, status)
        self.display_frame = undistorted

    # --- HUD (10Hz 전용 타이머, _on_image와 무관하게 동작) ------------------

    def _update_hud(self) -> None:
        """카메라 프레임 처리(_on_image)와 완전히 분리된 10Hz 타이머 콜백.
        여기서만 path_arc_length_m을 계산해서, 프레임 처리 콜백의 연산량은
        이번 변경으로 늘어나지 않는다(_on_image에 추가된 건 deque.append
        한 줄뿐)."""
        hud = np.zeros((HUD_HEIGHT, HUD_WIDTH, 3), dtype=np.uint8)

        if self._drive_status == 'flat':
            status_text, status_color = 'STATUS: FLAT', (0, 255, 0)      # 초록
        elif self._drive_status == 'slope':
            status_text, status_color = 'STATUS: SLOPE', (0, 165, 255)   # 주황(BGR)
        else:
            status_text, status_color = 'STATUS: UNKNOWN', (128, 128, 128)  # 회색

        roll_text = (
            f'ROLL: {self._side_slope_deg:+.1f} deg'
            if self._side_slope_deg is not None else 'ROLL: N/A')

        fps_text = f'FPS: {fps_from_timestamps(self._frame_times):.1f}'

        if self._latest_path is not None and self._latest_path.poses:
            xy = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in self._latest_path.poses],
                dtype=np.float64)
            path_len = path_arc_length_m(xy)
        else:
            path_len = 0.0
        path_len_text = f'PATH LEN: {path_len:.2f}m'

        # rmd_x8_driver_node.py 자체 docstring에 "wheel yaw rate is
        # intentionally NOT trusted"라고 명시돼 있음(vx만 EKF에 fuse됨,
        # robot_localization odom0_config 참고) — 여기 표시하는 값은
        # 디버그/비교용일 뿐 신뢰할 제어 신호가 아니다. 화면 텍스트는
        # 짧게 유지하고(패널 폭 제한) 이 캐비어트는 주석으로만 남긴다.
        wheel_yaw_rate_text = (
            f'WHEEL YAW: {self._wheel_yaw_rate:+.2f} rad/s'
            if self._wheel_yaw_rate is not None else 'WHEEL YAW: N/A')

        cmd_vel_text = (
            f'CMD_VEL_AUTO: lin={self._cmd_vel_auto_linear_x:+.2f} '
            f'ang={self._cmd_vel_auto_angular_z:+.2f}'
            if self._cmd_vel_auto_linear_x is not None else 'CMD_VEL_AUTO: N/A')

        # speed_limit=0.0은 nav2_msgs/SpeedLimit 규약상 "제한 없음"이라는
        # 별도 의미 -- 그냥 0.0%로 찍으면 "속도를 0으로 깎았다"로 오독하기
        # 쉬워 구분 표시.
        if self._speed_limit_pct is None:
            speed_limit_text = 'SPEED LIMIT: N/A'
        elif self._speed_limit_pct <= 0.0:
            speed_limit_text = 'SPEED LIMIT: NO LIMIT'
        else:
            speed_limit_text = f'SPEED LIMIT: {self._speed_limit_pct:.0f}%'

        y = 60
        cv2.putText(hud, status_text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                    1.1, status_color, 2, cv2.LINE_AA)
        for text in (roll_text, fps_text, path_len_text, wheel_yaw_rate_text,
                     cmd_vel_text, speed_limit_text):
            y += 60
            cv2.putText(hud, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2, cv2.LINE_AA)

        self.hud_frame = hud

    # --- 그리기 -------------------------------------------------------

    @staticmethod
    def _draw_path(img: np.ndarray, px: np.ndarray, valid: np.ndarray, w_img: int, h_img: int) -> None:
        pts = px.astype(np.int32)
        n = len(pts)
        for i in range(n - 1):
            if not (valid[i] and valid[i + 1]):
                continue  # 한쪽이라도 카메라 뒤쪽/소실점이면 그 세그먼트는 안 그림
            ok, c1, c2 = cv2.clipLine((0, 0, w_img, h_img), tuple(pts[i]), tuple(pts[i + 1]))
            if not ok:
                continue  # 두 점 다 화면 밖 -> 화면과 안 겹침
            cv2.line(img, c1, c2, RED_BGR, LINE_THICKNESS, lineType=cv2.LINE_AA)
        for i in range(n):
            if valid[i] and 0 <= pts[i, 0] < w_img and 0 <= pts[i, 1] < h_img:
                cv2.circle(img, tuple(pts[i]), 3, RED_BGR, -1, lineType=cv2.LINE_AA)

    @staticmethod
    def _draw_status(img: np.ndarray, lines: list) -> None:
        # cv2.putText 기본 폰트(Hershey)는 한글을 못 그리므로 화면 텍스트는
        # 영문/숫자만 쓴다 — 한글 상세 설명은 터미널 로그(get_logger)로만 낸다.
        y = 22
        for line in lines:
            cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y += 20

    # --- 연결 상태 점검 -----------------------------------------------

    def _warn_once(self, key: str, message: str) -> None:
        if not self._warned[key]:
            self._warned[key] = True
            self.get_logger().warn(message)

    def _connectivity_check(self) -> None:
        elapsed = time.monotonic() - self._start_wall
        if elapsed < NO_DATA_WARN_SEC:
            return
        for key, topic in self._topics.items():
            if not self._got[key] and not self._warned[key]:
                self._warned[key] = True
                hint = ''
                if key == 'bev_h':
                    hint = ' (flat_drive_node가 안 떠 있으면 이 토픽 자체가 없음)'
                elif key == 'speed_limit':
                    hint = ' (현재 발행자가 없음 -- 정상, N/A로 표시됨)'
                self.get_logger().warn(
                    f"{NO_DATA_WARN_SEC:.0f}초 넘게 '{topic}'에서 메시지가 안 들어옴.{hint} "
                    f"ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY/RMW_IMPLEMENTATION 및 "
                    f"`ros2 topic list`/`ros2 topic info {topic}`(QoS 확인)로 점검할 것.")

    # --- --dummy 모드: 로컬 스모크테스트용 더미 발행 -----------------------

    def _setup_dummy(self, args) -> None:
        img_qos = _sensor_data_qos_depth1()
        self._dummy_image_pub = self.create_publisher(CompressedImage, args.image_topic, img_qos)
        self._dummy_info_pub = self.create_publisher(CameraInfo, args.camera_info_topic, img_qos)
        self._dummy_bevh_pub = self.create_publisher(Float64MultiArray, args.bev_h_topic, _reliable_qos())
        self._dummy_path_pub = self.create_publisher(Path, args.path_topic, _reliable_qos())
        self._dummy_status_pub = self.create_publisher(String, args.drive_status_topic, _reliable_qos())
        self._dummy_side_slope_pub = self.create_publisher(Float32, args.side_slope_topic, _reliable_qos())
        self._dummy_wheel_odom_pub = self.create_publisher(
            Odometry, args.wheel_odom_topic, _wheel_odom_qos())
        self._dummy_cmd_vel_auto_pub = self.create_publisher(
            Twist, args.cmd_vel_auto_topic, _reliable_qos())
        self._dummy_speed_limit_pub = self.create_publisher(
            SpeedLimit, args.speed_limit_topic, _reliable_qos())

        self._dummy_w, self._dummy_h = 640, 480
        fx = fy = 500.0
        cx, cy = self._dummy_w / 2.0, self._dummy_h / 2.0
        self._dummy_k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        # 배럴 왜곡을 일부러 넣어서 undistort 단계가 눈에 보이게 함(격자무늬가 휨).
        self._dummy_d = np.array([-0.28, 0.08, 0.0, 0.0, 0.0], dtype=np.float64)

        # 지면(x_forward,y_left) <-> 화면 픽셀 4점 대응으로 h_g2i를 구성한다.
        # 실제 flat_drive.py의 R_BODY_TO_OPTICAL 등 내부 회전 규약을 그대로
        # 재현할 필요는 없음 — 이 더미의 목적은 "먼 점일수록 화면 위쪽/작게,
        # 카메라 뒤쪽 점은 w<=0으로 필터링됨"이라는 이 스크립트 쪽 투영/필터
        # 로직이 실제로 동작하는지 스모크테스트하는 것뿐이라 임의의 사다리꼴
        # 원근 대응으로 충분하다.
        ground_pts = np.float32([[1.0, 0.6], [1.0, -0.6], [4.0, 1.8], [4.0, -1.8]])
        image_pts = np.float32([[120, 470], [520, 470], [280, 260], [360, 260]])
        self._dummy_h_g2i = cv2.getPerspectiveTransform(ground_pts, image_pts)
        # getPerspectiveTransform은 동차행렬 스케일 모호성 때문에 전체 부호가
        # 임의로 나올 수 있다 — 실제 flat_drive.py의 h_g2i는 camera_matrix(양의
        # 대각)와 순수 회전으로 유도돼 "카메라 앞쪽 = w>0"이 물리적으로 보장되지만
        # (project_ground_points의 valid=w>0 전제), 이 더미는 4점 대응만으로 풀기
        # 때문에 그 보장이 없다. 캘리브레이션에 쓴 첫 점(카메라 앞쪽으로 알고 있는
        # 점)의 w 부호를 확인해서 필요하면 행렬 전체를 뒤집어 규약을 맞춘다(같은
        # 호모그래피, 픽셀 결과(u/w,v/w)는 동일하게 유지됨).
        probe_w = (self._dummy_h_g2i @ np.array([ground_pts[0][0], ground_pts[0][1], 1.0]))[2]
        if probe_w < 0:
            self._dummy_h_g2i = -self._dummy_h_g2i

        self._dummy_t = 0.0
        self.create_timer(1.0 / 15.0, self._publish_dummy_path_and_h)  # 실측 근사 15Hz
        self.create_timer(1.0 / 15.0, self._publish_dummy_image_and_info)
        self.get_logger().info(
            "--dummy 모드: 실제 로봇 없이 이미지/CameraInfo/bev_H/path/"
            "drive_status/side_slope_angle_deg/wheel_odom/cmd_vel_auto를 자체 발행합니다.")

    def _publish_dummy_image_and_info(self) -> None:
        now = self.get_clock().now().to_msg()
        w, h = self._dummy_w, self._dummy_h
        frame = np.full((h, w, 3), (40, 40, 40), dtype=np.uint8)
        step = 40
        for x in range(0, w, step):
            cv2.line(frame, (x, 0), (x, h), (90, 90, 90), 1)
        for y in range(0, h, step):
            cv2.line(frame, (0, y), (w, y), (90, 90, 90), 1)
        cv2.putText(frame, 'DUMMY CAMERA FEED', (16, h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

        ok, enc = cv2.imencode('.jpg', frame)
        img_msg = CompressedImage()
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'dummy_camera_color_optical_frame'
        img_msg.format = 'jpeg'
        img_msg.data = enc.tobytes() if ok else b''
        self._dummy_image_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width, info_msg.height = w, h
        info_msg.k = self._dummy_k.flatten().tolist()
        info_msg.d = self._dummy_d.tolist()
        self._dummy_info_pub.publish(info_msg)

    def _publish_dummy_path_and_h(self) -> None:
        self._dummy_t += 1.0 / 15.0
        now = self.get_clock().now().to_msg()

        h_msg = Float64MultiArray()
        # flat_drive.py와 동일하게 "역행렬"을 발행한다 — 이 뷰어가 받아서
        # 다시 뒤집는 double-inverse 경로 전체를 실제와 같은 방식으로 태운다.
        h_msg.data = np.linalg.inv(self._dummy_h_g2i).flatten().tolist()
        self._dummy_bevh_pub.publish(h_msg)

        # HUD 색상/값 전환이 눈에 보이게 — status는 3초 간격으로 flat/slope
        # 번갈아, roll은 -15~15도 사인파로 왔다갔다.
        status_msg = String()
        status_msg.data = 'slope' if int(self._dummy_t) % 6 < 3 else 'flat'
        self._dummy_status_pub.publish(status_msg)

        side_slope_msg = Float32()
        side_slope_msg.data = 15.0 * float(np.sin(self._dummy_t * 0.5))
        self._dummy_side_slope_pub.publish(side_slope_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.twist.twist.angular.z = 0.3 * float(np.sin(self._dummy_t * 0.7))
        self._dummy_wheel_odom_pub.publish(odom_msg)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5 + 0.2 * float(np.sin(self._dummy_t * 0.3))
        cmd_vel_msg.angular.z = 0.4 * float(np.cos(self._dummy_t * 0.4))
        self._dummy_cmd_vel_auto_pub.publish(cmd_vel_msg)

        # 0%~100% 사이를 오가되, 가끔 0.0("제한 없음" 특수값, 실제 노드와
        # 동일 규약)도 섞어서 NO LIMIT 표시 분기도 스모크테스트되게 함.
        speed_limit_msg = SpeedLimit()
        speed_limit_msg.header.stamp = now
        speed_limit_msg.percentage = True
        cycle = int(self._dummy_t) % 10
        speed_limit_msg.speed_limit = 0.0 if cycle < 2 else 40.0 + 30.0 * float(
            np.sin(self._dummy_t * 0.2))
        self._dummy_speed_limit_pub.publish(speed_limit_msg)

        # 완만한 S자 전방 경로 + 카메라 뒤쪽(x_forward<0) 이상치 한 점을 섞어서
        # w<=0 필터링이 실제로 동작하는지 매 프레임 스모크테스트한다.
        pts = [(-1.0, 0.0)]
        for i in range(15):
            x = 1.0 + i * 0.25
            y = 0.5 * np.sin(x * 0.8 + self._dummy_t)
            pts.append((x, float(y)))

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'dummy_camera_color_optical_frame'
        for x, y in pts:
            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self._dummy_path_pub.publish(path_msg)


# ---------------------------------------------------------------------------
# 메인 루프
# ---------------------------------------------------------------------------

def _save_snapshot(node: PathCameraOverlay) -> None:
    if node.display_frame is None:
        print('[snapshot] 아직 표시할 프레임이 없어 저장 스킵.')
        return
    SNAPSHOT_DIR.mkdir(exist_ok=True)
    ts = time.strftime('%Y%m%d_%H%M%S')
    out_path = SNAPSHOT_DIR / f'overlay_{ts}.png'
    cv2.imwrite(str(out_path), node.display_frame)
    print(f'[snapshot] 저장됨: {out_path}')


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--image-topic', default='/drive/camera/color/image_raw/compressed',
                         help='RGB 컬러 압축 이미지 토픽 (기본: flat_drive_node의 color_topic 기본값과 동일)')
    parser.add_argument('--camera-info-topic', default='/drive/camera/color/camera_info',
                         help='CameraInfo 토픽 (K/D 획득용)')
    parser.add_argument('--bev-h-topic', default='/bev/H', help='flat_drive_node가 발행하는 호모그래피 토픽')
    parser.add_argument('--path-topic', default='/path', help='덧씌울 경로 토픽')
    parser.add_argument('--drive-status-topic', default='/drive/status',
                         help='slope_decision.py가 발행하는 flat/slope 상태 토픽 (HUD STATUS용)')
    parser.add_argument('--side-slope-topic', default='/terrain/side_slope_angle_deg',
                         help='slope_decision.py가 발행하는 좌우 기울기 토픽 (HUD ROLL용)')
    parser.add_argument('--steer-lookahead-m', type=float, default=1.0,
                         help='approx_steering_deg()용 전방 거리(m, 기본 1.0) — 현재 HUD엔 표시 안 함, 함수 자체는 남겨둠')
    parser.add_argument('--wheel-odom-topic', default='/wheel/odom',
                         help='rmd_x8_driver_node.py가 발행하는 휠 오도메트리 토픽 (HUD WHEEL YAW용, '
                              'twist.twist.angular.z — 단 이 값은 소스 docstring상 EKF에서도 신뢰 안 함)')
    parser.add_argument('--cmd-vel-auto-topic', default='/cmd_vel_auto',
                         help='controller_server(MPPI)가 내는 자율주행 속도 명령 토픽 (HUD CMD_VEL_AUTO용, '
                              'nav2.launch.py에서 /cmd_vel -> /cmd_vel_auto로 remap됨)')
    parser.add_argument('--speed-limit-topic', default='/speed_limit',
                         help='nav2 속도제한 토픽 (HUD SPEED LIMIT용). 이 토픽을 '
                              '발행하던 slope_speed_limiter_node 삭제됨 -- 현재는 발행자가 '
                              '없어 항상 N/A로 표시된다')
    parser.add_argument('--dummy', action='store_true',
                         help='실제 로봇 없이 로컬에서 토픽들을 자체 발행해 스모크테스트')
    return parser


def main():
    args = _build_arg_parser().parse_args()

    rclpy.init()
    node = PathCameraOverlay(args, dummy=args.dummy)

    window = 'path_camera_overlay (q: quit, s: save snapshot)'
    hud_window = 'path_camera_overlay HUD'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(hud_window, cv2.WINDOW_NORMAL)

    print('실행 중... 뷰어 창에 포커스 준 상태에서 q 종료 / s 스냅샷 저장, Ctrl+C로도 종료')
    if not args.dummy:
        print(f"필수: flat_drive_node가 떠 있어야 '{args.bev_h_topic}'가 나옵니다 (없으면 경로 투영 불가).")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            if node.display_frame is not None:
                cv2.imshow(window, node.display_frame)
            if node.hud_frame is not None:
                cv2.imshow(hud_window, node.hud_frame)

            # q/s는 메인 창(경로 오버레이) 기준 그대로 — HUD는 정보 표시 전용이라 별도 키 없음.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                _save_snapshot(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
