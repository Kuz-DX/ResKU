#!/usr/bin/env python3
"""
path_ld_lp_visulizer.py — /path에 더해 pure pursuit의 lookahead distance(ld,
원)와 lookahead point(lp, 타겟점)를 RGB 카메라 화면에 같이 덧씌우는 노트북
전용 로컬 디버그 뷰어 (독립 실행 스크립트, path_visualizer.py의 자매 스크립트).

dolbotz 패키지 의존성 없음 — rclpy + sensor_msgs/nav_msgs/std_msgs +
opencv-python + numpy만 필요. path_visualizer.py와 마찬가지로 utils/ 아래
있지만 빌드/설치 없이 `python3 path_ld_lp_visulizer.py`로 바로 실행된다.
purepursuit_node(dolbotz/purepursuit.py) 코드는 import하지 않고, 그 안의
lookahead 계산 로직(_compute_pure_pursuit)만 여기 순수함수로 그대로
재현한다 — purepursuit_node는 ld/lp를 토픽으로 발행하지 않고 매 컨트롤
틱마다 내부에서만 계산하고 버리기 때문에(어떤 토픽으로도 안 나옴), 실제
운용 중인 노드를 건드리지 않고 확인하려면 이 방법뿐이다.

=== 필수: flat_drive_node가 항상 같이 떠 있어야 함 ===
path_visualizer.py와 동일한 이유 — /bev/H는 flat_drive_node만 발행한다.
없으면 경로/ld/lp 전부 투영을 못 하고 카메라 원본만 보여준다.

=== base_link vs camera_link 오프셋 ===
/path의 좌표(x_forward, y_left)는 camera_link 기준이다. 그런데
purepursuit_node는 /path를 정적 TF로 base_link로 옮긴 뒤에 lookahead
계산을 한다(purepursuit.py 모듈 docstring, reduced_odom_bringup.launch.py/
purepursuit.launch.py의 static_transform_publisher: base_link -> camera_link).
[카메라 TF] 카메라가 x,y는 base_link 중앙(0,0)에 맞춰 재장착돼서 한동안
이 x 오프셋이 0이었음(--base-to-camera-x-m 기본값도 0).
[2026-09-03 최신 실측] base_link 기준 x=+90.461mm(전방)이므로
--base-to-camera-x-m 기본값도 0.090461로 맞춘다.

=== ld(원)/lp(점) 계산 로직 — purepursuit.py._compute_pure_pursuit()와 동일 ===
1. 경로점을 근->원 순서로 훑어서, 로봇 원점으로부터의 거리가
   lookahead_distance_m 이상인 첫 점을 lp로 삼는다.
2. 경로 전체가 그 반경 안이면(짧은 경로) 경로의 마지막 점을 lp로 대신
   쓴다(purepursuit.py의 폴백과 동일).
3. 경로가 비어있으면 lp 없음.
ld는 로봇 원점 중심의 원으로 화면에 그린다 — 실제로는 지면 위 원이라
원근 투영을 거치면 타원/곡선처럼 보인다(project_ground_points로 여러 점을
찍어 이은 선).

=== 나머지(왜곡보정 순서, /bev/H 부호, QoS 등) ===
전부 path_visualizer.py와 동일 — 자세한 근거는 그 파일 상단 docstring 참고.
이 파일은 그 위에 ld/lp 오버레이만 추가한 것이라 중복 설명은 생략한다.

사용법:
  python3 path_ld_lp_visulizer.py                              # 실제 로봇/네트워크에 붙어서 구독
  python3 path_ld_lp_visulizer.py --lookahead-distance-m 0.8    # purepursuit_node와 다른 ld로 테스트
  python3 path_ld_lp_visulizer.py --dummy                       # 로컬 스모크테스트(더미 발행 내장)

키: q  종료
    s  현재 화면 스냅샷 PNG 저장 (path_ld_lp_overlay_snapshots/)
"""
import argparse
import time
from pathlib import Path as FsPath

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path

SNAPSHOT_DIR = FsPath(__file__).resolve().parent / 'path_ld_lp_overlay_snapshots'

RED_BGR = (0, 0, 255)          # 경로 선
CYAN_BGR = (255, 255, 0)       # lookahead distance 원
YELLOW_BGR = (0, 255, 255)     # lookahead point(타겟)
GREEN_BGR = (0, 255, 0)        # 로봇(base_link) 원점 마커
LINE_THICKNESS = 3
NO_DATA_WARN_SEC = 5.0


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
    (BEST_EFFORT/VOLATILE/depth=5)와 동일하되 depth만 1로 낮춘 프로필
    (path_visualizer.py의 _sensor_data_qos_depth1()와 동일 근거/구현 —
    dolbotz.utils.qos.SENSOR_DATA_QOS_DEPTH1과 스펙 동일, 이 파일은 dolbotz
    패키지 무의존 설계라 인라인으로 중복 정의)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


# ---------------------------------------------------------------------------
# 순수 계산 (ROS 의존성 없음)
# ---------------------------------------------------------------------------

def bev_h_to_ground_to_image(bev_h_flat) -> np.ndarray:
    """/bev/H로 받은 flatten(9,) 배열 -> ground-to-image 호모그래피 h_g2i.
    (path_visualizer.py와 동일 — 근거는 그쪽 파일 참고)"""
    h_i2g = np.asarray(bev_h_flat, dtype=np.float64).reshape(3, 3)
    return np.linalg.inv(h_i2g)


def project_ground_points(xy_forward_left: np.ndarray, h_g2i: np.ndarray):
    """(N,2) x_forward,y_left 지면 좌표 -> (N,2) 픽셀 좌표, (N,) valid bool.
    (path_visualizer.py와 동일)"""
    n = xy_forward_left.shape[0]
    if n == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty((0,), dtype=bool)
    homog = np.hstack([xy_forward_left, np.ones((n, 1), dtype=np.float64)])
    proj = (h_g2i @ homog.T).T
    w = proj[:, 2]
    valid = w > 1e-6
    px = np.zeros((n, 2), dtype=np.float64)
    px[valid, 0] = proj[valid, 0] / w[valid]
    px[valid, 1] = proj[valid, 1] / w[valid]
    return px, valid


def find_lookahead_point(xy_forward_left: np.ndarray, lookahead_m: float, robot_xy: tuple):
    """purepursuit.py._compute_pure_pursuit()의 타겟 선택 로직과 동일:
    robot_xy(로봇/base_link 원점, 경로와 같은 좌표계)로부터의 거리가
    lookahead_m 이상인 첫 경로점을 반환. 없으면(경로 전체가 반경 안)
    마지막 점을 대신 반환. 경로가 비어있으면 None."""
    if xy_forward_left.shape[0] == 0:
        return None
    rel = xy_forward_left - np.asarray(robot_xy, dtype=np.float64)
    dists = np.linalg.norm(rel, axis=1)
    beyond = np.flatnonzero(dists >= lookahead_m)
    idx = int(beyond[0]) if beyond.size > 0 else xy_forward_left.shape[0] - 1
    return float(xy_forward_left[idx, 0]), float(xy_forward_left[idx, 1])


def circle_ground_points(center_xy: tuple, radius_m: float, n_points: int = 72) -> np.ndarray:
    """center_xy 중심, 반지름 radius_m인 원 위의 점들을 (n_points,2)
    x_forward,y_left 좌표로 반환 — lookahead distance를 화면에 그리기 위함."""
    theta = np.linspace(0.0, 2.0 * np.pi, n_points, endpoint=True)
    cx, cy = center_xy
    x = cx + radius_m * np.cos(theta)
    y = cy + radius_m * np.sin(theta)
    return np.stack([x, y], axis=1)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def sync_delay_ms(stamp_a, stamp_b):
    ta, tb = stamp_to_sec(stamp_a), stamp_to_sec(stamp_b)
    if ta == 0.0 or tb == 0.0:
        return None
    return abs(ta - tb) * 1000.0


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------

class PathLdLpOverlay(Node):
    def __init__(self, args, dummy: bool = False):
        super().__init__('path_ld_lp_overlay')

        self._camera_matrix = None
        self._dist_coeffs = None
        self._h_g2i = None
        self._h_g2i_wall = None
        self._latest_path = None
        self.display_frame = None
        self._start_wall = time.monotonic()
        self._warned = {'image': False, 'camera_info': False, 'bev_h': False, 'path': False}

        self._lookahead_m = float(args.lookahead_distance_m)
        # camera_link 원점에서 본 로봇(base_link) 원점 위치 — docstring
        # "base_link vs camera_link 오프셋" 절 참고.
        self._robot_xy = (-float(args.base_to_camera_x_m), 0.0)

        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_camera_info, _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, args.image_topic, self._on_image, _sensor_data_qos_depth1())
        self.create_subscription(
            Float64MultiArray, args.bev_h_topic, self._on_bev_h, _reliable_qos())
        self.create_subscription(
            Path, args.path_topic, self._on_path, _reliable_qos())

        self._topics = {
            'image': args.image_topic, 'camera_info': args.camera_info_topic,
            'bev_h': args.bev_h_topic, 'path': args.path_topic,
        }
        self._got = {k: False for k in self._topics}

        if dummy:
            self._setup_dummy(args)

        self.create_timer(1.0, self._connectivity_check)

        self.get_logger().info(
            f'lookahead_distance_m={self._lookahead_m}m, '
            f'robot origin in path frame={self._robot_xy} '
            f'(base_to_camera_x_m={args.base_to_camera_x_m}m)')

    # --- 구독 콜백 --------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._got['camera_info'] = True
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f"CameraInfo 수신 완료 — fx={self._camera_matrix[0,0]:.1f} "
                f"fy={self._camera_matrix[1,1]:.1f}")

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

    def _on_image(self, msg: CompressedImage) -> None:
        self._got['image'] = True
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

        undistorted = cv2.undistort(raw, self._camera_matrix, self._dist_coeffs)
        h_img, w_img = undistorted.shape[:2]

        status = []
        if self._h_g2i is None:
            self._warn_once('bev_h', '/bev/H 없음 — flat_drive_node가 떠 있는지 확인할 것.')
            status.append('WAITING: no /bev/H -> check flat_drive_node (projection unavailable)')
        elif self._latest_path is None or not self._latest_path.poses:
            self._warn_once('path', f"{self._topics['path']}가 비어있거나 아직 안 들어옴.")
            status.append(f"WAITING: {self._topics['path']} empty/not received yet")
        else:
            xy = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in self._latest_path.poses],
                dtype=np.float64)

            # 경로
            px, valid = project_ground_points(xy, self._h_g2i)
            self._draw_path(undistorted, px, valid, w_img, h_img)

            # 로봇(base_link) 원점
            robot_px, robot_valid = project_ground_points(
                np.array([self._robot_xy], dtype=np.float64), self._h_g2i)
            if robot_valid[0]:
                cv2.circle(undistorted, tuple(robot_px[0].astype(np.int32)), 6,
                           GREEN_BGR, -1, lineType=cv2.LINE_AA)

            # lookahead distance 원
            circle_xy = circle_ground_points(self._robot_xy, self._lookahead_m)
            circle_px, circle_valid = project_ground_points(circle_xy, self._h_g2i)
            self._draw_circle(undistorted, circle_px, circle_valid, w_img, h_img)

            # lookahead point(타겟)
            lp = find_lookahead_point(xy, self._lookahead_m, self._robot_xy)
            lp_text = 'lp: N/A'
            if lp is not None:
                lp_px, lp_valid = project_ground_points(
                    np.array([lp], dtype=np.float64), self._h_g2i)
                if lp_valid[0] and 0 <= lp_px[0, 0] < w_img and 0 <= lp_px[0, 1] < h_img:
                    pt = tuple(lp_px[0].astype(np.int32))
                    cv2.circle(undistorted, pt, 9, YELLOW_BGR, -1, lineType=cv2.LINE_AA)
                    cv2.circle(undistorted, pt, 9, (0, 0, 0), 2, lineType=cv2.LINE_AA)
                    if robot_valid[0]:
                        cv2.line(undistorted, tuple(robot_px[0].astype(np.int32)), pt,
                                 YELLOW_BGR, 1, lineType=cv2.LINE_AA)
                lp_dist = float(np.hypot(lp[0] - self._robot_xy[0], lp[1] - self._robot_xy[1]))
                lp_text = f'lp: ({lp[0]:.2f}, {lp[1]:.2f})m  dist={lp_dist:.2f}m'

            delay_bits = []
            d_path = sync_delay_ms(msg.header.stamp, self._latest_path.header.stamp)
            if d_path is not None:
                delay_bits.append(f'path sync {d_path:.0f}ms')
            n_valid = int(valid.sum())
            status.append(
                f'projected {n_valid}/{len(valid)} path pts' +
                ('  |  ' + '  '.join(delay_bits) if delay_bits else ''))
            status.append(f'ld: {self._lookahead_m:.2f}m (cyan circle)  |  {lp_text} (yellow)')

        self._draw_status(undistorted, status)
        self.display_frame = undistorted

    # --- 그리기 -------------------------------------------------------

    @staticmethod
    def _draw_path(img: np.ndarray, px: np.ndarray, valid: np.ndarray, w_img: int, h_img: int) -> None:
        pts = px.astype(np.int32)
        n = len(pts)
        for i in range(n - 1):
            if not (valid[i] and valid[i + 1]):
                continue
            ok, c1, c2 = cv2.clipLine((0, 0, w_img, h_img), tuple(pts[i]), tuple(pts[i + 1]))
            if not ok:
                continue
            cv2.line(img, c1, c2, RED_BGR, LINE_THICKNESS, lineType=cv2.LINE_AA)
        for i in range(n):
            if valid[i] and 0 <= pts[i, 0] < w_img and 0 <= pts[i, 1] < h_img:
                cv2.circle(img, tuple(pts[i]), 3, RED_BGR, -1, lineType=cv2.LINE_AA)

    @staticmethod
    def _draw_circle(img: np.ndarray, px: np.ndarray, valid: np.ndarray, w_img: int, h_img: int) -> None:
        """지면 위 원을 원근 투영해서 그린다 — 화면상으로는 곡선/타원처럼
        보인다(project_ground_points로 찍은 점들을 순서대로 이음)."""
        pts = px.astype(np.int32)
        n = len(pts)
        for i in range(n - 1):
            if not (valid[i] and valid[i + 1]):
                continue
            ok, c1, c2 = cv2.clipLine((0, 0, w_img, h_img), tuple(pts[i]), tuple(pts[i + 1]))
            if not ok:
                continue
            cv2.line(img, c1, c2, CYAN_BGR, 2, lineType=cv2.LINE_AA)

    @staticmethod
    def _draw_status(img: np.ndarray, lines: list) -> None:
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
                hint = ' (flat_drive_node가 안 떠 있으면 이 토픽 자체가 없음)' if key == 'bev_h' else ''
                self.get_logger().warn(
                    f"{NO_DATA_WARN_SEC:.0f}초 넘게 '{topic}'에서 메시지가 안 들어옴.{hint}")

    # --- --dummy 모드 ---------------------------------------------------

    def _setup_dummy(self, args) -> None:
        img_qos = _sensor_data_qos_depth1()
        self._dummy_image_pub = self.create_publisher(CompressedImage, args.image_topic, img_qos)
        self._dummy_info_pub = self.create_publisher(CameraInfo, args.camera_info_topic, img_qos)
        self._dummy_bevh_pub = self.create_publisher(Float64MultiArray, args.bev_h_topic, _reliable_qos())
        self._dummy_path_pub = self.create_publisher(Path, args.path_topic, _reliable_qos())

        self._dummy_w, self._dummy_h = 640, 480
        fx = fy = 500.0
        cx, cy = self._dummy_w / 2.0, self._dummy_h / 2.0
        self._dummy_k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self._dummy_d = np.array([-0.28, 0.08, 0.0, 0.0, 0.0], dtype=np.float64)

        ground_pts = np.float32([[1.0, 0.6], [1.0, -0.6], [4.0, 1.8], [4.0, -1.8]])
        image_pts = np.float32([[120, 470], [520, 470], [280, 260], [360, 260]])
        self._dummy_h_g2i = cv2.getPerspectiveTransform(ground_pts, image_pts)
        probe_w = (self._dummy_h_g2i @ np.array([ground_pts[0][0], ground_pts[0][1], 1.0]))[2]
        if probe_w < 0:
            self._dummy_h_g2i = -self._dummy_h_g2i

        self._dummy_t = 0.0
        self.create_timer(1.0 / 15.0, self._publish_dummy_path_and_h)
        self.create_timer(1.0 / 15.0, self._publish_dummy_image_and_info)
        self.get_logger().info(
            "--dummy 모드: 실제 로봇 없이 이미지/CameraInfo/bev_H/path를 자체 발행합니다.")

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
        h_msg.data = np.linalg.inv(self._dummy_h_g2i).flatten().tolist()
        self._dummy_bevh_pub.publish(h_msg)

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

def _save_snapshot(node: PathLdLpOverlay) -> None:
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
    parser.add_argument('--image-topic', default='/drive/camera/color/image_raw/compressed')
    parser.add_argument('--camera-info-topic', default='/drive/camera/color/camera_info')
    parser.add_argument('--bev-h-topic', default='/bev/H')
    parser.add_argument('--path-topic', default='/path')
    parser.add_argument('--lookahead-distance-m', type=float, default=0.5,
                         help='purepursuit_node의 lookahead_distance_m 파라미터와 맞출 것 (기본값도 동일: 0.5)')
    parser.add_argument('--base-to-camera-x-m', type=float, default=0.090461,
                         help='base_link -> camera_link 전방 오프셋[m] (reduced_odom_bringup.launch.py/purepursuit.launch.py의 '
                              'static_transform_publisher CAD 실측값과 동일)')
    parser.add_argument('--dummy', action='store_true',
                         help='실제 로봇 없이 로컬에서 토픽들을 자체 발행해 스모크테스트')
    return parser


def main():
    args = _build_arg_parser().parse_args()

    rclpy.init()
    node = PathLdLpOverlay(args, dummy=args.dummy)

    window = 'path_ld_lp_overlay (q: quit, s: save snapshot)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    print('실행 중... 뷰어 창에 포커스 준 상태에서 q 종료 / s 스냅샷 저장, Ctrl+C로도 종료')
    if not args.dummy:
        print(f"필수: flat_drive_node가 떠 있어야 '{args.bev_h_topic}'가 나옵니다 (없으면 투영 불가).")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            if node.display_frame is not None:
                cv2.imshow(window, node.display_frame)

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
