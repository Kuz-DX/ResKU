"""path_camera_overlay_relay.py — /path를 카메라 원본 화면에 빨간 선으로
덧씌워 ROS 토픽으로 재발행하는 릴레이 노드 (RViz Image 디스플레이용).

=== 왜 필요한가 ===
utils/path_visualizer.py가 이미 이 기능(경로를 카메라 화면에 투영)을 하지만,
그건 cv2.imshow로 자체 창을 띄우는 노트북 전용 독립 실행 스크립트라(dolbotz
패키지 의존성 없음, `python3 path_visualizer.py`로 직접 실행) RViz에는 못
넣는다. 이 노드는 같은 투영 수학을 ROS Image 토픽으로 발행만 하는 버전 —
RViz의 다른 Image 디스플레이들(/bev/debug_overlay 등)과 나란히 한 화면에서
볼 수 있게 하기 위함.

핵심 투영 함수(bev_h_to_ground_to_image, project_ground_points)는
path_visualizer.py 것과 동일한 로직이지만 일부러 따로 복제했다 —
path_visualizer.py를 "dolbotz 패키지 의존성 전혀 없는 독립 스크립트"로
유지하려는 그 파일 자체의 설계 의도를 안 깨려는 목적(그 파일에서 import
해오면 이 노드가 설치될 때 그 파일도 dolbotz 패키지의 일부로 취급되는
꼴이 됨).

=== BEV 대비 이 뷰가 직관적인 이유 ===
RViz의 /bev/centerline_overlay 등은 진짜 위에서 내려다본 균일 축척
BEV(200px=6m)라, 실제로는 짧은 거리(1~2m대)도 좁은 조각으로만 보여서
"경로가 끊긴 것처럼" 보이기 쉽다. 이 노드가 만드는 화면은 카메라
원근 시점 그대로라 사람이 실제로 보는 눈높이/느낌과 일치해서 직관적으로
더 잘 읽힌다. 대신 원근
때문에 가까운 거리가 화면에서 과장되게 길어 보이는 착시가 있다는 점은
감안할 것(같은 이유로 path_visualizer.py HUD의 PATH LEN[m] 실측값과
같이 보는 걸 권장).

구독: color_topic       (sensor_msgs/CompressedImage, BEST_EFFORT/depth=1)
      camera_info_topic (sensor_msgs/CameraInfo,      BEST_EFFORT/depth=1)
      /bev/H             (std_msgs/Float64MultiArray,  RELIABLE) — flat_drive_node
      path_topic         (nav_msgs/Path,                RELIABLE) — 기본 /path
발행: /debug/path_camera_overlay (sensor_msgs/Image, bgr8)

사용법:
  ros2 run dolbotz path_camera_overlay_relay
  ros2 run dolbotz path_camera_overlay_relay --ros-args -p path_topic:=/flatdrive/planned_path
"""
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from cv_bridge import CvBridge

RED_BGR = (0, 0, 255)
LINE_THICKNESS = 3


def _reliable_qos() -> QoSProfile:
    """/bev/H, /path 발행자 쪽 기본 QoS(RELIABLE, depth 10)와 맞춘 프로필
    (path_visualizer.py의 _reliable_qos()와 동일 근거)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
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
# 순수 계산 (ROS 의존성 없음) — path_visualizer.py와 동일 로직(의도적 중복,
# 파일 상단 docstring 참고)
# ---------------------------------------------------------------------------

def bev_h_to_ground_to_image(bev_h_flat) -> np.ndarray:
    """/bev/H로 받은 flatten(9,) 배열 -> ground-to-image 호모그래피 h_g2i.
    flat_drive.py가 /bev/H에 발행하는 값은 h_i2g = inv(h_g2i)라 다시 뒤집는다."""
    h_i2g = np.asarray(bev_h_flat, dtype=np.float64).reshape(3, 3)
    return np.linalg.inv(h_i2g)


def project_ground_points(xy_forward_left: np.ndarray, h_g2i: np.ndarray):
    """(N,2) x_forward,y_left 지면 좌표 -> (N,2) 픽셀 좌표, (N,) valid bool.
    valid=False(w<=0, 카메라 뒤쪽/소실점 방향)는 호출부가 반드시 걸러서 그려야 함."""
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


def draw_path(img: np.ndarray, px: np.ndarray, valid: np.ndarray, w_img: int, h_img: int) -> None:
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


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------

class PathCameraOverlayRelay(Node):
    def __init__(self):
        super().__init__('path_camera_overlay_relay')

        self.declare_parameter('color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/drive/camera/color/camera_info')
        self.declare_parameter('bev_h_topic', '/bev/H')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('output_topic', '/debug/path_camera_overlay')

        color_topic = str(self.get_parameter('color_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        bev_h_topic = str(self.get_parameter('bev_h_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._h_g2i = None
        self._latest_path: Path | None = None

        self._pub = self.create_publisher(Image, output_topic, 10)

        self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, color_topic, self._on_image, _sensor_data_qos_depth1())
        self.create_subscription(
            Float64MultiArray, bev_h_topic, self._on_bev_h, _reliable_qos())
        self.create_subscription(
            Path, path_topic, self._on_path, _reliable_qos())

        self.get_logger().info(
            f'path_camera_overlay_relay ready — color={color_topic}, path={path_topic}, '
            f'bev_h={bev_h_topic} -> {output_topic}')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _on_bev_h(self, msg: Float64MultiArray) -> None:
        try:
            self._h_g2i = bev_h_to_ground_to_image(msg.data)
        except np.linalg.LinAlgError:
            self.get_logger().warn('/bev/H 역행렬 계산 실패(특이행렬) — 이번 값은 버림.')

    def _on_path(self, msg: Path) -> None:
        self._latest_path = msg

    def _on_image(self, msg: CompressedImage) -> None:
        if self._camera_matrix is None:
            return  # CameraInfo 아직 없음 -- 왜곡보정 불가, 발행 스킵

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        raw = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if raw is None:
            self.get_logger().error('이미지 디코드 실패.', throttle_duration_sec=5.0)
            return

        # flat_drive.py와 동일 순서: 왜곡보정 먼저, 그 위에 투영된 경로를 그린다
        # (ground_to_image_homography가 왜곡보정된 이미지 좌표계를 전제로 함).
        undistorted = cv2.undistort(raw, self._camera_matrix, self._dist_coeffs)
        h_img, w_img = undistorted.shape[:2]

        if self._h_g2i is not None and self._latest_path is not None and self._latest_path.poses:
            xy = np.array(
                [[p.pose.position.x, p.pose.position.y] for p in self._latest_path.poses],
                dtype=np.float64)
            px, valid = project_ground_points(xy, self._h_g2i)
            draw_path(undistorted, px, valid, w_img, h_img)

        out_msg = self._bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
        out_msg.header = msg.header
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathCameraOverlayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
