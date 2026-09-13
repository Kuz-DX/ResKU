#!/usr/bin/env python3
"""
terrain_viz_relay.py — 32FC1(float) 지형 이미지를 RViz Image 디스플레이가
바로 볼 수 있는 8bit 컬러맵 이미지로 정규화/재발행하는 순수 릴레이 노드.

=== 왜 필요한가 ===
gradient_map.py가 발행하는 다음 세 토픽은 전부 32FC1(픽셀=float32 물리값)이다
(gradient_map.py 소스 확인, cv2_to_imgmsg(..., encoding='32FC1') 한 호출을
gx/gy/magnitude/direction/slope_deg 다섯 개가 공유):
  /terrain/elevation_map       (elevation_map.py 발행, 단위 m, NaN=미측정 셀)
  /terrain/slope_deg           (gradient_map.py 발행, 단위 deg)
  /terrain/gradient_magnitude  (gradient_map.py 발행, 단위 deg/px 근사)
rviz_default_plugins의 Image 디스플레이는 32FC1을 min/max 자동정규화한
흑백으로는 띄울 수 있지만(버전에 따라 "Normalize Range" 옵션 필요),
이 프로젝트 데이터는 NaN 셀이 섞여 있어(min/max 계산이 NaN에 오염되면
전체가 검게 나오거나 자동정규화 자체가 깨짐 — elevation_map.py/gradient_map.py
양쪽 다 근접 이슈로 NaN이 흔함, 이번 세션에서 반복 확인된 사실) 그대로
띄우면 신뢰할 수 없다. 그래서 이 프로젝트 기존 스타일(utils/slope_visualizer.py,
arm_visualizer.py — 독립 rclpy 노드, 최소 의존성)대로 NaN을 먼저 걸러내고
남은 값만으로 min/max 정규화 후 컬러맵을 입혀 bgr8로 재발행한다.

=== 컬러맵/NaN 처리 ===
  1. np.isfinite()로 유효 픽셀만 골라 min/max 계산 (NaN/Inf는 계산에서 제외).
  2. 유효 픽셀이 하나도 없는 프레임(전부 NaN)은 발행을 건너뛰고 경고만 남김
     (검은 화면을 "값이 0"으로 오독하는 것보다 안전).
  3. 유효 범위를 0~255로 선형정규화 후 cv2.applyColorMap(COLORMAP_JET) 적용.
  4. NaN/Inf였던 픽셀은 컬러맵 이후 지정된 단색(기본 검정)으로 덮어써서
     "데이터 없음"과 "정규화 결과가 우연히 그 색"이 안 헷갈리게 한다.

=== 헤드리스 환경 대응 ===
cv2.imshow류 GUI 호출을 전혀 안 함 — 발행만 하는 순수 릴레이라 SSH/무헤드
환경(DISPLAY 없음)에서도 문제없이 계속 돈다.

=== 축 정렬 ===
gradient_map.py의 그리드는 row=좌우/col=전방인데, flat_drive.py의 BEV
이미지들(/bev/debug_overlay 등)은 row=전방(로봇이 하단, 위로 갈수록
멀어짐)/col=좌우다 — RViz에 나란히 띄워놓고 보면 gradient_map 쪽만 90도
돌아가 있어서 헷갈린다(실기에서 직접 확인된 문제). to_bev_orientation()이
발행 직전에 transpose+flip으로 flat_drive와 같은 축 배치로 맞춘다.

발행 토픽 (입력 토픽명 + '_viz' 접미사, sensor_msgs/Image, encoding='bgr8'):
  /terrain/elevation_map_viz
  /terrain/slope_deg_viz
  /terrain/gradient_magnitude_viz

사용법:
  ros2 run dolbotz terrain_viz_relay
  ros2 run dolbotz terrain_viz_relay --ros-args -p colormap:=turbo   # 컬러맵 변경(jet/turbo/viridis)
"""
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# cv2.COLORMAP_* 이름 -> 상수. TURBO/VIRIDIS는 OpenCV 4.1.2+ 필요 (Humble
# apt 배포 OpenCV는 이보다 신버전이라 문제 없음 — 혹시 없는 빌드면 아래
# getattr(..., None) 체크에서 걸러져 JET로 자동 폴백한다).
_COLORMAP_NAMES = {
    'jet': 'COLORMAP_JET',
    'turbo': 'COLORMAP_TURBO',
    'viridis': 'COLORMAP_VIRIDIS',
}

# NaN/Inf였던 픽셀을 컬러맵 적용 후 덮어쓸 색(BGR). 검정 — "데이터 없음"이
# 컬러맵 팔레트의 실제 값과 안 겹치게 하려는 목적(JET/TURBO/VIRIDIS 전부
# 팔레트 양끝이 검정에 가깝지 않아 육안 구분 잘 됨).
NO_DATA_BGR = (0, 0, 0)


def to_bev_orientation(arr: np.ndarray) -> np.ndarray:
    """gradient_map.py의 그리드 축(row=좌우, col=전방)을 flat_drive.py의 BEV
    이미지 축(row=전방·로봇이 하단·위로 갈수록 멀어짐, col=좌우)으로 맞춘다.

    gradient_map.py 그리드 규약 (elevation_map.py/gradient_map.py 소스 확인):
        col = floor(x_forward / resolution)                      -- 0=로봇, 커질수록 전방
        row = floor((grid_width_m/2 - y_left) / resolution)      -- 0=왼쪽 끝, 커질수록 오른쪽

    flat_drive.py의 bev_ground_projection_matrix() 규약:
        row(세로) = bev_height_px - x_forward/mpp   -- 0=먼 전방(위), 커질수록 로봇(아래)
        col(가로) = bev_width_px/2 - y_left/mpp     -- y_left>0(왼쪽)일수록 작은 col(왼쪽)

    즉 gradient_map 쪽만 90도 돌아가 있어서(전방이 가로축) flat_drive BEV류
    그림(전방이 세로축, 로봇이 하단)과 나란히 보면 헷갈린다 -- transpose로
    축을 맞바꾼 뒤(전방을 세로축으로), 세로축을 뒤집어서(가까운 전방이
    아래로 오게) flat_drive와 같은 "위=먼 전방, 아래=로봇" 배치로 만든다.
    좌우(가로축) 순서는 transpose만으로 이미 flat_drive와 같은 방향(왼쪽=왼쪽)
    이라 추가로 뒤집지 않는다.
    """
    return np.flipud(arr.T)


def normalize_and_colorize(arr: np.ndarray, colormap: int) -> np.ndarray | None:
    """(H,W) float32 배열(NaN/Inf 섞여있을 수 있음) -> (H,W,3) bgr8 컬러맵 이미지.

    유효(finite) 픽셀이 하나도 없으면 None을 반환한다 — 호출부는 이 경우
    발행을 건너뛰어야 한다(순수 함수라 로깅은 호출부 책임).
    """
    finite_mask = np.isfinite(arr)
    if not np.any(finite_mask):
        return None

    finite_vals = arr[finite_mask]
    vmin = float(finite_vals.min())
    vmax = float(finite_vals.max())

    if vmax > vmin:
        normalized = (arr - vmin) / (vmax - vmin)
    else:
        # 유효 픽셀이 전부 같은 값(또는 하나뿐) -- 나눗셈 0 방지, 중간값(회색)으로.
        normalized = np.full_like(arr, 0.5, dtype=np.float64)

    normalized = np.clip(normalized, 0.0, 1.0)
    normalized[~finite_mask] = 0.0  # 컬러맵 입력용 임시값 -- 아래서 NO_DATA_BGR로 다시 덮어씀
    gray_u8 = (normalized * 255.0).astype(np.uint8)

    colored = cv2.applyColorMap(gray_u8, colormap)
    colored[~finite_mask] = NO_DATA_BGR
    return colored


class TerrainVizRelay(Node):
    def __init__(self):
        super().__init__('terrain_viz_relay')

        self.declare_parameter('colormap', 'jet')
        colormap_name = str(self.get_parameter('colormap').value).lower()
        cv_const_name = _COLORMAP_NAMES.get(colormap_name)
        self._colormap = getattr(cv2, cv_const_name, None) if cv_const_name else None
        if self._colormap is None:
            if colormap_name not in _COLORMAP_NAMES:
                self.get_logger().warn(
                    f"colormap 파라미터 '{colormap_name}' 인식 안 됨 "
                    f"(가능: {list(_COLORMAP_NAMES)}) -- jet로 폴백.")
            self._colormap = cv2.COLORMAP_JET

        self._bridge = CvBridge()

        # (입력 토픽, 출력 토픽) 쌍 -- 전부 gradient_map.py/elevation_map.py가
        # 32FC1로 발행하는 것으로 소스에서 직접 확인한 토픽만 다룬다.
        relay_specs = [
            ('/terrain/elevation_map', '/terrain/elevation_map_viz'),
            ('/terrain/slope_deg', '/terrain/slope_deg_viz'),
            ('/terrain/gradient_magnitude', '/terrain/gradient_magnitude_viz'),
        ]

        self._pubs = {}
        for in_topic, out_topic in relay_specs:
            self._pubs[in_topic] = self.create_publisher(Image, out_topic, 10)
            self.create_subscription(
                Image, in_topic,
                lambda msg, in_topic=in_topic: self._on_image(msg, in_topic), 10)

        self.get_logger().info(
            f"terrain_viz_relay ready (colormap={colormap_name}). "
            f"릴레이 중: {', '.join(f'{i} -> {o}' for i, o in relay_specs)}")

    def _on_image(self, msg: Image, in_topic: str) -> None:
        try:
            arr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:  # noqa: BLE001 -- cv_bridge가 던지는 예외 타입이 버전별로 다양함
            self.get_logger().error(
                f"{in_topic} 디코드 실패({e}) -- 32FC1이 아닌 다른 인코딩으로 "
                f"바뀌었는지 확인할 것.", throttle_duration_sec=5.0)
            return

        arr = to_bev_orientation(arr)
        colored = normalize_and_colorize(arr, self._colormap)
        if colored is None:
            self.get_logger().warn(
                f'{in_topic} 프레임 전체가 NaN/Inf -- 이번 프레임은 발행 스킵.',
                throttle_duration_sec=5.0)
            return

        out_msg = self._bridge.cv2_to_imgmsg(colored, encoding='bgr8')
        out_msg.header = msg.header
        self._pubs[in_topic].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainVizRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
