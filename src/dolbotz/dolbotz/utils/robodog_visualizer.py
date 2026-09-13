#!/usr/bin/env python3
"""
robodog_visualizer.py — escort_follow_node(5구간, 선도 정찰로봇 추종)의 검출
결과를 RGB 카메라 화면에 덧씌우는 노트북 전용 로컬 디버그 뷰어 (독립 실행
스크립트). path_visualizer.py의 자매 스크립트 — 형식(독립 실행, argparse,
--dummy, HUD 창, 's' 스냅샷)은 그대로 따르되, escort_follow_node는
flat_drive_node의 BEV 호모그래피(/bev/H)를 전혀 안 쓰는 별개 파이프라인이라
투영 방식은 다르다(아래 "좌표 변환" 절 참고).

dolbotz 패키지 의존성 없음 — rclpy + sensor_msgs/nav_msgs/geometry_msgs +
mission_manager_interfaces + opencv-python + numpy만 필요(cv_bridge도 안 씀,
CompressedImage 바이트를 cv2.imdecode로 직접 디코드). utils/ 아래 있지만
dolbotz 패키지의 일부로 빌드/설치될 필요 없이
`python3 robodog_visualizer.py`로 바로 실행된다.

=== escort_follow.py에는 bbox 좌표를 담은 토픽이 따로 없다 ===
spring_ifof.py/summer_traffic.py/summer_supply.py와 달리 escort_follow.py는
vision_msgs/Detection2DArray류의 원시 bbox 토픽을 발행하지 않는다(직접 소스
확인). 대신:
  - /mission/escort_follow/debug_image/compressed — 이번 프레임에 탐지가
    "채택"됐을 때만(escort_follow.py._on_frames()의 accepted is not None
    분기, 즉 진짜 TRACKING일 때만) bbox+라벨이 이미 그려진 이미지가 나간다.
    PREDICTING/LOST 프레임에는 전혀 안 나간다(escort_follow.py 모듈
    docstring 발행 절: "항상 나가는 건 아님").
  - /mission/escort_follow/result, /path — 3D 상대좌표(target_point,
    카메라 광학 프레임 X/Y/Z)만 있고 픽셀 bbox 좌표는 없음.
그래서 이 뷰어는 두 소스를 함께 쓴다:
  1) 기본 화면은 항상 살아있는 /drive/camera/color/image_raw/compressed
     (escort_follow.py와 동일 토픽, raw 원본 — 아래 "왜곡보정 안 함" 참고).
  2) 최근(--debug-image-fresh-sec, 기본 0.5초) 안에 debug_image가 도착했으면
     그걸 그대로 보여준다 — 실제 검출기가 그린 정확한 bbox.
  3) 없으면 raw 화면 그대로 표시한다 — target_point 재투영 추정 마커는
     [2026-09-01, 사용자 결정] 제거함. escort_follow.py 내부의 PREDICTING은
     외부 스키마상 'tracking'으로만 나가 이 뷰어가 debug_image 신선도로
     역추론했었는데, 그 구분이 실제로는 큰 의미가 없다고 판단해 뺐다 —
     이제는 result.valid 값 하나만 보고 TRACKING/LOST 2단계로만 표시한다
     (아래 "HUD" 절 참고).

=== 좌표 변환: 단순 핀홀 재투영(호모그래피 없음) ===
flat_drive.py 기반 미션들(/bev/H 쓰는 path_visualizer.py 등)은 지면
평탄(flat-ground) 가정 호모그래피로 x_forward/y_left를 화면에 투영하지만,
escort_follow.py는 완전히 다른 방식으로 3D 좌표를 만든다(escort_follow.py
._detect_best_candidate() 직접 확인):
    u, v = bbox 중심 픽셀
    z = 그 지점의 depth(m)
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
즉 target_point(X,Y,Z)는 카메라 "광학" 프레임의 표준 핀홀 좌표(X=오른쪽,
Y=아래, Z=전방)다 — flat_drive의 "x=전방,y=좌측" body 규약이 아니다. 그래서
이 뷰어가 할 일은 저 식을 그대로 뒤집는 것뿐이다:
    u = cx + X * fx / z
    v = cy + Y * fy / z          (z<=0이면 카메라 뒤쪽 -> 투영 불가, 버림)
호모그래피 역행렬도, /bev/H 구독도 필요 없다 — CameraInfo의 K(fx/fy/cx/cy)
하나면 충분하다.

=== 왜곡보정(undistort) 안 함 ===
path_visualizer.py는 flat_drive.py가 왜곡보정된 이미지 좌표계 기준으로
호모그래피를 만들기 때문에 undistort를 거쳤지만, escort_follow.py는
_on_frames()에서 cv2.undistort()를 전혀 호출하지 않고 raw 이미지 그대로
bbox를 검출하고 depth를 샘플링한다(직접 소스 확인 — CameraInfo는 fx/fy/cx/cy
값만 꺼내 쓰고 D(왜곡계수)는 아예 안 읽음). 그래서 이 뷰어도 raw 이미지에
그대로 그린다 — undistort를 넣으면 오히려 escort_follow.py의 실제 좌표
계산과 기하학적으로 안 맞게 된다.

=== QoS ===
color_topic, camera_info_topic : BEST_EFFORT/depth=1 —
    escort_follow.py가 이 두 토픽을 구독하는 QoS(SENSOR_DATA_QOS_DEPTH1)와
    동일(직접 소스 확인).
debug_image_topic, result_topic, path_topic : RELIABLE, depth 10 —
    escort_follow.py가 create_publisher()에 QoS를 안 넘겨 ROS2 기본값
    (RELIABLE/KEEP_LAST/depth10)을 그대로 쓰므로 그것과 맞춘다
    (path_visualizer.py의 _reliable_qos()와 동일 근거/구현).
QoS가 안 맞으면 에러 없이 그 토픽만 조용히 콜백이 안 불리므로(원인 파악
어려움) 이 파일에서도 하드코딩해 실수를 방지한다.

=== HUD 사이드패널 ===
별도 창에 STATE/DIST(3D)/DEPTH(Z)/TARGET XY/PATH/FPS를 표시한다.
  STATE 색상 — result.valid 하나만 본다(2단계, [2026-09-01] PREDICTING
               구분 제거 — 위 "escort_follow.py에는 bbox 좌표를 담은 토픽이
               따로 없다" 절 참고):
               녹색 "TRACKING": result.valid=True(debug_image가 신선하지
               않아 화면에 실제 bbox가 안 보여도 초록으로 표시됨).
               빨강 "LOST": result.valid=False.
  DIST(3D) — target_point의 3D 노름(sqrt(X²+Y²+Z²)), 대회 규정 "간격" 판정에
             가장 가까운 값.
  DEPTH(Z) — target_point.z(카메라 정면 방향 거리) 단독 — DIST(3D)와 비교용.
  PATH — /path.poses 개수(2=활성, 0=빈 Path→purepursuit_node 즉시 정지,
         escort_follow.py._publish_path() 참고).

사용법:
  python3 robodog_visualizer.py                      # 실제 로봇/네트워크에 붙어서 구독
  python3 robodog_visualizer.py --image-topic ...     # 토픽명 오버라이드
  python3 robodog_visualizer.py --dummy               # 로컬 스모크테스트(더미 발행 내장, 실제 로봇 불필요)

키: q  종료
    s  현재 화면 스냅샷 PNG 저장 (robodog_overlay_snapshots/)

실행 전 확인:
  - escort_follow_node가 반드시 떠 있어야 함(mission_escort.launch.py 또는
    `ros2 run dolbotz escort_follow`).
  - ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY/RMW_IMPLEMENTATION이 젝슨/노트북 동일한지.
  - `ros2 topic list`에 /mission/escort_follow/result, /path, 컬러
    이미지/camera_info 토픽이 보이는지. 이 뷰어 실행 5초 뒤에도 안 들어오면
    터미널에 토픽별로 개별 경고가 뜬다.
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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mission_manager_interfaces.msg import MissionResult

SNAPSHOT_DIR = FsPath(__file__).resolve().parent / 'robodog_overlay_snapshots'

GREEN_BGR = (0, 255, 0)
RED_BGR = (0, 0, 255)
YELLOW_BGR = (0, 255, 255)
NO_DATA_WARN_SEC = 5.0
HUD_WIDTH = 460
HUD_HEIGHT = 320


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


def _reliable_qos() -> QoSProfile:
    """/mission/escort_follow/{result,debug_image/compressed}, /path
    발행자 쪽 기본 QoS(RELIABLE, depth 10)와 맞춘 프로필 —
    path_visualizer.py의 _reliable_qos()와 동일 근거."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.VOLATILE,
        depth=10,
    )


# ---------------------------------------------------------------------------
# 순수 계산 (ROS 의존성 없음) — 좌표 변환
# ---------------------------------------------------------------------------

def project_camera_point(xyz, fx: float, fy: float, cx: float, cy: float):
    """카메라 광학 프레임 (X,Y,Z) -> 픽셀 (u,v), valid.

    escort_follow.py._detect_best_candidate()의 X=(u-cx)*z/fx,
    Y=(v-cy)*z/fy를 그대로 뒤집는다(모듈 docstring "좌표 변환" 절 참고).
    z<=0이면(카메라 뒤쪽/원점) 투영 불가로 valid=False."""
    x, y, z = xyz
    if z <= 1e-6:
        return (0.0, 0.0), False
    u = cx + x * fx / z
    v = cy + y * fy / z
    return (u, v), True


def target_distance_3d(xyz) -> float:
    return float(np.linalg.norm(np.asarray(xyz, dtype=np.float64)))


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def fps_from_timestamps(timestamps) -> float:
    n = len(timestamps)
    if n < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    if span <= 0:
        return 0.0
    return n / span


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------

class RobodogCameraOverlay(Node):
    def __init__(self, args, dummy: bool = False):
        super().__init__('robodog_camera_overlay')

        self._debug_fresh_sec = float(args.debug_image_fresh_sec)

        self.fx = self.fy = self.cx = self.cy = None
        self.display_frame = None
        self.hud_frame = None
        self._raw_frame = None
        self._debug_frame = None
        self._debug_wall: float | None = None
        self._latest_result: MissionResult | None = None
        self._latest_path: Path | None = None
        self._start_wall = time.monotonic()
        self._frame_times: list = []
        self._warned = {
            'image': False, 'camera_info': False, 'result': False,
            'path': False, 'debug_image': False,
        }
        self._got = {k: False for k in self._warned}

        self.create_subscription(
            CameraInfo, args.camera_info_topic, self._on_camera_info, _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, args.image_topic, self._on_image, _sensor_data_qos_depth1())
        self.create_subscription(
            CompressedImage, args.debug_image_topic, self._on_debug_image, _reliable_qos())
        self.create_subscription(
            MissionResult, args.result_topic, self._on_result, _reliable_qos())
        self.create_subscription(
            Path, args.path_topic, self._on_path, _reliable_qos())

        self._topics = {
            'image': args.image_topic, 'camera_info': args.camera_info_topic,
            'debug_image': args.debug_image_topic, 'result': args.result_topic,
            'path': args.path_topic,
        }

        if dummy:
            self._setup_dummy(args)

        self.create_timer(1.0, self._connectivity_check)
        self.create_timer(0.1, self._update_hud)

    # --- 구독 콜백 --------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._got['camera_info'] = True
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"CameraInfo 수신 완료 — fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}")

    def _on_debug_image(self, msg: CompressedImage) -> None:
        self._got['debug_image'] = True
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self._debug_frame = frame
        self._debug_wall = time.monotonic()

    def _on_result(self, msg: MissionResult) -> None:
        self._got['result'] = True
        self._latest_result = msg

    def _on_path(self, msg: Path) -> None:
        self._got['path'] = True
        self._latest_path = msg

    def _on_image(self, msg: CompressedImage) -> None:
        self._got['image'] = True
        self._frame_times.append(time.monotonic())
        if len(self._frame_times) > 30:
            self._frame_times = self._frame_times[-30:]
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        raw = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if raw is None:
            self.get_logger().error('이미지 디코드 실패 — CompressedImage 포맷 확인 필요.')
            return
        self._raw_frame = raw

        # debug_image가 신선하면(진짜 검출 있었던 그 프레임) 그걸 그대로
        # 보여준다 — 실제 bbox가 이미 그려져 있음(모듈 docstring 참고).
        debug_age = (
            time.monotonic() - self._debug_wall
            if self._debug_wall is not None else None)
        if debug_age is not None and debug_age <= self._debug_fresh_sec:
            frame = self._debug_frame.copy()
            self._draw_status(frame, [f'LIVE bbox (debug_image age {debug_age*1000:.0f}ms)'])
            self.display_frame = frame
            return

        # 신선한 debug_image가 없음 — raw 화면 그대로 표시한다. 재투영
        # 마커는 [2026-09-01, 사용자 결정] 제거함(모듈 docstring 참고) —
        # HUD의 STATE(TRACKING/LOST)는 result.valid만으로 정해지고, 여기
        # 화면에는 더 이상 추정 위치를 안 그린다.
        frame = raw.copy()
        status = []
        if self._latest_result is None:
            self._warn_once('result', f"{self._topics['result']} 아직 안 들어옴.")
            status.append(f"WAITING: {self._topics['result']} not received yet")
        elif not self._latest_result.valid:
            status.append('LOST — no target')
        else:
            status.append('TRACKING (no live bbox this frame)')

        self._draw_status(frame, status)
        self.display_frame = frame

    # --- HUD (10Hz 전용 타이머) --------------------------------------------

    def _update_hud(self) -> None:
        hud = np.zeros((HUD_HEIGHT, HUD_WIDTH, 3), dtype=np.uint8)

        debug_age = (
            time.monotonic() - self._debug_wall
            if self._debug_wall is not None else None)
        is_live = debug_age is not None and debug_age <= self._debug_fresh_sec

        # [2026-09-01, 사용자 결정] PREDICTING 구분 제거 — result.valid만으로
        # TRACKING/LOST 2단계로 표시한다(모듈 docstring "HUD" 절 참고).
        # is_live는 라벨에 "실제 bbox가 화면에 보이는지"만 덧붙이는 용도.
        result = self._latest_result
        if result is None:
            state_text, state_color = 'STATE: N/A (no result yet)', (128, 128, 128)
        elif result.valid:
            bbox_note = 'live bbox' if is_live else 'no live bbox'
            state_text, state_color = f'STATE: TRACKING ({bbox_note})', GREEN_BGR
        else:
            state_text, state_color = 'STATE: LOST', RED_BGR

        if result is not None and (result.valid or is_live):
            p = result.target_point
            xyz = (p.x, p.y, p.z)
            dist_text = f'DIST(3D): {target_distance_3d(xyz):.2f} m'
            depth_text = f'DEPTH(Z): {p.z:.2f} m'
            target_text = f'TARGET X={p.x:+.2f} Y={p.y:+.2f} m'
        else:
            dist_text = 'DIST(3D): N/A'
            depth_text = 'DEPTH(Z): N/A'
            target_text = 'TARGET X=N/A Y=N/A'

        if self._latest_path is not None:
            n = len(self._latest_path.poses)
            path_text = f'PATH: ACTIVE ({n} poses)' if n > 0 else 'PATH: EMPTY (stopped)'
        else:
            path_text = 'PATH: N/A'

        fps_text = f'FPS: {fps_from_timestamps(self._frame_times):.1f}'

        y = 50
        cv2.putText(hud, state_text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, state_color, 2, cv2.LINE_AA)
        for text in (dist_text, depth_text, target_text, path_text, fps_text):
            y += 45
            cv2.putText(hud, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255), 2, cv2.LINE_AA)

        self.hud_frame = hud

    # --- 그리기 -------------------------------------------------------
    # _draw_predicted_marker()(재투영 추정 마커)는 [2026-09-01, 사용자 결정]
    # 제거함 — 모듈 docstring 참고.

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
                hint = ''
                if key == 'debug_image':
                    hint = ' (탐지가 한 번도 채택된 적 없으면 정상 — LOST/PREDICTING만 계속돼도 안 옴)'
                self.get_logger().warn(
                    f"{NO_DATA_WARN_SEC:.0f}초 넘게 '{topic}'에서 메시지가 안 들어옴.{hint} "
                    f"escort_follow_node가 떠 있는지, ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY/"
                    f"RMW_IMPLEMENTATION 및 `ros2 topic list`/`ros2 topic info {topic}`"
                    f"(QoS 확인)로 점검할 것.")

    # --- --dummy 모드: 로컬 스모크테스트용 더미 발행 -----------------------

    def _setup_dummy(self, args) -> None:
        img_qos = _sensor_data_qos_depth1()
        self._dummy_image_pub = self.create_publisher(CompressedImage, args.image_topic, img_qos)
        self._dummy_info_pub = self.create_publisher(CameraInfo, args.camera_info_topic, img_qos)
        self._dummy_debug_pub = self.create_publisher(
            CompressedImage, args.debug_image_topic, _reliable_qos())
        self._dummy_result_pub = self.create_publisher(
            MissionResult, args.result_topic, _reliable_qos())
        self._dummy_path_pub = self.create_publisher(Path, args.path_topic, _reliable_qos())

        self._dummy_w, self._dummy_h = 640, 480
        fx = fy = 500.0
        cx, cy = self._dummy_w / 2.0, self._dummy_h / 2.0
        self._dummy_k = (fx, fy, cx, cy)

        self._dummy_t = 0.0
        # 12초 주기: 0~5초 TRACKING(라이브 bbox), 5~9초 TRACKING인데
        # debug_image 없음(실물에서도 나는 정상 케이스 — 화면엔 그냥 raw만
        # 나오는지 확인용, [2026-09-01] 재투영 마커는 뺐음, 모듈 docstring
        # 참고), 9~12초 LOST — HUD 색상 전환/화면 갱신이 눈에 보이게.
        self.create_timer(1.0 / 15.0, self._publish_dummy)
        self.get_logger().info(
            "--dummy 모드: 실제 로봇 없이 이미지/CameraInfo/debug_image/"
            "result/path를 자체 발행합니다 (TRACKING(bbox)->TRACKING(no bbox)"
            "->LOST 12초 순환).")

    def _publish_dummy(self) -> None:
        self._dummy_t += 1.0 / 15.0
        now = self.get_clock().now().to_msg()
        w, h = self._dummy_w, self._dummy_h
        fx, fy, cx, cy = self._dummy_k

        frame = np.full((h, w, 3), (40, 40, 40), dtype=np.uint8)
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
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._dummy_info_pub.publish(info_msg)

        cycle = self._dummy_t % 12.0
        # 카메라 앞 2m 근방을 좌우로 오가는 타겟(3D, 카메라 광학 프레임).
        x = 0.8 * float(np.sin(self._dummy_t * 0.6))
        z = 2.0
        y = 0.1
        xyz = (x, y, z)

        result = MissionResult()
        result.header = img_msg.header
        result.mission_name = 'escort_follow'

        if cycle < 5.0:
            result.state = 'tracking'
            result.valid = True
            result.target_point.x, result.target_point.y, result.target_point.z = xyz
            self._dummy_result_pub.publish(result)

            (u, v), valid = project_camera_point(xyz, fx, fy, cx, cy)
            if valid:
                dbg = frame.copy()
                bw, bh = 60, 90
                x1, y1 = int(u - bw / 2), int(v - bh / 2)
                x2, y2 = int(u + bw / 2), int(v + bh / 2)
                cv2.rectangle(dbg, (x1, y1), (x2, y2), GREEN_BGR, 2)
                cv2.putText(dbg, f'go2 0.87 | Z={z:.2f}m', (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN_BGR, 2)
                ok2, enc2 = cv2.imencode('.jpg', dbg)
                dbg_msg = CompressedImage()
                dbg_msg.header = img_msg.header
                dbg_msg.format = 'jpeg'
                dbg_msg.data = enc2.tobytes() if ok2 else b''
                self._dummy_debug_pub.publish(dbg_msg)

            path_msg = Path()
            path_msg.header = img_msg.header
            origin = PoseStamped(); origin.header = img_msg.header; origin.pose.orientation.w = 1.0
            target = PoseStamped(); target.header = img_msg.header
            target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
            target.pose.orientation.w = 1.0
            path_msg.poses = [origin, target]
            self._dummy_path_pub.publish(path_msg)

        elif cycle < 9.0:
            # result는 계속 valid=True/tracking이지만 debug_image는 안 보냄
            # (실물 escort_follow.py의 내부 PREDICTING과 같은 입력 패턴 —
            # 화면엔 이제 raw만 나오고 마커는 안 그려지는지 확인용).
            result.state = 'tracking'
            result.valid = True
            result.target_point.x, result.target_point.y, result.target_point.z = xyz
            self._dummy_result_pub.publish(result)

            path_msg = Path()
            path_msg.header = img_msg.header
            origin = PoseStamped(); origin.header = img_msg.header; origin.pose.orientation.w = 1.0
            target = PoseStamped(); target.header = img_msg.header
            target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
            target.pose.orientation.w = 1.0
            path_msg.poses = [origin, target]
            self._dummy_path_pub.publish(path_msg)

        else:
            # LOST — 빈 Path, valid=False.
            result.state = 'lost'
            result.valid = False
            self._dummy_result_pub.publish(result)

            path_msg = Path()
            path_msg.header = img_msg.header
            self._dummy_path_pub.publish(path_msg)


# ---------------------------------------------------------------------------
# 메인 루프
# ---------------------------------------------------------------------------

def _save_snapshot(node: RobodogCameraOverlay) -> None:
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
                         help='RGB 컬러 압축 이미지 토픽 (기본: escort_follow.py의 color_topic 기본값과 동일)')
    parser.add_argument('--camera-info-topic', default='/drive/camera/color/camera_info',
                         help='CameraInfo 토픽 (K/재투영용)')
    parser.add_argument('--debug-image-topic', default='/mission/escort_follow/debug_image/compressed',
                         help='escort_follow_node가 발행하는, bbox 오버레이 이미 그려진 디버그 이미지')
    parser.add_argument('--result-topic', default='/mission/escort_follow/result',
                         help='mission_manager_interfaces/MissionResult 토픽 (state/valid/target_point)')
    parser.add_argument('--path-topic', default='/path',
                         help='escort_follow_node가 발행하는 2점짜리 Path (purepursuit_node 입력)')
    parser.add_argument('--debug-image-fresh-sec', type=float, default=0.5,
                         help='이 시간(초) 이내에 도착한 debug_image만 "라이브 bbox"로 취급 (기본 0.5)')
    parser.add_argument('--dummy', action='store_true',
                         help='실제 로봇 없이 로컬에서 토픽들을 자체 발행해 스모크테스트 '
                              '(TRACKING(bbox)->TRACKING(no bbox)->LOST 12초 순환)')
    return parser


def main():
    args = _build_arg_parser().parse_args()

    rclpy.init()
    node = RobodogCameraOverlay(args, dummy=args.dummy)

    window = 'robodog_camera_overlay (q: quit, s: save snapshot)'
    hud_window = 'robodog_camera_overlay HUD'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(hud_window, cv2.WINDOW_NORMAL)

    print('실행 중... 뷰어 창에 포커스 준 상태에서 q 종료 / s 스냅샷 저장, Ctrl+C로도 종료')
    if not args.dummy:
        print(f"escort_follow_node가 떠 있어야 '{args.result_topic}'/'{args.path_topic}'가 나옵니다.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            if node.display_frame is not None:
                cv2.imshow(window, node.display_frame)
            if node.hud_frame is not None:
                cv2.imshow(hud_window, node.hud_frame)

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
