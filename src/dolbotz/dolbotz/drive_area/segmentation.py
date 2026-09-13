"""
세그멘테이션 노드 — RGB 원근 이미지에서 주행가능영역('area' 클래스) 마스크를
뽑아 발행하는 전용 노드.

flat_drive.py/elevation_map.py/slope_decision.py가 공통으로 쓰는 마스크를 여기서
한 번만 추론해서 내보낸다 — 세 곳에서 각자 YOLO를 돌리면 임베디드 환경에서
추론이 중복됨. 이 노드가 죽으면 세 소비자가 다 같이 영향받으므로, 세그멘테이션
추론 + 마스크 발행 외의 다른 로직은 얹지 않는다.

반드시 undistort된 원근(perspective) 이미지에 추론을 돌린다 — 모델이 원근
이미지로 학습되었으므로(train_drive_area.py/Roboflow 데이터셋에 BEV·기하 변환
전처리가 없음을 확인함), BEV 등으로 미리 왜곡한 이미지에 돌리면 학습 분포를
벗어나 정확도가 떨어진다. BEV 투영, depth 좌표 정렬 등은 이 노드가 아니라 각
소비자(flat_drive.py의 BEV, elevation_map.py의 depth 필터링 등) 쪽 책임이다.

구독: /drive/camera/color/image_raw/compressed  sensor_msgs/CompressedImage
      /drive/camera/color/camera_info            sensor_msgs/CameraInfo (undistort용)
발행: /perception/drivable_mask  sensor_msgs/Image (mono8, 0/255)
      — undistort된 원본 해상도 그대로, 원근 이미지 좌표계 (BEV 아님)

파라미터:
  color_topic       str    기본값 /drive/camera/color/image_raw/compressed
  camera_info_topic str    기본값 /drive/camera/color/camera_info
  model_path        str    기본값 dolbotz.utils.paths.get_models_dir()/'dolbotz_seg_v2'/'best_320_int8_openvino_model'
                           — [2026-08-30] vision_marker 리포(별도 워크스페이스,
                           /home/j/vision_marker/drive_area/)에서 학습한
                           drive_area_v2.pt(YOLO26n-seg, 클래스 동일 {0: 'area'})를
                           이 리포로 복사해(config/models/dolbotz_seg_v2/best.pt)
                           train_drive_area.py/export_drive_area_optimized.py와
                           동일한 파라미터로 재export한 것 — 640 FP32
                           (best_openvino_model/), 320 FP32
                           (best_320_openvino_model/), 320 INT8
                           (best_320_int8_openvino_model/) 세 가지를 만들어뒀고,
                           기본값은 마지막(320 INT8)으로 잡는다.
                           [2026-09-02] INT8(320) 버전을 v2 학습에 실제로 쓴
                           검증셋(/home/j/vision_marker/drive_area/Drive-area-5/
                           data.yaml)으로 캘리브레이션해서 새로 만듦 —
                           그 전엔 FP32(320)가 기본값이었는데, v1의 INT8
                           export와 달리 양자화가 안 돼 있어서 가중치
                           3.9배(2.78MB->10.78MB)/추론 약 1.8배(8.3ms->4.7ms,
                           OpenVINO CPU 실측) 더 무거웠다. INT8 전환 검증:
                           검증셋 15장 기준 마스크 검출 개수 100% 동일, 평균
                           top confidence 거의 동일(0.9756->0.9743, 차이
                           0.0013) — 정확도 손실 없이 v1과 같은 수준으로
                           가벼워짐(mAP 등 다른 정량 지표는 실기 배포 전
                           추가 확인 권장).
                           # 이전 기본값(2026-08-27~2026-08-30): 아래 줄로
                           # 되돌리면 원래 모델(dolbotz_seg_v1, 신뢰성 검증됨 —
                           # mask mAP50-95 0.9903->0.9887, 추론 26.3ms->5.5ms
                           # 확인됨)로 복귀한다.
                           #   model_path 기본값: get_models_dir()/'dolbotz_seg_v1'/'best_320_int8_openvino_model'
                           #   imgsz 기본값: 320 (동일)
  imgsz             int    기본값 320 — model_path가 정적 imgsz로 export된
                           모델이라 반드시 그 export 크기와 일치해야 함
                           (다르면 shape 불일치로 추론이 깨짐). 640 FP32
                           모델로 되돌릴 땐 이 값도 640으로 같이 바꿀 것.
  conf_threshold    float  기본값 0.5
  (device는 파라미터가 아니며 'CPU'로 고정,
   launch 인자/ros2 param set 등 어떤 경로로도 바꿀 수 없음. GPU로 돌리고
   싶으면 __init__()의 self._device 리터럴을 직접 고칠 것.)
  bench_log_hz      float  기본값 1.0 — flat_drive.py와 동일한 perf_counter 기반
                           계측. segmentation 추론과 후속 단계의 지연을
                           구분하는 데 사용한다.
  snow_model_path   str    [2026-09-03, 겨울 미션 전용] 기본값 ''(빈 문자열
                           — 비활성). 값이 있으면 이 모델도 같이 로드해서,
                           area 마스크와 snow 마스크를 OR로 합쳐
                           /perception/drivable_mask 하나로 발행한다
                           (재현율 우선 — [2026-09-03, 사용자 결정]). 빙판길/
                           제설 구간에서 area 모델 단독으로 놓칠 수 있는
                           영역을 보강하는 목적. spring/summer/fall
                           미션에서는 이 값을 절대 안 넘기므로 완전히
                           비활성 — mission_winter.launch.py만 채운다
                           (perception_common.launch.py의 snow_model_path
                           launch 인자 참고). model_path와 마찬가지로
                           정적 imgsz export를 쓴다면 snow_imgsz도 맞출 것.
  snow_imgsz        int    기본값 640 — snow_model_path 모델 전용 imgsz
                           (imgsz와 별도 파라미터인 이유는 두 모델이 서로
                           다른 크기로 export됐을 가능성 때문 — 실제로
                           snow_v1은 640으로 학습됐고 area 모델(320)과
                           다르다, /home/j/vision_marker/snow/train_snow.py
                           IMGSZ 확인).
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from dolbotz.utils.qos import SENSOR_DATA_QOS_DEPTH1
from dolbotz.utils.paths import get_models_dir

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

# 세그멘테이션 모델의 유일한 클래스 (metadata.yaml: names: {0: area})
DRIVABLE_AREA_CLASS_NAME = 'area'

# [2026-09-03, 겨울 미션 전용] snow_v1(config/models/, {0: 'snow'} 단일
# 클래스) — 겨울 트랙의 빙판길/제설 구간에서 기존 area 모델이 놓칠 수 있는
# 영역을 보강하려고 area 마스크와 OR로 합친다(안전보다 재현율 우선 —
# [2026-09-03, 사용자 결정] 참고). SegmentationNode의 snow_model_path
# 파라미터(기본값 빈 문자열=비활성)로만 켜지며, 겨울 외 미션에서는 절대
# 안 쓴다(perception_common.launch.py를 spring/summer/fall이 그대로
# include하고, mission_winter.launch.py만 이 인자를 넘긴다).
SNOW_CLASS_NAME = 'snow'


# ---------------------------------------------------------------------------
# 순수 계산 — ROS 의존성 없음 (flat_drive.py의 기존 _segmentation_mask를 이관)
# ---------------------------------------------------------------------------

def segmentation_mask(
    model, image_bgr: np.ndarray, conf_threshold: float, device: str | None = None,
    imgsz: int = 320, target_class_name: str = DRIVABLE_AREA_CLASS_NAME,
) -> np.ndarray:
    """원본(왜곡보정된) 이미지에 세그멘테이션 추론을 돌려 target_class_name
    클래스의 바이너리(0/255) 마스크를 만든다. 모델은 정적 입력 크기로
    export되어 있으므로(metadata.yaml: dynamic=false) imgsz를 그 크기와
    정확히 맞춰야 한다 — 기본값 320은 best_320_int8_openvino_model 기준이다.
    SegmentationNode의 'imgsz' 파라미터 참고. 다른 export(예: 640 FP32
    best_openvino_model)를 쓰면 이 값도 같이 바꿔야 함.

    target_class_name: 기본은 area(주행가능영역) 모델용이지만, 겨울 미션의
    snow 모델처럼 클래스 이름이 다른 두 번째 모델에도 이 함수를 그대로
    재사용하려고 파라미터로 뺐다(SegmentationNode._on_image()의 snow 마스크
    계산 참고).

    model이 None(로드 실패)이면 빈(0) 마스크를 그대로 반환한다 — 호출부가
    "마스크 없음"과 "마스크는 있는데 트랙이 안 보임"을 굳이 구분할 필요
    없게 하기 위함이다(둘 다 아무것도 못 감이 안전한 기본값).

    device: OpenVINO 추론 디바이스('CPU'/'GPU'/'AUTO' 등, None이면 ultralytics
    기본값=CPU). SegmentationNode의 'device' 파라미터 참고.
    """
    h, w = image_bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    if model is None:
        return mask

    results = model(image_bgr, imgsz=imgsz, conf=conf_threshold, device=device, verbose=False)
    for r in results:
        if r.masks is None or r.boxes is None:
            continue
        for poly, cls_idx in zip(r.masks.xy, r.boxes.cls):
            cls_name = model.names.get(int(cls_idx), '')
            if cls_name != target_class_name:
                continue
            pts = poly.astype(np.int32)
            if pts.shape[0] >= 3:
                cv2.fillPoly(mask, [pts], 255)
    return mask


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class SegmentationNode(Node):
    """ROS2 wrapper: color+camera_info를 구독해 /perception/drivable_mask를 게시한다."""

    def __init__(self):
        super().__init__('segmentation_node')

        self.declare_parameter('color_topic', '/drive/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/drive/camera/color/camera_info')
        # [2026-08-30] dolbotz_seg_v2(vision_marker 리포에서 학습한
        # drive_area_v2.pt)로 교체 — 모듈 docstring의 model_path 절 참고.
        # [2026-09-02] v2의 기본 export(best_320_openvino_model)는 그동안
        # INT8 양자화가 안 돼 있어서(v1의 best_320_int8_openvino_model과
        # 달리 quantize: null) v1 대비 가중치 3.9배(2.78MB->10.78MB)/추론
        # 시간 약 1.8배(8.3ms->4.7ms, OpenVINO CPU 실측)로 더 무거웠다 —
        # Drive-area-5 검증셋(v2 학습에 쓴 것과 동일)으로 INT8 export를
        # 새로 만들어서 교체함. 15장 검증 결과 마스크 검출 개수 100% 동일,
        # 평균 top confidence 거의 동일(0.9756->0.9743, 차이 0.0013) —
        # 정확도 손실 없이 v1과 같은 수준으로 가벼워짐.
        self.declare_parameter(
            'model_path', str(get_models_dir() / 'dolbotz_seg_v2' / 'best_320_int8_openvino_model'))
        # self.declare_parameter(
        #     'model_path', str(get_models_dir() / 'dolbotz_seg_v2' / 'best_320_openvino_model'))
        # self.declare_parameter(
        #     'model_path', str(get_models_dir() / 'dolbotz_seg_v1' / 'best_320_int8_openvino_model'))
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('bench_log_hz', 1.0)
        # [2026-09-03, 겨울 미션 전용] 비어있으면(기본값) 완전히 비활성 —
        # 모듈 상단 SNOW_CLASS_NAME 주석 참고. perception_common.launch.py의
        # snow_model_path 인자로만 채워지고, mission_winter.launch.py만
        # 그 인자에 실제 값을 넘긴다.
        self.declare_parameter('snow_model_path', '')
        # snow_v1은 area 모델(320)과 달리 640으로 학습/export됨 —
        # 모듈 docstring의 snow_imgsz 절 참고.
        self.declare_parameter('snow_imgsz', 640)

        color_topic = str(self.get_parameter('color_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        model_path = str(self.get_parameter('model_path').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._conf_th = float(self.get_parameter('conf_threshold').value)
        self._device = 'CPU'
        bench_period = 1.0 / max(0.1, self.get_parameter('bench_log_hz').value)
        snow_model_path = str(self.get_parameter('snow_model_path').value)
        self._snow_imgsz = int(self.get_parameter('snow_imgsz').value)

        self._model = self._load_model(model_path)
        self._snow_model = self._load_model(snow_model_path) if snow_model_path else None
        self._bridge = CvBridge()
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._timings: list[tuple] = []

        qos = SENSOR_DATA_QOS_DEPTH1
        self._info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, qos)
        self._image_sub = self.create_subscription(
            CompressedImage, color_topic, self._on_image, qos)

        self._mask_pub = self.create_publisher(Image, '/perception/drivable_mask', 10)

        self._bench_timer = self.create_timer(bench_period, self._log_timing)

        self.get_logger().info(
            f'SegmentationNode ready — listening on {color_topic}, {camera_info_topic} '
            f'(device={self._device}, imgsz={self._imgsz}, model_path={model_path}, '
            f'snow_model_path={snow_model_path or "(비활성)"})')

    # ------------------------------------------------------------------

    def _load_model(self, path: str):
        if not path:
            self.get_logger().warn('model_path 미설정 — 세그멘테이션 모델 경로를 파라미터로 전달하세요')
            return None
        if not _YOLO_OK:
            self.get_logger().error('ultralytics 미설치 — pip install ultralytics openvino')
            return None
        # task='segment' 명시 — 경로에 'segment'가 없으면 detect로 오판되는 회귀 있었음
        model = YOLO(path, task='segment')
        self.get_logger().info(f'세그멘테이션 모델 로드 완료: {path}')
        return model

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape((3, 3))
            self._dist_coeffs = np.array(msg.d)

    def _on_image(self, msg: CompressedImage) -> None:
        if self._camera_matrix is None:
            self.get_logger().warn('CameraInfo 대기 중.', throttle_duration_sec=2.0)
            return
        if self._model is None:
            self.get_logger().warn('세그멘테이션 모델 미로드.', throttle_duration_sec=2.0)
            return

        t0 = time.perf_counter()

        try:
            cv_image = self._bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'CvBridge decode failed: {exc}')
            return

        t1 = time.perf_counter()

        undistorted = cv2.undistort(cv_image, self._camera_matrix, self._dist_coeffs)

        t2 = time.perf_counter()

        mask = segmentation_mask(
            self._model, undistorted, self._conf_th, self._device, self._imgsz)

        # [2026-09-03, 겨울 미션 전용] snow_model이 켜져 있으면(mission_winter
        # 에서만) area 마스크와 snow 마스크를 OR로 합친다 — 둘 중 하나라도
        # "주행 가능"으로 보면 인정(재현율 우선, [2026-09-03 사용자 결정]
        # 참고, 모듈 상단 SNOW_CLASS_NAME 주석과 동일 근거). 비활성이면
        # (spring/summer/fall) 이 블록 자체가 안 돌아서 기존과 완전히 동일.
        if self._snow_model is not None:
            snow_mask = segmentation_mask(
                self._snow_model, undistorted, self._conf_th, self._device,
                self._snow_imgsz, target_class_name=SNOW_CLASS_NAME)
            mask = cv2.bitwise_or(mask, snow_mask)

        t3 = time.perf_counter()

        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self._mask_pub.publish(mask_msg)

        t4 = time.perf_counter()

        self._timings.append((
            (t1 - t0) * 1e3,   # decode
            (t2 - t1) * 1e3,   # undistort
            (t3 - t2) * 1e3,   # infer (YOLO segmentation -- 원래 계측이 전혀 없던 구간)
            (t4 - t3) * 1e3,   # publish
            (t4 - t0) * 1e3,   # total
        ))

    def _log_timing(self) -> None:
        if not self._timings:
            return
        arr = np.array(self._timings, dtype=np.float64)
        mean = arr.mean(axis=0)
        p95 = np.percentile(arr, 95, axis=0)
        self.get_logger().info(
            f'[bench {len(self._timings)} frames] '
            f'mean decode={mean[0]:.2f} undistort={mean[1]:.2f} infer={mean[2]:.2f} '
            f'pub={mean[3]:.2f} total={mean[4]:.2f} ms | '
            f'p95_infer={p95[2]:.2f} p95_total={p95[4]:.2f} ms'
        )
        self._timings.clear()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
