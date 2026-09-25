#!/usr/bin/env python3

import json
import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String

from pupil_apriltags import Detector


class AprilTagCompressedNode(Node):

    def __init__(self):
        super().__init__('apriltag_compressed_node')

        # ============================================================
        # ROS Parameters
        # ============================================================
        self.declare_parameter(
            'input_topic',
            '/camera/camera/color/image_raw/compressed'
        )

        self.declare_parameter(
            'output_topic',
            '/apriltag/image/compressed'
        )

        self.declare_parameter(
            'centers_topic',
            '/apriltag/centers'
        )

        self.declare_parameter(
            'camera_info_topic',
            ''
        )

        # AprilTag 검은색 외곽 사각형 한 변의 실제 길이
        self.declare_parameter(
            'tag_size_cm',
            2.0
        )

        self.declare_parameter(
            'family',
            'tag36h11'
        )

        self.declare_parameter(
            'jpeg_quality',
            90
        )

        self.declare_parameter(
            'subpixel_refine',
            True
        )

        self.input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )

        self.output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )

        self.centers_topic = (
            self.get_parameter('centers_topic')
            .get_parameter_value()
            .string_value
        )

        self.camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )

        if not self.camera_info_topic:
            image_suffix = '/image_raw/compressed'
            if not self.input_topic.endswith(image_suffix):
                raise ValueError(
                    'camera_info_topic is required when input_topic does not '
                    f'end with {image_suffix!r}.'
                )
            self.camera_info_topic = (
                self.input_topic[:-len(image_suffix)] + '/camera_info'
            )

        self.tag_size_cm = (
            self.get_parameter('tag_size_cm')
            .get_parameter_value()
            .double_value
        )

        if self.tag_size_cm <= 0.0:
            raise ValueError('tag_size_cm must be greater than 0.')

        self.tag_size_m = self.tag_size_cm / 100.0
        self.camera_params = None
        self.camera_info_size = None

        self.family = (
            self.get_parameter('family')
            .get_parameter_value()
            .string_value
        )

        self.jpeg_quality = (
            self.get_parameter('jpeg_quality')
            .get_parameter_value()
            .integer_value
        )

        self.subpixel_refine = (
            self.get_parameter('subpixel_refine')
            .get_parameter_value()
            .bool_value
        )

        # ============================================================
        # AprilTag detector
        #
        # quad_decimate = 1.0
        #   -> 원본 해상도에서 검출
        #   -> 속도보다 위치 정밀도 우선
        #
        # refine_edges = 1
        #   -> tag edge 위치 보정
        # ============================================================
        self.detector = Detector(
            families=self.family,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ============================================================
        # ROS Subscriber / Publisher
        # ============================================================
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # 시각화 역시 CompressedImage만 발행
        self.image_pub = self.create_publisher(
            CompressedImage,
            self.output_topic,
            qos_profile_sensor_data
        )

        # 중심좌표를 별도의 JSON 문자열로 발행
        self.center_pub = self.create_publisher(
            String,
            self.centers_topic,
            10
        )

        self.get_logger().info(
            f'Input  : {self.input_topic}'
        )

        self.get_logger().info(
            f'Output : {self.output_topic}'
        )

        self.get_logger().info(
            f'Family : {self.family}'
        )

        self.get_logger().info(
            f'Tag size: {self.tag_size_cm:.2f} cm'
        )

        self.get_logger().info(
            f'Camera info: {self.camera_info_topic}'
        )

    def camera_info_callback(self, msg):
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning('Invalid focal length in CameraInfo.')
            return

        self.camera_params = (fx, fy, cx, cy)
        self.camera_info_size = (int(msg.width), int(msg.height))

        if not hasattr(self, '_camera_info_logged'):
            self.get_logger().info(
                f'CameraInfo received: fx={fx:.2f}, fy={fy:.2f}, '
                f'cx={cx:.2f}, cy={cy:.2f}'
            )
            self._camera_info_logged = True

    def camera_params_for_image(self, width, height):
        if self.camera_params is None:
            return None

        fx, fy, cx, cy = self.camera_params
        info_width, info_height = self.camera_info_size

        if info_width > 0 and info_height > 0:
            scale_x = width / info_width
            scale_y = height / info_height
            return (
                fx * scale_x,
                fy * scale_y,
                cx * scale_x,
                cy * scale_y
            )

        return self.camera_params

    # ================================================================
    # 두 선분이 이루는 직선의 교점 계산
    #
    # AprilTag의 중심 =
    #   corner 0 <-> corner 2 대각선
    #   corner 1 <-> corner 3 대각선
    #
    # 두 대각선의 교점
    # ================================================================
    @staticmethod
    def line_intersection(p1, p2, p3, p4):

        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denominator = (
            (x1 - x2) * (y3 - y4)
            -
            (y1 - y2) * (x3 - x4)
        )

        # 거의 평행한 경우
        if abs(denominator) < 1e-9:
            return None

        det1 = x1 * y2 - y1 * x2
        det2 = x3 * y4 - y3 * x4

        px = (
            det1 * (x3 - x4)
            -
            (x1 - x2) * det2
        ) / denominator

        py = (
            det1 * (y3 - y4)
            -
            (y1 - y2) * det2
        ) / denominator

        return np.array([px, py], dtype=np.float64)

    # ================================================================
    # Sub-pixel corner refinement
    # ================================================================
    def refine_corners(self, gray, corners):

        corners = np.asarray(
            corners,
            dtype=np.float32
        ).reshape(4, 2)

        if not self.subpixel_refine:
            return corners.astype(np.float64)

        h, w = gray.shape[:2]

        # cornerSubPix의 탐색 윈도우가 영상 밖으로 나가면
        # OpenCV 오류가 발생할 수 있으므로 검사
        margin = 7

        valid = (
            np.all(corners[:, 0] >= margin)
            and np.all(corners[:, 0] < w - margin)
            and np.all(corners[:, 1] >= margin)
            and np.all(corners[:, 1] < h - margin)
        )

        if not valid:
            return corners.astype(np.float64)

        refined = corners.reshape(-1, 1, 2).copy()

        criteria = (
            cv2.TERM_CRITERIA_EPS
            +
            cv2.TERM_CRITERIA_MAX_ITER,
            40,
            0.001
        )

        try:
            cv2.cornerSubPix(
                gray,
                refined,
                winSize=(5, 5),
                zeroZone=(-1, -1),
                criteria=criteria
            )

            return refined.reshape(4, 2).astype(np.float64)

        except cv2.error:
            return corners.astype(np.float64)

    # ================================================================
    # ROS callback
    # ================================================================
    def image_callback(self, msg):

        start_time = time.perf_counter()

        # ------------------------------------------------------------
        # CompressedImage -> OpenCV image
        # ------------------------------------------------------------
        np_data = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            np_data,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            self.get_logger().warning(
                'Failed to decode compressed image.'
            )
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # ------------------------------------------------------------
        # AprilTag Detection
        # ------------------------------------------------------------
        height, width = gray.shape[:2]
        camera_params = self.camera_params_for_image(width, height)
        estimate_pose = camera_params is not None

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=estimate_pose,
            camera_params=camera_params,
            tag_size=self.tag_size_m
        )

        result_list = []

        for detection in detections:

            # --------------------------------------------------------
            # Tag family
            # --------------------------------------------------------
            if isinstance(detection.tag_family, bytes):
                family = detection.tag_family.decode('utf-8')
            else:
                family = str(detection.tag_family)

            tag_id = int(detection.tag_id)

            # --------------------------------------------------------
            # AprilTag 원본 corner
            # +
            # OpenCV sub-pixel 보정
            # --------------------------------------------------------
            corners = self.refine_corners(
                gray,
                detection.corners
            )

            # --------------------------------------------------------
            # 정밀 중심좌표
            #
            # 대각선:
            # corner[0] -> corner[2]
            # corner[1] -> corner[3]
            # --------------------------------------------------------
            precise_center = self.line_intersection(
                corners[0],
                corners[2],
                corners[1],
                corners[3]
            )

            # 교점을 구할 수 없는 비정상적인 경우에는
            # AprilTag detector의 center 사용
            if precise_center is None:
                precise_center = np.asarray(
                    detection.center,
                    dtype=np.float64
                )

            u = float(precise_center[0])
            v = float(precise_center[1])

            position_cm = None
            distance_cm = None

            if estimate_pose and detection.pose_t is not None:
                pose_t_m = np.asarray(
                    detection.pose_t,
                    dtype=np.float64
                ).reshape(3)
                position_cm = pose_t_m * 100.0
                distance_cm = float(np.linalg.norm(position_cm))

            # --------------------------------------------------------
            # Visualization
            # --------------------------------------------------------

            polygon = np.round(
                corners
            ).astype(np.int32)

            # Tag 외곽선
            cv2.polylines(
                frame,
                [polygon],
                True,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            # 각 corner 표시
            for index, corner in enumerate(corners):

                cx = int(round(corner[0]))
                cy = int(round(corner[1]))

                cv2.circle(
                    frame,
                    (cx, cy),
                    4,
                    (255, 0, 255),
                    -1,
                    cv2.LINE_AA
                )

                cv2.putText(
                    frame,
                    str(index),
                    (cx + 5, cy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA
                )

            # --------------------------------------------------------
            # 중심 표시
            # --------------------------------------------------------
            center_draw = (
                int(round(u)),
                int(round(v))
            )

            # 원
            cv2.circle(
                frame,
                center_draw,
                7,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )

            # 십자선
            cross_size = 12

            cv2.line(
                frame,
                (
                    center_draw[0] - cross_size,
                    center_draw[1]
                ),
                (
                    center_draw[0] + cross_size,
                    center_draw[1]
                ),
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

            cv2.line(
                frame,
                (
                    center_draw[0],
                    center_draw[1] - cross_size
                ),
                (
                    center_draw[0],
                    center_draw[1] + cross_size
                ),
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

            # --------------------------------------------------------
            # 정보 표시
            # --------------------------------------------------------
            text1 = (
                f'{family}  ID:{tag_id}'
            )

            text2 = (
                f'center = ({u:.3f}, {v:.3f}) px'
            )

            text3 = (
                f'margin = {detection.decision_margin:.2f}'
            )

            if position_cm is not None:
                text4 = (
                    f'XYZ = ({position_cm[0]:.1f}, '
                    f'{position_cm[1]:.1f}, {position_cm[2]:.1f}) cm'
                )
                text5 = f'distance = {distance_cm:.1f} cm'
            else:
                text4 = 'distance = waiting for CameraInfo'
                text5 = None

            text_x = center_draw[0] + 15
            text_y = center_draw[1] - 25

            cv2.putText(
                frame,
                text1,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                text2,
                (text_x, text_y + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                text3,
                (text_x, text_y + 44),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                text4,
                (text_x, text_y + 66),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                1,
                cv2.LINE_AA
            )

            if text5 is not None:
                cv2.putText(
                    frame,
                    text5,
                    (text_x, text_y + 88),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

            # --------------------------------------------------------
            # ROS 중심좌표 데이터
            # --------------------------------------------------------
            result_list.append(
                {
                    'family': family,
                    'id': tag_id,

                    'center_px': {
                        'u': u,
                        'v': v
                    },

                    'detector_center_px': {
                        'u': float(detection.center[0]),
                        'v': float(detection.center[1])
                    },

                    'corners_px': [
                        {
                            'u': float(c[0]),
                            'v': float(c[1])
                        }
                        for c in corners
                    ],

                    'hamming': int(detection.hamming),

                    'decision_margin':
                        float(detection.decision_margin),

                    'position_camera_cm': None if position_cm is None else {
                        'x': float(position_cm[0]),
                        'y': float(position_cm[1]),
                        'z': float(position_cm[2])
                    },

                    'distance_camera_cm': distance_cm
                }
            )

        # ------------------------------------------------------------
        # FPS / processing time visualization
        # ------------------------------------------------------------
        processing_ms = (
            time.perf_counter() - start_time
        ) * 1000.0

        cv2.putText(
            frame,
            f'Tags: {len(detections)}',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            f'Processing: {processing_ms:.1f} ms',
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA
        )

        # ------------------------------------------------------------
        # Visualization -> JPEG compressed
        # ------------------------------------------------------------
        success, encoded = cv2.imencode(
            '.jpg',
            frame,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                self.jpeg_quality
            ]
        )

        if success:

            output_msg = CompressedImage()

            # 원본 카메라 timestamp 그대로 유지
            output_msg.header = msg.header

            output_msg.format = 'jpeg'

            output_msg.data = encoded.tobytes()

            self.image_pub.publish(
                output_msg
            )

        # ------------------------------------------------------------
        # 중심좌표 JSON publish
        # ------------------------------------------------------------
        center_msg = String()

        center_msg.data = json.dumps(
            {
                'stamp': {
                    'sec': int(msg.header.stamp.sec),
                    'nanosec': int(msg.header.stamp.nanosec)
                },

                'frame_id':
                    msg.header.frame_id,

                'detections':
                    result_list
            }
        )

        self.center_pub.publish(
            center_msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagCompressedNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
