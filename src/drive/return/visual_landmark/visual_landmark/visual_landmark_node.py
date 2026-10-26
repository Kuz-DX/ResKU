"""
visual_landmark_node

[capture_mode] 'moving'(주행 중 바로 촬영, 정지 안 함) / 'stop'(아래 흐름). launch의
landmark_capture_mode가 이 노드와 drive_cmd_mux에 같은 값을 넘긴다.

[OUTBOUND keyframe 촬영; RETURN 추정은 visual_localization_node] Back-to-Base 경로 중간의
landmark에서 후방 카메라 keyframe을 저장한다. 복귀(RETURN) 때 전방(로봇팔)
카메라로 비교하는 처리는 visual_localization_node가 담당한다. 이 노드는
주행 명령을 절대 내지 않고, 실패해도 Back-to-Base에는 영향이 없다.

촬영 흐름 (UI 버튼 한 번):
    /path/landmark rising edge (MANUAL_RECORDING / WAIT_RETURN_COMMAND에서)
      -> drive_cmd_mux가 즉시 manual 명령을 0으로 hold (이 노드가 죽어 있어도)
      -> 이 노드: /visual_landmark/hold=True 갱신하며
         정지 확인(odom) -> 잔진동 대기 -> 그 이후 도착한 후방 프레임 1장 저장
      -> /visual_landmark/hold=False -> mux가 manual 복귀
    자세한 단계/타임아웃은 capture_logic.py 참고.

저장 위치 (mission = /mission/origin_pose 하나):
    <db_root>/<YYYYmmdd_HHMMSS>/mission.yaml   T0(odom), 카메라 토픽
    <db_root>/<YYYYmmdd_HHMMSS>/L000/meta.yaml  id, 순서, mission pose, s_along_path,
                                               camera_info, extrinsic(base->camera), ...
    <db_root>/<YYYYmmdd_HHMMSS>/L000/rear.jpg   카메라가 보낸 압축 바이트 그대로(재인코딩 없음)

extrinsic(base_frame -> 이미지 frame_id)은 촬영 순간 TF에서 조회해 같이 저장한다.
카메라 static TF가 아직 없으면 null로 저장하고 event에 경고를 남긴다 -- 촬영 자체는
막지 않는다 (TF는 나중에 확정돼도 되지만, 매칭 단계에서 값이 바뀌었으면 경고해야 한다).

Subscribes:
    /path/landmark              std_msgs/Bool             UI 버튼 (rising edge)
    /mission/return/state       std_msgs/String
    /odometry/filtered          nav_msgs/Odometry
    /mission/origin_pose        geometry_msgs/Pose (TRANSIENT_LOCAL)  T0
    <rear_image_topic>          sensor_msgs/CompressedImage
    <rear_camera_info_topic>    sensor_msgs/CameraInfo
Publishes:
    /visual_landmark/hold       std_msgs/Bool    촬영 중 True(10 Hz), 끝나면 False
    /visual_landmark/event      std_msgs/String  JSON: captured / failed + reason
"""
import json
import math
import os
import time

import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from return_navigation.se2 import Pose2D
from visual_landmark.localization import transform, features, valid_transform
from visual_landmark.calibration import camera_model
from visual_landmark.adaptive import AutoKeyframes
import cv2
import numpy as np
from visual_landmark.capture_logic import CaptureSequencer, Odometer, PressDetector

# WAIT_RETURN_COMMAND means return is available; manual recording continues.
RECORDING_STATES = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}


def _quat_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr, cosr), math.asin(sinp), math.atan2(siny, cosy)


class VisualLandmarkNode(Node):
    def __init__(self):
        super().__init__('visual_landmark_node')
        self.declare_parameter('trigger_topic', '/path/landmark')
        self.declare_parameter('state_topic', '/mission/return/state')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('origin_topic', '/mission/origin_pose')
        self.declare_parameter('rear_image_topic', '/rear/camera/image_raw/compressed')
        self.declare_parameter('rear_camera_info_topic', '/rear/camera/camera_info')
        self.declare_parameter('hold_topic', '/visual_landmark/hold')
        self.declare_parameter('event_topic', '/visual_landmark/event')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('db_root', '~/.ros/visual_landmarks')
        # 'moving' = 주행 중 바로 촬영 (정지 안 함), 'stop' = mux가 정지시킨 뒤 촬영.
        # drive_cmd_mux의 landmark_capture_mode와 같은 값이어야 한다 (launch가 같이 넘김).
        self.declare_parameter('capture_mode', 'stop')
        # 정지 판정: /odometry/filtered twist (vx m/s, wz rad/s)
        self.declare_parameter('still_vx_mps', 0.02)
        self.declare_parameter('still_wz_radps', 0.02)
        self.declare_parameter('still_time_s', 0.5)
        self.declare_parameter('settle_s', 0.3)
        # drive_cmd_mux의 landmark_hold_max_s(6 s)보다 짧아야 이 노드가 먼저 정리한다.
        self.declare_parameter('capture_timeout_s', 5.0)
        # 버튼을 눌렀을 때 후방 프레임이 이 시간 안에 한 번도 안 왔으면 바로 실패.
        self.declare_parameter('image_stale_s', 1.0)

        for name, value in dict(enable_auto_keyframes=True, auto_keyframe_distance_m=1.25,
                                auto_keyframe_yaw_deg=12., auto_keyframe_max_interval_s=6.,
                                auto_keyframe_min_motion_for_timeout_m=.5,
                                auto_keyframe_retry_s=1., auto_keyframe_min_features=20,
                                auto_keyframe_blur_min=0.).items():
            self.declare_parameter(name, value)
        self.auto_enabled = self.get_parameter('enable_auto_keyframes').value
        self.auto_policy = AutoKeyframes(
            self.get_parameter('auto_keyframe_distance_m').value,
            self.get_parameter('auto_keyframe_yaw_deg').value,
            self.get_parameter('auto_keyframe_max_interval_s').value,
            self.get_parameter('auto_keyframe_min_motion_for_timeout_m').value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.auto_policy.distance, self.auto_policy.yaw_deg, self.auto_policy.interval, self.auto_policy.min_motion)):
            raise ValueError('auto keyframe thresholds must be positive')
        self.auto_next_try = 0.
        self._capture_source, self._capture_reason = 'manual', None
        self._capture_quality = None

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base_frame = p('base_frame')
        self.db_root = os.path.expanduser(p('db_root'))
        self.rear_image_topic = p('rear_image_topic')
        self.image_stale_s = float(p('image_stale_s'))
        self.capture_mode = str(p('capture_mode'))
        if self.capture_mode not in ('stop', 'moving'):
            self.get_logger().error(f"capture_mode must be 'stop' or 'moving', got "
                                    f"'{self.capture_mode}' -- using 'stop'.")
            self.capture_mode = 'stop'
        self.stop_mode = self.capture_mode == 'stop'
        self._press = PressDetector()
        self._odom_arrival = None
        self.seq = CaptureSequencer(
            still_vx=float(p('still_vx_mps')), still_wz=float(p('still_wz_radps')),
            still_time_s=float(p('still_time_s')), settle_s=float(p('settle_s')),
            timeout_s=float(p('capture_timeout_s')))
        self.odometer = Odometer()

        self._state = ''
        self._origin = None
        self._odom = None            # latest Odometry
        self._camera_info = None
        self._last_image_arrival = None
        self._mission_dir = None
        self._next_order = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hold_pub = self.create_publisher(Bool, p('hold_topic'), 10)
        self.event_pub = self.create_publisher(String, p('event_topic'), 10)
        self.create_subscription(Bool, p('trigger_topic'), self._on_trigger, 10)
        self.create_subscription(String, p('state_topic'), self._on_state, 10)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom, 10)
        self.create_subscription(Pose, p('origin_topic'), self._on_origin, latched)
        self.create_subscription(CompressedImage, self.rear_image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('rear_camera_info_topic'), self._on_camera_info,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'visual_landmark_node started (capture only, mode={self.capture_mode}). '
            f'rear image: {self.rear_image_topic}, db: {self.db_root}')

    # ------------------------------------------------------------------ #
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: String):
        if msg.data in RECORDING_STATES and self._state not in RECORDING_STATES:
            self._mission_dir = None
            self._next_order = 0
            self.odometer.reset()
            if hasattr(self, 'auto_policy'):
                self.auto_policy.reset()
        self._state = msg.data
        if self._state not in RECORDING_STATES and self.seq.busy:
            self._fail('recording ended during capture', hold_started=True)

    def _on_origin(self, msg: Pose):
        origin = Pose2D.from_ros_pose(msg)
        if self._origin is not None and (origin.x, origin.y, origin.yaw) == \
                (self._origin.x, self._origin.y, self._origin.yaw):
            return
        # 새 mission: landmark 번호와 경로 거리를 처음부터. 폴더는 첫 촬영 때 만든다.
        self._origin = origin
        self._mission_dir = None
        self._next_order = 0
        self.odometer.reset()
        if hasattr(self, 'auto_policy'):
            self.auto_policy.reset()
            self.auto_next_try = 0.

    def _on_odom(self, msg: Odometry):
        self._odom = msg
        self._odom_arrival = self._now()
        self.seq.on_odom(self._now(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self._origin is not None and self._state in RECORDING_STATES:
            m = Pose2D.from_ros_pose(msg.pose.pose).to_mission_frame(self._origin)
            self.odometer.update(m.x, m.y)

    def _on_camera_info(self, msg: CameraInfo):
        self._camera_info = msg

    def _on_trigger(self, msg: Bool):
        now = self._now()
        if not self._press.update(now, msg.data):
            return
        if self.seq.busy:
            self.get_logger().warn('Landmark capture already in progress -- button ignored.')
            return
        if self._state not in RECORDING_STATES:
            return self._fail(f'not recording (state "{self._state}")', hold_started=False)
        if self._origin is None or self._odom is None:
            return self._fail('no mission origin / odometry yet', hold_started=False)
        if self._last_image_arrival is None or now - self._last_image_arrival > self.image_stale_s:
            return self._fail(f'no rear image on {self.rear_image_topic}', hold_started=False)
        self._capture_source, self._capture_reason = 'manual', None
        self._capture_quality = None
        self.seq.start(now, wait_still=self.stop_mode)
        if self.stop_mode:
            self.hold_pub.publish(Bool(data=True))
            self.get_logger().info('Landmark capture started: waiting for the robot to stop.')
        else:
            self.get_logger().info('Landmark capture started (moving): taking the next frame.')

    def _tick(self):
        if not self.seq.busy:
            return
        if self.stop_mode:
            self.hold_pub.publish(Bool(data=True))
        reason = self.seq.tick(self._now())
        if reason is not None:
            self._fail(reason, hold_started=True)

    def _on_image(self, msg: CompressedImage):
        arrival = self._now()
        self._last_image_arrival = arrival
        if not self.seq.busy and getattr(self, 'auto_enabled', False):
            self._try_auto(msg, arrival)
        if not self.seq.accept_frame(arrival):
            return
        image_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if image_time <= 0 or not 0 <= arrival-image_time <= self.image_stale_s:
            return  # keep waiting for a fresh frame, never archive a queued old image
        if self.seq._frame_after is not None and image_time < self.seq._frame_after:
            return
        try:
            info = self._save_landmark(msg)
        except Exception as exc:  # 디스크/권한 오류도 주행을 막으면 안 된다
            self._fail(f'save failed: {exc}', hold_started=True)
            return
        self.seq.finish()
        if getattr(self, 'auto_enabled', False) and info.get('localization_ready'):
            self.auto_policy.accept(arrival, self.odometer.s, info['yaw'])
        if self.stop_mode:
            self.hold_pub.publish(Bool(data=False))
        self._event('captured', **info)
        self.get_logger().info(
            f"Landmark {info['id']} saved at mission ({info['x']:.2f}, {info['y']:.2f}, "
            f"{math.degrees(info['yaw']):.1f} deg), s={info['s_along_path']:.2f} m"
            + ('' if info['extrinsic_ok'] else ' -- WARNING: no camera TF, extrinsic not stored'))

    def _try_auto(self, msg, now):
        if (self._state not in RECORDING_STATES or self._origin is None or self._odom is None
                or now < self.auto_next_try):
            return
        pose = Pose2D.from_ros_pose(self._odom.pose.pose).to_mission_frame(self._origin)
        reason = self.auto_policy.reason(now, self.odometer.s, pose.yaw)
        if reason is None:
            return
        self.auto_next_try = now + max(.1, float(self.get_parameter('auto_keyframe_retry_s').value))
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp <= 0 or not 0 <= now-stamp <= self.image_stale_s:
                raise ValueError('stale automatic image')
            odom_stamp = self._odom.header.stamp.sec+self._odom.header.stamp.nanosec*1e-9
            if odom_stamp <= 0 or abs(stamp-odom_stamp) > .5 or not 0 <= now-self._odom_arrival <= .5:
                raise ValueError('stale automatic odometry')
            ci = self._camera_info
            if ci is None or ci.header.frame_id != msg.header.frame_id:
                raise ValueError('automatic CameraInfo/frame missing')
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError('automatic image decode failed')
            camera_model(ci.width, ci.height, ci.k, ci.d, ci.distortion_model, image.shape)
            blur = float(cv2.Laplacian(image, cv2.CV_64F).var())
            if blur < float(self.get_parameter('auto_keyframe_blur_min').value):
                raise ValueError('automatic blur gate')
            points, descriptors = features(image)
            minimum = max(20, int(self.get_parameter('auto_keyframe_min_features').value))
            if descriptors is None or len(points) < minimum:
                raise ValueError('automatic insufficient SIFT features')
            self._capture_source, self._capture_reason = 'automatic', reason
            self._capture_quality = dict(usable_features=len(points), laplacian_variance=blur)
            info = self._save_landmark(msg)
            if not info['localization_ready'] or not info['extrinsic_ok']:
                raise ValueError('automatic timestamped camera/base TF missing (archive retained)')
            np.savez_compressed(os.path.join(info['dir'], 'features.npz'),
                                points=points, descriptors=descriptors)
            self.auto_policy.accept(now, self.odometer.s, info['yaw'])
            self._event('captured', capture_source='automatic', capture_reason=reason, **info)
        except Exception as exc:
            # No hold or motion command even on failure. Invalid archives are never announced as references.
            self._event('automatic_rejected', reason=str(exc), capture_reason=reason)
        finally:
            self._capture_source, self._capture_reason = 'manual', None
            self._capture_quality = None

    # ------------------------------------------------------------------ #
    def _fail(self, reason: str, hold_started: bool):
        if hold_started:
            self.seq.finish()
        # mux는 이 노드가 응답 없으면 grace(1 s) 후 알아서 풀지만, 실패를 알면 즉시 푼다.
        self.hold_pub.publish(Bool(data=False))
        self._event('failed', reason=reason)
        self.get_logger().warn(f'Landmark capture failed: {reason}')

    def _event(self, kind: str, **fields):
        self.event_pub.publish(String(data=json.dumps({'event': kind, **fields})))

    def _ensure_mission_dir(self):
        if self._mission_dir is None:
            stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1000000000:09d}'
            self._mission_dir = os.path.join(self.db_root, stamp)
            os.makedirs(self._mission_dir, exist_ok=True)
            with open(os.path.join(self._mission_dir, 'mission.yaml'), 'w') as f:
                yaml.safe_dump({
                    'created': stamp,
                    'origin_odom': {'x': self._origin.x, 'y': self._origin.y,
                                    'yaw': self._origin.yaw},
                    'rear_image_topic': self.rear_image_topic,
                    'base_frame': self.base_frame,
                }, f, sort_keys=False)
        return self._mission_dir

    def _lookup_extrinsic(self, camera_frame: str, stamp=None):
        if not camera_frame:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, camera_frame, stamp if stamp is not None else Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        roll, pitch, yaw = _quat_to_rpy(q)
        return {'parent': self.base_frame, 'child': camera_frame,
                'xyz': [float(tr.x), float(tr.y), float(tr.z)],
                'quat_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
                'rpy': [float(roll), float(pitch), float(yaw)]}

    def _save_landmark(self, img: CompressedImage) -> dict:
        mission_dir = self._ensure_mission_dir()
        order = self._next_order
        lid = f'L{order:03d}'
        ldir = os.path.join(mission_dir, lid)
        os.makedirs(ldir, exist_ok=True)

        fmt = (img.format or '').lower()
        ext = '.png' if 'png' in fmt else '.jpg'
        image_file = 'rear' + ext
        with open(os.path.join(ldir, image_file), 'wb') as f:
            f.write(bytes(img.data))

        odom = self._odom
        odom_pose = Pose2D.from_ros_pose(odom.pose.pose)
        mission = odom_pose.to_mission_frame(self._origin)
        camera_frame = img.header.frame_id or (
            self._camera_info.header.frame_id if self._camera_info else '')
        extrinsic = self._lookup_extrinsic(camera_frame, Time.from_msg(img.header.stamp))
        ci = self._camera_info
        # Use image-time TF, including body roll/pitch and articulated camera pose.
        # Keep capture usable without TF, but localization must reject such records.
        mission_camera = None
        image_base = None
        try:
            t = self.tf_buffer.lookup_transform(
                odom.header.frame_id, camera_frame, Time.from_msg(img.header.stamp))
            tr, q = t.transform.translation, t.transform.rotation
            import numpy as np
            origin = transform([self._origin.x, self._origin.y, 0],
                               [0, 0, math.sin(self._origin.yaw/2), math.cos(self._origin.yaw/2)])
            bt = self.tf_buffer.lookup_transform(
                odom.header.frame_id, self.base_frame, Time.from_msg(img.header.stamp))
            bp, bq = bt.transform.translation, bt.transform.rotation
            image_base = transform([bp.x, bp.y, bp.z], [bq.x, bq.y, bq.z, bq.w])
            odom_pose = Pose2D(float(bp.x), float(bp.y), math.atan2(image_base[1, 0], image_base[0, 0]))
            mission = odom_pose.to_mission_frame(self._origin)
            mission_camera = (np.linalg.inv(origin) @ transform(
                [tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])).tolist()
        except TransformException:
            pass
        meta = {
            'schema_version': 3,
            'capture_source': getattr(self, '_capture_source', 'manual'),
            'automatic_capture_reason': getattr(self, '_capture_reason', None),
            'capture_quality': getattr(self, '_capture_quality', None),
            'mission_camera_transform': mission_camera,
            'image_time_base_transform': None if image_base is None else image_base.tolist(),
            'id': lid,
            'order': order,
            'image_file': image_file,
            'image_format': img.format,
            'image_stamp': img.header.stamp.sec + 1e-9 * img.header.stamp.nanosec,
            'camera_frame': camera_frame,
            'mission_pose': {'x': mission.x, 'y': mission.y, 'yaw': mission.yaw},
            'odom_pose': {'x': odom_pose.x, 'y': odom_pose.y, 'yaw': odom_pose.yaw},
            's_along_path': self.odometer.s,
            'capture_mode': 'moving' if getattr(self, '_capture_source', 'manual') == 'automatic' else self.capture_mode,
            # moving 모드 품질 판단용: 속도가 클수록 rolling shutter 흐림/휨이 크고,
            # odom과 이미지 도착 시각 차이 x 속도만큼 기록 위치가 어긋난다.
            'odom_sample_stamp': odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9,
            'odom_twist_at_capture': {'vx': float(odom.twist.twist.linear.x),
                                      'wz': float(odom.twist.twist.angular.z)},
            'odom_to_image_arrival_s': float(self._now() - self._odom_arrival),
            'extrinsic_base_to_camera': extrinsic,
            # Humble의 CameraInfo 배열은 numpy라 YAML이 못 쓴다 -> 순수 float로.
            'camera_info': None if ci is None else {
                'frame_id': ci.header.frame_id,
                'width': int(ci.width), 'height': int(ci.height),
                'distortion_model': ci.distortion_model,
                'd': [float(v) for v in ci.d], 'k': [float(v) for v in ci.k],
                'r': [float(v) for v in ci.r], 'p': [float(v) for v in ci.p],
            },
        }
        with open(os.path.join(ldir, 'meta.yaml'), 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False)
        self._next_order += 1
        return {'id': lid, 'order': order, 'dir': ldir, 'x': mission.x, 'y': mission.y,
                'yaw': mission.yaw, 's_along_path': self.odometer.s,
                'extrinsic_ok': extrinsic is not None, 'camera_info_ok': ci is not None,
                'localization_ready': bool(mission_camera is not None and image_base is not None and ci is not None
                and valid_transform(mission_camera) and valid_transform(image_base))}


def main(args=None):
    rclpy.init(args=args)
    node = VisualLandmarkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
