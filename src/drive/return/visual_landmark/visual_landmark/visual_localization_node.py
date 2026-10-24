"""Landmark RGB + current arm D435i RGB-D -> persistent mission<-odom correction.

Only landmarks captured in this process's current mission are loaded. No implicit
selection of historical missions. Failed matches leave the previous correction.
"""
import json
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from visual_landmark.localization import features, match, planar, transform


RECORDING_STATES = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}

def tf_matrix(t):
    p, q = t.transform.translation, t.transform.rotation
    return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class VisualLocalizationNode(Node):
    def __init__(self):
        super().__init__('visual_localization_node')
        for name, value in {
            'color_topic': '/arm/camera/color/image_raw/compressed',
            'depth_topic': '/arm/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/arm/camera/color/camera_info',
            'base_frame': 'base_link', 'odom_frame': 'odom',
            'max_correction_m': 3.0, 'max_correction_yaw_rad': 0.7,
            'candidate_radius_m': 12.0, 'processing_period_s': 1.0,
        }.items():
            self.declare_parameter(name, value)
        self.p = lambda n: self.get_parameter(n).value
        self.origin = None
        self.state = ''
        self.info = None
        self.landmarks = {}
        self.correction = None
        self.pending = None
        self.last = -float('inf')
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseStamped, '/visual_landmark/correction', 10)
        self.status = self.create_publisher(String, '/visual_landmark/localization_event', 10)
        self.create_subscription(Pose, '/mission/origin_pose', self.on_origin, latched)
        self.create_subscription(String, '/mission/return/state', self.on_state, 10)
        self.create_subscription(String, '/visual_landmark/event', self.on_landmark, 10)
        self.create_subscription(CameraInfo, self.p('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(self, CompressedImage, self.p('color_topic'),
                                                    qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, self.p('depth_topic'),
                                                    qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.04)
        self.sync.registerCallback(self.on_pair)

    def on_info(self, msg):
        self.info = msg

    def on_state(self, msg):
        if msg.data != self.state:
            self.pending = None
            if msg.data in RECORDING_STATES and self.state not in RECORDING_STATES:
                self.landmarks.clear()
                self.correction = None
        self.state = msg.data

    def on_origin(self, msg):
        q, p = msg.orientation, msg.position
        value = planar(transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        if self.origin is None or not np.allclose(value, self.origin, atol=1e-9):
            self.origin = value
            self.landmarks.clear()
            self.correction = None
            self.pending = None

    def event(self, kind, **fields):
        self.status.publish(String(data=json.dumps({'event': kind, **fields})))

    def on_landmark(self, msg):
        try:
            event = json.loads(msg.data)
            if event.get('event') != 'captured' or self.state not in RECORDING_STATES:
                return
            directory = Path(event['dir'])
            meta = yaml.safe_load((directory / 'meta.yaml').read_text())
            ci = meta['camera_info']
            if ci is None or meta.get('mission_camera_transform') is None:
                raise ValueError('landmark needs calibrated CameraInfo and timestamped camera TF')
            if ci.get('frame_id') != meta['camera_frame']:
                raise ValueError('reference calibration optical frame mismatch')
            if ci['k'][0] <= 0 or ci['k'][4] <= 0:
                raise ValueError('uncalibrated reference camera')
            if ci['distortion_model'] not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('unsupported reference distortion model')
            image = cv2.imread(str(directory / meta['image_file']), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (ci['height'], ci['width']):
                raise ValueError('invalid reference image/calibration size')
            points, descriptors = features(image)
            self.landmarks[meta['id']] = (
                (points, descriptors, np.array(ci['k']).reshape(3, 3), np.array(ci['d'])),
                np.array(meta['mission_camera_transform'], dtype=float))
            self.event('reference_loaded', id=meta['id'], features=len(points))
        except (ValueError, KeyError, OSError, cv2.error, TypeError) as exc:
            self.event('reference_rejected', reason=str(exc))

    def on_pair(self, color, depth):
        now = self.get_clock().now()
        if self.state not in {'FOLLOW_RETURN_PATH', 'RETURN_MANUAL', 'RETURN_RESUME_STOP', 'RETURN_INTERVENTION'} or self.origin is None or not self.landmarks or self.info is None:
            return
        if now.nanoseconds*1e-9 - self.last < self.p('processing_period_s'):
            return
        self.last = now.nanoseconds*1e-9
        try:
            stamp = Time.from_msg(color.header.stamp)
            if abs((now-stamp).nanoseconds*1e-9) > 0.5:
                raise ValueError('stale RGB-D pair')
            ci = self.info
            if ci.header.frame_id != color.header.frame_id or depth.header.frame_id != color.header.frame_id:
                raise ValueError('RGB, aligned depth and calibration must share optical frame')
            if ci.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('unsupported current distortion model')
            image = cv2.imdecode(np.frombuffer(color.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            z = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').astype(np.float32)
            if depth.encoding == '16UC1':
                z *= 0.001
            elif depth.encoding != '32FC1':
                raise ValueError('depth must be 16UC1 millimetres or 32FC1 metres')
            if image is None or image.shape != z.shape or image.shape != (ci.height, ci.width):
                raise ValueError('RGB-D/calibration resolution mismatch')
            if ci.k[0] <= 0 or ci.k[4] <= 0:
                raise ValueError('uncalibrated current camera')
            # Timestamped TF includes current arm joints and chassis roll/pitch.
            oc = tf_matrix(self.buffer.lookup_transform(self.p('odom_frame'), color.header.frame_id, stamp))
            ob = tf_matrix(self.buffer.lookup_transform(self.p('odom_frame'), self.p('base_frame'), stamp))
            current = features(image)
            prior = self.correction if self.correction is not None else np.linalg.inv(self.origin)
            ranked = []
            for lid, (reference, mc) in self.landmarks.items():
                if np.linalg.norm((prior @ oc)[:2, 3] - mc[:2, 3]) > self.p('candidate_radius_m'):
                    continue
                result = match(reference, current, z, np.array(ci.k).reshape(3, 3), np.array(ci.d))
                if result is None:
                    continue
                relative, count, error = result
                # reference<-current from PnP; mission<-base uses current articulated extrinsic.
                mb = mc @ relative @ np.linalg.inv(oc) @ ob
                correction = planar(mb) @ np.linalg.inv(planar(ob))
                delta = correction @ np.linalg.inv(prior)
                displacement = np.linalg.norm((correction @ ob)[:2, 3] - (prior @ ob)[:2, 3])
                yaw = abs(np.arctan2(delta[1, 0], delta[0, 0]))
                if displacement <= self.p('max_correction_m') and yaw <= self.p('max_correction_yaw_rad'):
                    ranked.append((count, -error, lid, correction))
            ranked.sort(key=lambda item: item[:2], reverse=True)
            if not ranked:
                self.pending = None
                self.event('no_match')
                return
            best = ranked[0]
            if len(ranked) > 1 and ranked[1][0] >= best[0]*0.85:
                raise ValueError('ambiguous landmarks')
            _, neg_error, lid, correction = best
            # Require two independent frames to agree before changing controller coordinates.
            if self.pending is None or self.pending[0] != lid:
                self.pending = (lid, correction)
                return
            previous = self.pending[1]
            delta = correction @ np.linalg.inv(previous)
            self.pending = (lid, correction)
            if np.linalg.norm((correction @ ob)[:2, 3] - (previous @ ob)[:2, 3]) > 0.25 or abs(np.arctan2(delta[1, 0], delta[0, 0])) > 0.1:
                return
            # Processing can be slow; never publish an old-frame result into live control.
            if (self.get_clock().now()-stamp).nanoseconds*1e-9 > 1.5:
                raise ValueError('localization processing too slow')
            self.correction = correction
            msg = PoseStamped()
            msg.header.stamp = color.header.stamp
            msg.header.frame_id = 'mission'
            msg.pose.position.x, msg.pose.position.y = map(float, correction[:2, 3])
            yaw = np.arctan2(correction[1, 0], correction[0, 0])
            msg.pose.orientation.z, msg.pose.orientation.w = float(np.sin(yaw/2)), float(np.cos(yaw/2))
            self.pub.publish(msg)
            self.event('localized', id=lid, inliers=best[0], reprojection_rmse=-neg_error)
        except Exception as exc:
            self.pending = None
            self.event('rejected', reason=str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = VisualLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
