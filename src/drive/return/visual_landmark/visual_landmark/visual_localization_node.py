"""Landmark RGB + current arm D435i RGB-D -> persistent mission<-odom correction.

Only landmarks captured in this process's current mission are loaded. No implicit
selection of historical missions. Failed matches leave the previous correction.
"""
import math
import json
from pathlib import Path
import time

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
from nav_msgs.msg import Path as RoutePath
from return_navigation.recovery import project_route
from return_navigation.se2 import Pose2D
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from visual_landmark.adaptive import candidate_window, reference_arcs
from visual_landmark.calibration import camera_model
from visual_landmark.localization import MatchOptions, features, match, planar, transform, valid_transform, pose_fields


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
            'publish_debug_matches': False, 'apply_correction': False, 'lock_pose_guard': True, 'expected_lock_pose_file': '',
            'lock_translation_tolerance_m': 0.02, 'lock_rotation_tolerance_rad': 0.035,
            'agreement_translation_m': 0.25, 'agreement_yaw_rad': 0.1,
            'agreement_max_interval_s': 2.5, 'ambiguity_ratio': 0.85,
            'route_backtrack_m': 2.0,
            'enable_adaptive_visual_check': True, 'max_visual_candidates': 3,
            'visual_check_timeout_s': 5., 'visual_check_max_frames': 4,
            'correction_min_translation_m': .03, 'correction_min_yaw_rad': .01,
        }.items():
            self.declare_parameter(name, value)
        self.p = lambda n: self.get_parameter(n).value
        defaults = MatchOptions()
        for key, value in vars(defaults).items():
            self.declare_parameter('match.'+key, value)
        self.match_options = MatchOptions(**{key: self.p('match.'+key) for key in vars(defaults)})
        if not 1 <= int(self.p('max_visual_candidates')) <= 7:
            raise ValueError('max_visual_candidates must be between 1 and 7')
        if self.p('visual_check_max_frames') < 2 or self.p('visual_check_timeout_s') <= 0:
            raise ValueError('check needs at least two frames and positive timeout')
        for name in ['correction_min_translation_m', 'correction_min_yaw_rad']:
            if not math.isfinite(self.p(name)) or self.p(name) < 0:
                raise ValueError('invalid correction deadband')
        self.debug_enabled = bool(self.p('publish_debug_matches'))
        self.check = None
        self.check_frames = 0
        self.check_expansion = 0
        self.route_progress = None
        self.cached_arcs = None
        self.lock_pose = None
        self.reference_images = {}
        self.reference_meta = {}
        self.route = []
        self.last_reference_arc = None
        self.pending_stamp = None
        self.origin = None
        self.state = ''
        self.info = None
        self.landmarks = {}
        self.correction = None
        self.pending = None
        self.search = None
        self.last = -float('inf')
        self.bridge = CvBridge()
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseStamped, '/visual_landmark/correction', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/visual_landmark/debug/matches/compressed', 1)
        self.status = self.create_publisher(String, '/visual_landmark/localization_event', 10)
        self.create_subscription(RoutePath, '/return_path', self.on_route, latched)
        self.create_subscription(Pose, '/mission/origin_pose', self.on_origin, latched)
        self.create_subscription(String, '/mission/return/state', self.on_state, 10)
        self.create_subscription(String, '/visual_landmark/event', self.on_landmark, 10)
        self.create_subscription(String, '/visual_landmark/search_request', self.on_search_request, 10)
        self.create_subscription(CameraInfo, self.p('camera_info_topic'), self.on_info,
                                 qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(self, CompressedImage, self.p('color_topic'),
                                                    qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, self.p('depth_topic'),
                                                    qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.04)
        self.sync.registerCallback(self.on_pair)
        self.create_subscription(String, '/visual_landmark/check_request', self.on_check_request, 10)
        self.create_subscription(String, '/visual_landmark/route_progress', self.on_progress, 1)
        self.create_timer(.1, self.check_timeout)

    def on_route(self, msg):
        self.route = [Pose2D.from_ros_pose(p.pose) for p in msg.poses]
        self.last_reference_arc = None
        self.route_progress = None
        self.cached_arcs = None

    def on_progress(self, msg):
        try:
            data = json.loads(msg.data)
            if math.isfinite(float(data['arc'])) and int(data['stamp_ns']) > 0:
                self.route_progress = (float(data['arc']), int(data['stamp_ns']))
        except (ValueError, TypeError, KeyError):
            pass

    def on_check_request(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('active') is False:
                if self.check and self.check['request'] == data.get('request'):
                    self.check = None
                    self.pending = None
                return
            now = self.get_clock().now().nanoseconds*1e-9
            if (not data.get('request') or self.check is not None or self.state != 'FOLLOW_RETURN_PATH'
                    or not 0 <= now-float(data['requested_at']) <= .5):
                return
            self.check = dict(data, deadline=now+min(float(data.get('timeout_s', 5.)), self.p('visual_check_timeout_s')))
            self.check_frames, self.check_expansion = 0, 0
            self.pending = None
            self.last = -float('inf')
            self.event('check_started', trigger=data.get('trigger'))
        except (ValueError, KeyError, TypeError):
            return

    def check_timeout(self):
        if self.check and self.get_clock().now().nanoseconds*1e-9 >= self.check['deadline']:
            self.event('skipped', reason='check timeout', check_complete=True)

    def check_lock_pose(self, current):
        if not bool(self.p('lock_pose_guard')):
            return
        if self.lock_pose is None:
            filename = self.p('expected_lock_pose_file')
            self.lock_pose = (np.array(yaml.safe_load(Path(filename).read_text())['base_from_camera'], dtype=float)
                              if filename else current.copy())
            if not valid_transform(self.lock_pose):
                raise ValueError('invalid expected lock transform')
        delta = np.linalg.inv(self.lock_pose) @ current
        angle = float(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1)))
        if np.linalg.norm(delta[:3, 3]) > self.p('lock_translation_tolerance_m') or angle > self.p('lock_rotation_tolerance_rad'):
            raise ValueError('D435i left its locked arm pose')

    def on_search_request(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('active'):
                if self.search is None or self.search.get('request') != data.get('request') or self.search.get('commit_after_ns') != data.get('commit_after_ns'):
                    self.pending = None
                self.search = data
            else:
                self.search = None
                self.pending = None
        except (ValueError, TypeError):
            return

    def on_info(self, msg):
        self.info = msg

    def on_state(self, msg):
        if msg.data != self.state:
            self.pending = None
            self.check = None
            if msg.data in RECORDING_STATES and self.state not in RECORDING_STATES:
                self.landmarks.clear()
                getattr(self, 'reference_images', {}).clear()
                self.correction = None
                self.lock_pose = None
                self.reference_meta.clear()
                self.cached_arcs = None
                self.route_progress = None
                self.route = []
                self.last_reference_arc = None
        self.state = msg.data

    def on_origin(self, msg):
        q, p = msg.orientation, msg.position
        value = planar(transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        if self.origin is None or not np.allclose(value, self.origin, atol=1e-9):
            self.origin = value
            self.check = None
            self.route_progress = None
            self.cached_arcs = None
            self.landmarks.clear()
            getattr(self, 'reference_images', {}).clear()
            self.correction = None
            self.pending = None
            self.lock_pose = None
            self.reference_meta.clear()
            self.last_reference_arc = None

    def event(self, kind, **fields):
        check = getattr(self, 'check', None)
        if check:
            fields.setdefault('check_request', check['request'])
            fields.setdefault('trigger', check.get('trigger'))
            if kind in {'validated', 'localized'}:
                fields['check_complete'] = True
            elif kind in {'no_match', 'rejected'}:
                self.check_expansion = 2
                self.pending = None
                fields.setdefault('check_complete', self.check_frames >= self.p('visual_check_max_frames'))
        self.status.publish(String(data=json.dumps({'event': kind, **fields})))
        if check and fields.get('check_complete'):
            self.check = None
            self.pending = None

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
            camera_model(ci['width'], ci['height'], ci['k'], ci['d'], ci['distortion_model'], image.shape)
            if not valid_transform(meta['mission_camera_transform']):
                raise ValueError('invalid reference camera transform')
            cache = directory / 'features.npz'
            if cache.is_file():
                with np.load(cache, allow_pickle=False) as stored:
                    points, descriptors = stored['points'], stored['descriptors']
                if (points.ndim != 2 or points.shape[1] != 2 or descriptors.shape != (len(points), 128)
                        or not np.all(np.isfinite(points)) or not np.all(np.isfinite(descriptors))):
                    raise ValueError('invalid cached reference SIFT')
            else:
                points, descriptors = features(image)
            if descriptors is None or len(points) < 20:
                raise ValueError('reference has fewer than 20 usable features')
            self.landmarks[meta['id']] = (
                (points, descriptors, np.array(ci['k']).reshape(3, 3), np.array(ci['d'])),
                np.array(meta['mission_camera_transform'], dtype=float))
            if getattr(self, 'debug_enabled', False):
                self.reference_images[meta['id']] = image
            self.reference_meta[meta['id']] = meta
            self.cached_arcs = None
            self.event('reference_loaded', id=meta['id'], features=len(points),
                       x=float(meta['mission_pose']['x']), y=float(meta['mission_pose']['y']))
        except (ValueError, KeyError, OSError, cv2.error, TypeError) as exc:
            self.event('reference_rejected', reason=str(exc))

    def publish_matches(self, lid, image, current, debug, diagnostics, header):
        # Only render actual matcher correspondences; never rerun SIFT/PnP.
        reference = self.reference_images.get(lid)
        if reference is None:
            return
        h = max(reference.shape[0], image.shape[0])
        w = reference.shape[1]
        canvas = np.zeros((h, w+image.shape[1], 3), np.uint8)
        canvas[:reference.shape[0], :w] = cv2.cvtColor(reference, cv2.COLOR_GRAY2BGR)
        canvas[:image.shape[0], w:] = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for x, y in current[0]:
            cv2.circle(canvas, (int(x)+w, int(y)), 2, (255, 180, 0), 1)
        for cp, rp in debug.get('pairs', [])[:150]:
            cv2.line(canvas, tuple(map(int, rp)), (int(cp[0])+w, int(cp[1])), (0, 150, 220), 1)
        pairs = debug.get('depth_pairs', [])
        for i in debug.get('inlier_indices', [])[:150]:
            cp, rp = pairs[i]
            cv2.line(canvas, tuple(map(int, rp)), (int(cp[0])+w, int(cp[1])), (0, 255, 0), 1)
        label = f"rear ref {lid} | front current | pairs={diagnostics.get('mutual_ratio_matches', 0)} inliers={diagnostics.get('inliers', 0)}"
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 65), (0, 0, 0), -1)
        cv2.putText(canvas, label, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        cv2.putText(canvas, f"FRAME ONLY: {diagnostics.get('reject_reason') or 'PnP passed'} | final validation: see status", (10, 49), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        self.debug_pub.publish(msg)

    def on_pair(self, color, depth):
        started = time.perf_counter()
        now = self.get_clock().now()
        if self.state not in {'FOLLOW_RETURN_PATH', 'RETURN_MANUAL', 'RETURN_RESUME_STOP', 'RETURN_INTERVENTION', 'RETURN_LANDMARK_SEARCH'} or self.origin is None or not self.landmarks or self.info is None:
            return
        adaptive = bool(self.p('enable_adaptive_visual_check'))
        if adaptive and self.state != 'RETURN_LANDMARK_SEARCH' and self.check is None:
            return  # No decode/SIFT/matching/PnP while normal odometry return runs.
        if self.check:
            self.check_timeout()
            if self.check is None:
                return
        if now.nanoseconds*1e-9 - self.last < self.p('processing_period_s'):
            return
        self.last = now.nanoseconds*1e-9
        if self.check:
            self.check_frames += 1
        try:
            stamp = Time.from_msg(color.header.stamp)
            if self.check and stamp.nanoseconds*1e-9 < self.check.get('requested_at', 0.):
                raise ValueError('image predates check request')
            if stamp.nanoseconds <= 0:
                raise ValueError('zero image timestamp')
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
            k, d = camera_model(ci.width, ci.height, ci.k, ci.d, ci.distortion_model, image.shape)
            # Timestamped TF includes current arm joints and chassis roll/pitch.
            oc = tf_matrix(self.buffer.lookup_transform(self.p('odom_frame'), color.header.frame_id, stamp))
            ob = tf_matrix(self.buffer.lookup_transform(self.p('odom_frame'), self.p('base_frame'), stamp))
            bc = np.linalg.inv(ob) @ oc
            if not valid_transform(bc) or not valid_transform(oc) or not valid_transform(ob):
                raise ValueError('invalid current camera/base TF')
            self.check_lock_pose(bc)
            self.event('frame', stamp_ns=stamp.nanoseconds)
            sift_started = time.perf_counter()
            current = features(image)
            sift_time = time.perf_counter()-sift_started
            prior = self.correction if self.correction is not None else np.linalg.inv(self.origin)
            ranked = []
            attempts = []
            if self.state == 'RETURN_LANDMARK_SEARCH' and self.search is not None:
                selected = [(self.search.get('id'), None)] if self.search.get('id') in self.landmarks else []
            else:
                progress = getattr(self, 'route_progress', None)
                if (not adaptive and (len(self.route) < 2 or progress is None)):
                    selected = sorted(((lid, None) for lid in self.landmarks),
                        key=lambda item: np.linalg.norm((prior @ oc)[:2, 3]-self.landmarks[item[0]][1][:2, 3]))[:max(1, min(7, self.p('max_visual_candidates')))]
                elif (len(self.route) < 2 or progress is None
                        or not 0 <= (now.nanoseconds-progress[1])*1e-9 <= .5):
                    raise ValueError('fresh controller route progress required')
                else:
                    if self.cached_arcs is None:
                        self.cached_arcs = reference_arcs(self.route, self.reference_meta)
                    selected = candidate_window(self.cached_arcs, progress[0], self.last_reference_arc,
                    self.p('route_backtrack_m'), self.p('max_visual_candidates'),
                    getattr(self, 'check_expansion', 0))
            self.event('candidates', candidate_ids=[lid for lid, _ in selected], current_sift_s=sift_time)
            for lid, arc in selected:
                reference, mc = self.landmarks[lid]
                if self.state == 'RETURN_LANDMARK_SEARCH' and self.search is not None and lid != self.search.get('id'):
                    continue
                if np.linalg.norm((prior @ oc)[:2, 3] - mc[:2, 3]) > self.p('candidate_radius_m'):
                    continue
                diagnostics = {'id': lid}
                debug = {} if getattr(self, 'debug_enabled', False) and self.debug_pub.get_subscription_count() else None
                result = match(reference, current, z, k, d, options=self.match_options, diagnostics=diagnostics, debug=debug)
                if debug is not None:
                    try:
                        self.publish_matches(lid, image, current, debug, diagnostics, color.header)
                    except Exception as exc:
                        self.get_logger().warning(f'Debug image failed: {exc}')
                attempts.append(diagnostics)
                if result is None:
                    continue
                relative, count, error = result
                # reference<-current from PnP; mission<-base uses current articulated extrinsic.
                mb = mc @ relative @ np.linalg.inv(oc) @ ob
                correction = planar(mb) @ np.linalg.inv(planar(ob))
                delta = correction @ np.linalg.inv(prior)
                displacement = np.linalg.norm((correction @ ob)[:2, 3] - (prior @ ob)[:2, 3])
                yaw = abs(np.arctan2(delta[1, 0], delta[0, 0]))
                diagnostics.update(correction_step_m=float(displacement), correction_step_yaw=float(yaw))
                if displacement <= self.p('max_correction_m') and yaw <= self.p('max_correction_yaw_rad'):
                    ranked.append((count, -error, lid, correction, relative, diagnostics, arc))
                else:
                    diagnostics['reject_reason'] = 'correction_step_limit'
            ranked.sort(key=lambda item: item[:2], reverse=True)
            if not ranked:
                self.pending = None
                self.event('no_match', attempts=attempts, current_features=len(current[0]), current_sift_s=sift_time,
                           processing_latency_s=time.perf_counter()-started)
                return
            best = ranked[0]
            if len(ranked) > 1 and ranked[1][0] >= best[0]*self.p('ambiguity_ratio'):
                raise ValueError('ambiguous landmarks')
            _, neg_error, lid, correction, relative, diagnostics, arc = best
            if self.state == 'RETURN_LANDMARK_SEARCH':
                if self.search is None:
                    return
                self.event('candidate', id=lid, request=self.search.get('request'), stamp_ns=stamp.nanoseconds)
                commit_after = self.search.get('commit_after_ns')
                if commit_after is None or stamp.nanoseconds < commit_after:
                    self.pending = None
                    return
            # Require two independent frames to agree before changing controller coordinates.
            if self.pending is None or self.pending[0] != lid or self.pending_stamp is None or not 0 < (stamp.nanoseconds-self.pending_stamp)*1e-9 <= self.p('agreement_max_interval_s'):
                self.pending = (lid, correction)
                self.pending_stamp = stamp.nanoseconds
                return
            previous = self.pending[1]
            delta = correction @ np.linalg.inv(previous)
            self.pending = (lid, correction)
            self.pending_stamp = stamp.nanoseconds
            if np.linalg.norm((correction @ ob)[:2, 3] - (previous @ ob)[:2, 3]) > self.p('agreement_translation_m') or abs(np.arctan2(delta[1, 0], delta[0, 0])) > self.p('agreement_yaw_rad'):
                return
            # Processing can be slow; never publish an old-frame result into live control.
            if (self.get_clock().now()-stamp).nanoseconds*1e-9 > 1.5:
                raise ValueError('localization processing too slow')
            if self.check and self.get_clock().now().nanoseconds*1e-9 >= self.check['deadline']:
                raise ValueError('check deadline exceeded before commit')
            meaningful = (diagnostics['correction_step_m'] >= self.p('correction_min_translation_m')
                          or diagnostics['correction_step_yaw'] >= self.p('correction_min_yaw_rad'))
            apply = bool(self.p('apply_correction')) and (meaningful or self.state == 'RETURN_LANDMARK_SEARCH')
            self.last_reference_arc = arc  # A validated anchor counts even in deadband/observation mode.
            if apply:
                self.correction = correction
            msg = PoseStamped()
            msg.header.stamp = color.header.stamp
            msg.header.frame_id = 'mission'
            msg.pose.position.x, msg.pose.position.y = map(float, correction[:2, 3])
            yaw = np.arctan2(correction[1, 0], correction[0, 0])
            msg.pose.orientation.z, msg.pose.orientation.w = float(np.sin(yaw/2)), float(np.cos(yaw/2))
            if apply:
                self.pub.publish(msg)
            self.event('localized' if apply else 'validated', attempts=attempts, id=lid, inliers=best[0], reprojection_rmse=-neg_error,
                       diagnostics=diagnostics, relative_pose=pose_fields(relative),
                       planar_correction=pose_fields(correction), correction_applied=apply, correction_deadband=not meaningful, current_sift_s=sift_time, candidate_count=len(attempts),
                       current_camera_tf=bc.tolist(), reference_camera_tf=self.landmarks[lid][1].tolist(),
                       reference_model=self.reference_meta[lid]['camera_info']['distortion_model'], current_model=ci.distortion_model,
                       reference_resolution=[self.reference_meta[lid]['camera_info']['width'], self.reference_meta[lid]['camera_info']['height']],
                       current_resolution=[ci.width, ci.height], processing_latency_s=time.perf_counter()-started,
                       stamp_ns=stamp.nanoseconds, request=self.search.get('request') if self.state == 'RETURN_LANDMARK_SEARCH' and self.search is not None else None)
        except Exception as exc:
            self.pending = None
            self.event('rejected', reason=str(exc), attempts=locals().get('attempts', []),
                       processing_latency_s=time.perf_counter()-started)


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
