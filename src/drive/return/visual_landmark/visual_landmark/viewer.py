"""Live SIFT preview and production match viewer. Never commands motion.

Preview features are independent diagnostics, not validated VPR matches.
"""
import json
import time
import textwrap
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


WINDOW_NAME = 'VPR Dashboard'


def fit_panel(frame, width, height):
    """Letterbox without stretching or cropping camera geometry."""
    scale = min(width/frame.shape[1], height/frame.shape[0])
    resized = cv2.resize(frame, (max(1, round(frame.shape[1]*scale)),
                                 max(1, round(frame.shape[0]*scale))))
    panel = np.full((height, width, 3), 20, np.uint8)
    x, y = (width-resized.shape[1])//2, (height-resized.shape[0])//2
    panel[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
    return panel


class FeaturePreview:
    """Bounded display-resolution SIFT; no descriptors, matching, depth or PnP."""
    def __init__(self, max_width=640, max_features=400):
        self.max_width = max_width
        self.sift = cv2.SIFT_create(nfeatures=max_features)

    def render(self, frame, enabled=True):
        scale = min(1., self.max_width/frame.shape[1])
        view = cv2.resize(frame, None, fx=scale, fy=scale)
        points = self.sift.detect(cv2.cvtColor(view, cv2.COLOR_BGR2GRAY), None) if enabled else []
        view = cv2.drawKeypoints(view, points, None, color=(255, 255, 0),
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return view, len(points)


class Viewer(Node):
    def __init__(self):
        super().__init__('vpr_viewer')
        self.frames = {}
        self.pending = {}
        self.declare_parameter('live_features', True)
        self.declare_parameter('preview_rate_hz', 5.0)
        self.declare_parameter('preview_max_width', 640)
        self.declare_parameter('preview_max_features', 400)
        rate = float(self.get_parameter('preview_rate_hz').value)
        if not np.isfinite(rate) or not 0 < rate <= 30:
            raise ValueError('preview_rate_hz must be in (0, 30]')
        width = int(self.get_parameter('preview_max_width').value)
        features = int(self.get_parameter('preview_max_features').value)
        if not 64 <= width <= 1280 or not 1 <= features <= 2000:
            raise ValueError('preview_max_width must be 64..1280; preview_max_features 1..2000')
        self.preview = FeaturePreview(width, features)
        self.preview_period = 1./rate
        self.last_decode = {}
        self.feature_counts = {}
        self.image_stamps = {}
        self.receive_counts = {}
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lines = {}
        for key, topic in {
            'Rear live': '/rear/camera/image_raw/compressed',
            'Front live': '/arm/camera/color/image_raw/compressed',
            'Matches': '/visual_landmark/debug/matches/compressed',
        }.items():
            self.create_subscription(CompressedImage, topic,
                lambda msg, key=key: self.receive(key, msg), image_qos)
        for key, topic in {
            'mission': '/mission/return/state',
            'localization': '/visual_landmark/localization_event',
            'scheduler': '/visual_landmark/scheduler_event',
            'capture': '/visual_landmark/event',
        }.items():
            self.create_subscription(String, topic,
                lambda msg, key=key: self.status(key, msg), 10)

    def receive(self, key, msg):
        # Decode at display time, replacing queued frames with the latest arrival.
        self.pending[key] = msg, time.monotonic()
        self.receive_counts[key] = self.receive_counts.get(key, 0) + 1

    def status(self, key, msg):
        try:
            data = json.loads(msg.data)
            summary = {k: data[k] for k in ('event', 'id', 'trigger', 'reason', 'inliers',
                'correction_applied', 'correction_step_m', 'correction_step_yaw', 'planar_correction') if k in data}
            if data.get('trigger') == 'stationary_debug':
                metrics = ('reference_features', 'current_features', 'mutual_ratio_matches',
                           'pnp_points', 'inliers', 'inlier_ratio', 'reprojection_rmse', 'reject_reason')
                attempts = data.get('attempts', [])
                if attempts:
                    self.lines['debug metrics'] = json.dumps([{k: a.get(k, 'not_run') for k in ('id',)+metrics}
                                                              for a in attempts], ensure_ascii=True)
            self.lines[key] = json.dumps(summary, ensure_ascii=True)
            if key == 'localization' and data.get('event') in {'validated', 'localized', 'no_match', 'rejected'}:
                self.lines['last result'] = self.lines[key]
        except (ValueError, TypeError):
            self.lines[key] = msg.data

    def show(self):
        now = time.monotonic()
        for key, (msg, arrival) in list(self.pending.items()):
            if key != 'Matches' and now-self.last_decode.get(key, -float('inf')) < self.preview_period:
                continue
            del self.pending[key]
            self.last_decode[key] = now
            try:
                frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('image decode failed')
                if key != 'Matches':
                    frame, count = self.preview.render(frame, bool(self.get_parameter('live_features').value))
                    self.feature_counts[key] = count
                self.frames[key] = frame, arrival
                self.image_stamps[key] = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                self.lines.pop(key+' error', None)
            except (cv2.error, ValueError) as error:
                self.lines[key+' error'] = str(error)
        panel = np.full((864, 1280, 3), (24, 24, 24), np.uint8)
        cv2.putText(panel, 'VPR Dashboard | cyan: live SIFT | green/amber: PnP inliers/descriptor matches | Q: close',
                    (15, 26), cv2.FONT_HERSHEY_SIMPLEX, .53, (230,230,230), 1)
        slots = {'Rear live': (0, 44, 640, 356), 'Front live': (640, 44, 640, 356),
                 'Matches': (0, 420, 800, 420)}
        lines = ['LIVE cyan=SIFT preview ONLY (not validated matches); Q closes viewer only',
                 'Matches: green=PnP inlier, amber=descriptor match; requires a VPR check.',
                 'Preview runs in viewer only; mission/odometry/corrections are unchanged.']
        ros_now = self.get_clock().now().nanoseconds*1e-9
        for key in ('Rear live', 'Front live', 'Matches'):
            if key in self.frames:
                frame, arrival = self.frames[key]
                scale = min(1., 1200/frame.shape[1], 650/frame.shape[0])
                view = cv2.resize(frame, None, fx=scale, fy=scale)
                age = now-arrival
                stamp = self.image_stamps.get(key, 0.)
                source_age = f'{ros_now-stamp:.2f}s' if stamp > 0 else 'unknown'
                count = self.feature_counts.get(key)
                title = f'{key}: SIFT={count} preview only' if count is not None else key
                detail = f'RX age {age:.1f}s | source age {source_age} | RX #{self.receive_counts.get(key, 0)}'
                if age > 2.:
                    detail = 'STALE / NO NEW DISPLAY FRAME | '+detail
                cv2.rectangle(view, (0, 0), (view.shape[1], 48), (0, 0, 0), -1)
                cv2.putText(view, title, (8,18), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,0), 1)
                cv2.putText(view, detail, (8,38), cv2.FONT_HERSHEY_SIMPLEX, .4,
                            (0,0,255) if age>2 else (255,255,255), 1)
                lines.append(title+' | '+detail)
            else:
                view = np.zeros((360, 640, 3), np.uint8)
                message = key+': WAITING FOR IMAGE TOPIC'
                cv2.putText(view, message, (15,165), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,200,255), 1)
                lines.append(message)
            x, y, width, height = slots[key]
            panel[y:y+height, x:x+width] = fit_panel(view, width, height)
            cv2.rectangle(panel, (x,y), (x+width-1,y+height-1), (80,80,80), 1)
        for key, value in self.lines.items():
            lines.append(key+': '+value)
        cv2.putText(panel, 'VPR status / diagnostics', (815, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,0), 1)
        y = 466
        for line in lines:
            for part in textwrap.wrap(line, width=57, break_long_words=True):
                if y > 828:
                    break
                cv2.putText(panel, part, (815,y), cv2.FONT_HERSHEY_SIMPLEX, .4, (230,230,230), 1)
                y += 18
            if y > 828:
                break
        cv2.putText(panel, 'Live preview points are not VPR validation. Matches update only during a check.',
                    (15, 858), cv2.FONT_HERSHEY_SIMPLEX, .46, (180,180,180), 1)
        cv2.imshow(WINDOW_NAME, panel)
        return cv2.waitKey(1)&255 != ord('q')


def main(args=None):
    rclpy.init(args=args)
    cv2.setNumThreads(1)
    node = Viewer()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 864)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            for _ in range(12):
                rclpy.spin_once(node, timeout_sec=0.)
            if not node.show():
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
