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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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


def quality_lines(data):
    """Actual production pipeline metrics; never substitute live preview counts."""
    attempts = data.get('attempts', [])
    selected = data.get('diagnostics') or next((a for a in attempts if a.get('id') == data.get('id')), None)
    if selected is None:
        selected = max(attempts, key=lambda a: (a.get('pnp_inliers', a.get('inliers', 0)),
                                               a.get('geometric_inliers', 0)), default={})
    def number(value, scale=1., digits=1):
        return '--' if value is None else f'{value*scale:.{digits}f}'
    def field(name):
        return selected.get(name, data.get(name))
    def count(name):
        value = field(name)
        return '--' if value is None else value
    status = data.get('event', 'waiting for VPR check')
    reason = selected.get('reject_reason') or data.get('reason') or '--'
    return [f'VPR: {status} | candidate: {selected.get("id", "--")}',
        f'reject: {reason}',
        f'correction applied: {data.get("correction_applied", False)}',
        'PRODUCTION metrics (not cyan preview)',
        f'features raw/grid: {count("feature_count_raw")} / {count("feature_count_grid_filtered")}',
        f'matches raw/mutual: {count("raw_matches")} / {count("mutual_ratio_matches")}',
        f'geometry/depth: {count("geometric_inliers")} / {count("depth_valid_matches")}',
        f'PnP points/inliers: {count("pnp_points")} / {count("pnp_inliers")}',
        f'inlier ratio: {number(field("inlier_ratio"), 100)}% | cells: {count("inlier_grid_cells")}',
        f'RMSE: {number(field("reprojection_rmse"), digits=2)}px',
        f'prior translation: {number(field("pose_prior_translation_error"), digits=3)}m',
        f'prior yaw: {number(field("pose_prior_yaw_error"), 180/np.pi)}deg',
        f'prior roll/pitch: {number(field("pose_prior_roll_error"), 180/np.pi)} / {number(field("pose_prior_pitch_error"), 180/np.pi)}deg',
        f'SIFT/match-depth: {number(data.get("current_sift_s"),1000)} / {number(field("matching_and_depth_s"),1000)}ms',
        f'PnP/total: {number(field("pnp_s"),1000)} / {number(data.get("processing_latency_s"),1000)}ms',
        f'pair skew/wait: {number(data.get("rgbd_stamp_skew_s"),1000)} / {number(data.get("pair_queue_wait_s"),1000)}ms',
        f'overwritten pairs: {data.get("pairs_overwritten", "--")}']


class FeaturePreview:
    """Bounded display-resolution SIFT; no descriptors, matching, depth or PnP."""
    def __init__(self, max_width=480, max_features=120):
        self.max_width = max_width
        self.max_features = max_features
        self.sift = cv2.SIFT_create(nfeatures=max_features)

    def render(self, frame, enabled=True):
        scale = min(1., self.max_width/frame.shape[1])
        view = cv2.resize(frame, None, fx=scale, fy=scale)
        points = self.sift.detect(cv2.cvtColor(view, cv2.COLOR_BGR2GRAY), None) if enabled else []
        points = sorted(points, key=lambda point: point.response, reverse=True)[:self.max_features]
        view = cv2.drawKeypoints(view, points, None, color=(255, 255, 0),
                                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return view, len(points)


class Viewer(Node):
    def __init__(self):
        super().__init__('vpr_viewer')
        self.quality_event = {}
        self.preview_pause_until = 0.
        self.declare_parameter('pause_preview_during_check', True)
        self.frames = {}
        self.pending = {}
        self.declare_parameter('live_features', False)
        self.declare_parameter('preview_rate_hz', 2.0)
        self.declare_parameter('preview_max_width', 480)
        self.declare_parameter('preview_max_features', 120)
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
        self.preview_modes = {}
        self.image_stamps = {}
        self.receive_counts = {}
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lines = {}
        self.mission_state = 'IDLE'
        self.create_subscription(CompressedImage, '/visual_landmark/debug/reference/compressed',
            lambda msg: self.receive('Stored rear reference', msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for key, topic in {
            'Rear live': '/rear/camera/image_raw/compressed',
            'Front live': '/arm/camera/color/image_raw/compressed',
            'Matches': '/visual_landmark/debug/matches/compressed',
        }.items():
            self.create_subscription(CompressedImage, topic,
                lambda msg, key=key: self.receive(key, msg),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == 'Matches' else image_qos)
        for key, topic in {
            'mission': '/mission/return/state',
            'localization': '/visual_landmark/localization_event',
            'scheduler': '/visual_landmark/scheduler_event',
            'capture': '/visual_landmark/event',
        }.items():
            self.create_subscription(String, topic,
                lambda msg, key=key: self.status(key, msg), 10)

    def receive(self, key, msg):
        if key in {'Stored rear reference', 'Matches'} and not msg.data:
            self.frames.pop(key, None)
            self.pending.pop(key, None)
            self.frames.pop('Matches', None)
            self.pending.pop('Matches', None)
            return
        # Decode at display time, replacing queued frames with the latest arrival.
        self.pending[key] = msg, time.monotonic()
        self.receive_counts[key] = self.receive_counts.get(key, 0) + 1

    def status(self, key, msg):
        if key == 'mission':
            if msg.data in {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'} and self.mission_state not in {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}:
                for name in ('Stored rear reference', 'Matches'):
                    self.frames.pop(name, None)
                    self.pending.pop(name, None)
            self.mission_state = msg.data
            self.lines[key] = msg.data
            return
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('status is not an object')
            if key == 'localization':
                if data.get('event') == 'check_started':
                    self.quality_event = data.copy()
                    self.preview_pause_until = time.monotonic()+float(data.get('timeout_s', 5.))+1.
                elif data.get('event') == 'candidates':
                    self.quality_event = data.copy()
                elif data.get('attempts') is not None:
                    self.quality_event.update(data)
                if data.get('check_complete'):
                    self.preview_pause_until = 0.
            summary = {k: data[k] for k in ('event', 'id', 'trigger', 'reason', 'inliers',
                'correction_applied', 'correction_step_m', 'correction_step_yaw', 'planar_correction') if k in data}
            if data.get('trigger') == 'stationary_debug':
                metrics = ('reference_features', 'current_features', 'mutual_ratio_matches',
                           'pnp_points', 'inliers', 'inlier_ratio', 'reprojection_rmse', 'reject_reason')
                attempts = data.get('attempts', [])
                if attempts:
                    self.lines['debug metrics'] = json.dumps([{k: a.get(k, 'not_run') for k in ('id',)+metrics}
                                                              for a in attempts], ensure_ascii=True)
            self.lines[key] = (str(data.get('reason') or data.get('event', '--'))
                               if key == 'capture' else json.dumps(summary, ensure_ascii=True))
            if key == 'localization' and data.get('event') in {'validated', 'localized', 'no_match', 'rejected'}:
                self.lines['last result'] = self.lines[key]
        except (ValueError, TypeError):
            self.lines[key] = msg.data

    def show(self):
        now = time.monotonic()
        for key, (msg, arrival) in list(self.pending.items()):
            if key not in {'Matches', 'Stored rear reference'} and now-self.last_decode.get(key, -float('inf')) < self.preview_period:
                continue
            del self.pending[key]
            self.last_decode[key] = now
            try:
                frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('image decode failed')
                if key not in {'Matches', 'Stored rear reference'}:
                    busy = (self.get_parameter('pause_preview_during_check').value
                            and now < self.preview_pause_until)
                    enabled = bool(self.get_parameter('live_features').value) and not busy
                    if enabled:
                        frame, count = self.preview.render(frame, True)
                    else:
                        count = 0
                    self.feature_counts[key] = count
                    self.preview_modes[key] = 'PAUSED for VPR' if busy else ('ON' if enabled else 'OFF')
                self.frames[key] = frame, arrival
                self.image_stamps[key] = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                self.lines.pop(key+' error', None)
            except (cv2.error, ValueError) as error:
                self.lines[key+' error'] = str(error)
        panel = np.full((864, 1280, 3), (24, 24, 24), np.uint8)
        cv2.putText(panel, 'VPR Dashboard | cyan: preview ONLY | green: PnP inlier | amber: mutual match | F: preview | Q: close',
                    (15, 26), cv2.FONT_HERSHEY_SIMPLEX, .53, (230,230,230), 1)
        returning = getattr(self, 'mission_state', 'IDLE') not in {'IDLE', 'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}
        left = 'Stored rear reference' if returning else 'Rear live'
        slots = {left: (0, 44, 640, 356), 'Front live': (640, 44, 640, 356),
                 'Matches': (0, 420, 800, 420)}
        lines = quality_lines(getattr(self, 'quality_event', {}))
        ros_now = self.get_clock().now().nanoseconds*1e-9
        for key in (left, 'Front live', 'Matches'):
            if key in self.frames:
                frame, arrival = self.frames[key]
                scale = min(1., 1200/frame.shape[1], 650/frame.shape[0])
                view = cv2.resize(frame, None, fx=scale, fy=scale)
                age = now-arrival
                stamp = self.image_stamps.get(key, 0.)
                source_age = f'{ros_now-stamp:.2f}s' if stamp > 0 else 'unknown'
                count = self.feature_counts.get(key)
                mode = getattr(self, 'preview_modes', {}).get(key, 'ON')
                title = (f'{key}: SIFT={count} preview only' if mode == 'ON' else f'{key}: preview {mode}') if count is not None else key
                detail = f'RX age {age:.1f}s | source age {source_age} | RX #{self.receive_counts.get(key, 0)}'
                stored = key in {'Stored rear reference', 'Matches'}
                if stored:
                    detail = f'SNAPSHOT result | last check {age:.1f}s ago | held until next check'
                if age > 2. and not stored:
                    detail = 'STALE / NO NEW DISPLAY FRAME | '+detail
                cv2.rectangle(view, (0, 0), (view.shape[1], 48), (0, 0, 0), -1)
                cv2.putText(view, title, (8,18), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,0), 1)
                cv2.putText(view, detail, (8,38), cv2.FONT_HERSHEY_SIMPLEX, .4,
                            (0,0,255) if age>2 and not stored else (255,255,255), 1)
                # Camera receipt/age lives in the image header, leaving room for pipeline metrics.
            else:
                view = np.zeros((360, 640, 3), np.uint8)
                message = ('WAITING FOR VPR CHECK / SELECTED REFERENCE' if key == 'Stored rear reference' else key+': WAITING FOR IMAGE TOPIC')
                cv2.putText(view, message, (15,165), cv2.FONT_HERSHEY_SIMPLEX, .55, (0,200,255), 1)
                if key == 'Matches':
                    state = getattr(self, 'mission_state', 'IDLE')
                    hints = ['STILL IMAGE comparison: stored rear / current front',
                             'Record: collect reference snapshots (distance 1.25m).',
                             'Return: comparison appears only during a VPR check.',
                             'Manual: /visual_landmark/debug_check after stopping.',
                             'Check localization status for missing inputs / rejection.',
                             'Publisher: visual_debug_matches:=true']
                    for row, hint in enumerate(hints):
                        cv2.putText(view, hint, (12, 195+row*24),
                                    cv2.FONT_HERSHEY_SIMPLEX, .43, (210,210,210), 1)
                lines.append(message)
            x, y, width, height = slots[key]
            panel[y:y+height, x:x+width] = fit_panel(view, width, height)
            cv2.rectangle(panel, (x,y), (x+width-1,y+height-1), (80,80,80), 1)
        lines = ['mission: '+self.lines.get('mission', 'waiting'),
                 'capture: '+self.lines.get('capture', 'waiting')[:45]] + lines
        cv2.putText(panel, 'VPR status / diagnostics', (815, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,0), 1)
        y = 466
        for line in lines:
            for part in textwrap.wrap(line, width=60, break_long_words=True):
                if y > 828:
                    break
                cv2.putText(panel, part, (815,y), cv2.FONT_HERSHEY_SIMPLEX, .37,
                            (0,100,255) if part.startswith('reject:') and not part.endswith('--') else (230,230,230), 1)
                y += 16
            if y > 828:
                break
        cv2.putText(panel, 'Live preview points are not VPR validation. Matches update only during a check.',
                    (15, 858), cv2.FONT_HERSHEY_SIMPLEX, .46, (180,180,180), 1)
        cv2.imshow(WINDOW_NAME, panel)
        key = cv2.waitKey(1)&255
        if key == ord('f'):
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('live_features', value=not self.get_parameter('live_features').value)])
        return key != ord('q')


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
