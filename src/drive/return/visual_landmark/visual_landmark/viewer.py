"""Read-only camera/match/status viewer. Does not extract features or command motion."""
import json
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class Viewer(Node):
    def __init__(self):
        super().__init__('vpr_viewer')
        self.frames = {}
        self.pending = {}
        self.displayed = {}
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
        for key, (msg, arrival) in list(self.pending.items()):
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                self.frames[key] = frame, arrival
        self.pending.clear()
        now = time.monotonic()
        panel = np.zeros((650, 1200, 3), np.uint8)
        lines = ['VPR trial | Q closes viewer only | green=PnP inlier, amber=descriptor match',
                 'Match image is one candidate/frame; only validated/localized means two-frame validation.']
        for key in ('Rear live', 'Front live', 'Matches'):
            if key in self.frames:
                frame, arrival = self.frames[key]
                if self.displayed.get(key) == arrival:
                    continue
                self.displayed[key] = arrival
                scale = min(1., 1200/frame.shape[1], 650/frame.shape[0])
                view = cv2.resize(frame, None, fx=scale, fy=scale)
                cv2.putText(view, f'{key}: {now-arrival:.1f}s since received at render', (10, view.shape[0]-12),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255) if now-arrival>2 else (255,255,0), 1)
                cv2.imshow(key, view)
            else:
                lines.append(key+': waiting for images')
        for key, value in self.lines.items():
            lines.append(key+': '+value)
        y = 25
        for line in lines:
            for offset in range(0, len(line), 135):
                cv2.putText(panel, line[offset:offset+135], (10,y), cv2.FONT_HERSHEY_SIMPLEX, .48, (230,230,230), 1)
                y += 22
        cv2.imshow('VPR status', panel)
        return cv2.waitKey(1)&255 != ord('q')


def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
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
