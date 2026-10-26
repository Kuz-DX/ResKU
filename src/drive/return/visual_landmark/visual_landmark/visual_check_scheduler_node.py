"""Optional watchdog. Never commands motion or changes mission state."""
import json
import math
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64
from diagnostic_msgs.msg import DiagnosticArray
from visual_landmark.adaptive import CheckWatchdog, RETURN, fresh


class VisualCheckScheduler(Node):
    def __init__(self):
        super().__init__('visual_check_scheduler_node')
        defaults = dict(enable_adaptive_visual_check=True, visual_check_distance_m=8.,
                        visual_check_retry_cooldown_s=5., visual_check_timeout_s=5.,
                        enable_slip_early_trigger=True, slip_wz_deadband_radps=.15,
                        slip_score_trigger=.35, slip_score_decay=.02,
                        wheel_timeout_s=.5, gyro_timeout_s=.5, diagnostics_timeout_s=2.,
                        odom_timeout_s=.5, visual_check_on_return_start=False)
        for n, v in defaults.items():
            self.declare_parameter(n, v)
        self.p = lambda n: self.get_parameter(n).value
        for n, v in defaults.items():
            if isinstance(v, float) and (not math.isfinite(self.p(n)) or self.p(n) < 0):
                raise ValueError('invalid scheduler parameter '+n)
        if self.p('visual_check_timeout_s') <= 0 or self.p('visual_check_distance_m') <= 0 or self.p('slip_score_trigger') <= 0:
            raise ValueError('watchdog distance, timeout and slip threshold must be positive')
        self.watch = CheckWatchdog(self.p('visual_check_distance_m'), self.p('visual_check_retry_cooldown_s'),
                                   self.p('visual_check_timeout_s'), self.p('slip_wz_deadband_radps'),
                                   self.p('slip_score_trigger'), self.p('slip_score_decay'))
        self.state, self.origin = '', None
        self.wheel = self.gyro = 0.
        self.wheel_time = self.gyro_time = self.diag_time = self.odom_time = None
        self.health = False
        self.start_check = False
        self.pub = self.create_publisher(String, '/visual_landmark/check_request', 10)
        self.status = self.create_publisher(String, '/visual_landmark/scheduler_event', 10)
        self.create_subscription(String, '/mission/return/state', self.on_state, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Pose, '/mission/origin_pose', self.on_origin, latched)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel, qos_profile_sensor_data)
        self.create_subscription(Float64, '/imu/gyro/z_corrected', self.on_gyro, 10)
        self.create_subscription(DiagnosticArray, '/odometry/diagnostics', self.on_diag, 10)
        self.create_subscription(String, '/visual_landmark/localization_event', self.on_result, 10)
        self.create_timer(.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def event(self, kind, **fields):
        self.status.publish(String(data=json.dumps(dict(event=kind, distance_since_last_valid_visual=self.watch.distance,
                                                       slip_score=self.watch.score, **fields))))

    def cancel(self):
        if self.watch.pending:
            token = self.watch.pending[0]
            self.pub.publish(String(data=json.dumps(dict(active=False, request=token))))
            self.watch.finish(token, self.now(), False)

    def on_origin(self, msg):
        key = (msg.position.x, msg.position.y, msg.orientation.z, msg.orientation.w)
        if key != self.origin:
            self.cancel()
            self.watch.reset()
            self.origin = key

    def on_state(self, msg):
        if msg.data in RETURN and self.state not in RETURN:
            self.watch.reset()
            self.start_check = bool(self.p('visual_check_on_return_start'))
        if msg.data != 'FOLLOW_RETURN_PATH':
            self.cancel()
        if msg.data not in RETURN:
            self.watch.reset()
        self.state = msg.data

    def on_odom(self, msg):
        now = self.now()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not fresh(now, stamp, self.p('odom_timeout_s')):
            return
        self.odom_time = stamp
        if self.state in RETURN:
            self.watch.odom(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_wheel(self, msg):
        self.wheel = msg.twist.twist.angular.z
        self.wheel_time = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_gyro(self, msg):
        # Float64 has no source timestamp; arrival freshness is the available contract.
        self.gyro, self.gyro_time = msg.data, self.now()

    def on_diag(self, msg):
        self.health = False
        self.diag_time = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        for status in msg.status:
            if status.name == 'reduced_odom/estimator':
                values = {v.key: v.value for v in status.values}
                try:
                    self.health = (float(values.get('gyro_calibrated', 0)) == 1 and
                                   float(values.get('wz_source_gyro', 0)) == 1 and
                                   float(values.get('imu_stale', 1)) == 0)
                except ValueError:
                    pass

    def on_result(self, msg):
        try:
            data = json.loads(msg.data)
            if not data.get('check_complete'):
                return
            success = data.get('event') in {'validated', 'localized'}
            latency = self.now()-self.watch.pending[1] if self.watch.pending else None
            if self.watch.finish(data.get('check_request'), self.now(), success):
                self.event('check_success' if success else 'check_failed', request=data.get('check_request'), check_latency_s=latency)
        except (ValueError, TypeError):
            return

    def tick(self):
        now = self.now()
        if not self.p('enable_adaptive_visual_check'):
            return
        expired = self.watch.expire(now)
        if expired:
            self.pub.publish(String(data=json.dumps(dict(active=False, request=expired))))
            self.event('check_timeout', request=expired)
        healthy = (self.health and self.state == 'FOLLOW_RETURN_PATH' and
                   fresh(now, self.diag_time, self.p('diagnostics_timeout_s')) and
                   fresh(now, self.wheel_time, self.p('wheel_timeout_s')) and
                   fresh(now, self.gyro_time, self.p('gyro_timeout_s')))
        self.watch.risk(now, self.wheel, self.gyro, healthy and self.p('enable_slip_early_trigger'))
        if self.state != 'FOLLOW_RETURN_PATH' or not fresh(now, self.odom_time, self.p('odom_timeout_s')):
            return
        trigger = self.watch.due(now, self.p('enable_slip_early_trigger'))
        if self.start_check and self.watch.pending is None and now >= self.watch.next_try:
            trigger = 'manual/debug'
        if trigger:
            token = uuid.uuid4().hex
            self.watch.start(token, now)
            self.start_check = False
            data = dict(active=True, request=token, trigger=trigger, requested_at=now,
                        timeout_s=self.p('visual_check_timeout_s'),
                        distance_since_last_valid_visual=self.watch.distance, slip_score=self.watch.score,
                        wheel_wz=self.wheel if math.isfinite(self.wheel) else None,
                        gyro_wz=self.gyro if math.isfinite(self.gyro) else None)
            self.pub.publish(String(data=json.dumps(data)))
            self.event('check_requested', **{k:v for k,v in data.items() if k not in {'active', 'distance_since_last_valid_visual', 'slip_score'}})


def main(args=None):
    rclpy.init(args=args)
    node = VisualCheckScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
