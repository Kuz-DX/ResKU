"""
return_path_follower_node

Fixed-frame pure-pursuit follower for the RETURN mission. Unlike
dolbotz/purepursuit.py (which expects a continuously-republished,
robot-relative /path and transforms it into base_link at receipt time --
architecturally wrong for a one-shot fixed path, since it would go stale
the instant the robot moves), this node stores /return_path exactly once,
in the fixed "mission" frame, and every control cycle re-derives the
robot-relative lookahead point from the CURRENT pose (odom -> mission via
the stored T0) instead of re-transforming the path itself. This is a new,
lightweight node (not a patch to dolbotz's node) since that module also
outputs the wrong message type/topic and carries escort-specific features
(bearing steering, distance-scaled speed, blind-turn recovery) not needed
here.

[Corner-aware Pure Pursuit + Rotate-To-Heading] Plain pure pursuit cuts
sharp corners: its lookahead point lands past the vertex and the robot
arcs (skid-steers) across it. So every vertex where the path heading
changes by >= corner_detection_angle_deg is treated as a TEMPORARY GOAL:

    TRACK_PATH      -- pure pursuit, but the lookahead target is never
                       allowed past the next corner vertex, and the robot
                       slows down toward it. The window of path indices
                       considered is [last handled corner, next corner], so
                       the corner can't be skipped and a handled corner is
                       never revisited.
    ROTATE_TO_PATH  -- entered once the robot REACHES the vertex
                       (within corner_reach_distance_m, or has passed it
                       along the incoming direction): linear.x = 0,
                       closed-loop rotate in place to the outgoing segment
                       heading using odometry yaw (no timers), then back to
                       TRACK_PATH toward the next corner / the goal.

A separate reactive check (heading vs. the current segment > 42 deg) also
enters ROTATE_TO_PATH, e.g. if the path starts facing the wrong way.

Subscribes:
    /return_path         (nav_msgs/Path, TRANSIENT_LOCAL, mission frame) -- once
    /odometry/filtered    (nav_msgs/Odometry, odom frame)
    /mission/origin_pose  (geometry_msgs/Pose, TRANSIENT_LOCAL) -- T0

Publishes:
    /cmd_vel_return_path  (geometry_msgs/Twist)

No goal-stop logic here (return_state_machine_node owns FOLLOW_RETURN_PATH
-> FINISHED via its own goal_tolerance_m check on the same /return_path)
-- this node just keeps tracking whatever's left of the path, decelerating
smoothly as it nears the final point.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, Path

from return_navigation.se2 import Pose2D, normalize_angle

TRACK_PATH = 'TRACK_PATH'
ROTATE_TO_PATH = 'ROTATE_TO_PATH'


class ReturnPathFollowerNode(Node):
    def __init__(self):
        super().__init__('return_path_follower_node')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('lookahead_distance_m', 0.6)
        self.declare_parameter('linear_speed_mps', 0.3)
        self.declare_parameter('min_linear_speed_mps', 0.05)
        self.declare_parameter('max_angular_speed_radps', 0.6)
        self.declare_parameter('goal_slowdown_radius_m', 0.8)
        self.declare_parameter('linear_accel_mps2', 0.5)
        self.declare_parameter('angular_accel_radps2', 1.5)

        # [Corner-aware + Rotate-To-Heading] TRACK_PATH<->ROTATE_TO_PATH
        # hysteresis and rotate-in-place control. Enter/exit thresholds are
        # deliberately different (42 deg enter, 9 deg exit) so a heading
        # error hovering near one boundary doesn't chatter between modes.
        self.declare_parameter('rotate_enter_angle_deg', 42.0)
        self.declare_parameter('rotate_exit_angle_deg', 9.0)
        # Real robot only turns ~30-40% of the commanded in-place rate
        # (skid slip), hence the higher gain/limits than sim needs.
        self.declare_parameter('rotate_kp', 2.0)
        self.declare_parameter('rotate_max_angular_speed_radps', 1.2)
        # floor so the commanded angular speed doesn't decay into the
        # motor's dead-band while still actively correcting a small error
        self.declare_parameter('rotate_min_angular_speed_radps', 0.15)
        self.declare_parameter('rotate_settle_time_s', 0.3)
        # A path vertex whose heading change is >= corner_detection_angle_deg
        # is a temporary goal: drive to it, stop, rotate, continue.
        self.declare_parameter('corner_detection_angle_deg', 45.0)
        self.declare_parameter('corner_reach_distance_m', 0.08)
        self.declare_parameter('corner_slowdown_radius_m', 0.4)
        self.declare_parameter('odom_timeout_s', 0.5)

        p = self.get_parameter
        self.control_period = 1.0 / float(p('control_rate_hz').value)
        self.lookahead_distance_m = float(p('lookahead_distance_m').value)
        self.linear_speed_mps = float(p('linear_speed_mps').value)
        self.min_linear_speed_mps = float(p('min_linear_speed_mps').value)
        self.max_angular_speed_radps = float(p('max_angular_speed_radps').value)
        self.goal_slowdown_radius_m = float(p('goal_slowdown_radius_m').value)
        self.linear_accel = float(p('linear_accel_mps2').value)
        self.angular_accel = float(p('angular_accel_radps2').value)

        self.rotate_enter_angle = math.radians(float(p('rotate_enter_angle_deg').value))
        self.rotate_exit_angle = math.radians(float(p('rotate_exit_angle_deg').value))
        self.rotate_kp = float(p('rotate_kp').value)
        self.rotate_max_angular_speed = float(p('rotate_max_angular_speed_radps').value)
        self.rotate_min_angular_speed = float(p('rotate_min_angular_speed_radps').value)
        self.rotate_settle_time_s = float(p('rotate_settle_time_s').value)
        self.corner_detection_angle = math.radians(float(p('corner_detection_angle_deg').value))
        self.corner_reach_distance_m = float(p('corner_reach_distance_m').value)
        self.corner_slowdown_radius_m = float(p('corner_slowdown_radius_m').value)
        self.odom_timeout_s = float(p('odom_timeout_s').value)

        self._path = []       # list[Pose2D], mission frame, fixed once received
        self._corner_angle = []  # list[float], parallel to _path; turn AT each vertex
        self._origin = None   # Pose2D, odom frame (T0)
        self._odom_pose = None  # Pose2D, odom frame
        self._last_odom_time = None

        self._corners = []    # indices of sharp-corner vertices, ascending
        self._corner_ptr = 0  # next unhandled entry of _corners
        self._floor = 0       # path index of the last handled corner

        self._mode = TRACK_PATH
        self._rotate_target_yaw = None
        self._rotate_settle_since = None
        self._rotating_corner = None  # corner idx being rotated at; None = reactive realign

        self._v_cmd = 0.0
        self._w_cmd = 0.0

        transient_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_return_path', 10)
        self.create_subscription(Path, '/return_path', self._on_return_path, transient_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Pose, '/mission/origin_pose', self._on_origin, transient_qos)

        self.create_timer(self.control_period, self._tick)

        self.get_logger().info('return_path_follower_node started (waiting for /return_path).')

    # ------------------------------------------------------------------ #
    def _on_return_path(self, msg: Path):
        # Stored once, as-is -- NOT re-transformed on every message like
        # dolbotz/purepursuit.py does. return_state_machine_node publishes
        # this exactly once per mission (TRANSIENT_LOCAL so a late
        # subscriber still gets it). Each pose's .yaw was already computed
        # there as the outgoing-segment heading (atan2 to the next point),
        # so corner_angle[i] below is just the difference between two
        # already-stored headings -- no new geometry needed.
        self._path = [Pose2D.from_ros_pose(ps.pose) for ps in msg.poses]
        self._corner_angle = [0.0] * len(self._path)
        for i in range(1, len(self._path) - 1):
            self._corner_angle[i] = normalize_angle(self._path[i].yaw - self._path[i - 1].yaw)
        self._corners = [i for i in range(1, len(self._path) - 1)
                         if abs(self._corner_angle[i]) >= self.corner_detection_angle]
        self._corner_ptr = 0
        self._floor = 0
        self._mode = TRACK_PATH
        self._rotating_corner = None
        self._rotate_target_yaw = None
        self._rotate_settle_since = None
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self.get_logger().info(
            f'/return_path received: {len(self._path)} points, '
            f'{len(self._corners)} sharp corners at indices {self._corners}.')

    def _on_odom(self, msg: Odometry):
        self._odom_pose = Pose2D.from_ros_pose(msg.pose.pose)
        self._last_odom_time = self.get_clock().now()

    def _on_origin(self, msg: Pose):
        self._origin = Pose2D.from_ros_pose(msg)

    # ------------------------------------------------------------------ #
    def _tick(self):
        if len(self._path) < 2 or self._odom_pose is None or self._origin is None:
            self._publish(0.0, 0.0)
            return

        odom_stale = (self._last_odom_time is None or
                      (self.get_clock().now() - self._last_odom_time).nanoseconds * 1e-9
                      > self.odom_timeout_s)
        if odom_stale:
            self._mode = TRACK_PATH  # reset so a fresh path/odom starts clean
            self._publish(0.0, 0.0)
            return

        current = self._odom_pose.to_mission_frame(self._origin)
        corner_idx = self._next_corner()
        hi = corner_idx if corner_idx is not None else len(self._path) - 1
        nearest_idx = self._nearest_index(current, self._floor, hi)

        if self._mode == TRACK_PATH:
            # heading_error is measured against the INCOMING segment while a
            # corner is still ahead (path[corner].yaw is already the
            # outgoing direction), and never against segments before the
            # last handled corner.
            ref_idx = nearest_idx
            if corner_idx is not None:
                ref_idx = min(nearest_idx, corner_idx - 1)
            ref_idx = max(self._floor, min(ref_idx, len(self._path) - 1))
            heading_error = normalize_angle(self._path[ref_idx].yaw - current.yaw)

            if corner_idx is not None and self._reached_corner(current, corner_idx):
                self._enter_rotate(self._path[corner_idx].yaw, corner_idx,
                                   f'reached corner {corner_idx} '
                                   f'({math.degrees(self._corner_angle[corner_idx]):.0f} deg turn)')
            elif abs(heading_error) > self.rotate_enter_angle:
                self._enter_rotate(self._path[ref_idx].yaw, None,
                                   f'heading_error={math.degrees(heading_error):.1f} deg')

        if self._mode == ROTATE_TO_PATH:
            self._tick_rotate_to_path(current)
        else:
            self._tick_track_path(current, nearest_idx, corner_idx)

    def _next_corner(self):
        if self._corner_ptr < len(self._corners):
            return self._corners[self._corner_ptr]
        return None

    def _reached_corner(self, current: Pose2D, corner_idx: int) -> bool:
        vertex = self._path[corner_idx]
        if current.distance_to(vertex) <= self.corner_reach_distance_m:
            return True
        # Passed the vertex along the incoming direction (missed it
        # sideways) -- rotate now rather than driving away from it.
        inc = self._path[corner_idx - 1].yaw
        along = ((current.x - vertex.x) * math.cos(inc)
                 + (current.y - vertex.y) * math.sin(inc))
        return along >= 0.0

    def _enter_rotate(self, target_yaw: float, corner_idx, reason: str):
        self._mode = ROTATE_TO_PATH
        self._rotating_corner = corner_idx
        self._rotate_target_yaw = target_yaw
        self._rotate_settle_since = None
        # Hard-zero both commands right at the switch (not ramped) -- the
        # "never linear.x and angular.z at the same time" rule is an
        # absolute safety requirement, not a smoothness preference.
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self.get_logger().info(f'TRACK_PATH -> ROTATE_TO_PATH ({reason})')

    def _tick_track_path(self, current: Pose2D, nearest_idx: int, corner_idx):
        hi = corner_idx if corner_idx is not None else len(self._path) - 1
        target = self._lookahead_point(current, nearest_idx, hi)
        goal = self._path[-1]

        # target expressed in the robot's current local frame (reusing the
        # same SE2 composition used for mission-frame conversion, just with
        # "current" as the momentary origin instead of T0).
        local = target.to_mission_frame(current)
        look_dist = math.hypot(local.x, local.y)

        if look_dist < 1e-3:
            curvature = 0.0
        else:
            curvature = 2.0 * local.y / (look_dist * look_dist)

        # decelerate toward whichever stop point is next: the corner vertex
        # (we stop there to rotate) or, if none is left, the goal.
        if corner_idx is not None:
            stop_dist = current.distance_to(self._path[corner_idx])
            radius = self.corner_slowdown_radius_m
        else:
            stop_dist = current.distance_to(goal)
            radius = self.goal_slowdown_radius_m
        if stop_dist < radius:
            scale = max(stop_dist / radius, 0.0)
            v_target = max(self.min_linear_speed_mps, self.linear_speed_mps * scale)
        else:
            v_target = self.linear_speed_mps

        w_target = max(-self.max_angular_speed_radps,
                        min(self.max_angular_speed_radps, v_target * curvature))

        self._v_cmd = self._ramp(self._v_cmd, v_target, self.linear_accel)
        self._w_cmd = self._ramp(self._w_cmd, w_target, self.angular_accel)

        self._publish(self._v_cmd, self._w_cmd)

    def _tick_rotate_to_path(self, current: Pose2D):
        # Safety: linear.x is always exactly 0 in this mode (hard-zeroed at
        # the TRACK_PATH -> ROTATE_TO_PATH switch, see _tick) -- never
        # ramped down concurrently with a nonzero angular.z.
        self._v_cmd = 0.0

        yaw_error = normalize_angle(self._rotate_target_yaw - current.yaw)

        if abs(yaw_error) <= 1e-4:
            w_target = 0.0
        else:
            raw = self.rotate_kp * yaw_error
            sign = 1.0 if raw >= 0.0 else -1.0
            magnitude = min(self.rotate_max_angular_speed, max(self.rotate_min_angular_speed, abs(raw)))
            w_target = sign * magnitude

        self._w_cmd = self._ramp(self._w_cmd, w_target, self.angular_accel)
        self._publish(self._v_cmd, self._w_cmd)

        now = self.get_clock().now()
        if abs(yaw_error) > self.rotate_exit_angle:
            self._rotate_settle_since = None
            return
        if self._rotate_settle_since is None:
            self._rotate_settle_since = now
            return
        settled_s = (now - self._rotate_settle_since).nanoseconds * 1e-9
        if settled_s >= self.rotate_settle_time_s:
            self._mode = TRACK_PATH
            self._rotate_settle_since = None
            if self._rotating_corner is not None:
                self._floor = self._rotating_corner
                self._corner_ptr += 1
                self._rotating_corner = None
            # Hard-zero here too (same reasoning as the TRACK->ROTATE
            # switch): without this, leftover angular velocity from the
            # rotation ramp carries into TRACK_PATH's own curvature-based
            # w_target and compounds with it, overshooting well past the
            # heading we just spent time settling onto.
            self._w_cmd = 0.0
            self.get_logger().info(
                f'ROTATE_TO_PATH -> TRACK_PATH (yaw_error={math.degrees(yaw_error):.1f} deg)')

    def _ramp(self, current: float, target: float, max_rate: float) -> float:
        max_delta = max_rate * self.control_period
        delta = max(-max_delta, min(max_delta, target - current))
        return current + delta

    def _nearest_index(self, current: Pose2D, lo: int, hi: int) -> int:
        best_idx = lo
        best_dist = float('inf')
        for i in range(lo, hi + 1):
            d = current.distance_to(self._path[i])
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _lookahead_point(self, current: Pose2D, nearest_idx: int, hi: int) -> Pose2D:
        for i in range(nearest_idx, hi + 1):
            if current.distance_to(self._path[i]) >= self.lookahead_distance_m:
                return self._path[i]
        return self._path[hi]

    def _publish(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
