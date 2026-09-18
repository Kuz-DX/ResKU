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

from return_navigation.se2 import Pose2D


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

        p = self.get_parameter
        self.control_period = 1.0 / float(p('control_rate_hz').value)
        self.lookahead_distance_m = float(p('lookahead_distance_m').value)
        self.linear_speed_mps = float(p('linear_speed_mps').value)
        self.min_linear_speed_mps = float(p('min_linear_speed_mps').value)
        self.max_angular_speed_radps = float(p('max_angular_speed_radps').value)
        self.goal_slowdown_radius_m = float(p('goal_slowdown_radius_m').value)
        self.linear_accel = float(p('linear_accel_mps2').value)
        self.angular_accel = float(p('angular_accel_radps2').value)

        self._path = []       # list[Pose2D], mission frame, fixed once received
        self._origin = None   # Pose2D, odom frame (T0)
        self._odom_pose = None  # Pose2D, odom frame

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
        # subscriber still gets it).
        self._path = [Pose2D.from_ros_pose(ps.pose) for ps in msg.poses]
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self.get_logger().info(f'/return_path received: {len(self._path)} points.')

    def _on_odom(self, msg: Odometry):
        self._odom_pose = Pose2D.from_ros_pose(msg.pose.pose)

    def _on_origin(self, msg: Pose):
        self._origin = Pose2D.from_ros_pose(msg)

    # ------------------------------------------------------------------ #
    def _tick(self):
        if len(self._path) < 2 or self._odom_pose is None or self._origin is None:
            self._publish(0.0, 0.0)
            return

        current = self._odom_pose.to_mission_frame(self._origin)

        nearest_idx = self._nearest_index(current)
        target = self._lookahead_point(current, nearest_idx)
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

        dist_to_goal = current.distance_to(goal)
        if dist_to_goal < self.goal_slowdown_radius_m:
            scale = max(dist_to_goal / self.goal_slowdown_radius_m, 0.0)
            v_target = max(self.min_linear_speed_mps, self.linear_speed_mps * scale)
        else:
            v_target = self.linear_speed_mps

        w_target = max(-self.max_angular_speed_radps,
                        min(self.max_angular_speed_radps, v_target * curvature))

        self._v_cmd = self._ramp(self._v_cmd, v_target, self.linear_accel)
        self._w_cmd = self._ramp(self._w_cmd, w_target, self.angular_accel)

        self._publish(self._v_cmd, self._w_cmd)

    def _ramp(self, current: float, target: float, max_rate: float) -> float:
        max_delta = max_rate * self.control_period
        delta = max(-max_delta, min(max_delta, target - current))
        return current + delta

    def _nearest_index(self, current: Pose2D) -> int:
        best_idx = 0
        best_dist = float('inf')
        for i, pose in enumerate(self._path):
            d = current.distance_to(pose)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _lookahead_point(self, current: Pose2D, nearest_idx: int) -> Pose2D:
        for i in range(nearest_idx, len(self._path)):
            if current.distance_to(self._path[i]) >= self.lookahead_distance_m:
                return self._path[i]
        return self._path[-1]

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
