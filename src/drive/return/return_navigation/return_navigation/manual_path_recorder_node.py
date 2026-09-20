"""
manual_path_recorder_node

Records the robot's actual manual-drive path as a nav_msgs/Path in a fixed
"mission" frame, using /odometry/filtered (wheel+IMU fused pose, odom
frame). The mission frame's origin T0 is captured once when recording
starts (the pose the robot was at when MANUAL driving began) so the
recorded path's first point is always (x=0, y=0, yaw=0) -- see se2.py.
odom itself, and reduced_odom_node's EKF state, are never touched/reset.

Subscribes:
    /odometry/filtered            (nav_msgs/Odometry)
    /manual_path_recorder/command (std_msgs/String) -- "START"/"STOP"/"CLEAR"/"SAVE"

Publishes:
    /mission/origin_pose  (geometry_msgs/Pose, TRANSIENT_LOCAL) -- T0, once per START
    /recorded_path_raw    (nav_msgs/Path, frame_id="mission")   -- every gated sample
    /recorded_path        (nav_msgs/Path, frame_id="mission")   -- _process_path(raw)

Waypoint gating (a new point is appended only if ANY holds since the last
stored point): distance >= distance_threshold_m, |yaw delta| >=
yaw_threshold_rad, or elapsed >= max_time_interval_s. The very first point
of a recording is always (0,0,0) regardless of these thresholds.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from return_navigation.se2 import Pose2D, normalize_angle


class ManualPathRecorderNode(Node):
    def __init__(self):
        super().__init__('manual_path_recorder_node')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('command_topic', '/manual_path_recorder/command')
        self.declare_parameter('raw_path_topic', '/recorded_path_raw')
        self.declare_parameter('processed_path_topic', '/recorded_path')
        self.declare_parameter('origin_pose_topic', '/mission/origin_pose')
        self.declare_parameter('mission_frame_id', 'mission')
        self.declare_parameter('distance_threshold_m', 0.3)
        self.declare_parameter('yaw_threshold_rad', 0.26)   # ~15 deg
        self.declare_parameter('max_time_interval_s', 2.0)
        self.declare_parameter('min_point_spacing_m', 0.1)

        p = self.get_parameter
        self.mission_frame_id = p('mission_frame_id').value
        self.distance_threshold_m = float(p('distance_threshold_m').value)
        self.yaw_threshold_rad = float(p('yaw_threshold_rad').value)
        self.max_time_interval_s = float(p('max_time_interval_s').value)
        self.min_point_spacing_m = float(p('min_point_spacing_m').value)

        self._recording = False
        self._origin = None            # Pose2D, odom frame, set on START
        self._last_odom_pose = None    # Pose2D, odom frame, updated every sample
        self._raw_poses = []           # list[Pose2D], mission frame
        self._last_stored_pose = None  # Pose2D, mission frame
        self._last_stored_time = None

        origin_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.origin_pub = self.create_publisher(Pose, p('origin_pose_topic').value, origin_qos)
        self.raw_path_pub = self.create_publisher(Path, p('raw_path_topic').value, 10)
        self.processed_path_pub = self.create_publisher(Path, p('processed_path_topic').value, 10)

        self.create_subscription(Odometry, p('odom_topic').value, self._on_odom, 10)
        self.create_subscription(String, p('command_topic').value, self._on_command, 10)

        self.get_logger().info('manual_path_recorder_node started (waiting for START command).')

    # ------------------------------------------------------------------ #
    def _on_command(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self._start_recording()
        elif cmd == 'STOP':
            self._recording = False
            self.get_logger().info(f'Recording stopped. {len(self._raw_poses)} waypoints captured.')
        elif cmd == 'CLEAR':
            self._raw_poses = []
            self._origin = None
            self._last_stored_pose = None
            self._last_stored_time = None
            self._recording = False
            self._publish_paths()
            self.get_logger().info('Recorded path cleared.')
        elif cmd == 'SAVE':
            # 추후 디스크 저장용 훅 -- P0 범위에서는 stub.
            self.get_logger().info(f'SAVE requested ({len(self._raw_poses)} waypoints) -- not yet implemented.')
        else:
            self.get_logger().warn(f'Unknown manual_path_recorder command: "{msg.data}"')

    def _start_recording(self):
        if self._last_odom_pose is None:
            self.get_logger().error('START requested but no /odometry/filtered received yet -- ignored.')
            return

        self._origin = Pose2D(self._last_odom_pose.x, self._last_odom_pose.y, self._last_odom_pose.yaw)
        self._raw_poses = [Pose2D(0.0, 0.0, 0.0)]  # mission origin, always the first point
        self._last_stored_pose = self._raw_poses[0]
        self._last_stored_time = self.get_clock().now()
        self._recording = True

        self.origin_pub.publish(self._origin.to_ros_pose())
        self._publish_paths()
        self.get_logger().info(
            f'Recording started. Mission origin (odom frame): '
            f'x={self._origin.x:.3f} y={self._origin.y:.3f} yaw={self._origin.yaw:.3f}')

    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry):
        self._last_odom_pose = Pose2D.from_ros_pose(msg.pose.pose)

        if not self._recording or self._origin is None:
            return

        current = self._last_odom_pose.to_mission_frame(self._origin)

        distance = current.distance_to(self._last_stored_pose)
        yaw_delta = abs(normalize_angle(current.yaw - self._last_stored_pose.yaw))
        elapsed_s = (self.get_clock().now() - self._last_stored_time).nanoseconds * 1e-9

        if (distance >= self.distance_threshold_m
                or yaw_delta >= self.yaw_threshold_rad
                or elapsed_s >= self.max_time_interval_s):
            self._raw_poses.append(current)
            self._last_stored_pose = current
            self._last_stored_time = self.get_clock().now()
            self._publish_paths()

    # ------------------------------------------------------------------ #
    def _process_path(self, raw_poses):
        """Merge points closer than min_point_spacing_m to the last KEPT
        point (first point of each cluster wins).

        Turning in place fires the yaw gate repeatedly at (almost) the same
        spot, stacking points a few mm apart. The return path derives each
        point's heading from atan2(next - this), so such stacked points get
        random headings (measured: -134, -170, -152, -85, -74 deg across
        one physical corner) and the follower spins back and forth. The
        cluster's position IS the corner vertex, so keeping its first
        point preserves the corner geometry.

        INVARIANT for any future resampling/smoothing added here: a point
        whose heading changes sharply from its neighbor (a recorded sharp
        corner) must never be dropped -- return_path_follower_node relies
        on /recorded_path (via /return_path's per-point yaw) to detect and
        rotate-in-place at those corners. Thinning by distance/count alone
        would silently erase that geometry and corner-cutting would come
        back regardless of follower-side fixes."""
        processed = []
        for pose in raw_poses:
            if processed and pose.distance_to(processed[-1]) < self.min_point_spacing_m:
                continue
            processed.append(pose)
        return processed

    def _to_path_msg(self, poses) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.mission_frame_id
        for pose in poses:
            ps = PoseStamped()
            ps.header.frame_id = self.mission_frame_id
            ps.pose = pose.to_ros_pose()
            path.poses.append(ps)
        return path

    def _publish_paths(self):
        self.raw_path_pub.publish(self._to_path_msg(self._raw_poses))
        self.processed_path_pub.publish(self._to_path_msg(self._process_path(self._raw_poses)))


def main(args=None):
    rclpy.init(args=args)
    node = ManualPathRecorderNode()
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
