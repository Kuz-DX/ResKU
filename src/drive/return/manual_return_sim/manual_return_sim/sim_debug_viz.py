"""
sim_debug_viz

Pre-flight virtual test tool (NOT for production launch). Two jobs:

1. Broadcasts a debug-only "odom -> mission" TF from /mission/origin_pose,
   so RViz can actually render /recorded_path_raw, /recorded_path,
   /return_path (all stamped in the "mission" frame) against Fixed Frame
   "odom". Production nodes (manual_path_recorder_node,
   return_path_follower_node) deliberately do NOT broadcast this TF --
   they do the mission-frame math directly (see se2.py) to avoid TF-timing
   edge cases -- so without this node those Path topics silently fail to
   render in RViz (no error, just nothing drawn).

2. Keeps a running, never-cleared history of each mission cycle's
   recorded path, return path, and the ACTUAL trajectory driven during
   FOLLOW_RETURN_PATH (sampled from /odometry/filtered), published as
   MarkerArrays -- so you can visually compare the planned return path
   against what the robot actually did, across multiple record/return
   cycles, instead of the live topics which get cleared/overwritten each
   cycle by return_state_machine_node's normal re-arm behavior.

3. Once per second during TURN_180 / FOLLOW_RETURN_PATH, drops a labelled
   dot on the actual trace ("12s yaw=70 path=82 err=+12 xte=+0.05") on
   /debug/yaw_labels and prints the same row to the console, so the yaw
   history can be read second by second next to the RViz picture. err =
   (heading of the nearest planned segment) - (robot yaw); xte = signed
   distance to that segment (+ = robot is to the left of the path).
   A live text (state / yaw / v,w) follows the robot on /debug/live_yaw.

Works against the real robot too: it only needs /odometry/filtered,
/mission/* and /return_path, all visible over DDS from another PC.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from return_navigation.se2 import Pose2D, normalize_angle

RECORDING_STATES = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}
SAMPLED_STATES = {'TURN_180', 'FOLLOW_RETURN_PATH'}
MAX_LABELS = 600


class SimDebugVizNode(Node):
    def __init__(self):
        super().__init__('sim_debug_viz')

        self.declare_parameter('mission_frame_id', 'mission')
        self.declare_parameter('odom_frame_id', 'odom')
        self.mission_frame_id = self.get_parameter('mission_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        transient_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._origin = None  # Pose2D, odom frame (T0), from /mission/origin_pose

        self._state = ''
        self._prev_state = ''
        self._latest_recorded_path = None  # Path, last message seen
        self._cycle_index = 0

        self._actual_trace = []  # list[Pose2D], mission frame, sampled during FOLLOW_RETURN_PATH

        self.recorded_history_pub = self.create_publisher(
            MarkerArray, '/debug/recorded_path_history', 10)
        self.return_history_pub = self.create_publisher(
            MarkerArray, '/debug/return_path_history', 10)
        self.actual_history_pub = self.create_publisher(
            MarkerArray, '/debug/actual_return_trace_history', 10)
        self.yaw_labels_pub = self.create_publisher(MarkerArray, '/debug/yaw_labels', 10)
        self.live_yaw_pub = self.create_publisher(Marker, '/debug/live_yaw', 10)

        self._return_path = []      # list[Pose2D], mission frame (planned return path)
        self._pose = None           # Pose2D, mission frame, latest odometry
        self._cmd = (0.0, 0.0)      # latest /cmd_vel_return_path (v, w)
        self._sample_t0 = None      # clock time when TURN_180/FOLLOW sampling began
        self._labels = []           # accumulated label/dot markers
        self._label_id = 0
        self._last_live_pub = 0.0

        self.create_subscription(Pose, '/mission/origin_pose', self._on_origin, transient_qos)
        self.create_subscription(String, '/mission/return/state', self._on_state, 10)
        self.create_subscription(Path, '/recorded_path', self._on_recorded_path, 10)
        self.create_subscription(Path, '/return_path', self._on_return_path, transient_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_return_path', self._on_cmd, 10)
        self.create_timer(1.0, self._tick_1s)

        self.get_logger().info(
            'sim_debug_viz started -- publishing odom->mission debug TF and '
            'cross-cycle path history markers.')

    # ------------------------------------------------------------------ #
    def _on_origin(self, msg: Pose):
        self._origin = Pose2D.from_ros_pose(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.mission_frame_id
        t.transform.translation.x = self._origin.x
        t.transform.translation.y = self._origin.y
        qx, qy, qz, qw = _yaw_to_quat(self._origin.yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self._tf_broadcaster.sendTransform(t)

    def _on_recorded_path(self, msg: Path):
        self._latest_recorded_path = msg

    def _on_cmd(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_return_path(self, msg: Path):
        self._return_path = [Pose2D.from_ros_pose(ps.pose) for ps in msg.poses]
        if len(msg.poses) >= 2:
            self._publish_history(self.return_history_pub, msg.poses,
                                   (1.0, 0.53, 0.0), f'return_{self._cycle_index}')

    def _on_odom(self, msg: Odometry):
        if self._origin is None:
            return
        pose = Pose2D.from_ros_pose(msg.pose.pose).to_mission_frame(self._origin)
        self._pose = pose
        self._publish_live(pose)
        if self._state != 'FOLLOW_RETURN_PATH':
            return
        self._actual_trace.append(pose)

    def _on_state(self, msg: String):
        self._state = msg.data

        # Recording just ended -- snapshot the final recorded path before
        # the next cycle's CLEAR wipes it.
        if (self._prev_state in RECORDING_STATES and self._state not in RECORDING_STATES
                and self._latest_recorded_path is not None
                and len(self._latest_recorded_path.poses) >= 2):
            self._publish_history(self.recorded_history_pub, self._latest_recorded_path.poses,
                                   (0.1, 0.9, 0.1), f'recorded_{self._cycle_index}')

        # FOLLOW_RETURN_PATH just ended -- snapshot what was actually driven.
        if self._prev_state == 'FOLLOW_RETURN_PATH' and self._state != 'FOLLOW_RETURN_PATH':
            if len(self._actual_trace) >= 2:
                poses = [_pose2d_to_stamped_pose(p) for p in self._actual_trace]
                self._publish_history(self.actual_history_pub, poses,
                                       (0.0, 0.8, 1.0), f'actual_{self._cycle_index}')
            self._actual_trace = []
            self._cycle_index += 1

        self._prev_state = self._state

    # ------------------------------------------------------------------ #
    def _nearest_segment(self, pose: Pose2D):
        """(heading, signed cross-track) of the planned segment closest to pose."""
        path = self._return_path
        best = None
        for a, b in zip(path, path[1:]):
            dx, dy = b.x - a.x, b.y - a.y
            length2 = dx * dx + dy * dy
            if length2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / length2))
            px, py = a.x + t * dx, a.y + t * dy
            dist = math.hypot(pose.x - px, pose.y - py)
            if best is None or dist < best[0]:
                cross = dx * (pose.y - a.y) - dy * (pose.x - a.x)
                side = 1.0 if cross >= 0.0 else -1.0  # + = robot on the left of the path
                best = (dist, math.atan2(dy, dx), side * dist)
        if best is None:
            return None
        return best[1], best[2]

    def _tick_1s(self):
        if self._state not in SAMPLED_STATES or self._pose is None:
            self._sample_t0 = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._sample_t0 is None:
            self._sample_t0 = now
            self.get_logger().info(
                '  t(s)  state                 yaw(deg) path(deg)  err(deg)   xte(m)     v(m/s)  w(rad/s)')
        t = now - self._sample_t0
        yaw = math.degrees(self._pose.yaw)
        seg = self._nearest_segment(self._pose) if self._state == 'FOLLOW_RETURN_PATH' else None
        v, w = self._cmd
        if seg is not None:
            path_h, xte = seg
            err = math.degrees(normalize_angle(path_h - self._pose.yaw))
            row = (f'{t:6.0f}  {self._state:<20} {yaw:9.1f} {math.degrees(path_h):9.1f} '
                   f'{err:+9.1f} {xte:+9.3f} {v:10.2f} {w:+9.2f}')
            text = f'{t:.0f}s  yaw={yaw:.0f}\npath={math.degrees(path_h):.0f} err={err:+.0f}\nxte={xte:+.2f}m'
        else:
            row = (f'{t:6.0f}  {self._state:<20} {yaw:9.1f} {"-":>9} {"-":>9} {"-":>9} '
                   f'{"-":>10} {"-":>9}')
            text = f'{t:.0f}s  yaw={yaw:.0f}'
        self.get_logger().info(row)
        self._add_label(text)

    def _add_label(self, text: str):
        stamp = self.get_clock().now().to_msg()
        x, y = self._pose.x, self._pose.y
        dot = Marker()
        dot.header.frame_id = self.mission_frame_id
        dot.header.stamp = stamp
        dot.ns = f'yaw_dot_{self._cycle_index}'
        dot.id = self._label_id
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x, dot.pose.position.y = x, y
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.09
        dot.color.r, dot.color.g, dot.color.b, dot.color.a = 1.0, 1.0, 0.0, 1.0
        label = Marker()
        label.header = dot.header
        label.ns = f'yaw_text_{self._cycle_index}'
        label.id = self._label_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.2
        label.pose.orientation.w = 1.0
        label.scale.z = 0.14
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = text
        self._label_id += 1
        self._labels.extend([dot, label])
        self._labels = self._labels[-MAX_LABELS:]
        array = MarkerArray()
        array.markers = list(self._labels)
        self.yaw_labels_pub.publish(array)

    def _publish_live(self, pose: Pose2D):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_live_pub < 0.2:
            return
        self._last_live_pub = now
        m = Marker()
        m.header.frame_id = self.mission_frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'live_yaw'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pose.x, pose.y, 0.55
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 1.0, 1.0, 1.0
        m.text = (f'{self._state}\nyaw={math.degrees(pose.yaw):.1f}\n'
                  f'v={self._cmd[0]:.2f} w={self._cmd[1]:+.2f}')
        self.live_yaw_pub.publish(m)

    # ------------------------------------------------------------------ #
    def _publish_history(self, publisher, poses, rgb, ns):
        marker = Marker()
        marker.header.frame_id = self.mission_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*rgb, 0.9)
        marker.pose.orientation.w = 1.0
        for p in poses:
            pos = p.pose.position
            marker.points.append(Point(x=pos.x, y=pos.y, z=0.0))
        array = MarkerArray()
        array.markers.append(marker)
        publisher.publish(array)


def _yaw_to_quat(yaw: float):
    import math
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _pose2d_to_stamped_pose(p: Pose2D):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose = p.to_ros_pose()
    return ps


def main(args=None):
    rclpy.init(args=args)
    node = SimDebugVizNode()
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
