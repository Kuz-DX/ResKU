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
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Pose, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from return_navigation.se2 import Pose2D

RECORDING_STATES = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}


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

        self.create_subscription(Pose, '/mission/origin_pose', self._on_origin, transient_qos)
        self.create_subscription(String, '/mission/return/state', self._on_state, 10)
        self.create_subscription(Path, '/recorded_path', self._on_recorded_path, 10)
        self.create_subscription(Path, '/return_path', self._on_return_path, transient_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)

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

    def _on_return_path(self, msg: Path):
        if len(msg.poses) >= 2:
            self._publish_history(self.return_history_pub, msg.poses,
                                   (1.0, 0.53, 0.0), f'return_{self._cycle_index}')

    def _on_odom(self, msg: Odometry):
        if self._state != 'FOLLOW_RETURN_PATH' or self._origin is None:
            return
        pose = Pose2D.from_ros_pose(msg.pose.pose).to_mission_frame(self._origin)
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
