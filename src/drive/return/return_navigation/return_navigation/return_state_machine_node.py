"""
return_state_machine_node

Owns the manual-recording -> autonomous-return mission state machine, and is
the SOLE publisher of /cmd_vel_return (avoids two nodes racing to publish
the same topic -- see manual_return_bringup.launch.py / project plan).

States (published continuously as std_msgs/String on /mission/return/state):

    IDLE -> MANUAL_RECORDING -> WAIT_RETURN_COMMAND -> STOP_BEFORE_TURN
         -> TURN_180 -> FOLLOW_RETURN_PATH -> FINISHED -> IDLE (loops)

    IDLE->MANUAL_RECORDING:            once /odometry/filtered is available
    MANUAL_RECORDING->WAIT_RETURN_COMMAND:
        once the recorder has captured >=2 waypoints (i.e. the robot has
        actually moved) -- an early RETURN trigger before that is ignored.
    WAIT_RETURN_COMMAND->STOP_BEFORE_TURN:
        on /mission/return/trigger rising edge
    STOP_BEFORE_TURN->TURN_180:
        |vx| and |wz| (from /odometry/filtered) below threshold, sustained
    TURN_180->FOLLOW_RETURN_PATH:
        closed-loop 180-degree turn complete (yaw error within tolerance,
        sustained) -- NOT timer-based. On this transition, /return_path is
        built once (reverse + re-yaw of /recorded_path) and published.
    FOLLOW_RETURN_PATH->FINISHED:
        current mission-frame position within goal_tolerance_m of the
        final /return_path pose
    FINISHED->IDLE:
        after finished_hold_s (purely for observability -- functionally
        identical to IDLE for drive_cmd_mux_node, both mean "manual
        passthrough")

Publishes:
    /mission/return/state          (std_msgs/String)
    /cmd_vel_return                 (geometry_msgs/Twist)  -- sole publisher
    /manual_path_recorder/command   (std_msgs/String)       -- START/STOP
    /return_path                    (nav_msgs/Path, TRANSIENT_LOCAL)

Subscribes:
    /odometry/filtered      (nav_msgs/Odometry)
    /mission/return/trigger  (std_msgs/Bool)
    /mission/origin_pose     (geometry_msgs/Pose, TRANSIENT_LOCAL)
    /recorded_path           (nav_msgs/Path)
    /cmd_vel_return_path      (geometry_msgs/Twist) -- from return_path_follower_node
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

from return_navigation.se2 import Pose2D, normalize_angle

IDLE = 'IDLE'
MANUAL_RECORDING = 'MANUAL_RECORDING'
WAIT_RETURN_COMMAND = 'WAIT_RETURN_COMMAND'
STOP_BEFORE_TURN = 'STOP_BEFORE_TURN'
TURN_180 = 'TURN_180'
FOLLOW_RETURN_PATH = 'FOLLOW_RETURN_PATH'
FINISHED = 'FINISHED'


class ReturnStateMachineNode(Node):
    def __init__(self):
        super().__init__('return_state_machine_node')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('min_waypoints_for_return', 2)

        self.declare_parameter('stop_vx_threshold_mps', 0.03)
        self.declare_parameter('stop_wz_threshold_radps', 0.03)
        self.declare_parameter('stop_settle_duration_s', 0.5)

        self.declare_parameter('turn_kp', 1.0)
        self.declare_parameter('turn_w_max_radps', 0.6)
        self.declare_parameter('turn_yaw_tolerance_rad', 0.035)  # ~2 deg
        self.declare_parameter('turn_settle_duration_s', 0.3)

        self.declare_parameter('follow_relay_timeout_s', 0.5)
        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('finished_hold_s', 2.0)

        p = self.get_parameter
        self.control_period = 1.0 / float(p('control_rate_hz').value)
        self.min_waypoints_for_return = int(p('min_waypoints_for_return').value)
        self.stop_vx_threshold = float(p('stop_vx_threshold_mps').value)
        self.stop_wz_threshold = float(p('stop_wz_threshold_radps').value)
        self.stop_settle_duration = float(p('stop_settle_duration_s').value)
        self.turn_kp = float(p('turn_kp').value)
        self.turn_w_max = float(p('turn_w_max_radps').value)
        self.turn_yaw_tolerance = float(p('turn_yaw_tolerance_rad').value)
        self.turn_settle_duration = float(p('turn_settle_duration_s').value)
        self.follow_relay_timeout = float(p('follow_relay_timeout_s').value)
        self.goal_tolerance_m = float(p('goal_tolerance_m').value)
        self.finished_hold_s = float(p('finished_hold_s').value)

        # ---- runtime state ----
        self._state = IDLE
        self._state_entered_time = self.get_clock().now()

        self._odom_pose = None       # Pose2D, odom frame
        self._odom_vx = 0.0
        self._odom_wz = 0.0
        self._origin = None          # Pose2D, odom frame (T0)
        self._recorded_path = []     # list[Pose2D], mission frame, forward order

        self._trigger_pending = False
        self._prev_trigger = False

        self._stop_settle_since = None
        self._turn_yaw_start = None
        self._turn_yaw_target = None
        self._turn_settle_since = None

        self._return_path = []       # list[Pose2D], mission frame, PN..P0 order
        self._follow_cmd = Twist()
        self._follow_cmd_time = None

        transient_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.state_pub = self.create_publisher(String, '/mission/return/state', 10)
        self.cmd_vel_return_pub = self.create_publisher(Twist, '/cmd_vel_return', 10)
        self.recorder_cmd_pub = self.create_publisher(String, '/manual_path_recorder/command', 10)
        self.return_path_pub = self.create_publisher(Path, '/return_path', transient_qos)

        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Bool, '/mission/return/trigger', self._on_trigger, 10)
        self.create_subscription(Pose, '/mission/origin_pose', self._on_origin, transient_qos)
        self.create_subscription(Path, '/recorded_path', self._on_recorded_path, 10)
        self.create_subscription(Twist, '/cmd_vel_return_path', self._on_follow_cmd, 10)

        self.create_timer(self.control_period, self._tick)

        self.get_logger().info('return_state_machine_node started in IDLE.')

    # ------------------------------------------------------------------ #
    # subscriptions
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry):
        self._odom_pose = Pose2D.from_ros_pose(msg.pose.pose)
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_wz = msg.twist.twist.angular.z

    def _on_trigger(self, msg: Bool):
        if msg.data and not self._prev_trigger:
            self._trigger_pending = True
        self._prev_trigger = msg.data

    def _on_origin(self, msg: Pose):
        self._origin = Pose2D.from_ros_pose(msg)

    def _on_recorded_path(self, msg: Path):
        self._recorded_path = [Pose2D.from_ros_pose(ps.pose) for ps in msg.poses]

    def _on_follow_cmd(self, msg: Twist):
        self._follow_cmd = msg
        self._follow_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------ #
    def _go_to(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'[return_state_machine] {self._state} -> {new_state}')
            self._state = new_state
            self._state_entered_time = self.get_clock().now()

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self._state_entered_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------ #
    def _tick(self):
        self.state_pub.publish(String(data=self._state))

        if self._state == IDLE:
            self._tick_idle()
        elif self._state == MANUAL_RECORDING:
            self._tick_manual_recording()
        elif self._state == WAIT_RETURN_COMMAND:
            self._tick_wait_return_command()
        elif self._state == STOP_BEFORE_TURN:
            self._tick_stop_before_turn()
        elif self._state == TURN_180:
            self._tick_turn_180()
        elif self._state == FOLLOW_RETURN_PATH:
            self._tick_follow_return_path()
        elif self._state == FINISHED:
            self._tick_finished()

    def _tick_idle(self):
        if self._odom_pose is not None:
            self.recorder_cmd_pub.publish(String(data='START'))
            self._go_to(MANUAL_RECORDING)

    def _tick_manual_recording(self):
        self._trigger_pending = False  # too early -- ignore/absorb any press
        if len(self._recorded_path) >= self.min_waypoints_for_return:
            self._go_to(WAIT_RETURN_COMMAND)

    def _tick_wait_return_command(self):
        if self._trigger_pending:
            self._trigger_pending = False
            self.recorder_cmd_pub.publish(String(data='STOP'))
            self._stop_settle_since = None
            self._go_to(STOP_BEFORE_TURN)

    def _tick_stop_before_turn(self):
        self.cmd_vel_return_pub.publish(Twist())  # belt-and-suspenders zero; mux also forces zero here

        stopped = (abs(self._odom_vx) <= self.stop_vx_threshold
                   and abs(self._odom_wz) <= self.stop_wz_threshold)
        now = self.get_clock().now()
        if not stopped:
            self._stop_settle_since = None
            return
        if self._stop_settle_since is None:
            self._stop_settle_since = now
            return
        settled_s = (now - self._stop_settle_since).nanoseconds * 1e-9
        if settled_s >= self.stop_settle_duration:
            self._turn_yaw_start = self._odom_pose.yaw if self._odom_pose else 0.0
            self._turn_yaw_target = normalize_angle(self._turn_yaw_start + math.pi)
            self._turn_settle_since = None
            self._go_to(TURN_180)

    def _tick_turn_180(self):
        if self._odom_pose is None:
            self.cmd_vel_return_pub.publish(Twist())
            return

        yaw_error = normalize_angle(self._turn_yaw_target - self._odom_pose.yaw)
        cmd = Twist()
        cmd.angular.z = max(-self.turn_w_max, min(self.turn_w_max, self.turn_kp * yaw_error))
        self.cmd_vel_return_pub.publish(cmd)

        now = self.get_clock().now()
        if abs(yaw_error) > self.turn_yaw_tolerance:
            self._turn_settle_since = None
            return
        if self._turn_settle_since is None:
            self._turn_settle_since = now
            return
        settled_s = (now - self._turn_settle_since).nanoseconds * 1e-9
        if settled_s >= self.turn_settle_duration:
            self._build_and_publish_return_path()
            self._go_to(FOLLOW_RETURN_PATH)

    def _tick_follow_return_path(self):
        fresh = (self._follow_cmd_time is not None and
                  (self.get_clock().now() - self._follow_cmd_time).nanoseconds * 1e-9
                  <= self.follow_relay_timeout)
        if fresh:
            self.cmd_vel_return_pub.publish(self._follow_cmd)
        else:
            self.cmd_vel_return_pub.publish(Twist())
            self.get_logger().warn('/cmd_vel_return_path is stale -- commanding zero.',
                                    throttle_duration_sec=1.0)

        if not self._return_path or self._odom_pose is None or self._origin is None:
            return
        current_mission = self._odom_pose.to_mission_frame(self._origin)
        goal = self._return_path[-1]
        if current_mission.distance_to(goal) <= self.goal_tolerance_m:
            self.cmd_vel_return_pub.publish(Twist())
            self._go_to(FINISHED)

    def _tick_finished(self):
        self.cmd_vel_return_pub.publish(Twist())
        if self._elapsed_in_state() >= self.finished_hold_s:
            self.recorder_cmd_pub.publish(String(data='CLEAR'))
            self._return_path = []
            self._go_to(IDLE)

    # ------------------------------------------------------------------ #
    def _build_and_publish_return_path(self):
        """Reverse /recorded_path (P0..PN -> PN..P0) and recompute each
        point's yaw from the direction to the NEXT point along the new
        (reversed) order -- not simply recorded_yaw + pi. The final point
        reuses the previous segment's yaw."""
        reversed_poses = list(reversed(self._recorded_path))
        n = len(reversed_poses)
        result = []
        for i in range(n):
            x, y = reversed_poses[i].x, reversed_poses[i].y
            if i < n - 1:
                yaw = math.atan2(reversed_poses[i + 1].y - y, reversed_poses[i + 1].x - x)
            elif result:
                yaw = result[-1].yaw
            else:
                yaw = 0.0
            result.append(Pose2D(x, y, yaw))

        self._return_path = result

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'mission'
        for pose in result:
            ps = PoseStamped()
            ps.header.frame_id = 'mission'
            ps.pose = pose.to_ros_pose()
            path_msg.poses.append(ps)
        self.return_path_pub.publish(path_msg)

        self.get_logger().info(f'/return_path published: {n} points (reversed from /recorded_path).')


def main(args=None):
    rclpy.init(args=args)
    node = ReturnStateMachineNode()
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
