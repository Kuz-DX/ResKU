"""
return_state_machine_node

Owns the manual-recording -> autonomous-return mission state machine, and is
the SOLE publisher of /cmd_vel_return (avoids two nodes racing to publish
the same topic -- see manual_return_bringup.launch.py / project plan).

States (published continuously as std_msgs/String on /mission/return/state):

    IDLE -> MANUAL_RECORDING -> WAIT_RETURN_COMMAND -> STOP_BEFORE_TURN
         -> TURN_180 -> FOLLOW_RETURN_PATH -> FINISHED -> IDLE (loops)

    [reverse_return, 2026-09] When true, STOP_BEFORE_TURN goes straight to
    FOLLOW_RETURN_PATH -- TURN_180 is skipped entirely, and the robot backs
    up along /return_path instead (return_path_follower_node's own
    reverse_drive parameter, set from the SAME launch arg, does the actual
    backward driving -- see that node's docstring for why this is safe to
    do with the plain pure-pursuit curvature formula unchanged). Default
    false = existing behaviour, byte-for-byte unchanged.

    IDLE->MANUAL_RECORDING:
        on /path/record rising edge, once /odometry/filtered is available.
        [UI interface, 2026-09] recording is no longer auto-started the
        instant odometry appears -- the operator/UI marks the origin
        explicitly by publishing /path/record=true. A stale True already
        latched in from an earlier (too-early) press cannot re-trigger this
        on its own: _go_to() clears _prev_record_trigger on every entry to
        IDLE, so only an actual False->True edge counts (see _on_record_trigger).
    MANUAL_RECORDING->WAIT_RETURN_COMMAND:
        once the recorder has captured >=2 waypoints (i.e. the robot has
        actually moved) -- an early RETURN trigger before that is ignored.
    WAIT_RETURN_COMMAND->STOP_BEFORE_TURN:
        on /path/return rising edge
    STOP_BEFORE_TURN->TURN_180 (reverse_return=false, default):
        |vx| and |wz| (from /odometry/filtered) below threshold, sustained
    STOP_BEFORE_TURN->FOLLOW_RETURN_PATH (reverse_return=true):
        same stop condition, but TURN_180 is skipped -- /return_path is
        built and published here instead of at the end of TURN_180
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
    /path/record             (std_msgs/Bool)  -- UI trigger, start recording at (0,0)
    /path/return             (std_msgs/Bool)  -- UI trigger, start Return to Base
    /emergency_stop          (std_msgs/Bool)  -- UI emergency shutdown trigger
    /mission/origin_pose     (geometry_msgs/Pose, TRANSIENT_LOCAL)
    /recorded_path           (nav_msgs/Path)
    /cmd_vel_return_path      (geometry_msgs/Twist) -- from return_path_follower_node

[UI boolean interface, 2026-09] /path/record and /path/return are level
signals from the UI team's spec ("default false, reset to false again after
the action runs") -- but THIS NODE only ever acts on a False->True EDGE
(_on_record_trigger/_on_return_trigger below), never on a sustained True.
This is deliberate and load-bearing: whether or not the UI actually resets
its own value back to False afterward, a stale True sitting on the topic
can never fire twice in a row here, and a fresh press is never missed
because _prev_record_trigger/_prev_trigger are cleared every time the node
re-enters the state that is waiting for that trigger (IDLE / 
WAIT_RETURN_COMMAND) -- see _go_to(). The UI must still publish False after
True for its own "boolean resets to default" semantics to be true on ITS
side, but this node's correctness does not depend on it doing so.
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

        # manual_return_bringup sets effective_track_width_m=1.58 so that a
        # commanded w is (roughly) the real rotation rate on the robot; these
        # are therefore real rates: 0.5 rad/s ~ 29 deg/s.
        self.declare_parameter('turn_kp', 1.5)
        self.declare_parameter('turn_w_max_radps', 0.5)
        self.declare_parameter('turn_yaw_tolerance_rad', 0.087)  # ~5 deg
        # 'left' = counter-clockwise (+angular.z), 'right' = clockwise.
        # Fixed on purpose: a 180 deg error has no natural shortest
        # direction, so leaving it to sensor noise picks a random side.
        self.declare_parameter('turn_direction', 'left')
        self.declare_parameter('turn_settle_duration_s', 0.3)
        # See module docstring [reverse_return].
        self.declare_parameter('reverse_return', False)

        self.declare_parameter('follow_relay_timeout_s', 0.5)
        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('finished_hold_s', 2.0)
        # Emergency 수신 직후 launch를 내리기 전에 safety zero가 드라이버까지
        # 전달될 시간을 확보한다. 20 Hz 기본 주기에서 5회 발행된다.
        self.declare_parameter('emergency_stop_hold_s', 0.25)

        p = self.get_parameter
        self.control_period = 1.0 / float(p('control_rate_hz').value)
        self.min_waypoints_for_return = int(p('min_waypoints_for_return').value)
        self.stop_vx_threshold = float(p('stop_vx_threshold_mps').value)
        self.stop_wz_threshold = float(p('stop_wz_threshold_radps').value)
        self.stop_settle_duration = float(p('stop_settle_duration_s').value)
        self.turn_kp = float(p('turn_kp').value)
        direction = str(p('turn_direction').value).strip().lower()
        if direction not in ('left', 'right'):
            raise ValueError(f"turn_direction must be 'left' or 'right', got '{direction}'")
        self.turn_sign = 1.0 if direction == 'left' else -1.0
        self.turn_w_max = float(p('turn_w_max_radps').value)
        self.turn_yaw_tolerance = float(p('turn_yaw_tolerance_rad').value)
        self.turn_settle_duration = float(p('turn_settle_duration_s').value)
        self.reverse_return = bool(p('reverse_return').value)
        self.follow_relay_timeout = float(p('follow_relay_timeout_s').value)
        self.goal_tolerance_m = float(p('goal_tolerance_m').value)
        self.finished_hold_s = float(p('finished_hold_s').value)
        self.emergency_stop_hold_s = float(p('emergency_stop_hold_s').value)

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
        self._record_trigger_pending = False
        self._prev_record_trigger = False

        self._stop_settle_since = None
        self._turn_prev_yaw = None
        self._turn_accum = 0.0  # signed rotation done since TURN_180 began
        self._turn_settle_since = None

        self._return_path = []       # list[Pose2D], mission frame, PN..P0 order
        self._follow_cmd = Twist()
        self._follow_cmd_time = None
        self._emergency_stop_active = False
        self._emergency_stop_started = None

        transient_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.state_pub = self.create_publisher(String, '/mission/return/state', 10)
        self.cmd_vel_return_pub = self.create_publisher(Twist, '/cmd_vel_return', 10)
        # rmd_x8_driver_node가 일반 /cmd_vel보다 우선 처리하는 안전 입력이다.
        self.cmd_vel_safety_pub = self.create_publisher(Twist, '/cmd_vel_safety', 10)
        self.recorder_cmd_pub = self.create_publisher(String, '/manual_path_recorder/command', 10)
        self.return_path_pub = self.create_publisher(Path, '/return_path', transient_qos)

        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Bool, '/path/record', self._on_record_trigger, 10)
        self.create_subscription(Bool, '/path/return', self._on_return_trigger, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop, 10)
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

    def _on_record_trigger(self, msg: Bool):
        if msg.data and not self._prev_record_trigger:
            self._record_trigger_pending = True
        self._prev_record_trigger = msg.data

    def _on_return_trigger(self, msg: Bool):
        if msg.data and not self._prev_trigger:
            self._trigger_pending = True
        self._prev_trigger = msg.data

    def _on_emergency_stop(self, msg: Bool):
        if not msg.data or self._emergency_stop_active:
            return
        self._emergency_stop_active = True
        self._emergency_stop_started = self.get_clock().now()
        # 콜백 시점에도 즉시 한 번 발행하고, _tick()에서 종료 전까지 반복한다.
        self.cmd_vel_return_pub.publish(Twist())
        self.cmd_vel_safety_pub.publish(Twist())
        self.get_logger().fatal(
            '/emergency_stop=true 수신: 모터 0 명령 후 manual_return launch를 종료합니다.')

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
            if new_state == WAIT_RETURN_COMMAND:
                # Publishers that only ever send True (keyboard, `topic pub -1`)
                # never produce a False->True edge after their first press, so a
                # stale True from an earlier (too-early, absorbed) press would
                # otherwise block every later trigger.
                self._prev_trigger = False
            if new_state == IDLE:
                # Same reasoning as above, for /path/record.
                self._prev_record_trigger = False
                # A /path/record edge that arrived in any OTHER state (e.g. an
                # accidental second press while recording) sets the pending
                # flag but nothing there consumes it; without this clear it
                # would silently auto-start the next recording the instant
                # the cycle returns to IDLE, with no new press.
                self._record_trigger_pending = False

    def _elapsed_in_state(self) -> float:
        return (self.get_clock().now() - self._state_entered_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------ #
    def _tick(self):
        if self._emergency_stop_active:
            self._tick_emergency_stop()
            return

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

    def _tick_emergency_stop(self):
        # mux의 현재 상태/입력과 무관하게 드라이버 최우선 safety 채널을 0으로
        # 유지한다. return 채널도 동시에 0으로 내려 정상 종료 중 재가속을 막는다.
        self.cmd_vel_return_pub.publish(Twist())
        self.cmd_vel_safety_pub.publish(Twist())

        elapsed = (
            (self.get_clock().now() - self._emergency_stop_started).nanoseconds * 1e-9
            if self._emergency_stop_started is not None else 0.0
        )
        if elapsed >= self.emergency_stop_hold_s:
            self.get_logger().fatal('긴급정지 0 명령 전달 완료: 노드를 종료합니다.')
            # manual_return_bringup.launch.py의 OnProcessExit가 이 정상 종료를
            # 감지해 launch 전체(rmd_x8_driver 포함)를 shutdown한다.
            rclpy.shutdown()

    def _tick_idle(self):
        if self._record_trigger_pending and self._odom_pose is not None:
            self._record_trigger_pending = False
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
            if self.reverse_return:
                self._build_and_publish_return_path()
                self._go_to(FOLLOW_RETURN_PATH)
                return
            self._turn_prev_yaw = self._odom_pose.yaw if self._odom_pose else 0.0
            self._turn_accum = 0.0
            self._turn_settle_since = None
            self._go_to(TURN_180)

    def _tick_turn_180(self):
        if self._odom_pose is None:
            self.cmd_vel_return_pub.publish(Twist())
            return

        yaw = self._odom_pose.yaw
        self._turn_accum += normalize_angle(yaw - self._turn_prev_yaw)
        self._turn_prev_yaw = yaw
        yaw_error = self.turn_sign * math.pi - self._turn_accum
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
