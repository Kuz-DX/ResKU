"""Passive Servo/controller diagnostics; never publishes motor commands."""
import math
import time

from control_msgs.msg import JointJog, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory

WATCH = ('base_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'shoulder_joint')


def number(values, index):
    """Missing/nonfinite fields stay NA rather than becoming a false zero."""
    if index is None or index >= len(values):
        return None
    value = float(values[index])
    return value if math.isfinite(value) else None


def field(point, attribute, index):
    return number(getattr(point, attribute, ()), index)


def indexed(names, name):
    return names.index(name) if name in names else None


def fmt(value):
    return 'NA' if value is None else f'{value:.9f}'


def state_point(message, modern, legacy):
    """Humble versions may contain both schemas but populate only one."""
    candidate = getattr(message, modern, None)
    if candidate is not None and len(candidate.positions):
        return candidate, modern
    return getattr(message, legacy, None), legacy


class TrajectoryDiagnostics:
    def __init__(self, node, jog_topic, state_topic, clock=time.monotonic):
        self.node, self.clock = node, clock
        self.trajectory = None
        self.states = {}
        self.joints = None
        self.jog = None
        self.last_motion = -math.inf
        self.previous_positions = {}
        self.last_trajectory_at = None
        self.delta = {}
        self.trajectory_dt = None
        self.status = None
        self.subscriptions = [
            node.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.on_trajectory, 1),
            node.create_subscription(JointState, state_topic, self.on_joints, qos_profile_sensor_data),
            node.create_subscription(JointJog, jog_topic, self.on_jog, 1),
            node.create_subscription(Int8, '/servo_node/status', self.on_status, 1),
        ]
        for topic in ('/arm_controller/controller_state',):
            self.subscriptions.append(node.create_subscription(
                JointTrajectoryControllerState, topic,
                lambda msg, source=topic: self.on_controller(msg, source), 1))
        self.timer = node.create_timer(0.5, self.report)

    def on_jog(self, message):
        now = self.clock()
        self.jog = (now, message)
        if any(abs(v) > 1e-9 for v in message.velocities if math.isfinite(v)):
            self.last_motion = now

    def on_trajectory(self, message):
        now = self.clock()
        current = {}
        if message.points:
            for name in WATCH:
                current[name] = field(message.points[0], 'positions', indexed(message.joint_names, name))
        self.delta = {name: current[name] - self.previous_positions[name]
                      if current.get(name) is not None and self.previous_positions.get(name) is not None
                      else None for name in WATCH}
        self.trajectory_dt = None if self.last_trajectory_at is None else now - self.last_trajectory_at
        self.previous_positions, self.last_trajectory_at = current, now
        self.trajectory = (now, message)

    def on_controller(self, message, source):
        self.states[source] = (self.clock(), message)

    def on_joints(self, message):
        self.joints = (self.clock(), message)

    def on_status(self, message):
        self.status = (self.clock(), message.data)

    def report(self):
        now = self.clock()
        # Silent at rest. Keep one second of stop-tail diagnostics after input.
        if now - self.last_motion > 1.0:
            return
        state_source, state_sample = max(self.states.items(), key=lambda pair: pair[1][0]) if self.states else ('NA', None)
        trajectory = self.trajectory[1] if self.trajectory else None
        controller = state_sample[1] if state_sample else None
        joints = self.joints[1] if self.joints else None
        jog = self.jog[1] if self.jog else None
        first = trajectory.points[0] if trajectory and trajectory.points else None
        last = trajectory.points[-1] if first else None
        reference, ref_schema = state_point(controller, 'reference', 'desired')
        actual, actual_schema = state_point(controller, 'feedback', 'actual')
        age = lambda sample: fmt(now - sample[0]) if sample else 'NA'
        for name in WATCH:
            ti = indexed(trajectory.joint_names, name) if trajectory else None
            ci = indexed(controller.joint_names, name) if controller else None
            ji = indexed(joints.name, name) if joints else None
            vi = indexed(jog.joint_names, name) if jog else None
            velocity = field(jog, 'velocities', vi)
            expected = None if velocity is None or self.trajectory_dt is None else velocity * self.trajectory_dt
            duration = first.time_from_start if first else None
            last_duration = last.time_from_start if last else None
            self.node.get_logger().info(
                f'DIAG_TRAJ joint={name} traj_index={ti} jog_v={fmt(velocity)} '
                f'p0={fmt(field(first, "positions", ti))} v0={fmt(field(first, "velocities", ti))} '
                f'dp_msg={fmt(self.delta.get(name))} expected_dp={fmt(expected)} '
                f'rx_dt={fmt(self.trajectory_dt)} t0={duration.sec if duration else "NA"}:'
                f'{duration.nanosec if duration else "NA"} '
                f'plast={fmt(field(last, "positions", ti))} '
                f'tlast={last_duration.sec if last_duration else "NA"}:'
                f'{last_duration.nanosec if last_duration else "NA"} '
                f'desired={fmt(field(reference, "positions", ci))} '
                f'actual={fmt(field(actual, "positions", ci))} '
                f'controller_output={fmt(field(getattr(controller, "output", None), "positions", ci))} '
                f'error={fmt(field(getattr(controller, "error", None), "positions", ci))} '
                f'js_p={fmt(field(joints, "position", ji))} js_v={fmt(field(joints, "velocity", ji))} '
                f'source={state_source} schema={ref_schema}/{actual_schema} '
                f'age_traj={age(self.trajectory)} age_ctrl={age(state_sample)} age_js={age(self.joints)} '
                f'servo_status={self.status[1] if self.status else "NA"} age_status={age(self.status)}')
