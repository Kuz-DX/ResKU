#!/usr/bin/env python3
"""Run reviewed joint paths from L2 + face-button requests via the mode manager."""
import json
import math
from pathlib import Path
import sys
import time
import uuid

source = Path(__file__).resolve().parents[2]/'tool_change_min'
if (source/'tool_change_min/kinematics.py').is_file():
    sys.path.insert(0, str(source))

import numpy as np
import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
try:
    from tool_change_min.kinematics import ArmKinematics, ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ArmKinematics, ARM_JOINTS
from arm_mode_core import TCP_ENTRY_TOLERANCE


def load_path(path, model, margin=.02):
    """Load captured points with recorded or manager-limited execution timing."""
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f'{path}: YAML root must be a mapping')
    if data.get('execution_enabled') is not True:
        return None
    if data.get('joint_names') != list(ARM_JOINTS):
        raise ValueError(f'{path}: invalid joint order')
    timing_mode = data.get('timing_mode', 'recorded')
    if timing_mode not in ('recorded', 'joint_limits'):
        raise ValueError(f'{path}: timing_mode must be recorded or joint_limits')
    segment_limits = data.get('joint_speed_limits_rad_s')
    if segment_limits is not None:
        if (timing_mode != 'joint_limits' or not isinstance(segment_limits, list)
                or len(segment_limits) != len(ARM_JOINTS)
                or not all(type(v) in (int, float) and math.isfinite(v)
                           and 0 < v <= .12 for v in segment_limits)):
            raise ValueError(f'{path}: invalid joint_speed_limits_rad_s for joint_limits timing')
    raw = data.get('points')
    if not isinstance(raw, list) or len(raw) < 2 or len(raw) > 1000:
        raise ValueError(f'{path}: expected 2..1000 points')
    times, points = [], []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f'{path}: point {index} must be a mapping')
        t = item.get('time_from_start_sec')
        q = item.get('positions_rad')
        if (not isinstance(t, (int, float)) or isinstance(t, bool) or not math.isfinite(t)
                or not isinstance(q, list) or len(q) != len(ARM_JOINTS)
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                           and math.isfinite(v) for v in q)):
            raise ValueError(f'{path}: invalid point {index}')
        q = np.asarray(q, dtype=float)
        if not model.within_limits(q, margin):
            raise ValueError(f'{path}: point {index} violates joint margin')
        times.append(float(t))
        points.append(q)
    if times[0] != 0. or any(not a < b for a, b in zip(times, times[1:])):
        raise ValueError(f'{path}: times must start at 0 and increase')
    durations = [b-a for a, b in zip(times, times[1:])]
    if timing_mode == 'recorded' and any(d >= 120. for d in durations):
        raise ValueError(f'{path}: recorded segment times must be <120s')
    if timing_mode == 'joint_limits':
        # Request the minimum positive duration accepted by the timed path
        # pipeline. ModeCore._next_waypoint stretches each segment to the
        # slowest joint's travel/speed limit, preserving synchronized motion.
        # Acceleration, jerk and feedback pacing remain owned by the manager.
        # Capture pauses must not become commanded travel times.
        durations = [.02]*len(durations)
        if segment_limits is not None:
            # Slow the whole taught segment together. This retains waypoint
            # geometry and the existing manager acceleration/jerk/lead guards.
            # Reverse paths reuse these durations in reverse order.
            durations = [max(.02, float(np.max(np.abs(b-a)/segment_limits)))
                         for a, b in zip(points, points[1:])]
            if any(d >= 120. for d in durations):
                raise ValueError(f'{path}: speed-limited segment time must be <120s')
    after = data.get('after_path')
    if after is not None:
        if (not isinstance(after, dict) or
                set(after) != {'ee_close_rad', 'return_to_start'} or
                after['return_to_start'] is not True or
                type(after['ee_close_rad']) not in (int, float) or
                not math.isfinite(after['ee_close_rad']) or
                not -2.351592547829 <= after['ee_close_rad'] <= 2.839398438376):
            raise ValueError(f'{path}: invalid close-and-return action')
    return dict(start=points[0], waypoints=points[1:],
                durations=durations, total=sum(durations),
                after_path=after)


def reverse_path(path):
    """Retrace all taught points, excluding any automatically planned entry."""
    points = [path['start'], *path['waypoints']]
    return dict(start=points[-1], waypoints=list(reversed(points[:-1])),
                durations=list(reversed(path['durations'])), total=path['total'])


def requested_path(axes, buttons, previous, l2_axis=2, threshold=-.5,
                   first_button=2, third_button=0, second_button=1):
    """Return a new 0..2 path request, plus current combo edge state."""
    # Linux PlayStation mapping: triangle=2, circle=1, X=0; square=3 is HOME.
    path_buttons = (first_button, second_button, third_button)
    valid = (len(axes) > l2_axis and len(buttons) > max(path_buttons)
             and all(math.isfinite(v) for v in axes)
             and all(v in (0, 1) for v in buttons))
    current = [False]*3
    if valid and axes[l2_axis] < threshold:
        current = [bool(buttons[index]) for index in path_buttons]
    edges = [i for i, held in enumerate(current) if held and not previous[i]]
    return (edges[0] if len(edges) == 1 else None), current


def execution_plan(path, measured, tolerance=.03, max_delta=.10, approach_speed=.05):
    """Prepend a bounded, timed approach when feedback is near the taught start."""
    error = float(np.max(np.abs(np.asarray(measured)-path['start'])))
    if error > max_delta:
        raise ValueError(
            'start mismatch: %.4f rad > %.4f rad automatic approach limit' %
            (error, max_delta))
    waypoints = list(path['waypoints'])
    durations = list(path['durations'])
    approach_duration = 0.
    if error > tolerance or not waypoints:
        approach_duration = max(.5, error/approach_speed)
        waypoints.insert(0, path['start'])
        durations.insert(0, approach_duration)
    return dict(start=np.asarray(measured, dtype=float).copy(),
                waypoints=waypoints, durations=durations,
                total=path['total']+approach_duration, start_error=error)


def nearest_path_suffix(path, measured):
    """Resume forward from the closest point on a taught joint-space segment."""
    points = [np.asarray(path['start'], dtype=float), *path['waypoints']]
    measured = np.asarray(measured, dtype=float)
    candidates = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        delta = end-start
        length_sq = float(np.dot(delta, delta))
        fraction = (float(np.clip(np.dot(measured-start, delta)/length_sq, 0., 1.))
                    if length_sq > 1e-12 else 0.)
        projected = start+fraction*delta
        candidates.append((float(np.max(np.abs(projected-measured))),
                           -index-fraction, index, fraction, projected))
    _, _, index, fraction, projected = min(candidates, key=lambda item: item[:2])
    if fraction >= 1.-1e-9:
        index += 1
        projected = points[index]
        waypoints = points[index+1:]
        durations = list(path['durations'][index:])
    else:
        waypoints = points[index+1:]
        durations = [max(.02, path['durations'][index]*(1.-fraction)),
                     *path['durations'][index+1:]]
    return dict(start=projected, waypoints=waypoints, durations=durations,
                total=sum(durations), entry_index=index)


def collision_check_budget(sample_count, minimum_timeout, service_rate=50.):
    """Allow serial MoveIt validity calls enough time, while keeping a hard ceiling."""
    if sample_count < 1 or service_rate <= 0.:
        raise ValueError('invalid collision-check budget input')
    return min(120., max(float(minimum_timeout), sample_count/service_rate+2.))


def execution_timeout(path, measured, speed_limits):
    """Account for manager speed retiming and ramp/settling overhead.

    The manager independently enforces joint stall and waypoint timeouts.
    """
    speeds = np.asarray(speed_limits, dtype=float)
    if speeds.shape != (6,) or not np.all(np.isfinite(speeds)) or np.any(speeds <= 0):
        raise ValueError('invalid manager path speed limits')
    start = np.asarray(measured, dtype=float)
    duration = 0.
    for goal, nominal in zip(path['waypoints'], path['durations']):
        duration += max(nominal, float(np.max(np.abs(goal-start)/speeds)))
        start = goal
    return duration+5.*len(path['waypoints'])+30.


def planned_entry(path, trajectory, measured, minimum_duration=.02):
    """Convert a MoveIt joint trajectory into a managed path prefix."""
    names = list(trajectory.joint_names)
    if len(names) != len(set(names)) or any(name not in names for name in ARM_JOINTS):
        raise ValueError('MoveIt entry plan has invalid joint names')
    indices = [names.index(name) for name in ARM_JOINTS]
    waypoints, durations = [], []
    previous = np.asarray(measured, dtype=float)
    previous_time = 0.
    for point in trajectory.points:
        if len(point.positions) != len(names):
            raise ValueError('MoveIt entry plan has invalid positions')
        current = np.asarray([point.positions[i] for i in indices], dtype=float)
        stamp = float(point.time_from_start.sec) + float(point.time_from_start.nanosec)*1e-9
        if not np.all(np.isfinite(current)) or not math.isfinite(stamp) or stamp < previous_time:
            raise ValueError('MoveIt entry plan has invalid timing')
        if float(np.max(np.abs(current-previous))) > 1e-9:
            waypoints.append(current)
            durations.append(max(minimum_duration, stamp-previous_time))
            previous = current
        previous_time = stamp
    if not waypoints:
        raise ValueError('MoveIt entry plan is empty')
    if float(np.max(np.abs(waypoints[-1]-path['start']))) > .01:
        raise ValueError('MoveIt entry plan did not reach the taught start')
    waypoints[-1] = np.asarray(path['start'], dtype=float)
    waypoints.extend(path['waypoints'])
    durations.extend(path['durations'])
    return dict(start=np.asarray(measured, dtype=float).copy(),
                waypoints=waypoints, durations=durations,
                total=sum(durations), start_error=float(np.max(
                    np.abs(np.asarray(measured)-path['start']))))


def isolate_shoulder_zero_crossings(path, half_width=.08, isolated_speed=.015,
                                     shoulder_index=1):
    """Stage a full shoulder zero crossing while every other joint is stopped."""
    if not 0 < half_width <= .25 or not 0 < isolated_speed <= .05:
        raise ValueError('invalid shoulder isolation parameters')
    segment_start = np.asarray(path['start'], dtype=float)
    output, durations, stops = [], [], []
    for segment_goal, duration in zip(path['waypoints'], path['durations']):
        goal = np.asarray(segment_goal, dtype=float)
        a, b = segment_start[shoulder_index], goal[shoulder_index]
        ascending = a < 0. < b
        descending = b < 0. < a
        if not (ascending or descending):
            output.append(goal)
            durations.append(duration)
            stops.append(False)
            segment_start = goal
            continue
        entry_value, exit_value = ((max(a, -half_width), min(b, half_width))
                                   if ascending else
                                   (min(a, half_width), max(b, -half_width)))
        entry_fraction = (entry_value-a)/(b-a)
        exit_fraction = (exit_value-a)/(b-a)
        entry = segment_start+entry_fraction*(goal-segment_start)
        nominal_exit = segment_start+exit_fraction*(goal-segment_start)
        isolated_exit = entry.copy()
        isolated_exit[shoulder_index] = exit_value
        phases = (
            (entry, max(.02, duration*entry_fraction), True),
            (isolated_exit, max(.5, abs(exit_value-entry_value)/isolated_speed), True),
            (nominal_exit, max(.5, duration*(exit_fraction-entry_fraction)), True),
            (goal, max(.02, duration*(1.-exit_fraction)), False),
        )
        for point, phase_duration, stop in phases:
            if output and np.max(np.abs(point-output[-1])) < 1e-12:
                stops[-1] = stops[-1] or stop
                continue
            output.append(point)
            durations.append(float(phase_duration))
            stops.append(stop)
        segment_start = goal
    if len(output) > 1000:
        raise ValueError('shoulder isolation expands path beyond 1000 waypoints')
    return dict(path, waypoints=output, durations=durations,
                stop_at_waypoints=stops, total=sum(durations))


class ToolTrajectoryJoy(Node):
    def __init__(self):
        super().__init__('tool_trajectory_joy')
        def param(name, default):
            return self.declare_parameter(name, default).value
        hardware = str(param('hardware_config', ''))
        description = str(param('robot_description', ''))
        self.model = ArmKinematics('', hardware, description=description)
        directory = Path(str(param('trajectory_dir', '')))
        self.margin = float(param('limit_margin_rad', .02))
        self.start_tolerance = float(param('start_tolerance_rad', .03))
        self.max_start_delta = float(param('max_start_delta_rad', .10))
        self.approach_speed = float(param('start_approach_speed_rad_s', .05))
        self.collision_step = float(param('collision_step_rad', .005))
        self.validation_rmd_drift = float(param('validation_rmd_drift_rad', .005))
        self.collision_timeout = float(param('collision_timeout_sec', 10.))
        self.planning_timeout = float(param('entry_planning_timeout_sec', 8.))
        self.planning_attempts = int(param('entry_planning_attempts', 3))
        self.planning_velocity_scale = float(param('entry_planning_velocity_scale', .08))
        # 0.25 rad/s^2 from joint_limits.yaml * 0.4 = 0.10 rad/s^2.  Keep the
        # MoveIt entry plan aligned with the loaded-arm runtime path limiter.
        self.planning_acceleration_scale = float(param('entry_planning_acceleration_scale', .4))
        self.shoulder_isolation_enabled = bool(param('shoulder_isolation_enabled', True))
        self.shoulder_isolation_half_width = float(param(
            'shoulder_isolation_half_width_rad', .08))
        self.shoulder_isolation_speed = float(param(
            'shoulder_isolation_speed_rad_s', .015))
        self.l2_axis = int(param('l2_axis', 2))
        self.l2_threshold = float(param('l2_pressed_below', -.5))
        # Linux PlayStation: path 0=triangle (2), 1=circle (1), 2=X (0).
        self.first_button = int(param('first_path_button', 2))
        self.second_button = int(param('second_path_button', 1))
        self.third_button = int(param('third_path_button', 0))
        if (not 0 < self.margin <= .1 or not 0 < self.start_tolerance <= self.max_start_delta <= .25
                or not 0 < self.approach_speed <= .12
                or not 0 < self.collision_step <= .01
                or not .001 <= self.validation_rmd_drift <= self.collision_step
                or not .1 < self.collision_timeout <= 60.
                or not .5 <= self.planning_timeout <= 30. or not 1 <= self.planning_attempts <= 10
                or not 0 < self.planning_velocity_scale <= .2
                or not 0 < self.planning_acceleration_scale <= 1.
                or not 0 < self.shoulder_isolation_half_width <= .25
                or not 0 < self.shoulder_isolation_speed <= .05
                or self.l2_axis < 0 or self.first_button < 0
                or self.third_button < 0 or self.second_button < 0
                or len({self.first_button, self.second_button, self.third_button}) != 3
                or 3 in (self.first_button, self.second_button, self.third_button)):
            raise ValueError('invalid tool trajectory joystick parameters')
        self.paths = {}
        for path_id in range(3):
            path = directory/f'tool{path_id}_trajectory.yaml'
            loaded = load_path(path, self.model, self.margin)
            if loaded is not None:
                self.paths[path_id] = loaded
        self.publisher = self.create_publisher(String, '/arm/input/tool_auto', 1)
        self.diagnostics = self.create_publisher(String, '/arm/tool_trajectory/status', 1)
        self.mode = self.create_client(Trigger, '/arm/set_mode/tool_auto')
        self.cancel = self.create_client(Trigger, '/arm/cancel')
        self.collision = self.create_client(GetStateValidity, '/check_state_validity')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/arm/control_status', self.on_status, qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 1)
        self.create_subscription(Joy, '/joy', self.on_joy, 1)
        self.docking_command = self.create_publisher(Bool, '/arm/docking', 10)
        docking_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/arm/docked', self.on_docked, docking_qos)
        self.create_subscription(String, '/arm/docking_status', self.on_docking_status,
                                 docking_qos)
        self.ui_held = {(path_id, desired): False for path_id in range(3)
                        for desired in (False, True)}
        for path_id in range(3):
            self.create_subscription(
                Bool, f'/arm/tool{path_id}_docking',
                lambda msg, p=path_id: self.on_ui_request(msg, p, True), 10)
            self.create_subscription(
                Bool, f'/arm/tool{path_id}_release',
                lambda msg, p=path_id: self.on_ui_request(msg, p, False), 10)
        self.docked_state = None
        self.docking_status = 'UNKNOWN'
        self.docked_feedback_sequence = 0
        self.actuation_timeout = float(param('docking_feedback_timeout_sec', 10.))
        if not .5 <= self.actuation_timeout <= 60.:
            raise ValueError('docking_feedback_timeout_sec must be in [0.5, 60]')
        self.create_timer(.02, self.tick)
        self.status, self.status_time = {}, -math.inf
        self.full_state, self.state_time = None, -math.inf
        self.combos = [False]*3
        self.reset()
        enabled = ','.join(map(str, self.paths)) or 'none'
        self.get_logger().info(f'L2 tool trajectories enabled: {enabled}')

    def reset(self):
        self.stage = 'idle'
        self.path_id = self.path = self.snapshot = self.original_epoch = None
        self.requests, self.future = [], None
        self.request_index = 0
        self.started = None
        self.execution_id = None
        self.entry_path = None
        self.requested_docked_state = None
        self.use_nearest_entry = False
        self.command_feedback_sequence = None
        self.actuation_deadline = None
        self.collision_deadline = None
        self.sequence_phase = 'approach'

    def report(self, state, reason):
        self.diagnostics.publish(String(data=json.dumps(dict(
            state=state, reason=reason, path_id=self.path_id))))
        # rclpy associates severity with the Python call site.  Calling a
        # dynamically selected logger method from one site raises when a later
        # message changes severity, so keep the sites distinct.
        if state == 'failed':
            self.get_logger().error(reason)
        else:
            self.get_logger().info(reason)

    def fail(self, reason):
        if (self.execution_id is not None and self.status.get('mode') == 'tool_auto'
                and self.status.get('execution_id') == self.execution_id
                and self.status.get('epoch') is not None):
            self.publisher.publish(String(data=json.dumps(dict(
                epoch=self.status['epoch'], execution_id=self.execution_id, cancel=True))))
        elif self.stage in ('request_mode', 'wait_mode') and self.cancel.service_is_ready():
            self.cancel.call_async(Trigger.Request())
        self.report('failed', reason)
        self.reset()

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError
            self.status, self.status_time = value, time.monotonic()
        except (ValueError, TypeError):
            self.status, self.status_time = {}, -math.inf

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if (len(msg.name) != len(msg.position) or len(values) != len(msg.name)
                or not all(j in values for j in ARM_JOINTS)
                or not all(math.isfinite(v) for v in msg.position)):
            self.full_state, self.state_time = None, -math.inf
            return
        self.full_state, self.state_time = values, time.monotonic()

    def on_joy(self, msg):
        request, self.combos = requested_path(
            msg.axes, msg.buttons, self.combos, self.l2_axis,
            self.l2_threshold, self.first_button, self.third_button, self.second_button)
        if request is None:
            return
        self.request_path(request, nearest_entry=True)

    def on_ui_request(self, msg, path_id, desired_docked):
        key = (path_id, desired_docked)
        rising = bool(msg.data) and not self.ui_held[key]
        self.ui_held[key] = bool(msg.data)
        if rising:
            self.request_path(path_id, desired_docked, nearest_entry=True)

    def request_path(self, request, desired_docked=None, nearest_entry=False):
        if self.stage != 'idle':
            self.report('ignored', 'tool trajectory request ignored while another path is active')
            return
        self.requested_docked_state = desired_docked
        self.use_nearest_entry = nearest_entry
        close_and_return = self.paths.get(request, {}).get('after_path') is not None
        if close_and_return and desired_docked is False:
            self.path_id = request
            self.report('blocked', 'this path defines ee close and return, not tool release')
            self.reset()
            return
        if desired_docked is not None and not close_and_return:
            if self.docked_state is None:
                self.path_id = request
                self.report('blocked', 'docking mechanism state is UNKNOWN; initialize it first')
                self.path_id = None
                self.requested_docked_state = None
                return
            if self.docked_state == desired_docked:
                self.path_id = request
                state = 'docked' if desired_docked else 'released'
                self.report('completed', f'tool already {state}; servo and arm were not moved')
                self.reset()
                return
        if request not in self.paths:
            self.path_id = request
            self.report('blocked', f'tool path {request} is not execution-enabled')
            self.path_id = None
            return
        now = time.monotonic()
        s = self.status
        available = (now-self.status_time < .3 and now-self.state_time < .3
                     and self.full_state is not None and s.get('ready')
                     and s.get('startup_complete') and s.get('focus') == 'arm'
                     and s.get('entry_alignment_complete', True)
                     and s.get('mode') != 'fault' and not s.get('transitioning')
                     and s.get('requested', s.get('mode')) == s.get('mode'))
        if not available:
            self.path_id = request
            self.report('blocked', 'fresh ready arm focus is required: '+
                'mode=%s ready=%s focus=%s startup=%s alignment=%s transitioning=%s '
                'status_age=%.3fs feedback_age=%.3fs; %s' %
                (s.get('mode'), s.get('ready'), s.get('focus'), s.get('startup_complete'),
                 s.get('entry_alignment_complete'), s.get('transitioning'),
                 now-self.status_time, now-self.state_time, s.get('blocker') or s.get('reason', '')))
            self.path_id = None
            return
        if s.get('mode') == 'hold':
            self.begin(request)
        elif self.cancel.service_is_ready():
            self.path_id = request
            self.future = self.cancel.call_async(Trigger.Request())
            self.stage, self.started = 'request_hold', now
            self.report('stopping', f'requesting HOLD before tool path {request}')
        else:
            self.path_id = request
            self.report('blocked', 'arm cancel/HOLD service unavailable')
            self.reset()

    def healthy_hold(self, now):
        s = self.status
        if (now-self.status_time >= .3 or now-self.state_time >= .3
                or self.full_state is None or not s.get('ready') or not s.get('startup_complete')
                or not s.get('entry_alignment_complete', True)
                or s.get('focus') != 'arm' or s.get('mode') != 'hold' or s.get('transitioning')
                or s.get('requested', s.get('mode')) != 'hold'):
            raise RuntimeError('fresh stopped HOLD with arm focus is required')
        return s

    def begin(self, path_id, planned_path=None):
        now = time.monotonic()
        try:
            s = self.healthy_hold(now)
            measured = np.asarray([self.full_state[j] for j in ARM_JOINTS])
            if planned_path is None:
                source_path = (nearest_path_suffix(self.paths[path_id], measured)
                               if self.use_nearest_entry else self.paths[path_id])
                self.entry_path = source_path
                error = float(np.max(np.abs(measured-source_path['start'])))
                if error > self.max_start_delta:
                    self.request_entry_plan(path_id, source_path, measured, error, s)
                    return
                path = execution_plan(source_path, measured, self.start_tolerance,
                                      self.max_start_delta, self.approach_speed)
            else:
                path = planned_path
            if self.shoulder_isolation_enabled and not path.get('cartesian_line', False):
                path = isolate_shoulder_zero_crossings(
                    path, self.shoulder_isolation_half_width,
                    self.shoulder_isolation_speed)
            if not self.collision.service_is_ready() or not self.mode.service_is_ready():
                raise RuntimeError('tool mode or MoveIt collision service unavailable')
            self.path_id, self.path = path_id, path
            self.snapshot = dict(self.full_state)
            self.original_epoch = s['epoch']
            starts = [measured, *path['waypoints'][:-1]]
            self.requests = []
            samples = 0
            if 'ee_target_rad' in path:
                ee_start = self.snapshot['ee_joint']
                count = max(1, math.ceil(abs(path['ee_target_rad']-ee_start)/.02))
                if count > 1000:
                    raise ValueError('ee collision sampling budget exceeded')
                states = []
                for ee in np.linspace(ee_start, path['ee_target_rad'], count+1):
                    values = dict(self.snapshot, ee_joint=float(ee))
                    # MoveIt derives rack positions from the ee mimic joint.
                    for name in ('rack_left_joint', 'rack_right_joint'):
                        values.pop(name, None)
                    states.append(values)
            else:
                states = []
                for start, target in zip(starts, path['waypoints']):
                    count = max(1, math.ceil(float(np.max(np.abs(target-start)))/self.collision_step))
                    samples += count+1
                    if samples > 20000:
                        raise ValueError('collision sampling budget exceeded')
                    for q in np.linspace(start, target, count+1):
                        values = dict(self.snapshot)
                        values.update(zip(ARM_JOINTS, map(float, q)))
                        states.append(values)
            for values in states:
                req = GetStateValidity.Request()
                req.robot_state.joint_state.name = list(values)
                req.robot_state.joint_state.position = list(values.values())
                req.robot_state.is_diff = True
                self.requests.append(req)
            self.stage, self.started, self.request_index = 'checking', now, 0
            budget = collision_check_budget(len(self.requests), self.collision_timeout)
            self.collision_deadline = now+budget
            self.future = self.collision.call_async(self.requests[0])
            approach = ('; approaching taught start from %.4f rad' % path['start_error']
                        if path['start_error'] > self.start_tolerance else '')
            entry = getattr(self, 'entry_path', None)
            resume = ('; resuming from segment %d' % (entry['entry_index']+1)
                      if (getattr(self, 'use_nearest_entry', False) and isinstance(entry, dict)
                          and 'entry_index' in entry and 'ee_target_rad' not in path)
                      else '')
            self.report('checking',
                        f'checking tool path {path_id} ({len(self.requests)} samples, '
                        f'{budget:.1f}s budget){resume}{approach}')
        except (ValueError, RuntimeError, KeyError) as exc:
            self.path_id = path_id
            self.begin_failed(str(exc))

    def request_entry_plan(self, path_id, path, measured, error, status):
        if not self.planner.service_is_ready():
            raise RuntimeError('MoveIt entry planner unavailable')
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_timeout
        request.max_velocity_scaling_factor = self.planning_velocity_scale
        request.max_acceleration_scaling_factor = self.planning_acceleration_scale
        request.start_state.joint_state.name = list(self.full_state)
        request.start_state.joint_state.position = list(self.full_state.values())
        request.start_state.is_diff = True
        goal = Constraints()
        for name, position in zip(ARM_JOINTS, path['start']):
            goal.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(position),
                tolerance_above=.005, tolerance_below=.005, weight=1.))
        request.goal_constraints = [goal]
        self.path_id = path_id
        self.snapshot = dict(self.full_state)
        self.original_epoch = status['epoch']
        self.future = self.planner.call_async(GetMotionPlan.Request(motion_plan_request=request))
        self.stage, self.started = 'planning_entry', time.monotonic()
        self.report('planning',
                    f'planning collision-free entry to tool path {path_id} from {error:.4f} rad')

    def begin_failed(self, reason):
        self.report('blocked', reason)
        self.reset()

    def path_completed(self):
        after = self.paths[self.path_id].get('after_path')
        if after is not None:
            if self.sequence_phase == 'approach':
                self.sequence_phase = 'closing'
                measured = np.asarray([self.full_state[j] for j in ARM_JOINTS])
                self.report('closing', 'approach complete; checking ee close motion')
                self.begin(self.path_id, dict(
                    start=measured, waypoints=[measured], durations=[.5],
                    total=.5, start_error=0., ee_target_rad=after['ee_close_rad']))
            elif self.sequence_phase == 'closing':
                self.sequence_phase = 'returning'
                # Use the full taught reverse path even if UI entry started midway.
                path = execution_plan(reverse_path(self.paths[self.path_id]),
                                      [self.full_state[j] for j in ARM_JOINTS],
                                      self.start_tolerance, self.max_start_delta,
                                      self.approach_speed)
                self.report('returning', 'ee close confirmed; checking reverse path')
                self.begin(self.path_id, path)
            else:
                self.report('completed', f'tool path {self.path_id} closed ee and returned; HOLD')
                self.reset()
            return
        if self.requested_docked_state is not None:
            if (self.docking_command.get_subscription_count() == 0
                    or self.count_publishers('/arm/docked') == 0):
                raise RuntimeError('Arduino command/status connection unavailable')
            self.snapshot = dict(self.full_state)
            self.command_feedback_sequence = self.docked_feedback_sequence
            self.actuation_deadline = time.monotonic()+self.actuation_timeout
            self.send_docking_pulse(self.requested_docked_state)
            self.stage = 'wait_docked'
            action = 'dock' if self.requested_docked_state else 'release'
            self.report('actuating', f'Arduino {action} pulse sent; waiting for /arm/docked')
            return
        self.report('completed', f'tool path {self.path_id} completed; HOLD')
        self.reset()

    def on_docked(self, msg):
        # Compatibility fallback for an older bridge. The authoritative status
        # prevents a transient-local stale Bool from overwriting UNKNOWN.
        if not hasattr(self, 'docking_status') or self.docking_status not in (
                'UNKNOWN', 'MOVING', 'ERROR'):
            self.docked_state = bool(msg.data)
            self.docked_feedback_sequence += 1

    def on_docking_status(self, msg):
        self.docking_status = msg.data
        if msg.data == 'DOCKED':
            self.docked_state = True
            self.docked_feedback_sequence += 1
        elif msg.data == 'RELEASED':
            self.docked_state = False
            self.docked_feedback_sequence += 1
        elif msg.data in ('UNKNOWN', 'MOVING', 'ERROR'):
            self.docked_state = None

    def send_docking_pulse(self, desired):
        # This is a desired-state command. Arduino reports completion after its
        # timed servo motion; it is not physical latch-sensor confirmation.
        self.docking_command.publish(Bool(data=desired))

    def snapshot_delta(self):
        if self.full_state is None or self.snapshot is None:
            return None
        if self.full_state.keys() != self.snapshot.keys():
            return None
        return {joint: abs(self.full_state[joint]-value)
                for joint, value in self.snapshot.items()}

    def snapshot_tolerances(self, validation=False):
        tolerances = dict(zip(ARM_JOINTS, TCP_ENTRY_TOLERANCE))
        # ee_joint is a separate DYNAMIXEL velocity axis and is not commanded
        # by a six-joint arm path.  Its stopped feedback has the same encoder
        # hunting seen on the arm DYNAMIXEL axes, so do not apply the 0.001-rad
        # RMD fallback to it.
        tolerances['ee_joint'] = .02
        if validation:
            for joint in ARM_JOINTS[1:4]:
                tolerances[joint] = self.validation_rmd_drift
        return tolerances

    def unchanged(self, validation=False):
        delta = ToolTrajectoryJoy.snapshot_delta(self)
        if delta is None:
            return False
        tolerances = ToolTrajectoryJoy.snapshot_tolerances(self, validation)
        return all(change <= tolerances.get(joint, .001)
                   for joint, change in delta.items())

    def snapshot_change_reason(self, validation=False):
        delta = ToolTrajectoryJoy.snapshot_delta(self)
        if not delta:
            return 'joint feedback unavailable or joint set changed'
        tolerances = ToolTrajectoryJoy.snapshot_tolerances(self, validation)
        joint = max(delta, key=lambda name: delta[name]/tolerances.get(name, .001))
        return (f'largest normalized change {joint}={delta[joint]:.6f} rad '
                f'(limit {tolerances.get(joint, .001):.6f})')

    def tick(self):
        if self.stage == 'idle':
            return
        now = time.monotonic()
        try:
            if self.stage == 'wait_docked':
                self.healthy_hold(now)
                if not self.unchanged():
                    raise RuntimeError('arm departed final docking-preparation pose')
                if now >= self.actuation_deadline:
                    raise TimeoutError('Arduino docking completion timeout')
                if (self.docked_feedback_sequence > self.command_feedback_sequence
                        and self.docked_state == self.requested_docked_state):
                    state = 'docked' if self.requested_docked_state else 'released'
                    self.report(state, f'Arduino {state} state confirmed; remain HOLD')
                    self.reset()
                return
            if self.stage == 'request_hold':
                if not self.future.done():
                    if now-self.started > 3.:
                        raise TimeoutError('HOLD request timed out')
                    return
                result = self.future.result()
                if result is None or not result.success:
                    raise RuntimeError('HOLD request rejected')
                self.stage = 'wait_hold'
                return
            if self.stage == 'wait_hold':
                if (self.status.get('mode') == 'hold' and not self.status.get('transitioning')):
                    path_id = self.path_id
                    self.begin(path_id)
                    return
                if now-self.started > 3.:
                    raise TimeoutError('arm did not reach HOLD')
                return
            if self.stage == 'planning_entry':
                s = self.healthy_hold(now)
                if s.get('epoch') != self.original_epoch:
                    raise RuntimeError('arm ownership changed during entry planning')
                if not self.unchanged(validation=True):
                    raise RuntimeError('robot moved during entry planning; '
                                       + self.snapshot_change_reason(validation=True))
                if now-self.started > self.planning_timeout+1.:
                    raise TimeoutError('MoveIt entry planning timed out')
                if not self.future.done():
                    return
                result = self.future.result()
                if (result is None or
                        result.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS):
                    code = ('none' if result is None else
                            result.motion_plan_response.error_code.val)
                    raise RuntimeError(f'MoveIt entry planning failed ({code})')
                path_id = self.path_id
                plan = planned_entry(self.entry_path,
                                     result.motion_plan_response.trajectory.joint_trajectory,
                                     [self.snapshot[j] for j in ARM_JOINTS])
                self.begin(path_id, plan)
                return
            if self.stage == 'checking':
                s = self.healthy_hold(now)
                if s.get('epoch') != self.original_epoch:
                    raise RuntimeError('arm ownership changed during path validation')
                if not self.unchanged(validation=True):
                    raise RuntimeError(
                        'robot state changed beyond stopped-feedback tolerance during '
                        'path validation; '+self.snapshot_change_reason(validation=True))
                if now > self.collision_deadline:
                    raise TimeoutError('tool path collision check timed out')
                if not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.valid:
                    pairs = sorted({' <-> '.join(sorted((c.contact_body_1, c.contact_body_2)))
                                    for c in getattr(result, 'contacts', [])})
                    detail = ', '.join(pairs[:6]) or 'no contacts; check joint bounds/scene'
                    raise ValueError('tool path collision/state invalid at sample %d/%d: %s' %
                                     (self.request_index+1, len(self.requests), detail))
                self.request_index += 1
                if self.request_index < len(self.requests):
                    self.future = self.collision.call_async(self.requests[self.request_index])
                    return
                self.future = self.mode.call_async(Trigger.Request())
                self.stage = 'request_mode'
                self.started = now
                return
            if self.stage == 'request_mode':
                if not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.success:
                    raise RuntimeError('tool_auto mode request rejected')
                self.stage = 'wait_mode'
                return
            if self.stage == 'wait_mode':
                s = self.status
                if now-self.status_time >= .3 or now-self.state_time >= .3:
                    raise RuntimeError('feedback became stale before path execution')
                if s.get('mode') != 'tool_auto' or s.get('transitioning'):
                    if now-self.started > self.collision_timeout:
                        raise TimeoutError('tool_auto transition timed out')
                    return
                if not self.unchanged(validation=True):
                    raise RuntimeError('robot moved before path execution; '
                                       + self.snapshot_change_reason(validation=True))
                self.execution_id = 'tool-path-'+uuid.uuid4().hex
                command = dict(epoch=s['epoch'], execution_id=self.execution_id)
                if 'ee_target_rad' in self.path:
                    self.execution_timeout = 65.
                    command['ee_target_rad'] = self.path['ee_target_rad']
                    if 'ee_speed_rad_s' in self.path:
                        command['ee_speed_rad_s'] = self.path['ee_speed_rad_s']
                else:
                    self.execution_timeout = execution_timeout(
                        self.path, [self.full_state[j] for j in ARM_JOINTS],
                        s.get('path_speed_limits_rad_s', [.04]*6))
                    command.update(
                        waypoints=[q.tolist() for q in self.path['waypoints']],
                        durations=self.path['durations'],
                        stop_at_waypoints=self.path.get('stop_at_waypoints'))
                self.publisher.publish(String(data=json.dumps(command)))
                self.stage = 'executing'
                self.started = now
                self.report('executing', f'executing tool path {self.path_id}')
                return
            if self.stage == 'executing':
                s = self.status
                if now-self.status_time >= .3 or now-self.state_time >= .3:
                    raise RuntimeError('feedback became stale during path execution')
                if s.get('last_execution_id') == self.execution_id:
                    if s.get('last_execution_result') != 'succeeded':
                        raise RuntimeError('managed tool path aborted: '
                                           +(s.get('last_execution_reason') or s.get('reason') or 'unknown'))
                    if s.get('mode') == 'hold' and not s.get('transitioning'):
                        self.path_completed()
                        return
                elif s.get('mode') != 'tool_auto':
                    raise RuntimeError('tool path ownership lost')
                if now-self.started > self.execution_timeout:
                    raise TimeoutError('tool path completion timed out')
        except (ValueError, RuntimeError, TimeoutError, KeyError) as exc:
            self.fail(str(exc))


def main():
    rclpy.init()
    node = None
    try:
        node = ToolTrajectoryJoy()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
