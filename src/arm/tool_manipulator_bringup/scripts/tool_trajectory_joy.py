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
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity
try:
    from tool_change_min.kinematics import ArmKinematics, ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ArmKinematics, ARM_JOINTS
from arm_mode_core import TCP_ENTRY_TOLERANCE


def load_path(path, model, margin=.02):
    """Load one captured path; the zero-time point is its required start pose."""
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f'{path}: YAML root must be a mapping')
    if data.get('execution_enabled') is not True:
        return None
    if data.get('joint_names') != list(ARM_JOINTS):
        raise ValueError(f'{path}: invalid joint order')
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
    if times[0] != 0. or any(not a < b or b-a >= 120. for a, b in zip(times, times[1:])):
        raise ValueError(f'{path}: times must start at 0 and increase; each segment must be <120s')
    return dict(start=points[0], waypoints=points[1:],
                durations=[b-a for a, b in zip(times, times[1:])], total=times[-1])


def requested_path(axes, buttons, previous, l2_axis=2, threshold=-.5, first_button=2):
    """Return a new 0..2 path request, plus current combo edge state."""
    valid = (len(axes) > l2_axis and len(buttons) > first_button+2
             and all(math.isfinite(v) for v in axes)
             and all(v in (0, 1) for v in buttons))
    current = [False]*3
    if valid and axes[l2_axis] < threshold:
        current = [bool(buttons[first_button+i]) for i in range(3)]
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
    if error > tolerance:
        approach_duration = max(.5, error/approach_speed)
        waypoints.insert(0, path['start'])
        durations.insert(0, approach_duration)
    return dict(waypoints=waypoints, durations=durations,
                total=path['total']+approach_duration, start_error=error)


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
        self.collision_timeout = float(param('collision_timeout_sec', 10.))
        self.l2_axis = int(param('l2_axis', 2))
        self.l2_threshold = float(param('l2_pressed_below', -.5))
        # This controller's physical button 1 is reported as ROS Joy button 2.
        # Path 0 starts there; the parameter remains overridable per controller.
        self.first_button = int(param('first_path_button', 2))
        if (not 0 < self.margin <= .1 or not 0 < self.start_tolerance <= self.max_start_delta <= .25
                or not 0 < self.approach_speed <= .12
                or not 0 < self.collision_step <= .01 or not .1 < self.collision_timeout <= 60.
                or self.l2_axis < 0 or self.first_button < 0):
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
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/arm/control_status', self.on_status, qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 1)
        self.create_subscription(Joy, '/joy', self.on_joy, 1)
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
        if self.execution_id is not None and self.status.get('epoch') is not None:
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
            self.l2_threshold, self.first_button)
        if request is None:
            return
        if self.stage != 'idle':
            self.report('ignored', 'tool trajectory request ignored while another path is active')
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
                     and s.get('mode') != 'fault' and not s.get('transitioning')
                     and s.get('requested', s.get('mode')) == s.get('mode'))
        if not available:
            self.path_id = request
            self.report('blocked', 'fresh ready arm focus is required')
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
                or s.get('focus') != 'arm' or s.get('mode') != 'hold' or s.get('transitioning')
                or s.get('requested', s.get('mode')) != 'hold'):
            raise RuntimeError('fresh stopped HOLD with arm focus is required')
        return s

    def begin(self, path_id, planned_path=None):
        now = time.monotonic()
        try:
            s = self.healthy_hold(now)
            measured = np.asarray([self.full_state[j] for j in ARM_JOINTS])
            path = (execution_plan(self.paths[path_id], measured, self.start_tolerance,
                                   self.max_start_delta, self.approach_speed)
                    if planned_path is None else planned_path)
            if not self.collision.service_is_ready() or not self.mode.service_is_ready():
                raise RuntimeError('tool mode or MoveIt collision service unavailable')
            self.path_id, self.path = path_id, path
            self.snapshot = dict(self.full_state)
            self.original_epoch = s['epoch']
            starts = [measured, *path['waypoints'][:-1]]
            self.requests = []
            samples = 0
            for start, target in zip(starts, path['waypoints']):
                count = max(1, math.ceil(float(np.max(np.abs(target-start)))/self.collision_step))
                samples += count+1
                if samples > 20000:
                    raise ValueError('collision sampling budget exceeded')
                for q in np.linspace(start, target, count+1):
                    req = GetStateValidity.Request()
                    values = dict(self.snapshot)
                    values.update(zip(ARM_JOINTS, map(float, q)))
                    req.robot_state.joint_state.name = list(values)
                    req.robot_state.joint_state.position = list(values.values())
                    req.robot_state.is_diff = True
                    self.requests.append(req)
            self.stage, self.started, self.request_index = 'checking', now, 0
            self.future = self.collision.call_async(self.requests[0])
            approach = ('; approaching taught start from %.4f rad' % path['start_error']
                        if path['start_error'] > self.start_tolerance else '')
            self.report('checking',
                        f'checking tool path {path_id} ({len(self.requests)} samples){approach}')
        except (ValueError, RuntimeError, KeyError) as exc:
            self.path_id = path_id
            self.begin_failed(str(exc))

    def begin_failed(self, reason):
        self.report('blocked', reason)
        self.reset()

    def path_completed(self):
        self.report('completed', f'tool path {self.path_id} completed; HOLD')
        self.reset()

    def unchanged(self):
        if self.full_state is None or self.full_state.keys() != self.snapshot.keys():
            return False
        tolerances = dict(zip(ARM_JOINTS, TCP_ENTRY_TOLERANCE))
        return all(abs(self.full_state[j]-v) <= tolerances.get(j, .001)
                   for j, v in self.snapshot.items())

    def tick(self):
        if self.stage == 'idle':
            return
        now = time.monotonic()
        try:
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
            if self.stage == 'checking':
                s = self.healthy_hold(now)
                if s.get('epoch') != self.original_epoch:
                    raise RuntimeError('arm ownership changed during path validation')
                if not self.unchanged():
                    raise RuntimeError('robot state changed beyond stopped-feedback tolerance during path validation')
                if now-self.started > self.collision_timeout:
                    raise TimeoutError('tool path collision check timed out')
                if not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.valid:
                    raise ValueError('tool path collision/state invalid')
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
                if not self.unchanged():
                    raise RuntimeError('robot moved before path execution')
                self.execution_id = 'tool-path-'+uuid.uuid4().hex
                self.publisher.publish(String(data=json.dumps(dict(
                    epoch=s['epoch'], execution_id=self.execution_id,
                    waypoints=[q.tolist() for q in self.path['waypoints']],
                    durations=self.path['durations']))))
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
                        raise RuntimeError('managed tool path aborted')
                    if s.get('mode') == 'hold' and not s.get('transitioning'):
                        self.path_completed()
                        return
                elif s.get('mode') != 'tool_auto':
                    raise RuntimeError('tool path ownership lost')
                if now-self.started > self.path['total']+30.:
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
