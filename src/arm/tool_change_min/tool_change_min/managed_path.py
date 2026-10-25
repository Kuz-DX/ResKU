"""Tool path producer. Only the mode manager publishes joint commands."""
import json
import math
import time
import uuid

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity


class ManagedPath:
    def __init__(self, node, model, names, margin, group):
        self.node, self.model, self.names, self.margin = node, model, list(names), margin
        self.status = ({}, -math.inf)
        self.state = (None, -math.inf)
        self.failed = False
        self.publisher = node.create_publisher(String, '/arm/input/tool_auto', 1)
        self.mode = node.create_client(Trigger, '/arm/set_mode/tool_auto', callback_group=group)
        self.collision = node.create_client(GetStateValidity, '/check_state_validity', callback_group=group)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/arm/control_status', self.on_status, qos, callback_group=group)
        node.create_subscription(JointState, '/joint_states', self.on_state, 1, callback_group=group)

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict): raise ValueError('invalid status')
            self.status = (value, time.monotonic())
        except ValueError:
            self.status = ({}, -math.inf)

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if (len(msg.name) != len(msg.position) or len(values) != len(msg.name)
                or not all(n in values for n in self.names) or not np.all(np.isfinite(msg.position))):
            self.state = (None, -math.inf)
        else:
            self.state = (values, time.monotonic())

    def healthy(self):
        s, stamp = self.status
        if (time.monotonic()-stamp >= .3 or not s.get('ready') or s.get('focus') != 'arm'
                or not s.get('startup_complete') or s.get('mode') == 'fault'):
            raise RuntimeError('managed control unavailable, inactive focus or FAULT')
        return s

    def feedback(self):
        values, stamp = self.state
        if values is None or time.monotonic()-stamp >= .3:
            raise RuntimeError('managed path feedback missing/stale')
        return values

    def wait(self, future, deadline):
        while not future.done():
            self.healthy()
            self.feedback()
            if time.monotonic() >= deadline:
                future.cancel()
                raise TimeoutError('managed path service deadline')
            time.sleep(.01)
        if time.monotonic() >= deadline:
            raise TimeoutError('managed path service deadline')
        return future.result()

    def send(self, trajectory, timeout):
        if self.failed:
            raise RuntimeError('managed executor fault latched; inspect and restart')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid path timeout')
        deadline = time.monotonic()+timeout
        s = self.healthy()
        if s.get('mode') != 'hold' or s.get('transitioning'):
            raise ValueError('explicit stopped HOLD required before tool movement')
        original_epoch = s['epoch']
        snapshot = dict(self.feedback())
        start = np.array([snapshot[j] for j in self.names])
        if not self.model.within_limits(start, self.margin):
            raise ValueError('path start outside margin')
        if list(trajectory.joint_names) != self.names or not 1 <= len(trajectory.points) <= 1000:
            raise ValueError('invalid trajectory joint order/point count')
        header = getattr(trajectory, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            raise ValueError('scheduled trajectory stamps are not supported by managed paths')
        points, durations, previous = [], [], 0.
        for p in trajectory.points:
            if p.time_from_start.sec < 0 or not 0 <= p.time_from_start.nanosec < 1000000000:
                raise ValueError('invalid trajectory duration encoding')
            t = p.time_from_start.sec+p.time_from_start.nanosec*1e-9
            if not math.isfinite(t) or not previous < t or t-previous >= 120:
                raise ValueError('trajectory times must increase; each segment <120s')
            if not self.model.within_limits(p.positions, self.margin):
                raise ValueError('waypoint violates limits')
            if any(p.velocities) or any(p.accelerations) or any(p.effort):
                raise ValueError('managed path accepts position-only trajectories')
            points.append(list(p.positions))
            durations.append(t-previous)
            previous = t
        if previous >= timeout:
            raise ValueError('trajectory duration must leave time for collision checks and settling')
        if not self.collision.service_is_ready() or not self.mode.service_is_ready():
            raise RuntimeError('managed mode/collision services unavailable')
        # Validate the entire path before publishing. Check actual joint lines,
        # including both endpoints, using all received robot joints.
        samples = 0
        for point in points:
            target = np.asarray(point)
            count = max(1, math.ceil(float(np.max(np.abs(target-start)))/.005))
            samples += count+1
            if samples > 20000:
                raise ValueError('collision sampling budget exceeded')
            for q in np.linspace(start, target, count+1):
                now_status = self.healthy()
                if now_status['epoch'] != original_epoch or now_status.get('mode') != 'hold':
                    raise RuntimeError('ownership changed during collision validation')
                req = GetStateValidity.Request()
                values = dict(snapshot)
                values.update(zip(self.names, map(float, q)))
                req.robot_state.joint_state.name = list(values)
                req.robot_state.joint_state.position = list(values.values())
                req.robot_state.is_diff = True
                result = self.wait(self.collision.call_async(req), deadline)
                if result is None or not result.valid:
                    raise ValueError('tool path collision/state invalid')
            start = target
        current = self.feedback()
        if current.keys() != snapshot.keys() or any(abs(current[j]-v) > .001 for j, v in snapshot.items()):
            raise RuntimeError('robot moved during collision validation')
        result = self.wait(self.mode.call_async(Trigger.Request()), deadline)
        if result is None or not result.success:
            raise RuntimeError('tool_auto mode request rejected')
        while True:
            s = self.healthy()
            if s.get('mode') == 'tool_auto' and not s.get('transitioning'):
                break
            if s.get('requested') not in ('hold', 'tool_auto') or time.monotonic() >= deadline:
                raise RuntimeError('tool_auto entry interrupted or timed out')
            time.sleep(.01)
        epoch, execution = s['epoch'], uuid.uuid4().hex
        try:
            current = self.feedback()
            if current.keys() != snapshot.keys() or any(abs(current[j]-v) > .001 for j, v in snapshot.items()):
                raise RuntimeError('robot moved before path execution')
            self.publisher.publish(String(data=json.dumps(dict(epoch=epoch, execution_id=execution,
                waypoints=points, durations=durations))))
            while time.monotonic() < deadline:
                s = self.healthy()
                self.feedback()
                if s.get('last_execution_id') == execution:
                    if s.get('last_execution_result') != 'succeeded':
                        raise RuntimeError('managed path aborted')
                    if s.get('mode') == 'hold' and not s.get('transitioning'):
                        return 'managed path completed and stopped'
                elif s.get('epoch') != epoch or s.get('mode') != 'tool_auto':
                    raise RuntimeError('managed path ownership lost')
                time.sleep(.01)
            raise TimeoutError('managed path completion timeout')
        except Exception:
            self.failed = True
            self.publisher.publish(String(data=json.dumps(dict(epoch=epoch, execution_id=execution, cancel=True))))
            raise
