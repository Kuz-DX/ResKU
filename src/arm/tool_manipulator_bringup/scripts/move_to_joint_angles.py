#!/usr/bin/env python3
"""Move to six entered joint angles through the managed, collision-checked path."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
import uuid

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


JOINTS = (
    'base_joint', 'shoulder_joint', 'elbow_joint',
    'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Move through the managed tool_auto gateway to six entered joint angles.')
    parser.add_argument('angles', nargs=6, type=float, metavar='ANGLE',
                        help='base shoulder elbow wrist_pitch wrist_roll wrist_yaw')
    parser.add_argument('--degrees', action='store_true', help='Interpret ANGLE values as degrees')
    parser.add_argument('--duration', type=float,
                        help='Requested synchronized duration in seconds; safety limits may extend it')
    parser.add_argument('--limit-margin', type=float, default=.02,
                        help='Required distance inside soft limits in radians (default: 0.02)')
    parser.add_argument('--collision-step', type=float, default=.005,
                        help='Maximum joint-space collision sample step in radians (default: 0.005)')
    parser.add_argument('--timeout', type=float, default=120.,
                        help='Total operation timeout in seconds (default: 120)')
    parser.add_argument('--hardware', type=Path,
                        help='hardware.yaml override (default: package config/hardware.yaml)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Validate input and print the target without contacting ROS services')
    return parser.parse_args(remove_ros_args(args=sys.argv if argv is None else argv)[1:])


def load_limits(path: Path):
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return {name: tuple(map(float, data['joints'][name]['soft_limit_rad'])) for name in JOINTS}


def checked_target(values, limits, margin):
    target = np.asarray(values, dtype=float)
    if target.shape != (6,) or not np.isfinite(target).all():
        raise ValueError('six finite joint angles are required')
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('limit margin must be finite and nonnegative')
    errors = []
    for name, value in zip(JOINTS, target):
        low, high = limits[name]
        safe_low, safe_high = low+margin, high-margin
        if not safe_low <= value <= safe_high:
            errors.append(f'{name}={value:.6f}, allowed=[{safe_low:.6f}, {safe_high:.6f}]')
    if errors:
        raise ValueError('target violates joint limits: '+'; '.join(errors))
    return target


class JointAngleRunner(Node):
    def __init__(self):
        super().__init__('move_to_joint_angles')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = None
        self.status_time = -math.inf
        self.state = None
        self.state_time = -math.inf
        self.create_subscription(String, '/arm/control_status', self.on_status, qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.collision = self.create_client(GetStateValidity, '/check_state_validity')
        self.mode = self.create_client(Trigger, '/arm/set_mode/tool_auto')
        self.cancel = self.create_client(Trigger, '/arm/cancel')
        self.command = self.create_publisher(String, '/arm/input/tool_auto', 1)

    def on_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError
            self.status, self.status_time = value, time.monotonic()
        except (TypeError, ValueError):
            self.status, self.status_time = None, -math.inf

    def on_state(self, message):
        if (len(message.name) != len(message.position)
                or len(set(message.name)) != len(message.name)):
            return
        values = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if (not all(name in values for name in JOINTS)
                or not all(math.isfinite(v) for v in values.values())
                or len(message.velocity) != len(message.name)
                or not all(math.isfinite(v) for v in velocities.values())):
            return
        self.state = (values, velocities)
        self.state_time = time.monotonic()

    def spin_until(self, predicate, deadline, reason):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
            if predicate():
                return
        raise TimeoutError(reason)

    def fresh_hold(self):
        now = time.monotonic()
        if self.status is None or self.state is None or now-self.status_time >= .3 or now-self.state_time >= .3:
            raise RuntimeError('fresh control status and joint feedback are required')
        s = self.status
        _, velocity = self.state
        if (s.get('mode') != 'hold' or s.get('requested', 'hold') != 'hold'
                or s.get('transitioning') or s.get('focus') != 'arm' or not s.get('ready')
                or not s.get('startup_complete') or not s.get('entry_alignment_complete', True)):
            raise RuntimeError(
                'stopped ready HOLD with arm focus is required: '
                f"mode={s.get('mode')} requested={s.get('requested')} "
                f"focus={s.get('focus')} ready={s.get('ready')} "
                f"startup={s.get('startup_complete')} "
                f"alignment={s.get('entry_alignment_complete')} "
                f"transitioning={s.get('transitioning')} "
                f"reason={s.get('blocker') or s.get('reason', '')}")
        if max(abs(velocity[name]) for name in JOINTS) > .02:
            raise RuntimeError('arm feedback must be stopped before validation')
        return s, dict(self.state[0])

    def call(self, client, request, deadline, unavailable):
        if not client.wait_for_service(timeout_sec=min(2., max(0., deadline-time.monotonic()))):
            raise RuntimeError(unavailable)
        future = client.call_async(request)
        self.spin_until(future.done, deadline, unavailable+' timed out')
        result = future.result()
        if result is None:
            raise RuntimeError(unavailable)
        return result

    def prepare_hold(self, deadline):
        """Stop another healthy arm mode before validating the requested move."""
        now = time.monotonic()
        if self.status is None or self.state is None or now-self.status_time >= .3 or now-self.state_time >= .3:
            raise RuntimeError('fresh control status and joint feedback are required')
        s = self.status
        if s.get('focus') != 'arm':
            raise RuntimeError('arm focus is required; press Options to select arm')
        if s.get('mode') == 'fault':
            raise RuntimeError('manager fault: '+str(s.get('reason', 'inspect control_status')))
        if not s.get('ready') or not s.get('startup_complete'):
            raise RuntimeError('controller readiness and startup recovery must complete before motion')
        if not s.get('entry_alignment_complete', True):
            raise RuntimeError('entry alignment must complete before motion')
        if s.get('mode') == 'hold' and s.get('requested', 'hold') == 'hold' and not s.get('transitioning'):
            return
        response = self.call(self.cancel, Trigger.Request(), deadline, '/arm/cancel unavailable')
        if not response.success:
            raise RuntimeError('HOLD request rejected: '+response.message)
        self.spin_until(
            lambda: self.status is not None and self.status.get('mode') == 'hold'
            and self.status.get('requested', 'hold') == 'hold'
            and not self.status.get('transitioning'),
            deadline, 'timed out waiting for HOLD')

    def collision_check(self, start, target, step, deadline):
        if not math.isfinite(step) or step <= 0:
            raise ValueError('collision step must be finite and positive')
        begin = np.asarray([start[name] for name in JOINTS])
        count = max(1, math.ceil(float(np.max(np.abs(target-begin)))/step))
        if count+1 > 20000:
            raise ValueError('collision sampling budget exceeded')
        for index, q in enumerate(np.linspace(begin, target, count+1)):
            values = dict(start)
            values.update(zip(JOINTS, map(float, q)))
            # MoveIt derives these mimic joints from ee_joint.
            values.pop('rack_left_joint', None)
            values.pop('rack_right_joint', None)
            request = GetStateValidity.Request()
            request.robot_state.joint_state.name = list(values)
            request.robot_state.joint_state.position = list(values.values())
            request.robot_state.is_diff = True
            result = self.call(self.collision, request, deadline,
                               'MoveIt state-validity service unavailable')
            if not result.valid:
                pairs = sorted({' <-> '.join(sorted((c.contact_body_1, c.contact_body_2)))
                                for c in result.contacts})
                raise RuntimeError('collision/state invalid at sample %d/%d: %s' %
                                   (index+1, count+1, ', '.join(pairs[:6]) or 'no contact detail'))
        return count+1

    def run(self, target, duration, collision_step, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if duration is not None and (not math.isfinite(duration) or not 0 < duration < 120):
            raise ValueError('duration must be in (0, 120) seconds')
        deadline = time.monotonic()+timeout
        self.spin_until(lambda: self.status is not None and self.state is not None, deadline,
                        'initial feedback timed out')
        self.prepare_hold(deadline)
        status, snapshot = self.fresh_hold()
        epoch = status['epoch']
        samples = self.collision_check(snapshot, target, collision_step, deadline)
        status2, current = self.fresh_hold()
        if status2.get('epoch') != epoch or max(abs(current[n]-snapshot[n]) for n in JOINTS) > .005:
            raise RuntimeError('arm state or ownership changed during collision validation')
        response = self.call(self.mode, Trigger.Request(), deadline, 'tool_auto mode service unavailable')
        if not response.success:
            raise RuntimeError('tool_auto request rejected: '+response.message)
        self.spin_until(lambda: self.status is not None and self.status.get('mode') == 'tool_auto'
                        and not self.status.get('transitioning'), deadline,
                        'tool_auto transition timed out')
        execution_id = 'joint-angle-'+uuid.uuid4().hex
        command = dict(epoch=self.status['epoch'], execution_id=execution_id,
                       waypoints=[target.tolist()], stop_at_waypoints=[True])
        if duration is not None:
            command['durations'] = [duration]
        self.command.publish(String(data=json.dumps(command)))
        self.get_logger().info(
            f'executing collision-checked target ({samples} samples), id={execution_id}')
        self.spin_until(lambda: self.status is not None
                        and self.status.get('last_execution_id') == execution_id,
                        deadline, 'joint-angle execution timed out')
        if self.status.get('last_execution_result') != 'succeeded':
            raise RuntimeError('managed execution failed: '+str(self.status.get('reason', 'unknown')))
        self.get_logger().info('joint-angle target reached; manager returned to HOLD')


def main():
    args = parse_args()
    hardware = args.hardware or (Path(get_package_share_directory(
        'tool_manipulator_bringup'))/'config/hardware.yaml')
    try:
        limits = load_limits(hardware)
        values = np.radians(args.angles) if args.degrees else args.angles
        target = checked_target(values, limits, args.limit_margin)
        print('target_rad:', ' '.join(f'{value:.9f}' for value in target))
        if args.dry_run:
            return 0
        rclpy.init()
        node = JointAngleRunner()
        try:
            node.run(target, args.duration, args.collision_step, args.timeout)
            return 0
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except (KeyError, OSError, TypeError, ValueError, RuntimeError, TimeoutError) as exc:
        print('ERROR:', exc, file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
