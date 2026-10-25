#!/usr/bin/env python3
"""Read-only periodic JointState recorder. Ctrl-C saves; never sends motion."""
import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_roll_joint', 'wrist_yaw_joint']


class Recorder(Node):
    def __init__(self, topic, period, timeout):
        super().__init__('capture_tool_trajectory')
        self.period, self.timeout = period, timeout
        self.started = self.last_recorded = self.last_received = None
        self.points = []
        self.error = None
        self.create_subscription(JointState, topic, self.receive, qos_profile_sensor_data)
        self.create_timer(min(period, .1), self.watchdog)

    def receive(self, msg):
        if self.error:
            return
        now = time.monotonic()
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.error = 'malformed joint state; recording stopped'
            return
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in JOINTS):
            self.error = 'missing/nonfinite arm joint feedback; recording stopped'
            return
        if self.last_received is not None and now-self.last_received > self.timeout:
            self.error = 'feedback gap; recording stopped'
            return
        self.last_received = now
        if self.started is None:
            self.started = now
            self.get_logger().info('Recording started. Ctrl-C saves the disabled YAML.')
        if self.last_recorded is None or now-self.last_recorded >= self.period:
            self.points.append(dict(time_from_start_sec=round(now-self.started, 6),
                                    positions_rad=[float(values[j]) for j in JOINTS]))
            self.last_recorded = now

    def watchdog(self):
        if self.last_received is not None and time.monotonic()-self.last_received > self.timeout:
            self.error = 'feedback timeout; recording stopped'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--period', type=float, default=.1, help='minimum sample interval, seconds')
    parser.add_argument('--timeout', type=float, default=.5, help='maximum feedback gap, seconds')
    parser.add_argument('--topic', default='/joint_states')
    args = parser.parse_args()
    if not (math.isfinite(args.period) and math.isfinite(args.timeout)
            and 0 < args.period <= args.timeout):
        parser.error('require 0 < period <= timeout, both finite')
    if not args.output.parent.is_dir():
        parser.error('output parent directory must exist')
    if args.output.exists():
        parser.error('output exists; use a new filename to preserve previous recordings')
    rclpy.init()
    node = Recorder(args.topic, args.period, args.timeout)
    print('Waiting for complete arm feedback. Move using your existing controller; Ctrl-C to save.')
    try:
        while rclpy.ok() and not node.error:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not node.points:
        print('No samples; no file written.')
        return 1
    # Exclusive creation prevents accidental overwrite, including a concurrent recorder.
    with args.output.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(dict(execution_enabled=False, joint_names=JOINTS,
                            recording_complete=node.error is None,
                            recording_error=node.error,
                            sample_period_sec=args.period, points=node.points),
                       stream, sort_keys=False)
    print(f'Saved {len(node.points)} points to {args.output} (execution disabled).')
    if node.error:
        print(node.error+'; saved partial recording for inspection only')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
