#!/usr/bin/env python3
ㅈ"""Print current joints on terminal Enter. No saving and no motor commands."""
import argparse
import math
import select
import sys
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_roll_joint', 'wrist_yaw_joint']


class Capture(Node):
    def __init__(self, topic, timeout):
        super().__init__('capture_tool_trajectory')
        self.timeout = timeout
        self.positions = self.received = self.started = None
        self.create_subscription(JointState, topic, self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        self.positions = None
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in JOINTS):
            return
        self.positions = [float(values[j]) for j in JOINTS]
        self.received = time.monotonic()

    def capture(self):
        now = time.monotonic()
        if self.positions is None or now-self.received > self.timeout:
            print('Feedback missing, invalid or stale; no capture', flush=True)
            return
        if self.started is None:
            self.started = now
        print(yaml.safe_dump([dict(time_from_start_sec=round(now-self.started, 6),
                                   positions_rad=list(self.positions))],
                             sort_keys=False, default_flow_style=None).strip(), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--timeout', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    rclpy.init()
    node = Capture(args.topic, args.timeout)
    print('Enter: print pose | q + Enter: quit | no automatic saving', flush=True)
    print('joint_names: '+str(JOINTS), flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip().lower() == 'q':
                    break
                if not line.strip():
                    node.capture()
                else:
                    print('Press Enter to capture, or q to quit.', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
