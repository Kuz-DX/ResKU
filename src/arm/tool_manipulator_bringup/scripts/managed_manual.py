#!/usr/bin/env python3
"""Manual input producer; controller ownership belongs to the mode manager."""
import json
import signal
import time

import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from manual import Manual


class ManagedManual(Manual):
    local_tool_paths = False
    def __init__(self):
        self.manager_status = {}
        self.manager_time = -float('inf')
        self.manager_epoch = None
        self.pending_ee = 0.
        super().__init__()
        # TCP-ready alignment is owned by the managed workflow, not this producer.
        self.entry_alignment_complete = True
        # Base construction does not publish before feedback initialization.
        self.destroy_publisher(self.publisher)
        self.destroy_publisher(self.ee_velocity_publisher)
        self.input_pub = self.create_publisher(String, '/arm/input/manual', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/arm/control_status', self.on_manager, qos)

    def on_manager(self, msg):
        try:
            self.manager_status = json.loads(msg.data)
            self.manager_time = time.monotonic()
        except (ValueError, TypeError):
            self.manager_status = {}

    def poll_controller(self, now):
        status = self.manager_status
        allowed = (now-self.manager_time < .3 and status.get('ready') and
                   status.get('mode') == 'manual' and not status.get('transitioning'))
        self.controllers = status.get('controllers', {})
        if not allowed or not self.valid_measured(now):
            self.path_points = None
            self.controller_ready = False
            self.phase = 'waiting'
            self.neutral_required = True
            self.manager_epoch = None
            return
        if self.manager_epoch != status.get('epoch'):
            self.path_points = None
            self.manager_epoch = status['epoch']
            self.target = self.measured()
            self.stop_target = None
            self.stop_pose_active = False
            self.neutral_required = True
            self.pending_ee = 0.
            self.startup_recovering, self.startup_complete = False, True
        self.phase, self.controller_ready = 'ready', True

    def publish_ee_velocity(self, velocity):
        self.pending_ee = float(velocity)

    def hold(self):
        if (self.controller_ready and self.target is not None and self.manager_epoch
                and time.monotonic()-self.manager_time < .3):
            self.input_pub.publish(String(data=json.dumps(dict(
                epoch=self.manager_epoch, positions=self.target.tolist(),
                ee_velocity=self.pending_ee, fault=bool(self.fault or self.hardware_fault)))))

    def shutdown_controller(self):
        # No controller services or raw command publications from this producer.
        pass


def main():
    rclpy.init()
    node = ManagedManual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
