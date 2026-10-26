#!/usr/bin/env python3
"""Bridge Bool docking requests to an Arduino motor controller over serial."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial


class ArduinoDockingBridge(Node):
    def __init__(self):
        super().__init__('arduino_docking_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.serial = None
        self.last_open_attempt = 0.0
        self.last_query = 0.0
        self.pending = False
        self.last_command = False
        self.last_arm_command = False
        self.state = None
        self.rx_buffer = bytearray()
        self.publishers = [self.create_publisher(Bool, topic, 10)
                           for topic in ('/docked', '/arm/docked')]
        self.create_subscription(Bool, '/docking', self.on_command, 10)
        self.create_subscription(Bool, '/arm/docking', self.on_arm_command, 10)
        self.create_timer(0.02, self.poll)

    def on_arm_command(self, msg):
        # The existing coordinator sends either True->False or False->True.
        # Only the True edge requests a state toggle.
        rising = msg.data and not self.last_arm_command
        self.last_arm_command = msg.data
        if rising:
            self.trigger()

    def on_command(self, msg):
        rising = msg.data and not self.last_command
        self.last_command = msg.data
        if rising:
            self.trigger()

    def trigger(self):
        if self.serial is None or self.pending or self.state is None:
            self.get_logger().warning('Docking request ignored: disconnected, busy, or state unknown')
            return
        try:
            self.serial.write(b'T\n')
            self.pending = True
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def publish_state(self):
        msg = Bool(data=self.state)
        for publisher in self.publishers:
            publisher.publish(msg)

    def disconnect(self, exc):
        self.get_logger().error(f'Arduino connection lost: {exc}')
        if self.serial is not None:
            self.serial.close()
        self.serial = None
        self.pending = False
        self.state = None
        self.rx_buffer.clear()

    def poll(self):
        if self.serial is None:
            now = time.monotonic()
            if now - self.last_open_attempt < 1.0:
                return
            self.last_open_attempt = now
            try:
                self.serial = serial.Serial(self.port, self.baud, timeout=0,
                                            write_timeout=0.2, exclusive=True)
                # Opening USB serial can reset the Arduino. Its boot state is STATE 0.
                self.state = None
                self.pending = False
                self.rx_buffer.clear()
                self.serial.write(b'Q\n')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f'Arduino connection unavailable: {exc}')
            return
        try:
            if self.state is None and time.monotonic() - self.last_query >= 1.0:
                self.serial.write(b'Q\n')
                self.last_query = time.monotonic()
            self.rx_buffer.extend(self.serial.read(min(self.serial.in_waiting, 256)))
            if len(self.rx_buffer) > 256:
                self.rx_buffer.clear()
                self.get_logger().warning('Arduino response exceeded 256 bytes')
            while b'\n' in self.rx_buffer:
                raw, _, rest = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(rest)
                line = raw.decode('ascii', errors='replace').strip()
                if line in ('STATE 0', 'STATE 1'):
                    self.state = line == 'STATE 1'
                    self.pending = False
                    self.publish_state()
                elif line == 'STOPPED':
                    self.pending = False
                elif line and line not in ('MOVING', 'BUSY'):
                    self.get_logger().warning(f'Arduino response: {line}')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)


def main():
    rclpy.init()
    node = ArduinoDockingBridge()
    try:
        rclpy.spin(node)
    finally:
        if node.serial is not None:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
