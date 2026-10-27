#!/usr/bin/env python3
"""Bridge Bool docking requests to an Arduino motor controller over serial."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
        self.state = None
        self.rx_buffer = bytearray()
        self.protocol = None
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_publishers = [self.create_publisher(Bool, topic, state_qos)
                                  for topic in ('/docked', '/arm/docked')]
        self._status_publisher = self.create_publisher(
            String, '/arm/docking_status', state_qos)
        self.create_subscription(Bool, '/docking', self.on_command, 10)
        self.create_subscription(Bool, '/arm/docking', self.on_arm_command, 10)
        self.create_timer(0.02, self.poll)

    def on_arm_command(self, msg):
        # Bool is the desired state, not a toggle: true=dock, false=release.
        self.command_state(bool(msg.data))

    def on_command(self, msg):
        # /docking is a one-shot request; separate `ros2 topic pub --once`
        # invocations must each work without an intervening False message.
        if msg.data:
            if self.state is None:
                self.get_logger().warning('Legacy toggle rejected while state is UNKNOWN')
            else:
                self.command_state(not self.state)

    def command_state(self, desired):
        if self.serial is None or self.pending:
            self.get_logger().warning('Docking request ignored: disconnected or busy')
            return
        if self.state is not None and self.state == desired:
            self.get_logger().info('Docking mechanism already in requested state')
            self.publish_state()
            return
        try:
            self.serial.write(b'D\n' if desired else b'R\n')
            self.pending = True
            self.publish_status('MOVING')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def publish_state(self):
        if self.state is None:
            self.publish_status('UNKNOWN')
            return
        msg = Bool(data=self.state)
        for publisher in self._state_publishers:
            publisher.publish(msg)
        self.publish_status('DOCKED' if self.state else 'RELEASED')

    def publish_status(self, status):
        self._status_publisher.publish(String(data=status))

    def disconnect(self, exc):
        self.get_logger().error(f'Arduino connection lost: {exc}')
        if self.serial is not None:
            self.serial.close()
        self.serial = None
        self.pending = False
        self.state = None
        self.rx_buffer.clear()
        self.protocol = None
        self.publish_status('UNKNOWN')

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
                self.protocol = None
                self.serial.write(b'Q\n')
                self.publish_status('UNKNOWN')
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f'Arduino connection unavailable: {exc}')
            return
        try:
            if self.protocol is None and time.monotonic() - self.last_query >= 1.0:
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
                    self.protocol = 'docking'
                    self.state = line == 'STATE 1'
                    self.pending = False
                    self.publish_state()
                elif line == 'STATE UNKNOWN':
                    self.protocol = 'docking'
                    self.state = None
                    self.pending = False
                    self.publish_status('UNKNOWN')
                elif ('MG996R D9 calibration ready' in line
                      or line.startswith('ERROR: send 0..180')
                      or line.startswith('ANGLE ')):
                    # Never retain the old calibration-sketch fallback here: it
                    # commanded 180 degrees as soon as the serial port opened.
                    # The production firmware must support D/R/Q and boot UNKNOWN.
                    self.pending = False
                    self.publish_status('ERROR')
                    self.get_logger().error(
                        'Legacy calibration firmware detected; automatic motion is disabled. '
                        'Flash Arduino/DockingMotor/DockingMotor.ino')
                elif line == 'STOPPED':
                    self.pending = False
                elif line.startswith('ERROR '):
                    self.pending = False
                    self.publish_status('ERROR')
                    self.get_logger().error(f'Arduino rejected docking request: {line}')
                elif line and line not in ('MOVING', 'BUSY'):
                    self.get_logger().warning(f'Arduino response: {line}')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)


def main():
    rclpy.init()
    node = ArduinoDockingBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.serial is not None:
            node.serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
