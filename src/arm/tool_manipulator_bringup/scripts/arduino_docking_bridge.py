#!/usr/bin/env python3
"""Bridge Bool docking requests to an Arduino motor controller over serial."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
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
        self.last_arm_command = False
        self.state = None
        self.rx_buffer = bytearray()
        self.protocol = None
        self.calibration_target = None
        self.calibration_deadline = None
        self._state_publishers = [self.create_publisher(Bool, topic, 10)
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
        # /docking is a one-shot request; separate `ros2 topic pub --once`
        # invocations must each work without an intervening False message.
        if msg.data:
            self.trigger()

    def trigger(self):
        if self.serial is None or self.pending or self.state is None:
            self.get_logger().warning('Docking request ignored: disconnected, busy, or state unknown')
            return
        try:
            if self.protocol == 'calibration':
                self.calibration_target = not self.state
                angle = 90 if self.calibration_target else 180
                self.calibration_deadline = None
                self.serial.write(f'{angle}\n'.encode('ascii'))
            else:
                self.serial.write(b'T\n')
            self.pending = True
        except (serial.SerialException, OSError) as exc:
            self.disconnect(exc)

    def publish_state(self):
        msg = Bool(data=self.state)
        for publisher in self._state_publishers:
            publisher.publish(msg)

    def disconnect(self, exc):
        self.get_logger().error(f'Arduino connection lost: {exc}')
        if self.serial is not None:
            self.serial.close()
        self.serial = None
        self.pending = False
        self.state = None
        self.rx_buffer.clear()
        self.protocol = None
        self.calibration_target = None
        self.calibration_deadline = None

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
                self.calibration_target = None
                self.calibration_deadline = None
                self.serial.write(b'Q\n')
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
                elif ('MG996R D9 calibration ready' in line
                      or line.startswith('ERROR: send 0..180')):
                    if self.protocol is None:
                        self.protocol = 'calibration'
                        self.calibration_target = False
                        self.pending = True
                        self.serial.write(b'180\n')
                        self.get_logger().info(
                            'Calibration sketch detected; commanding initial 180 degrees')
                elif line.startswith('ANGLE ') and self.protocol == 'calibration':
                    angle = 90 if self.calibration_target else 180
                    if self.pending and line == f'ANGLE {angle} (commanded)':
                        self.calibration_deadline = time.monotonic() + 3.0
                elif line == 'STOPPED':
                    self.pending = False
                elif line.startswith('ERROR '):
                    self.pending = False
                    self.get_logger().error(f'Arduino rejected docking request: {line}')
                elif line and line not in ('MOVING', 'BUSY'):
                    self.get_logger().warning(f'Arduino response: {line}')
            if (self.protocol == 'calibration' and self.pending
                    and self.calibration_deadline is not None
                    and time.monotonic() >= self.calibration_deadline):
                self.state = self.calibration_target
                self.pending = False
                self.calibration_deadline = None
                self.publish_state()
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
