#!/usr/bin/env python3
"""Bridge /gripper/box commands to the Gripper_BOX Arduino over serial.

ROS node: /arm/gripper_gox
Subscription: /gripper/box (std_msgs/msg/Int32)
  0: stop immediately
  1: run in the original direction for three seconds, then publish 0 at 4 seconds
  2: run in the reverse direction for three seconds, then publish 0 at 4 seconds

The Arduino firmware owns the three-second timeout so the motors still stop if
this ROS process exits or its serial connection is interrupted after a command.
"""

import threading

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Int32


class GripperGox(Node):
    """Forward validated gripper commands to the Arduino."""

    VALID_COMMANDS = (0, 1, 2)

    def __init__(self) -> None:
        super().__init__('gripper_gox', namespace='/arm')

        self.declare_parameter('port', '/dev/GRIPPER_BOX')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/gripper/box')
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('reset_delay_sec', 4.0)

        self._port = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._topic = str(self.get_parameter('topic').value)
        reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self._reset_delay = float(self.get_parameter('reset_delay_sec').value)

        if reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval_sec must be greater than zero')
        if self._reset_delay <= 0.0:
            raise ValueError('reset_delay_sec must be greater than zero')

        self._serial = None
        self._serial_lock = threading.Lock()
        self._shutting_down = False
        self._reset_timer = None

        self._reset_publisher = self.create_publisher(Int32, self._topic, 10)
        self._subscription = self.create_subscription(
            Int32, self._topic, self._command_callback, 10)
        self._reconnect_timer = self.create_timer(
            reconnect_interval, self._try_open_serial)

        self._try_open_serial()
        self.get_logger().info(
            f'listening on {self._topic}; serial={self._port}@{self._baud}')

    def _command_callback(self, msg: Int32) -> None:
        command = int(msg.data)
        if command not in self.VALID_COMMANDS:
            self.get_logger().warning(
                f'ignoring invalid gripper command {command}; expected 0, 1, or 2')
            return

        if command == 0:
            self._cancel_reset_timer()
        else:
            self._schedule_topic_reset()

        if not self._send_command(command):
            self.get_logger().warning(
                f'gripper command {command} not sent: serial is disconnected')

    def _schedule_topic_reset(self) -> None:
        """Publish a stop command four seconds after the latest motion command."""
        self._cancel_reset_timer()
        self._reset_timer = self.create_timer(
            self._reset_delay, self._publish_reset_command)

    def _cancel_reset_timer(self) -> None:
        timer = self._reset_timer
        if timer is None:
            return
        self._reset_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _publish_reset_command(self) -> None:
        self._cancel_reset_timer()
        self._reset_publisher.publish(Int32(data=0))
        self.get_logger().info(
            f'published gripper reset command 0 after {self._reset_delay:g}s')

    def _try_open_serial(self) -> None:
        if self._shutting_down:
            return

        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                return

            try:
                connection = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=0.05,
                    write_timeout=0.2,
                    exclusive=True,
                )
                try:
                    connection.dtr = False
                    connection.rts = False
                except (AttributeError, OSError, SerialException):
                    # Some USB serial adapters do not expose modem control.
                    pass
                connection.reset_input_buffer()
                connection.reset_output_buffer()
                self._serial = connection
            except (OSError, ValueError, SerialException) as exc:
                self._serial = None
                self.get_logger().warning(
                    f'cannot open gripper serial port {self._port}: {exc}',
                    throttle_duration_sec=5.0)
                return

        self.get_logger().info(
            f'opened gripper serial port {self._port} @ {self._baud}')

    def _send_command(self, command: int) -> bool:
        payload = f'{command}\n'.encode('ascii')

        with self._serial_lock:
            connection = self._serial
            if connection is None or not connection.is_open:
                return False

            try:
                connection.write(payload)
                connection.flush()
            except (OSError, ValueError, SerialException) as exc:
                self.get_logger().error(f'gripper serial write failed: {exc}')
                self._close_serial_locked()
                return False

        self.get_logger().info(f'sent gripper command: {command}')
        return True

    def _close_serial_locked(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, SerialException):
                pass
        self._serial = None

    def shutdown(self) -> None:
        """Request a stop, then release the serial device."""
        self._shutting_down = True
        self._cancel_reset_timer()
        with self._serial_lock:
            connection = self._serial
            if connection is not None and connection.is_open:
                try:
                    connection.write(b'0\n')
                    connection.flush()
                except (OSError, ValueError, SerialException):
                    pass
            self._close_serial_locked()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperGox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
