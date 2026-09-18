"""
sim_rmd_x8_hardware

Pre-flight virtual test tool (NOT for production launch). Simulates the
physical hardware that rmd_x8_driver_node normally talks to over CAN:

  - Two fake RMD-X8 motors on vcan0: parses the same 0xA2/0xB3/0x9A frames
    the real driver sends, integrates simple zero-slip physics, and replies
    with the same frame formats rmd_x8_driver.rmd_x8_protocol expects
    (parse_a2_9c_reply / parse_status1_reply / parse_set_timeout_reply).
  - A fake /imu (sensor_msgs/Imu), tracking the same simulated skid-steer
    motion as the fake motors (so reduced_odom's IMU-yaw fusion sees a
    consistent world, not an independent random signal).

This lets the REAL rmd_x8_driver_node (CAN protocol/reconnect logic) and
the REAL reduced_odom_node (EKF fusion) run unmodified against vcan0,
instead of bypassing them with a fake /odometry/filtered directly. Only
the physical motors and the physical IMU are replaced.

Requires a vcan0 interface to already exist (one-time, needs root):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
"""

import math
import struct
import threading
import time

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from rmd_x8_driver import rmd_x8_protocol as proto


class FakeMotor:
    """Zero-slip, instant-tracking simulation of one RMD-X8 output shaft."""

    def __init__(self, name: str):
        self.name = name
        self.lock = threading.Lock()
        self.commanded_output_shaft_dps = 0.0
        self.angle_deg = 0.0  # unwrapped, output shaft
        self.last_update = time.monotonic()

    def _integrate(self):
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        self.angle_deg += self.commanded_output_shaft_dps * dt

    def apply_speed_command(self, output_shaft_dps: float):
        with self.lock:
            self._integrate()
            self.commanded_output_shaft_dps = output_shaft_dps

    def stop(self):
        self.apply_speed_command(0.0)

    def snapshot(self):
        with self.lock:
            self._integrate()
            wrapped = int(round(self.angle_deg)) % 65536
            if wrapped > 32767:
                wrapped -= 65536
            return self.commanded_output_shaft_dps, wrapped


class FakeRmdX8HardwareNode(Node):
    def __init__(self):
        super().__init__('sim_rmd_x8_hardware')

        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('left_motor_can_id', 1)
        self.declare_parameter('right_motor_can_id', 2)
        self.declare_parameter('left_direction_sign', 1.0)
        self.declare_parameter('right_direction_sign', -1.0)
        self.declare_parameter('wheel_radius_m', 0.1125)
        self.declare_parameter('external_gear_ratio', 1.0)
        self.declare_parameter('effective_track_width_m', 0.4904)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('imu_rate_hz', 20.0)

        p = self.get_parameter
        can_interface = p('can_interface').value
        left_id = int(p('left_motor_can_id').value)
        right_id = int(p('right_motor_can_id').value)
        self.left_dir = float(p('left_direction_sign').value)
        self.right_dir = float(p('right_direction_sign').value)
        self.wheel_radius_m = float(p('wheel_radius_m').value)
        self.gear_ratio = float(p('external_gear_ratio').value)
        self.track_width = float(p('effective_track_width_m').value)

        self.left = FakeMotor('left')
        self.right = FakeMotor('right')
        self._send_id_map = {
            proto.motor_send_id(left_id): (self.left, left_id),
            proto.motor_send_id(right_id): (self.right, right_id),
        }
        self._reply_id_map = {
            left_id: proto.motor_reply_id(left_id),
            right_id: proto.motor_reply_id(right_id),
        }

        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(
                f"Failed to open '{can_interface}': {exc}. "
                f"Create it first: sudo ip link add dev {can_interface} type vcan "
                f"&& sudo ip link set up {can_interface}")
            raise

        self._notifier = can.Notifier(self.bus, [self._on_can_message])

        # ---- fake IMU, driven by the same simulated skid-steer motion ----
        self._world_yaw = 0.0
        self._last_yaw_update = time.monotonic()
        self.imu_pub = self.create_publisher(Imu, p('imu_topic').value, 10)
        imu_rate = float(p('imu_rate_hz').value)
        self.create_timer(1.0 / imu_rate, self._publish_imu)

        self.get_logger().info(
            f'sim_rmd_x8_hardware started on {can_interface} '
            f'(left_id=0x{proto.motor_send_id(left_id):X}, right_id=0x{proto.motor_send_id(right_id):X}) '
            f'-- pre-flight virtual test tool, NOT for production use.')

    # ------------------------------------------------------------------ #
    def _on_can_message(self, msg: can.Message):
        entry = self._send_id_map.get(msg.arbitration_id)
        if entry is None or len(msg.data) != 8:
            return
        motor, can_id = entry
        data = bytes(msg.data)
        cmd = data[0]

        if cmd == proto.CMD_SPEED_CONTROL:
            _, speed_control = struct.unpack('<Bxxxi', data)
            output_shaft_dps = speed_control / 100.0
            motor.apply_speed_command(output_shaft_dps)
            self._reply_speed(motor, can_id)
        elif cmd == proto.CMD_MOTOR_STOP or cmd == proto.CMD_MOTOR_SHUTDOWN:
            motor.stop()
        elif cmd == proto.CMD_READ_STATUS_1:
            self._reply_status1(can_id)
        elif cmd == proto.CMD_SET_TIMEOUT:
            _, timeout_ms = struct.unpack('<BxxxI', data)
            self._reply(can_id, proto.build_set_timeout_command(timeout_ms))

    def _reply_speed(self, motor: FakeMotor, can_id: int):
        speed_dps, angle_deg = motor.snapshot()
        payload = struct.pack('<Bbhhh', proto.CMD_SPEED_CONTROL, 25,
                               0, int(round(speed_dps)), angle_deg)
        self._reply(can_id, payload)

    def _reply_status1(self, can_id: int):
        # temp=25C, mos_temp=30C, brake_released=True, voltage=12.6V, no errors
        payload = struct.pack('<BbbBHH', proto.CMD_READ_STATUS_1, 25, 30, 1, 126, 0)
        self._reply(can_id, payload)

    def _reply(self, can_id: int, payload: bytes):
        reply_arb_id = self._reply_id_map[can_id]
        message = can.Message(arbitration_id=reply_arb_id, data=payload, is_extended_id=False)
        try:
            self.bus.send(message)
        except can.CanError as exc:
            self.get_logger().warn(f'sim reply send failed: {exc}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ #
    def _publish_imu(self):
        left_dps, _ = self.left.snapshot()
        right_dps, _ = self.right.snapshot()
        # undo direction_sign to recover wheel-frame (both positive = forward)
        wheel_left_dps = left_dps * self.left_dir
        wheel_right_dps = right_dps * self.right_dir
        v_left = math.radians(wheel_left_dps) * self.wheel_radius_m / self.gear_ratio
        v_right = math.radians(wheel_right_dps) * self.wheel_radius_m / self.gear_ratio
        wz = (v_right - v_left) / self.track_width if self.track_width > 1e-6 else 0.0

        now = time.monotonic()
        dt = now - self._last_yaw_update
        self._last_yaw_update = now
        if 0.0 < dt < 1.0:
            self._world_yaw += wz * dt

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = math.sin(self._world_yaw / 2.0)
        imu.orientation.w = math.cos(self._world_yaw / 2.0)
        imu.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu.angular_velocity.z = wz
        imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.linear_acceleration.z = 9.81
        imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.imu_pub.publish(imu)

    def destroy_node(self):
        try:
            self._notifier.stop()
            self.bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeRmdX8HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
