#!/usr/bin/env python3
"""Read actuator feedback and publish calibrated ROS joint states.

This node is read-only: it queries RMD actuators over CAN and Dynamixels over
TTL, and never sends a position, torque, velocity, or configuration command.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from dynamixel_sdk import PacketHandler, PortHandler
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rmd_sdk import rmd_sdk_py
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


ADDR_PRESENT_POSITION = 132
ADDR_HARDWARE_ERROR_STATUS = 70
DXL_PULSE_TO_RAD = 2.0 * math.pi / 4096.0
COMM_SUCCESS = 0


@dataclass(frozen=True)
class RmdJoint:
    """Calibration and actuator handle for one RMD joint."""

    name: str
    actuator: Any
    sign: float
    q_offset: float


@dataclass(frozen=True)
class DxlJoint:
    """Calibration and bus ID for one Dynamixel joint."""

    name: str
    motor_id: int
    zero_offset: float
    direction: float
    wraparound: bool


def hardware_error_details(status: int) -> str:
    """Translate known Dynamixel hardware-error bits into readable labels."""
    error_bits = (
        (0x01, "input voltage"),
        (0x02, "motor Hall sensor"),
        (0x04, "overheating"),
        (0x08, "motor encoder"),
        (0x10, "electrical shock / insufficient power"),
        (0x20, "overload"),
    )
    details = [label for mask, label in error_bits if status & mask]
    return ", ".join(details) if details else "unknown"


def calibrate_rmd(raw_deg: float, sign: float, q_offset: float) -> float:
    """Apply the same sign and zero-offset conversion as ros2_control."""
    return sign * (math.radians(raw_deg) - q_offset)


def calibrate_dxl(
    raw_pulse: int, zero_offset: float, direction: float, wraparound: bool
) -> float:
    """Convert the raw Dynamixel pulse count into a calibrated joint angle."""
    raw_rad = raw_pulse * DXL_PULSE_TO_RAD - math.pi
    angle = direction * (raw_rad - zero_offset)
    return math.remainder(angle, 2.0 * math.pi) if wraparound else angle


def signed_int32(value: int) -> int:
    """Interpret an unsigned 32-bit SDK result as a signed encoder count."""
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


class RmdJointStateBridge(Node):
    """Poll read-only RMD and Dynamixel feedback into ROS topics."""

    def __init__(self) -> None:
        super().__init__("rmd_joint_state_bridge")

        self.declare_parameter("can_ifname", "can_arm")
        self.declare_parameter("shoulder_actuator_id", 4)
        self.declare_parameter("elbow_actuator_id", 5)
        self.declare_parameter("wrist_actuator_id", 6)
        self.declare_parameter("rmd_wrist_joint_name", "wrist_pitch_joint")
        self.declare_parameter("shoulder_sign", -1.0)
        self.declare_parameter("elbow_sign", -1.0)
        self.declare_parameter("wrist_sign", 1.0)
        self.declare_parameter("shoulder_q_offset", 1.6524777357882314)
        self.declare_parameter("elbow_q_offset", 0.08866272600131195)
        self.declare_parameter("wrist_q_offset", 4.135732038446131)

        self.declare_parameter("dxl_port_name", "/dev/ttyUSB0")
        self.declare_parameter("dxl_baud_rate", 1000000)
        self.declare_parameter("base_dxl_id", 1)
        self.declare_parameter("wrist_roll_dxl_id", 2)
        self.declare_parameter("wrist_yaw_dxl_id", 3)
        self.declare_parameter("gripper_dxl_id", 4)
        self.declare_parameter("base_zero_offset", -3.124718864923051)
        self.declare_parameter("base_direction", 1.0)
        self.declare_parameter("base_wraparound", False)
        self.declare_parameter("wrist_roll_zero_offset", -3.1185829417715083)
        self.declare_parameter("wrist_roll_direction", 1.0)
        self.declare_parameter("wrist_roll_wraparound", False)
        self.declare_parameter("wrist_yaw_zero_offset", -2.8792819388613484)
        self.declare_parameter("wrist_yaw_direction", -1.0)
        self.declare_parameter("wrist_yaw_wraparound", False)
        self.declare_parameter("gripper_zero_offset", -0.085902924122)
        self.declare_parameter("gripper_direction", 1.0)
        self.declare_parameter("gripper_wraparound", False)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._last_warning: dict[str, float] = {}

        can_ifname = str(self.get_parameter("can_ifname").value)
        self.get_logger().info(f"Opening RMD CAN interface '{can_ifname}'...")
        self._can_driver = rmd_sdk_py.CanDriver(can_ifname)
        self._rmd_joints = (
            RmdJoint(
                "shoulder_joint",
                rmd_sdk_py.ActuatorInterface(
                    self._can_driver,
                    int(self.get_parameter("shoulder_actuator_id").value),
                ),
                float(self.get_parameter("shoulder_sign").value),
                float(self.get_parameter("shoulder_q_offset").value),
            ),
            RmdJoint(
                "elbow_joint",
                rmd_sdk_py.ActuatorInterface(
                    self._can_driver,
                    int(self.get_parameter("elbow_actuator_id").value),
                ),
                float(self.get_parameter("elbow_sign").value),
                float(self.get_parameter("elbow_q_offset").value),
            ),
            RmdJoint(
                str(self.get_parameter("rmd_wrist_joint_name").value),
                rmd_sdk_py.ActuatorInterface(
                    self._can_driver,
                    int(self.get_parameter("wrist_actuator_id").value),
                ),
                float(self.get_parameter("wrist_sign").value),
                float(self.get_parameter("wrist_q_offset").value),
            ),
        )

        self._dxl_port: PortHandler | None = None
        self._dxl_packet = PacketHandler(2.0)
        dxl_port_name = str(self.get_parameter("dxl_port_name").value)
        self.get_logger().info(f"Opening Dynamixel port '{dxl_port_name}'...")
        dxl_port = PortHandler(dxl_port_name)
        try:
            port_open = dxl_port.openPort()
            port_ready = port_open and dxl_port.setBaudRate(
                int(self.get_parameter("dxl_baud_rate").value)
            )
        except Exception as exc:
            port_ready = False
            self.get_logger().error(
                f"Dynamixel port '{dxl_port_name}' unavailable: {exc}; "
                "base/wrist_yaw/gripper feedback will be absent"
            )
        if port_ready:
            self._dxl_port = dxl_port
        else:
            self.get_logger().error(
                f"Dynamixel port '{dxl_port_name}' unavailable; "
                "base/wrist_yaw/gripper feedback will be absent"
            )
            try:
                dxl_port.closePort()
            except Exception:
                pass

        self._dxl_joints = [
            DxlJoint(
                "base_joint",
                int(self.get_parameter("base_dxl_id").value),
                float(self.get_parameter("base_zero_offset").value),
                float(self.get_parameter("base_direction").value),
                bool(self.get_parameter("base_wraparound").value),
            )
        ]
        optional_joints = (
            (
                "wrist_roll_joint",
                "wrist_roll_dxl_id",
                "wrist_roll_zero_offset",
                "wrist_roll_direction",
                "wrist_roll_wraparound",
            ),
            (
                "wrist_yaw_joint",
                "wrist_yaw_dxl_id",
                "wrist_yaw_zero_offset",
                "wrist_yaw_direction",
                "wrist_yaw_wraparound",
            ),
            (
                "gripper_joint",
                "gripper_dxl_id",
                "gripper_zero_offset",
                "gripper_direction",
                "gripper_wraparound",
            ),
        )
        for name, id_key, offset_key, direction_key, wrap_key in optional_joints:
            motor_id = int(self.get_parameter(id_key).value)
            if motor_id >= 0:
                self._dxl_joints.append(
                    DxlJoint(
                        name,
                        motor_id,
                        float(self.get_parameter(offset_key).value),
                        float(self.get_parameter(direction_key).value),
                        bool(self.get_parameter(wrap_key).value),
                    )
                )

        self._joint_states_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self._rmd_angle_pub = self.create_publisher(
            Float64MultiArray, "/arm/joint_angle_deg", 10
        )
        self._rmd_raw_angle_pub = self.create_publisher(
            Float64MultiArray, "/arm/rmd_raw_angle_deg", 10
        )
        self._rmd_encoder_pub = self.create_publisher(
            Float64MultiArray, "/arm/rmd_encoder", 10
        )
        self._dxl_angle_pub = self.create_publisher(
            Float64MultiArray, "/arm/dxl_angle_deg", 10
        )
        self._dxl_encoder_pub = self.create_publisher(
            Float64MultiArray, "/arm/dxl_encoder", 10
        )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and greater than zero")
        self._timer = self.create_timer(
            1.0 / rate_hz,
            self._on_timer,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            "rmd_joint_state_bridge ready (read-only, no position commands)."
        )

    def _warn_throttle(self, key: str, message: str) -> None:
        """Log a repeated hardware failure no more than once every two seconds."""
        now = time.monotonic()
        if now - self._last_warning.get(key, -math.inf) >= 2.0:
            self._last_warning[key] = now
            self.get_logger().warning(message)

    def _read_dynamixel_pulse(self, motor_id: int) -> int | None:
        """Read a signed present-position pulse count, or skip failed feedback."""
        if self._dxl_port is None:
            return None
        try:
            raw, comm_result, error = self._dxl_packet.read4ByteTxRx(
                self._dxl_port, motor_id, ADDR_PRESENT_POSITION
            )
        except Exception as exc:
            self._warn_throttle(
                f"dxl-read-{motor_id}", f"Dynamixel id={motor_id} read failed: {exc}"
            )
            return None

        if comm_result != COMM_SUCCESS or error != 0:
            if comm_result == COMM_SUCCESS and error & 0x80:
                try:
                    hardware_status, status_result, _ = self._dxl_packet.read1ByteTxRx(
                        self._dxl_port, motor_id, ADDR_HARDWARE_ERROR_STATUS
                    )
                except Exception:
                    status_result = -1
                    hardware_status = 0
                if status_result == COMM_SUCCESS:
                    self._warn_throttle(
                        f"dxl-hardware-{motor_id}",
                        f"Dynamixel id={motor_id} Hardware Error Status: "
                        f"0x{hardware_status:02X} ({hardware_error_details(hardware_status)}). "
                        "Check the cause and power-cycle the base motor; torque remains "
                        "disabled until reset.",
                    )
            tx_rx_result = self._dxl_packet.getTxRxResult(comm_result)
            self._warn_throttle(
                f"dxl-read-{motor_id}",
                f"Dynamixel id={motor_id} read failed "
                f"(comm={comm_result}, error={error}): {tx_rx_result}",
            )
            return None
        return signed_int32(int(raw))

    def _on_timer(self) -> None:
        """Poll available motors and publish joint states and diagnostic arrays."""
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()

        rmd_angle_deg: list[float] = []
        rmd_raw_angle_deg: list[float] = []
        rmd_encoder: list[float] = []
        for joint in self._rmd_joints:
            try:
                raw_deg = float(joint.actuator.getMultiTurnAngle())
                encoder = int(joint.actuator.getMultiTurnEncoderOriginalPosition())
                calibrated_rad = calibrate_rmd(raw_deg, joint.sign, joint.q_offset)
            except Exception as exc:
                self._warn_throttle(
                    f"rmd-read-{joint.name}", f"RMD '{joint.name}' read failed: {exc}"
                )
                continue

            joint_states.name.append(joint.name)
            joint_states.position.append(calibrated_rad)
            rmd_angle_deg.append(math.degrees(calibrated_rad))
            rmd_raw_angle_deg.append(raw_deg)
            rmd_encoder.append(float(encoder))

        self._rmd_angle_pub.publish(Float64MultiArray(data=rmd_angle_deg))
        self._rmd_raw_angle_pub.publish(Float64MultiArray(data=rmd_raw_angle_deg))
        self._rmd_encoder_pub.publish(Float64MultiArray(data=rmd_encoder))

        dxl_angle_deg: list[float] = []
        dxl_encoder: list[float] = []
        for joint in self._dxl_joints:
            raw_pulse = self._read_dynamixel_pulse(joint.motor_id)
            if raw_pulse is None:
                continue
            calibrated_rad = calibrate_dxl(
                raw_pulse,
                joint.zero_offset,
                joint.direction,
                joint.wraparound,
            )
            joint_states.name.append(joint.name)
            joint_states.position.append(calibrated_rad)
            dxl_angle_deg.append(math.degrees(calibrated_rad))
            dxl_encoder.append(float(raw_pulse))

        self._dxl_angle_pub.publish(Float64MultiArray(data=dxl_angle_deg))
        self._dxl_encoder_pub.publish(Float64MultiArray(data=dxl_encoder))
        if joint_states.name:
            self._joint_states_pub.publish(joint_states)

    def destroy_node(self) -> bool:
        """Close the optional serial port before releasing ROS node resources."""
        if self._dxl_port is not None:
            try:
                self._dxl_port.closePort()
            except Exception as exc:
                self.get_logger().warning(f"Failed to close Dynamixel port: {exc}")
            self._dxl_port = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RmdJointStateBridge | None = None
    try:
        node = RmdJointStateBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("rmd_joint_state_bridge").fatal(
            f"Fatal error: {exc}"
        )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()