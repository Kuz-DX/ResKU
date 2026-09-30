#!/usr/bin/env python3
"""Read safety-related hardware values without sending motion commands."""

import argparse
import glob
import math
import socket
import struct
import sys
import time
from pathlib import Path

import yaml


DXL_REGISTERS = {
    "model_number": (0, 2),
    "firmware_version": (6, 1),
    "operating_mode": (11, 1),
    "current_limit_raw": (38, 2),
    "velocity_limit_raw": (44, 4),
    "torque_enable": (64, 1),
    "profile_acceleration_raw": (108, 4),
    "profile_velocity_raw": (112, 4),
}
EXPECTED_DXL_MODELS = {"XH540-W270-T": 1100, "MX-106R": 321, "MX-106T": 321}
RMD_POLICY_FIELDS = (
    "velocity_limit_rad_s",
    "current_limit_a",
    "current_release_threshold_a",
    "current_retreat_velocity_rad_s",
    "current_max_retreat_distance_rad",
    "current_limit_duration_ms",
)
CAN_FRAME = struct.Struct("=IB3x8s")


class ReadError(RuntimeError):
    pass


def default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("tool_manipulator_bringup")) / "config/hardware.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config/hardware.yaml"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Dynamixel safety registers and sample RMD 0x9C status without writes."
    )
    parser.add_argument("--config", type=Path, help="hardware.yaml path")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--dxl-only", action="store_true")
    group.add_argument("--rmd-only", action="store_true")
    parser.add_argument("--rmd-duration", type=float, default=10.0)
    parser.add_argument("--rmd-period", type=float, default=0.10)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def load_config(path):
    try:
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ReadError(f"cannot load {path}: {exc}") from exc
    if not isinstance(config, dict) or not isinstance(config.get("buses"), dict):
        raise ReadError(f"invalid hardware config: {path}")
    if not isinstance(config.get("joints"), dict):
        raise ReadError(f"missing joints mapping: {path}")
    return config


def vendor_joints(config, vendor):
    return [
        (name, joint) for name, joint in config["joints"].items()
        if isinstance(joint, dict) and joint.get("vendor") == vendor
    ]


def print_dry_run(config, use_dxl, use_rmd):
    print("DRY RUN: no serial/CAN device will be opened.\n")
    if use_dxl:
        bus = config["buses"].get("dynamixel_u2d2", {})
        print(f"Dynamixel: {bus.get('device')} @ {bus.get('baud_rate')}, protocol {bus.get('protocol')}")
        for name, joint in vendor_joints(config, "dynamixel"):
            print(f"  {name}: ID {joint.get('id')}, model {joint.get('model')}")
        for field, (address, size) in DXL_REGISTERS.items():
            print(f"  {field}: address={address}, size={size}")
        print()
    if use_rmd:
        bus = config["buses"].get("rmd_can", {})
        print(f"RMD: {bus.get('interface')} @ {bus.get('bitrate')} bit/s")
        for name, joint in vendor_joints(config, "rmd"):
            motor_id = int(joint["id"])
            print(f"  {name}: ID {motor_id}, request=0x{0x140 + motor_id:03X}, response=0x{0x240 + motor_id:03X}, command=0x9C")


def dxl_read(packet, port, motor_id, field):
    address, size = DXL_REGISTERS[field]
    method = {1: packet.read1ByteTxRx, 2: packet.read2ByteTxRx, 4: packet.read4ByteTxRx}[size]
    value, comm_result, packet_error = method(port, motor_id, address)
    if comm_result != 0:
        raise ReadError(f"{field}: {packet.getTxRxResult(comm_result)}")
    if packet_error:
        raise ReadError(f"{field}: {packet.getRxPacketError(packet_error)}")
    return int(value)


def read_dynamixel(config):
    try:
        import dynamixel_sdk
    except ImportError as exc:
        raise ReadError("dynamixel_sdk unavailable; source /opt/ros/humble/setup.bash") from exc
    bus = config["buses"].get("dynamixel_u2d2", {})
    device = str(bus.get("device", ""))
    if not device or not Path(device).exists():
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*"))
        candidate_text = ", ".join(candidates) if candidates else "none"
        raise ReadError(
            f"configured Dynamixel device does not exist: {device or '<empty>'}; "
            f"serial candidates: {candidate_text}. Connect/power U2D2 and check its USB cable."
        )
    port = dynamixel_sdk.PortHandler(device)
    packet = dynamixel_sdk.PacketHandler(float(bus.get("protocol", 2.0)))
    try:
        opened = port.openPort()
    except Exception as exc:
        raise ReadError(
            f"cannot open {device}: {exc}; stop other hardware nodes and check dialout permission"
        ) from exc
    if not opened:
        raise ReadError(f"cannot open {device}; stop other hardware nodes and check dialout permission")
    values, errors = {}, []
    try:
        if not port.setBaudRate(int(bus.get("baud_rate", 0))):
            raise ReadError(f"cannot set baud rate on {device}")
        for name, joint in vendor_joints(config, "dynamixel"):
            motor_id = int(joint["id"])
            try:
                item = {field: dxl_read(packet, port, motor_id, field) for field in DXL_REGISTERS}
                expected = EXPECTED_DXL_MODELS.get(str(joint.get("model")))
                if expected is not None and item["model_number"] != expected:
                    raise ReadError(f"model {item['model_number']} != expected {expected}; table not trusted")
                values[name] = item
            except (KeyError, TypeError, ValueError, ReadError) as exc:
                errors.append(f"{name} (ID {motor_id}): {exc}")
    finally:
        port.closePort()
    return values, errors


def print_dynamixel(values, errors):
    print("=== Dynamixel register read ===")
    for name, item in values.items():
        print(f"{name}: model={item['model_number']}, firmware={item['firmware_version']}, operating_mode={item['operating_mode']}, torque_enable={item['torque_enable']}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    print("\n# Copy candidate; verify motor specification and safety margin first.")
    print("joints:")
    for name, item in values.items():
        print(f"  {name}:")
        for field in ("current_limit_raw", "velocity_limit_raw", "profile_acceleration_raw", "profile_velocity_raw"):
            print(f"    {field}: {item[field]}")
    print()


def receive_rmd_status(can_socket, response_id, deadline):
    while time.monotonic() < deadline:
        can_socket.settimeout(max(0.001, deadline - time.monotonic()))
        try:
            frame = can_socket.recv(CAN_FRAME.size)
        except socket.timeout as exc:
            raise ReadError(f"timeout waiting for 0x{response_id:03X}") from exc
        can_id, dlc, data = CAN_FRAME.unpack(frame)
        if (can_id & 0x7FF) != response_id or dlc < 8 or data[0] != 0x9C:
            continue
        temperature_c = struct.unpack_from("<b", data, 1)[0]
        current_a = struct.unpack_from("<h", data, 2)[0] * 0.01
        velocity_rad_s = math.radians(struct.unpack_from("<h", data, 4)[0])
        return current_a, velocity_rad_s, temperature_c
    raise ReadError(f"timeout waiting for 0x{response_id:03X}")


def sample_rmd(config, duration, period):
    if duration <= 0 or period <= 0:
        raise ReadError("--rmd-duration and --rmd-period must be greater than zero")
    interface = str(config["buses"].get("rmd_can", {}).get("interface", ""))
    if not interface:
        raise ReadError("buses.rmd_can.interface is missing")
    joints = vendor_joints(config, "rmd")
    observed = {
        name: {"samples": 0, "peak_abs_current_a": 0.0, "peak_abs_velocity_rad_s": 0.0,
               "min_temperature_c": None, "max_temperature_c": None}
        for name, _ in joints
    }
    errors = []
    try:
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.bind((interface,))
    except OSError as exc:
        raise ReadError(f"cannot open SocketCAN {interface}: {exc}") from exc
    end_time = time.monotonic() + duration
    try:
        while time.monotonic() < end_time:
            round_start = time.monotonic()
            for name, joint in joints:
                motor_id = int(joint["id"])
                try:
                    request = bytes((0x9C, 0, 0, 0, 0, 0, 0, 0))
                    can_socket.send(CAN_FRAME.pack(0x140 + motor_id, 8, request))
                    current, velocity, temperature = receive_rmd_status(
                        can_socket, 0x240 + motor_id, time.monotonic() + max(0.05, period))
                    item = observed[name]
                    item["samples"] += 1
                    item["peak_abs_current_a"] = max(item["peak_abs_current_a"], abs(current))
                    item["peak_abs_velocity_rad_s"] = max(item["peak_abs_velocity_rad_s"], abs(velocity))
                    item["min_temperature_c"] = temperature if item["min_temperature_c"] is None else min(item["min_temperature_c"], temperature)
                    item["max_temperature_c"] = temperature if item["max_temperature_c"] is None else max(item["max_temperature_c"], temperature)
                except (OSError, ValueError, ReadError) as exc:
                    errors.append(f"{name} (ID {motor_id}): {exc}")
            time.sleep(max(0.0, period - (time.monotonic() - round_start)))
    finally:
        can_socket.close()
    return observed, errors


def print_rmd(observed, errors):
    print("=== RMD 0x9C observations ===")
    print("# Measured peaks only; these are NOT safe limits.")
    print("rmd_observed:")
    for name, item in observed.items():
        print(f"  {name}:")
        print(f"    samples: {item['samples']}")
        print(f"    peak_abs_current_a: {item['peak_abs_current_a']:.3f}")
        print(f"    peak_abs_velocity_rad_s: {item['peak_abs_velocity_rad_s']:.6f}")
        print(f"    min_temperature_c: {item['min_temperature_c']}")
        print(f"    max_temperature_c: {item['max_temperature_c']}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    print("\n# RMD software protection policy: engineering decision required")
    print("joints:")
    for name in observed:
        print(f"  {name}:")
        for field in RMD_POLICY_FIELDS:
            print(f"    {field}: null")
    print("\nRMD 관측 최대값을 그대로 리밋으로 사용하지 마십시오. 정격/피크 전류, 감속기·링크 허용 하중, 정상 궤적 로그와 안전 여유로 보호값을 결정해야 합니다.")


def main():
    args = parse_args()
    path = args.config.resolve() if args.config else default_config_path()
    try:
        config = load_config(path)
        use_dxl, use_rmd = not args.rmd_only, not args.dxl_only
        print(f"config: {path}")
        print("READ ONLY: no register/config write and no motion command is sent.")
        print("Run exclusively: stop real_control, TCP teleop and joint-state bridges first.\n")
        if args.dry_run:
            print_dry_run(config, use_dxl, use_rmd)
            return 0
        failed = False
        if use_dxl:
            values, errors = read_dynamixel(config)
            print_dynamixel(values, errors)
            failed = failed or bool(errors) or not values
        if use_rmd:
            observed, errors = sample_rmd(config, args.rmd_duration, args.rmd_period)
            print_rmd(observed, errors)
            failed = failed or bool(errors) or any(item["samples"] == 0 for item in observed.values())
        return 2 if failed else 0
    except (KeyError, TypeError, ValueError, ReadError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
