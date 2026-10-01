#!/usr/bin/env python3
"""Teach the current, stationary arm pose as zero in hardware.yaml.

This node only consumes the read-only rmd_joint_state_bridge diagnostic topics.
It never opens a motor device and never publishes a motor command.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import statistics
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray
import yaml

from zero_calibration import (
    ARM_JOINTS,
    DXL_JOINTS,
    RMD_JOINTS,
    CalibrationError,
    ZeroChange,
    plan_zero_update,
)


class CurrentZeroCapture(Node):
    def __init__(self, sample_count: int) -> None:
        super().__init__("set_current_zero")
        self._sample_count = sample_count
        self.rmd_samples: list[tuple[float, ...]] = []
        self.dxl_samples: list[tuple[int, ...]] = []
        self.create_subscription(
            Float64MultiArray, "/arm/rmd_raw_angle_deg", self._rmd_callback, 10)
        self.create_subscription(
            Float64MultiArray, "/arm/dxl_encoder", self._dxl_callback, 10)

    def _rmd_callback(self, message: Float64MultiArray) -> None:
        if len(self.rmd_samples) >= self._sample_count:
            return
        if len(message.data) != len(RMD_JOINTS):
            self.get_logger().warn(
                "RMD raw sample rejected: expected 3 values, got %d" % len(message.data),
                throttle_duration_sec=1.0,
            )
            return
        values = tuple(float(value) for value in message.data)
        if all(math.isfinite(value) for value in values):
            self.rmd_samples.append(values)

    def _dxl_callback(self, message: Float64MultiArray) -> None:
        if len(self.dxl_samples) >= self._sample_count:
            return
        if len(message.data) != len(DXL_JOINTS):
            self.get_logger().warn(
                "Dynamixel raw sample rejected: expected 4 values, got %d" % len(message.data),
                throttle_duration_sec=1.0,
            )
            return
        values = tuple(float(value) for value in message.data)
        if not all(math.isfinite(value) and abs(value - round(value)) <= 0.25 for value in values):
            self.get_logger().warn(
                "Dynamixel raw sample rejected: values must be finite integer pulses",
                throttle_duration_sec=1.0,
            )
            return
        self.dxl_samples.append(tuple(round(value) for value in values))


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, required=True,
        help="hardware.yaml to inspect/update (use the source-tree file)",
    )
    parser.add_argument(
        "--joints", nargs="+", choices=ARM_JOINTS + ("ee_joint",),
        default=list(ARM_JOINTS),
        help="joints to zero; default is the six arm joints (ee_joint excluded)",
    )
    parser.add_argument(
        "--samples", type=int, default=10,
        help="number of consecutive bridge samples to inspect (default: 10)",
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0,
        help="seconds to wait for complete raw samples (default: 5.0)",
    )
    parser.add_argument(
        "--max-rmd-spread-deg", type=float, default=0.05,
        help="maximum accepted sample spread per RMD axis (default: 0.05 deg)",
    )
    parser.add_argument(
        "--max-dxl-spread", type=int, default=2,
        help="maximum accepted sample spread per Dynamixel axis (default: 2 pulses)",
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="write hardware.yaml; without this flag only preview changes",
    )
    args = parser.parse_args(remove_ros_args(argv)[1:])
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.max_rmd_spread_deg < 0.0:
        parser.error("--max-rmd-spread-deg must be nonnegative")
    if args.max_dxl_spread < 0:
        parser.error("--max-dxl-spread must be nonnegative")
    if len(set(args.joints)) != len(args.joints):
        parser.error("--joints must not contain duplicates")
    return args


def _needs_rmd(joints: list[str]) -> bool:
    return any(name in RMD_JOINTS for name in joints)


def _needs_dxl(joints: list[str]) -> bool:
    return any(name in DXL_JOINTS for name in joints)


def _collect(node: CurrentZeroCapture, args: argparse.Namespace) -> None:
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rmd_ready = not _needs_rmd(args.joints) or len(node.rmd_samples) >= args.samples
        dxl_ready = not _needs_dxl(args.joints) or len(node.dxl_samples) >= args.samples
        if rmd_ready and dxl_ready:
            return
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))

    missing = []
    if _needs_rmd(args.joints) and len(node.rmd_samples) < args.samples:
        missing.append(
            f"/arm/rmd_raw_angle_deg ({len(node.rmd_samples)}/{args.samples} samples)")
    if _needs_dxl(args.joints) and len(node.dxl_samples) < args.samples:
        missing.append(f"/arm/dxl_encoder ({len(node.dxl_samples)}/{args.samples} samples)")
    raise CalibrationError("timed out waiting for " + ", ".join(missing))


def _median_and_validate(
    node: CurrentZeroCapture, args: argparse.Namespace,
) -> tuple[dict[str, float], dict[str, int]]:
    rmd_values: dict[str, float] = {}
    dxl_values: dict[str, int] = {}

    if _needs_rmd(args.joints):
        for index, name in enumerate(RMD_JOINTS):
            values = [sample[index] for sample in node.rmd_samples]
            spread = max(values) - min(values)
            if name in args.joints and spread > args.max_rmd_spread_deg:
                raise CalibrationError(
                    f"{name} moved during capture: spread {spread:.6f} deg exceeds "
                    f"{args.max_rmd_spread_deg:.6f} deg")
            rmd_values[name] = float(statistics.median(values))

    if _needs_dxl(args.joints):
        for index, name in enumerate(DXL_JOINTS):
            values = [sample[index] for sample in node.dxl_samples]
            spread = max(values) - min(values)
            if name in args.joints and spread > args.max_dxl_spread:
                raise CalibrationError(
                    f"{name} moved during capture: spread {spread} pulses exceeds "
                    f"{args.max_dxl_spread} pulses")
            dxl_values[name] = round(statistics.median(values))

    return rmd_values, dxl_values


def _load_config(path: Path) -> dict:
    try:
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise CalibrationError(f"cannot read {path}: {exc}") from exc
    if not isinstance(config, dict):
        raise CalibrationError(f"{path} must contain a YAML map")
    return config


def _write_config(path: Path, config: dict) -> Path:
    if not path.is_file():
        raise CalibrationError(f"config is not a regular file: {path}")
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = path.with_name(f"{path.name}.bak-{timestamp}")
    temporary_name: str | None = None
    try:
        shutil.copy2(path, backup)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as stream:
            temporary_name = stream.name
            yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_name, path.stat().st_mode)
        os.replace(temporary_name, path)
    except OSError as exc:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink()
            except OSError:
                pass
        raise CalibrationError(f"cannot update {path}: {exc}") from exc
    return backup


def _print_changes(changes: list[ZeroChange]) -> None:
    print("\n영점 변경 예정값:")
    for change in changes:
        if change.field == "q_offset_rad":
            print(
                f"  {change.joint}: q_offset_rad "
                f"{change.old_value:.12f} -> {change.new_value:.12f}")
        else:
            print(
                f"  {change.joint}: zero_raw {change.old_value} -> {change.new_value}; "
                f"soft_limit_raw {list(change.old_raw_limits or ())} -> "
                f"{list(change.new_raw_limits or ())}")
    print("  soft_limit_rad: 변경 없음")


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv if argv is None else argv
    args = _parse_args(argv)
    path = args.config.expanduser().resolve()
    try:
        config = _load_config(path)
        rclpy.init(args=argv)
        node = CurrentZeroCapture(args.samples)
        try:
            print("현재 자세를 유지한 채 raw 엔코더 샘플을 수집합니다...")
            _collect(node, args)
            rmd_values, dxl_values = _median_and_validate(node, args)
        finally:
            node.destroy_node()
            rclpy.shutdown()

        updated, changes = plan_zero_update(
            config, args.joints, rmd_values, dxl_values)
        print(f"config: {path}")
        _print_changes(changes)
        if not args.apply:
            print("\n미리보기만 완료했습니다. 저장하려면 같은 명령에 --apply를 추가하세요.")
            return 0

        backup = _write_config(path, updated)
        _load_config(path)
        print(f"\n저장 완료: {path}")
        print(f"백업 파일: {backup}")
        print("새 영점은 다음 real_control 기동부터 적용됩니다.")
        return 0
    except (CalibrationError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
