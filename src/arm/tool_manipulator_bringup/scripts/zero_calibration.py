"""Pure helpers for teaching the current arm pose as joint zero."""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import Mapping, Sequence


RMD_JOINTS = ("shoulder_joint", "elbow_joint", "wrist_pitch_joint")
DXL_JOINTS = ("base_joint", "wrist_roll_joint", "wrist_yaw_joint", "ee_joint")
ARM_JOINTS = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
)


class CalibrationError(ValueError):
    """Raised when calibration input or hardware.yaml is invalid."""


@dataclass(frozen=True)
class ZeroChange:
    joint: str
    field: str
    old_value: float | int
    new_value: float | int
    old_raw_limits: tuple[int, int] | None = None
    new_raw_limits: tuple[int, int] | None = None


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CalibrationError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise CalibrationError(f"{field} must be a finite number")
    return number


def _raw_limits(value: object, field: str) -> tuple[int, int]:
    if not isinstance(value, list) or len(value) != 2:
        raise CalibrationError(f"{field} must contain exactly two integer values")
    limits = []
    for item in value:
        number = _finite_number(item, field)
        if not number.is_integer():
            raise CalibrationError(f"{field} must contain exactly two integer values")
        limits.append(int(number))
    if limits[0] >= limits[1]:
        raise CalibrationError(f"{field} must be ordered [min, max]")
    return limits[0], limits[1]


def plan_zero_update(
    config: Mapping[str, object],
    selected_joints: Sequence[str],
    rmd_raw_deg: Mapping[str, float],
    dxl_raw: Mapping[str, int],
) -> tuple[dict, list[ZeroChange]]:
    """Return an updated config while preserving each joint's usable limit range.

    RMD limits are already expressed relative to joint zero, so only q_offset_rad
    changes. Dynamixel raw limits are absolute encoder pulses, so they move by the
    same pulse delta as zero_raw. soft_limit_rad remains unchanged for both types.
    """
    if not isinstance(config, Mapping):
        raise CalibrationError("hardware config root must be a map")
    joints = config.get("joints")
    if not isinstance(joints, Mapping):
        raise CalibrationError("hardware config must contain a joints map")

    unknown = sorted(set(selected_joints) - set(RMD_JOINTS) - set(DXL_JOINTS))
    if unknown:
        raise CalibrationError(f"unknown joint(s): {', '.join(unknown)}")
    if not selected_joints:
        raise CalibrationError("at least one joint must be selected")
    if len(set(selected_joints)) != len(selected_joints):
        raise CalibrationError("selected joints must not contain duplicates")

    updated = copy.deepcopy(dict(config))
    updated_joints = updated["joints"]
    changes: list[ZeroChange] = []

    for name in selected_joints:
        spec = updated_joints.get(name)
        if not isinstance(spec, dict):
            raise CalibrationError(f"joints.{name} must be a map")

        if name in RMD_JOINTS:
            old_offset = _finite_number(
                spec.get("q_offset_rad"), f"joints.{name}.q_offset_rad")
            if name not in rmd_raw_deg:
                raise CalibrationError(f"no RMD raw angle received for {name}")
            new_offset = math.radians(
                _finite_number(rmd_raw_deg[name], f"raw angle for {name}"))
            spec["q_offset_rad"] = new_offset
            changes.append(ZeroChange(name, "q_offset_rad", old_offset, new_offset))
            continue

        old_zero_number = _finite_number(spec.get("zero_raw"), f"joints.{name}.zero_raw")
        if not old_zero_number.is_integer():
            raise CalibrationError(f"joints.{name}.zero_raw must be an integer")
        if name not in dxl_raw:
            raise CalibrationError(f"no Dynamixel raw pulse received for {name}")
        new_zero_number = _finite_number(dxl_raw[name], f"raw pulse for {name}")
        if not new_zero_number.is_integer():
            raise CalibrationError(f"raw pulse for {name} must be an integer")

        old_zero = int(old_zero_number)
        new_zero = int(new_zero_number)
        old_limits = _raw_limits(
            spec.get("soft_limit_raw"), f"joints.{name}.soft_limit_raw")
        delta = new_zero - old_zero
        new_limits = (old_limits[0] + delta, old_limits[1] + delta)
        spec["zero_raw"] = new_zero
        spec["soft_limit_raw"] = list(new_limits)
        changes.append(ZeroChange(
            name,
            "zero_raw",
            old_zero,
            new_zero,
            old_raw_limits=old_limits,
            new_raw_limits=new_limits,
        ))

    return updated, changes
