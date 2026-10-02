"""Pure helpers shared by the current-platform supply grasp controller."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence


def fixed_grasp_z(base_height_m: float, box_height_m: float,
                  grasp_height_ratio: float, tcp_offset_z_m: float = 0.0) -> float:
    """Return a ground-box TCP height expressed in ``base_actuator``."""
    values = (base_height_m, box_height_m, grasp_height_ratio, tcp_offset_z_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("fixed grasp geometry values must be finite")
    if base_height_m < 0.0 or box_height_m <= 0.0:
        raise ValueError("base height must be non-negative and box height positive")
    if not 0.0 <= grasp_height_ratio <= 1.0:
        raise ValueError("box grasp height ratio must be in [0, 1]")
    return box_height_m * grasp_height_ratio - base_height_m + tcp_offset_z_m


def normalized_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError("orientation must contain four finite XYZW values")
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm < 1.0e-12:
        raise ValueError("orientation quaternion must be non-zero")
    return tuple(float(value) / norm for value in values)


def pitch_offset_quaternion(
    orientation_xyzw: Sequence[float], offset_rad: float
) -> tuple[float, float, float, float]:
    """Pre-multiply a base-frame Y rotation onto an XYZW quaternion."""
    x, y, z, w = normalized_quaternion(orientation_xyzw)
    if not math.isfinite(float(offset_rad)):
        raise ValueError("orientation pitch offset must be finite")
    sy = math.sin(float(offset_rad) / 2.0)
    cy = math.cos(float(offset_rad) / 2.0)
    return normalized_quaternion((cy * x + sy * z, cy * y + sy * w,
                                  cy * z - sy * x, cy * w - sy * y))


def checked_joint_vector(names: Sequence[str], values: Iterable[float],
                         limits: Mapping[str, Sequence[float]], label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != len(names) or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {len(names)} finite joint values")
    for name, value in zip(names, result):
        lower, upper = map(float, limits[name])
        if not lower <= value <= upper:
            raise ValueError(
                f"{label}.{name}={value:.6f} outside [{lower:.6f}, {upper:.6f}]")
    return result


def required_duration(names: Sequence[str], current: Sequence[float],
                      target: Sequence[float], velocity_limits: Mapping[str, float],
                      requested_s: float, padding_s: float = 0.5) -> float:
    values = (requested_s, padding_s, *current, *target)
    if len(current) != len(names) or len(target) != len(names):
        raise ValueError("current and target joint vectors must match joint names")
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("trajectory duration inputs must be finite")
    if requested_s <= 0.0 or padding_s < 0.0:
        raise ValueError("requested duration must be positive and padding non-negative")
    duration = float(requested_s)
    for name, start, goal in zip(names, current, target):
        velocity = float(velocity_limits[name])
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError(f"invalid velocity limit for {name}")
        duration = max(duration, abs(float(goal) - float(start)) / velocity + padding_s)
    return duration
