#!/usr/bin/env python3
"""Pure helpers shared by arm named-pose tools.

This module deliberately has no ROS imports so configuration checks can run in
unit tests and before a node is started.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


def load_soft_limits(path: Path, joint_names: Sequence[str]) -> dict[str, tuple[float, float]]:
    """Load finite, ordered soft limits for exactly the requested joints."""
    try:
        with path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: invalid YAML: {exc}") from exc
    if not isinstance(config, dict):
        raise ValueError(f"{path}: YAML root must be a mapping")

    joints = config.get("joints")
    if not isinstance(joints, dict):
        raise ValueError(f"{path}: joints mapping is required")

    limits: dict[str, tuple[float, float]] = {}
    for name in joint_names:
        spec = joints.get(name)
        raw = spec.get("soft_limit_rad") if isinstance(spec, dict) else None
        if not isinstance(raw, list) or len(raw) != 2:
            raise ValueError(f"{path}: {name}.soft_limit_rad must contain [lower, upper]")
        lower, upper = (float(raw[0]), float(raw[1]))
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError(f"{path}: invalid {name}.soft_limit_rad={raw}")
        limits[name] = (lower, upper)
    return limits


def validate_joint_positions(
    positions: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
    margin_rad: float = 0.0,
    *,
    label: str = "pose",
) -> None:
    """Reject missing, non-finite, or limit-adjacent joint positions."""
    if not math.isfinite(margin_rad) or margin_rad < 0.0:
        raise ValueError("limit margin must be finite and nonnegative")

    errors = []
    for name, (lower, upper) in limits.items():
        if margin_rad * 2.0 >= upper - lower:
            errors.append(
                f"{name}: margin {margin_rad:.6f} leaves no valid interval "
                f"inside [{lower:.6f}, {upper:.6f}]"
            )
            continue
        value = positions.get(name)
        if value is None or not math.isfinite(float(value)):
            errors.append(f"{name}: finite position is required")
            continue
        value = float(value)
        safe_lower = lower + margin_rad
        safe_upper = upper - margin_rad
        if value < safe_lower or value > safe_upper:
            errors.append(
                f"{name}={value:.6f} outside "
                f"[{safe_lower:.6f}, {safe_upper:.6f}]"
            )
    if errors:
        raise ValueError(f"{label} violates arm soft limits: " + "; ".join(errors))


def pose_errors(
    current: Mapping[str, float],
    target: Mapping[str, float],
    joint_names: Sequence[str],
) -> dict[str, float]:
    """Return absolute per-joint errors, rejecting incomplete states."""
    errors: dict[str, float] = {}
    for name in joint_names:
        actual = current.get(name)
        goal = target.get(name)
        if actual is None or goal is None:
            raise ValueError(f"missing joint position: {name}")
        actual = float(actual)
        goal = float(goal)
        if not (math.isfinite(actual) and math.isfinite(goal)):
            raise ValueError(f"non-finite joint position: {name}")
        errors[name] = abs(actual - goal)
    return errors
