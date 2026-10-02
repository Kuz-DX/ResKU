"""Fail-closed, stage-scoped loading for the tool 1 teach-pose file."""
from pathlib import Path
from typing import Any

import yaml


STAGES = (
    "home", "docking_wait", "ee_align", "tool1_pre", "tool1_target", "lock",
    "return_docking_wait", "return_home",
)
STOP_AFTER_STATES = frozenset((*STAGES, "done"))

_NAMED_POSE_BY_STAGE = {
    "home": "home",
    "docking_wait": "docking_wait",
    "tool1_pre": "tool1_pre",
    "return_docking_wait": "docking_wait",
    "return_home": "home",
}


def load_poses(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: root must be a mapping")
    return data


def null_paths(value: Any, path: str = "") -> list[str]:
    """Return dotted paths for every deliberately unset configuration value."""
    if value is None:
        return [path or "<root>"]
    if isinstance(value, dict):
        return [p for key, item in value.items()
                for p in null_paths(item, f"{path}.{key}" if path else str(key))]
    if isinstance(value, list):
        return [p for index, item in enumerate(value)
                for p in null_paths(item, f"{path}[{index}]")]
    return []


def _at_path(data: dict[str, Any], dotted_path: str) -> Any:
    value: Any = data
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            raise KeyError(dotted_path)
        value = value[key]
    return value


def stop_after_state(data: dict[str, Any]) -> str:
    try:
        stop_after = _at_path(data, "development.stop_after_state")
    except KeyError as exc:
        raise ValueError("missing required configuration key: development.stop_after_state") from exc
    if stop_after not in STOP_AFTER_STATES:
        allowed = ", ".join((*STAGES, "done"))
        raise ValueError(f"development.stop_after_state must be one of: {allowed}")
    return str(stop_after)


def _required_paths(stop_after: str) -> list[str]:
    """Return only values needed before the requested development stop."""
    through = len(STAGES) if stop_after == "done" else STAGES.index(stop_after) + 1
    required = [
        "joint_names",
        "motion.ik_limit_margin_rad",
        "feedback.joint_state_max_age_s",
    ]
    for stage in STAGES[:through]:
        required.append(f"timeouts_s.{stage}")
        pose = _NAMED_POSE_BY_STAGE.get(stage)
        if pose is not None:
            required.extend((f"named_poses.{pose}", "motion.named_pose_duration_s"))
        elif stage == "ee_align":
            required.extend(("motion.ee_align.reference_raw",
                             "motion.ee_align.duration_s",
                             "motion.ee_align.tolerance_rad"))
        elif stage == "tool1_target":
            required.extend((
                "cartesian_targets.tool1_target.frame_id",
                "cartesian_targets.tool1_target.position_m",
                "cartesian_targets.tool1_target.orientation_xyzw",
                "motion.insert.speed_m_s",
                "motion.ik_step_m",
                "motion.ik_orientation_step_rad",
            ))
        elif stage == "lock":
            required.extend(("motion.yaw_delta_rad", "motion.lock_duration_s"))
    return required


def validate_request_config(data: dict[str, Any], tool_id: int = 1) -> None:
    """Check the selected tool's required stages before issuing any motion."""
    if tool_id not in (0, 1):
        raise ValueError(f"unsupported tool ID: {tool_id}")
    stop_after = stop_after_state(data)
    missing = []
    for path in _required_paths(stop_after):
        dotted_path = path.replace("tool1_", f"tool{tool_id}_")
        try:
            missing.extend(null_paths(_at_path(data, dotted_path), dotted_path))
        except KeyError:
            missing.append(dotted_path)
    if missing:
        raise ValueError("unset required configuration keys: " + ", ".join(missing))


def require_for_stop_state(path: str | Path) -> dict[str, Any]:
    """Load a safe partial configuration for the selected development stage.

    A stage may not execute until its own pose/motion/timeout values are set.
    Values for later stages are intentionally allowed to remain ``null``.
    """
    data = load_poses(path)
    validate_request_config(data)
    return data
