"""Canonical task IDs shared by the arm Python nodes."""

GRIPPER = 0
DRILL = 1
SUPPLY_BOX = 2

TOOL_NAMES = {GRIPPER: "gripper", DRILL: "drill", SUPPLY_BOX: "supply_box"}
TOOL_IDS = tuple(TOOL_NAMES)


def tool_name(tool_id):
    try:
        return TOOL_NAMES[tool_id]
    except (KeyError, TypeError) as exc:
        raise ValueError(f"unknown tool id {tool_id!r}; expected 0, 1 or 2") from exc


def tool_label(tool_id):
    return f"tool{tool_id} ({tool_name(tool_id)})"
