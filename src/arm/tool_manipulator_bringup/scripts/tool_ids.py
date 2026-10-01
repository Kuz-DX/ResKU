"""Logical tool/tag IDs shared by the tool-change nodes."""

GRIPPER_TOOL_ID = 0
DRILL_TOOL_ID = 1

# Logical tool IDs and physical AprilTag IDs intentionally match. IDs 2 and
# above are not part of the autonomous tool-change contract.
TOOL_TAG_IDS = {
    GRIPPER_TOOL_ID: 0,
    DRILL_TOOL_ID: 1,
}
SUPPORTED_TOOL_IDS = frozenset(TOOL_TAG_IDS)
SUPPORTED_TAG_IDS = frozenset(TOOL_TAG_IDS.values())

NO_TOOL_ID = 99
UNKNOWN_TOOL_ID = -2
