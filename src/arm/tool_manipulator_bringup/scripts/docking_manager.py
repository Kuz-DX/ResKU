#!/usr/bin/env python3
"""Action-level admission, cancellation, and outcome authority for tool docking."""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import yaml
from enum import Enum

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32, String
from tool_manipulator_bringup.action import Dock


class State(str, Enum):
    IDLE = 'idle'
    ATTACH_REQUESTED = 'attach_requested'
    DOCKING = 'docking'
    ATTACHED = 'attached'
    UNKNOWN = 'unknown'
    CANCELLED_HOLD = 'cancelled_hold'
    FAILED = 'failed'


class DockingManager(Node):
    """Single owner of external docking requests; motion executor stays separate."""
    ATTACH, DETACH = 0, 1
    NONE, UNKNOWN = -1, -2

    def __init__(self) -> None:
        super().__init__('docking_manager')
        for name, default in (
            ('action_name', '/dock'), ('tools_config_file', ''), ('selected_tool_id_topic', '/selected_tool_id'),
            ('active_tool_id_topic', '/active_tool_id'), ('motion_status_topic', '/docking_motion_status'),
            ('attachment_status_topic', '/tool_attachment_status'),
('cancel_topic', '/tool_change/cancel'),
            ('action_timeout_sec', 120.0),
        ):
            self.declare_parameter(name, default)
        self.enabled_tools = self._load_enabled_tools(self.get_parameter('tools_config_file').value)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.state, self.reason = State.UNKNOWN, 'restart_requires_operator_recovery'
        self.active_tool_id, self.requested_tool_id = self.UNKNOWN, self.NONE
        self.active_goal = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/docking_status', latched)
        self.selected_pub = self.create_publisher(Int32, self.get_parameter('selected_tool_id_topic').value, 10)
        self.cancel_pub = self.create_publisher(Bool, self.get_parameter('cancel_topic').value, latched)
        self.create_subscription(Int32, self.get_parameter('active_tool_id_topic').value, self._active_cb, 10, callback_group=self.group)
        self.create_subscription(String, self.get_parameter('motion_status_topic').value, self._motion_cb, 10, callback_group=self.group)
        self.create_subscription(String, self.get_parameter('attachment_status_topic').value, self._scene_cb, 10, callback_group=self.group)
        self.server = ActionServer(self, Dock, self.get_parameter('action_name').value,
                                   execute_callback=self._execute, goal_callback=self._goal,
                                   cancel_callback=self._cancel, callback_group=self.group)
        self._publish()

    @staticmethod
    def _load_enabled_tools(filename: str) -> set[int]:
        if not filename:
            raise RuntimeError('tools_config_file is required')
        try:
            with Path(filename).open(encoding='utf-8') as stream:
                tools = (yaml.safe_load(stream) or {}).get('tools', {})
            return {int(tool_id) for tool_id, spec in tools.items() if isinstance(spec, dict) and spec.get('enabled', False)}
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(f'cannot read tools_config_file: {exc}') from exc

    def _goal(self, goal) -> GoalResponse:
        with self.lock:
            valid = (goal.mode == self.ATTACH and goal.tool_id in self.enabled_tools and self.active_tool_id == self.NONE)
            if self.active_goal is not None or not valid:
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        self.cancel_pub.publish(Bool(data=True))
        self._set(State.CANCELLED_HOLD, 'cancel_requested_hold')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            self.active_goal = goal_handle
        if goal.mode == self.ATTACH:
            self.requested_tool_id = int(goal.tool_id)
            self.cancel_pub.publish(Bool(data=False))
            self._set(State.ATTACH_REQUESTED, f'tool_{goal.tool_id}')
            self.selected_pub.publish(Int32(data=goal.tool_id))

        deadline = time.monotonic() + float(self.get_parameter('action_timeout_sec').value)
        while rclpy.ok() and time.monotonic() < deadline:
            feedback = Dock.Feedback()
            feedback.state, feedback.active_tool_id, feedback.detail = self.state.value, self.active_tool_id, self.reason
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested or self.state == State.CANCELLED_HOLD:
                goal_handle.canceled()
                return self._result(False, 'cancelled_hold')
            if goal.mode == self.ATTACH and self.state == State.ATTACHED and self.active_tool_id == goal.tool_id:
                goal_handle.succeed()
                return self._result(True, 'attached')
            if self.state in (State.FAILED, State.UNKNOWN):
                goal_handle.abort()
                return self._result(False, self.reason)
            self.changed.wait(timeout=0.1)
            self.changed.clear()
        self.cancel_pub.publish(Bool(data=True))
        self._set(State.CANCELLED_HOLD, 'action_timeout_hold')
        goal_handle.abort()
        return self._result(False, 'timeout_hold')

    def _result(self, success: bool, reason: str):
        result = Dock.Result()
        result.success, result.active_tool_id = success, self.active_tool_id
        result.final_state, result.reason = self.state.value, reason
        with self.lock:
            self.active_goal = None
            self.requested_tool_id = self.NONE
        return result

    def _active_cb(self, msg: Int32) -> None:
        self.active_tool_id = int(msg.data)
        if self.active_tool_id == self.UNKNOWN:
            self._set(State.UNKNOWN, 'tool_state_unknown_recovery_required')
        elif self.active_tool_id >= 0 and self.requested_tool_id == self.active_tool_id:
            self._set(State.ATTACHED, 'scene_attached_after_physical_lock')
        elif self.active_tool_id == self.NONE and self.state == State.UNKNOWN:
            self._set(State.IDLE, 'operator_confirmed_empty')

    def _motion_cb(self, msg: String) -> None:
        if msg.data.startswith('failed:'):
            self._set(State.FAILED, msg.data)
        elif self.state == State.ATTACH_REQUESTED and msg.data.startswith(('coarse_moving', 'fine_xy', 'descending', 'lock_rotating', 'retreat')):
            self._set(State.DOCKING, msg.data)

    def _scene_cb(self, msg: String) -> None:
        if msg.data.startswith('unknown:'):
            self._set(State.UNKNOWN, msg.data)
        elif msg.data.startswith('recovered:') and self.active_tool_id == self.NONE:
            self._set(State.IDLE, msg.data)
        elif msg.data.startswith(('attach_failed:', 'detach_failed:', 'rejected:')):
            self._set(State.FAILED, msg.data)

    def _set(self, state: State, reason: str) -> None:
        self.state, self.reason = state, reason
        self.changed.set()
        self._publish()

    def _publish(self) -> None:
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state.value, 'active_tool_id': self.active_tool_id,
            'requested_tool_id': self.requested_tool_id, 'reason': self.reason,
        }, separators=(',', ':'))))


def main() -> None:
    rclpy.init()
    node = DockingManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
