#!/usr/bin/env python3
"""Topic-to-action coordinator for autonomous tag-1 drill attach/detach.

/tool_change/request is a desired final state, not a low-level motion command:
  1  -> drill attached
  99 -> no tool attached (release drill into its rack)
"""
from __future__ import annotations

import json
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String
from tool_manipulator_bringup.action import Dock
from tool_ids import DRILL_TOOL_ID, NO_TOOL_ID, UNKNOWN_TOOL_ID


class DrillToolChangeCoordinator(Node):
    ATTACH = 0
    DETACH = 1

    def __init__(self) -> None:
        super().__init__('drill_tool_change_coordinator')
        self.declare_parameter('request_topic', '/tool_change/request')
        self.declare_parameter('active_tool_id_topic', '/active_tool_id')
        self.declare_parameter('action_name', '/dock')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.active_tool_id = UNKNOWN_TOOL_ID
        self.busy = False
        self.requested_state = UNKNOWN_TOOL_ID
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/tool_change/state', latched)
        self.create_subscription(
            Int32, self.get_parameter('active_tool_id_topic').value,
            self._active_cb, 10, callback_group=self.group)
        self.create_subscription(
            Int32, self.get_parameter('request_topic').value,
            self._request_cb, 10, callback_group=self.group)
        self.client = ActionClient(
            self, Dock, self.get_parameter('action_name').value,
            callback_group=self.group)
        self._publish('unknown', 'waiting_for_active_tool_state')

    def _active_cb(self, msg: Int32) -> None:
        self.active_tool_id = int(msg.data)

    def _request_cb(self, msg: Int32) -> None:
        target = int(msg.data)
        if target not in (DRILL_TOOL_ID, NO_TOOL_ID):
            self._publish('rejected', 'tag1_drill_only', target)
            return
        with self.lock:
            if self.busy:
                self._publish('rejected', 'tool_change_busy', target)
                return
            if self.active_tool_id == UNKNOWN_TOOL_ID:
                self._publish('rejected', 'active_tool_state_unknown', target)
                return
            if self.active_tool_id == target:
                self._publish('complete', 'already_in_requested_state', target)
                return
            if target == DRILL_TOOL_ID and self.active_tool_id != NO_TOOL_ID:
                self._publish('rejected', f'tool_{self.active_tool_id}_must_be_removed_first', target)
                return
            if target == NO_TOOL_ID and self.active_tool_id != DRILL_TOOL_ID:
                self._publish('rejected', 'only_tag1_drill_detach_is_supported', target)
                return
            self.busy = True
            self.requested_state = target
        mode = self.ATTACH if target == DRILL_TOOL_ID else self.DETACH
        self._publish('waiting_for_action', 'dock_action_server', target)
        if not self.client.server_is_ready():
            self.client.wait_for_server(timeout_sec=1.0)
        if not self.client.server_is_ready():
            self._finish(False, 'dock_action_server_unavailable')
            return
        goal = Dock.Goal()
        goal.tool_id = DRILL_TOOL_ID
        goal.mode = mode
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self._finish(False, f'goal_error:{exc}')
            return
        if not handle.accepted:
            self._finish(False, 'dock_goal_rejected')
            return
        self._publish('running', 'dock_goal_accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, message) -> None:
        feedback = message.feedback
        self._publish(feedback.state, feedback.detail)

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:
            self._finish(False, f'result_error:{exc}')
            return
        self.active_tool_id = int(result.active_tool_id)
        self._finish(bool(result.success), result.reason)

    def _finish(self, success: bool, reason: str) -> None:
        target = self.requested_state
        with self.lock:
            self.busy = False
            self.requested_state = UNKNOWN_TOOL_ID
        self._publish('complete' if success else 'failed', reason, target)

    def _publish(self, state: str, reason: str, requested_state: int | None = None) -> None:
        target = self.requested_state if requested_state is None else requested_state
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'active_tool_id': self.active_tool_id,
            'requested_tool_id': target,
            'reason': reason,
        }, separators=(',', ':'))))


def main() -> None:
    rclpy.init()
    node = DrillToolChangeCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
