#!/usr/bin/env python3
"""High-level, fail-closed coordinator for attach/release tool changes.

It owns request admission and persistent user-visible state.  Motion remains in
apriltag_tool_docking_node; planning-scene ownership remains in
ToolAttachmentManager.  A collision object is detached only after an external
physical release confirmation arrives at ToolAttachmentManager.
"""
from __future__ import annotations

import json
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32, String


class ChangeState(str, Enum):
    IDLE = 'idle'
    ATTACH_REQUESTED = 'attach_requested'
    DOCKING = 'docking'
    ATTACHED = 'attached'
    RELEASE_REQUESTED = 'release_requested'
    WAITING_FOR_RELEASE_CONFIRMATION = 'waiting_for_release_confirmation'
    CANCELLED_HOLD = 'cancelled_hold'
    FAILED = 'failed'


class ToolChangeStateMachine(Node):
    """Coordinate UI intent without inventing a physical release completion."""

    def __init__(self):
        super().__init__('tool_change_state_machine')
        for name, value in (
            ('command_topic', '/tool_change/command'),
            ('selected_tool_id_topic', '/selected_tool_id'),
            ('active_tool_id_topic', '/active_tool_id'),
            ('docking_status_topic', '/docking_status'),
            ('attachment_status_topic', '/tool_attachment_status'),
            ('release_request_topic', '/tool_release_request'),
            ('cancel_topic', '/tool_change/cancel'),
        ):
            self.declare_parameter(name, value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/tool_change/state', latched)
        self.selected_pub = self.create_publisher(
            Int32, self.get_parameter('selected_tool_id_topic').value, 10)
        self.release_pub = self.create_publisher(
            Bool, self.get_parameter('release_request_topic').value, latched)
        self.cancel_pub = self.create_publisher(
            Bool, self.get_parameter('cancel_topic').value, latched)
        self.create_subscription(String, self.get_parameter('command_topic').value, self._command_cb, 10)
        self.create_subscription(Int32, self.get_parameter('active_tool_id_topic').value, self._active_cb, 10)
        self.create_subscription(String, self.get_parameter('docking_status_topic').value, self._docking_cb, 10)
        self.create_subscription(String, self.get_parameter('attachment_status_topic').value, self._attachment_cb, 10)
        self.state = ChangeState.IDLE
        self.active_tool_id = 0
        self.requested_tool_id: int | None = None
        self._publish('ready')

    def _command_cb(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command.startswith('attach:'):
            try:
                self._request_attach(int(command.split(':', 1)[1]))
            except ValueError:
                self._reject('attach_requires_integer_tool_id')
        elif command == 'release':
            self._request_release()
        elif command == 'cancel':
            self._cancel()
        elif command == 'reset':
            self._reject('restart_docking_node_required_after_cancel')
        else:
            self._reject('unknown_command')

    def _request_attach(self, tool_id: int) -> None:
        if tool_id <= 0:
            self._reject('tool_id_must_be_positive')
        elif self.active_tool_id:
            self._reject(f'tool_{self.active_tool_id}_must_be_released_first')
        elif self.state not in (ChangeState.IDLE, ChangeState.FAILED, ChangeState.CANCELLED_HOLD):
            self._reject('change_already_in_progress')
        else:
            self.requested_tool_id = tool_id
            self.cancel_pub.publish(Bool(data=False))
            self._set(ChangeState.ATTACH_REQUESTED, f'awaiting_docking:tool_{tool_id}')
            # Existing docking + attachment nodes use this established contract.
            self.selected_pub.publish(Int32(data=tool_id))

    def _request_release(self) -> None:
        if not self.active_tool_id:
            self._reject('no_active_tool')
        elif self.state not in (ChangeState.ATTACHED, ChangeState.CANCELLED_HOLD, ChangeState.FAILED):
            self._reject('change_already_in_progress')
        else:
            self._set(ChangeState.RELEASE_REQUESTED, f'tool_{self.active_tool_id}')
            # A future physical release executor must move through its taught
            # release path and publish /tool_release_complete only after unlock.
            self.release_pub.publish(Bool(data=True))
            self._set(ChangeState.WAITING_FOR_RELEASE_CONFIRMATION, 'physical_unlock_required')

    def _cancel(self) -> None:
        # This is a request-to-hold. The docking node stops Servo motion
        # immediately; an in-flight FollowJointTrajectory is not falsely claimed
        # cancelled until its controller exposes a real cancel result.
        self.cancel_pub.publish(Bool(data=True))
        self.selected_pub.publish(Int32(data=0))
        self.requested_tool_id = None
        self._set(ChangeState.CANCELLED_HOLD, 'hold_requested')

    def _active_cb(self, msg: Int32) -> None:
        previous = self.active_tool_id
        self.active_tool_id = int(msg.data)
        if self.active_tool_id and self.state in (ChangeState.ATTACH_REQUESTED, ChangeState.DOCKING):
            self._set(ChangeState.ATTACHED, f'tool_{self.active_tool_id}_collision_attached')
        elif previous and not self.active_tool_id and self.state == ChangeState.WAITING_FOR_RELEASE_CONFIRMATION:
            self.requested_tool_id = None
            self._set(ChangeState.IDLE, 'physical_release_confirmed_collision_detached')

    def _docking_cb(self, msg: String) -> None:
        text = msg.data
        if text.startswith('failed:'):
            self._set(ChangeState.FAILED, text)
        elif self.state == ChangeState.ATTACH_REQUESTED and (
                text.startswith('coarse_moving') or text in ('fine_xy_aligning', 'descending', 'lock_rotating')):
            self._set(ChangeState.DOCKING, text)

    def _attachment_cb(self, msg: String) -> None:
        text = msg.data
        if text.startswith(('attach_failed:', 'detach_failed:', 'rejected:')):
            self._set(ChangeState.FAILED, text)

    def _reject(self, reason: str) -> None:
        self._publish(f'rejected:{reason}')

    def _set(self, state: ChangeState, reason: str) -> None:
        self.state = state
        self._publish(reason)

    def _publish(self, reason: str) -> None:
        self.state_pub.publish(String(data=json.dumps({
            'state': self.state.value,
            'active_tool_id': self.active_tool_id,
            'requested_tool_id': self.requested_tool_id,
            'reason': reason,
        }, separators=(',', ':'))))


def main() -> None:
    rclpy.init()
    node = ToolChangeStateMachine()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
