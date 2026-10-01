#!/usr/bin/env python3
"""Execute the UI-triggered tag-1 approach, alignment, and short descent.

The UI publishes the selected AprilTag ID to ``/selected_tool_id``.  A tag-1
request is accepted once per 99/other-ID -> 1 transition.  The node sends the
taught joint-space stages through arm_controller, waits for visual-servo's
stable alignment acknowledgement, then sends a bounded downward Servo twist.
"""
from __future__ import annotations

import math
import threading
import time
from enum import Enum, auto

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class State(Enum):
    IDLE = auto()
    HOME = auto()
    BACK = auto()
    TAG1_WAIT = auto()
    ALIGNING = auto()
    DESCENDING = auto()
    COMPLETE = auto()
    HOLD = auto()


class UiTag1Docking(Node):
    """A small, UI-owned state machine for a single taught tag-1 approach."""

    def __init__(self) -> None:
        super().__init__('ui_tag1_docking')
        self._declare_parameters()
        self._validate_parameters()
        self._lock = threading.RLock()
        self.state = State.IDLE
        self.request_armed = True
        self.cancelled = threading.Event()
        self.latest_positions: dict[str, float] = {}
        self.latest_joint_state_time = 0.0
        self.align_started = 0.0
        self.descent_started = 0.0

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('arm_action').value)
        self.ee_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter('ee_action').value)
        self.visual_servo_client = self.create_client(
            SetBool, self.get_parameter('visual_servo_enable_service').value)
        self.twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter('servo_twist_topic').value, 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, status_qos)
        self.create_subscription(
            Int32, self.get_parameter('request_topic').value, self._request_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('cancel_topic').value, self._cancel_cb, 10)
        self.create_subscription(
            Bool, self.get_parameter('servo_aligned_topic').value, self._aligned_cb, 10)
        self.create_subscription(
            String, self.get_parameter('hardware_fault_topic').value, self._fault_cb, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_state_topic').value, self._joint_state_cb, 20)
        self.create_service(Trigger, '~/reset', self._reset_cb)
        self.create_timer(0.05, self._tick)
        self._publish_status('idle')

    def _declare_parameters(self) -> None:
        p = self.declare_parameter
        p('request_topic', '/selected_tool_id')
        p('target_tag_id', 1)
        p('cancel_topic', '/tool_change/cancel')
        p('hardware_fault_topic', '/control/hardware_fault')
        p('joint_state_topic', '/joint_states')
        p('arm_action', '/arm_controller/follow_joint_trajectory')
        p('joint_names', [
            'base_joint', 'shoulder_joint', 'elbow_joint',
            'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint',
        ])
        p('arm_action', '/arm_controller/follow_joint_trajectory')
        p('ee_action', '/ee_controller/follow_joint_trajectory')
        # These correspond to SRDF home, dock_wait3, and tagid1 respectively.
        p('home_goal', [0.0, 1.526290430869, 1.522974340197,
                        -1.643751229155, 0.055223308364, 0.010737865515, 3.153864])
        p('back_goal', [2.819457, 0.864811, 1.531177,
                        -1.282643, -0.000000, -0.038350, 3.153864])
        p('tag1_wait_goal', [2.175185, 0.718203, 1.566433,
                             -1.463459, 0.556835, -1.050777, 3.153864])
        p('max_joint_speed_rad_s', 0.15)
        p('min_segment_duration_sec', 1.0)
        p('action_wait_timeout_sec', 10.0)
        p('visual_servo_enable_service', '/visual_servo_node/enable')
        p('servo_aligned_topic', '/servo_aligned')
        p('align_timeout_sec', 15.0)
        p('servo_twist_topic', '/servo_node/delta_twist_cmds')
        p('descent_frame', 'base_actuator')
        p('descent_direction', [0.0, 0.0, -1.0])
        p('descent_distance_m', 0.010)
        p('descent_speed_mps', 0.002)
        p('descent_timeout_sec', 10.0)
        p('status_topic', '/ui_tag1_docking/status')

    def _validate_parameters(self) -> None:
        joints = list(self.get_parameter('joint_names').value)
        for name in ('home_goal', 'back_goal', 'tag1_wait_goal'):
            goal = self.get_parameter(name).value
            if len(goal) != len(joints) or not all(math.isfinite(float(v)) for v in goal):
                raise RuntimeError(f'{name} must contain one finite radian value per joint')
        for name in ('max_joint_speed_rad_s', 'min_segment_duration_sec',
                     'action_wait_timeout_sec', 'align_timeout_sec',
                     'descent_distance_m', 'descent_speed_mps', 'descent_timeout_sec'):
            if float(self.get_parameter(name).value) <= 0.0:
                raise RuntimeError(f'{name} must be positive')
        direction = self.get_parameter('descent_direction').value
        if len(direction) != 3 or math.sqrt(sum(float(v) ** 2 for v in direction)) < 1e-9:
            raise RuntimeError('descent_direction must be a non-zero 3-vector')
        if not self.get_parameter('descent_frame').value:
            raise RuntimeError('descent_frame is required')

    def _request_cb(self, msg: Int32) -> None:
        target = int(self.get_parameter('target_tag_id').value)
        if int(msg.data) != target:
            self.request_armed = True
            return
        if not self.request_armed:
            return
        with self._lock:
            if self.state not in (State.IDLE, State.COMPLETE):
                self._publish_status('ignored:busy')
                return
            self.request_armed = False
            self.cancelled.clear()
            self._set_state(State.HOME)
        threading.Thread(target=self._sequence_worker, daemon=True).start()

    def _sequence_worker(self) -> None:
        for state, parameter in ((State.HOME, 'home_goal'), (State.BACK, 'back_goal'),
                                 (State.TAG1_WAIT, 'tag1_wait_goal')):
            if self.cancelled.is_set():
                return
            with self._lock:
                self._set_state(state)
            if not self._execute_goal(list(self.get_parameter(parameter).value)):
                self._hold(f'{state.name.lower()}_trajectory')
                return
        with self._lock:
            if self.cancelled.is_set():
                return
            self.align_started = time.monotonic()
            self._set_state(State.ALIGNING)
            self._set_visual_servo(True)

    def _joint_state_cb(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        joints = self.get_parameter('joint_names').value
        if all(name in values and math.isfinite(float(values[name])) for name in joints):
            with self._lock:
                self.latest_positions = {name: float(values[name]) for name in joints}
                self.latest_joint_state_time = time.monotonic()

    def _aligned_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self.state != State.ALIGNING:
                return
            self._set_visual_servo(False)
            self.descent_started = time.monotonic()
            self._set_state(State.DESCENDING)

    def _cancel_cb(self, msg: Bool) -> None:
        if msg.data:
            self._hold('cancel')

    def _fault_cb(self, msg: String) -> None:
        if msg.data.strip():
            self._hold('hardware_fault')

    def _tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            if self.state == State.ALIGNING:
                if now - self.align_started > float(self.get_parameter('align_timeout_sec').value):
                    self._hold('align_timeout')
                return
            if self.state != State.DESCENDING:
                return
            elapsed = now - self.descent_started
            if elapsed > float(self.get_parameter('descent_timeout_sec').value):
                self._hold('descent_timeout')
                return
            if elapsed >= float(self.get_parameter('descent_distance_m').value) / float(self.get_parameter('descent_speed_mps').value):
                self._stop_servo()
                self._set_state(State.COMPLETE)
                return
            raw = [float(v) for v in self.get_parameter('descent_direction').value]
            magnitude = math.sqrt(sum(v * v for v in raw))
            speed = float(self.get_parameter('descent_speed_mps').value)
            self._publish_twist([speed * value / magnitude for value in raw])

    def _execute_goal(self, target: list[float]) -> bool:
        with self._lock:
            current = [self.latest_positions.get(name) for name in self.get_parameter('joint_names').value]
            stamp = self.latest_joint_state_time
        if any(value is None for value in current) or time.monotonic() - stamp > 1.0:
            self.get_logger().error('No fresh complete /joint_states sample')
            return False
        timeout = float(self.get_parameter('action_wait_timeout_sec').value)
        if not self.arm_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('arm_controller action server unavailable')
            return False
        speed = float(self.get_parameter('max_joint_speed_rad_s').value)
        duration = max(float(self.get_parameter('min_segment_duration_sec').value),
                       max(abs(goal - actual) for goal, actual in zip(target, current)) / speed)
        time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.joint_names = list(self.get_parameter('joint_names').value[:6])
        arm_goal.trajectory.points = [JointTrajectoryPoint(
            positions=target[:6], time_from_start=time_from_start)]
        ee_goal = FollowJointTrajectory.Goal()
        ee_goal.trajectory.joint_names = ['ee_joint']
        ee_goal.trajectory.points = [JointTrajectoryPoint(
            positions=[target[6]], time_from_start=time_from_start)]
        if not self.ee_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('ee_controller action server unavailable')
            return False
        arm_handle = self._future_result(self.arm_client.send_goal_async(arm_goal), timeout)
        ee_handle = self._future_result(self.ee_client.send_goal_async(ee_goal), timeout)
        if (arm_handle is None or not arm_handle.accepted or
                ee_handle is None or not ee_handle.accepted):
            return False
        arm_result = self._future_result(arm_handle.get_result_async(), duration + timeout)
        ee_result = self._future_result(ee_handle.get_result_async(), duration + timeout)
        return (arm_result is not None and ee_result is not None and
                arm_result.status == GoalStatus.STATUS_SUCCEEDED and
                ee_result.status == GoalStatus.STATUS_SUCCEEDED and not self.cancelled.is_set())

    @staticmethod
    def _future_result(future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        return future.result() if done.wait(timeout) else None

    def _set_visual_servo(self, enabled: bool) -> None:
        if not self.visual_servo_client.service_is_ready():
            self.get_logger().error('visual-servo enable service unavailable')
            self.cancelled.set()
            return
        request = SetBool.Request(data=enabled)
        self.visual_servo_client.call_async(request)

    def _publish_twist(self, linear: list[float]) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('descent_frame').value
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = linear
        self.twist_pub.publish(message)

    def _stop_servo(self) -> None:
        self._publish_twist([0.0, 0.0, 0.0])

    def _hold(self, reason: str) -> None:
        with self._lock:
            self.cancelled.set()
            self._set_visual_servo(False)
            self._stop_servo()
            self._set_state(State.HOLD, reason)

    def _reset_cb(self, _request, response):
        with self._lock:
            if self.state != State.HOLD:
                response.success, response.message = False, 'reset is only allowed while held'
                return response
            self.cancelled.clear()
            self._set_state(State.IDLE)
            response.success, response.message = True, 'ready for a new UI request'
            return response

    def _set_state(self, state: State, reason: str = '') -> None:
        self.state = state
        self._publish_status(state.name.lower() + (f':{reason}' if reason else ''))
        self.get_logger().info(f'UI tag-1 state -> {state.name}' + (f' ({reason})' if reason else ''))

    def _publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = UiTag1Docking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
