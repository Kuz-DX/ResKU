#!/usr/bin/env python3
"""UI-triggered six-axis taught replay; never commands EE or declares attachment."""
from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_pose_safety import validate_joint_positions
from tag1_recorded_path import ARM_JOINTS, compile_sequence, read_yaml, segment_duration


def default_config(filename):
    source = Path(__file__).resolve().parents[1] / 'config' / filename
    if source.is_file():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('tool_manipulator_bringup')) / 'config' / filename)


def duration(seconds):
    nanoseconds = math.ceil(seconds * 1_000_000_000)
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


class RecordedPathNode(Node):
    def __init__(self):
        super().__init__('ui_tag1_docking')
        p = self.declare_parameter
        p('recorded_path_file', default_config('tag1_recorded_path.yaml'))
        p('hardware_config_file', default_config('hardware.yaml'))
        p('tools_config_file', default_config('tools.yaml'))
        p('request_topic', '/selected_tool_id')
        p('cancel_topic', '/tool_change/cancel')
        p('hardware_fault_topic', '/control/hardware_fault')
        p('joint_state_topic', '/joint_states')
        p('status_topic', '/ui_tag1_docking/status')
        p('arm_action', '/arm_controller/follow_joint_trajectory')
        for name, value in (
            ('max_joint_speed_rad_s', 0.15), ('min_segment_duration_sec', 1.0),
            ('start_tolerance_rad', 0.05), ('goal_tolerance_rad', 0.01),
            ('max_start_velocity_rad_s', 0.02), ('joint_state_timeout_sec', 1.0),
            ('action_wait_timeout_sec', 10.0),
        ):
            p(name, value)
            actual = self.value(name)
            if not math.isfinite(actual) or actual <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.busy = False
        self.unresolved_goal = False
        self.handle = None
        self.worker = None
        self.armed = True
        self.state = 'idle'
        self.fault = ''
        self.positions, self.velocities, self.stamp = {}, {}, 0.0
        self.active_limits = None
        self.client = ActionClient(self, FollowJointTrajectory, self.value('arm_action'))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, self.value('status_topic'), qos)
        self.create_subscription(Int32, self.value('request_topic'), self.request, 10)
        self.create_subscription(Bool, self.value('cancel_topic'), self.cancel, 10)
        self.create_subscription(String, self.value('hardware_fault_topic'), self.hardware_fault, 10)
        self.create_subscription(JointState, self.value('joint_state_topic'), self.joint_state, 20)
        self.create_service(Trigger, '~/reset', self.reset)
        self.create_timer(0.05, self.watchdog)
        self.publish('idle')

    def value(self, name):
        return self.get_parameter(name).value

    def publish(self, state):
        self.state = state
        self.status.publish(String(data=state))
        self.get_logger().info(state)

    def joint_state(self, msg):
        positions, velocities = dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))
        if all(name in positions and math.isfinite(positions[name]) and
               name in velocities and math.isfinite(velocities[name]) for name in ARM_JOINTS):
            with self.lock:
                self.positions, self.velocities = positions, velocities
                self.stamp = time.monotonic()

    def request(self, msg):
        with self.lock:
            if msg.data != 1:
                self.armed = True
                return
            if not self.armed or self.busy or self.unresolved_goal or self.state.startswith('hold'):
                return
            self.armed = False
            self.busy = True
            self.cancelled.clear()
            self.publish('preflight')
            self.worker = threading.Thread(target=self.run_sequence, daemon=True)
            self.worker.start()

    def check_current(self, limits, stopped=False):
        if self.fault:
            raise ValueError(f'hardware fault: {self.fault}')
        if time.monotonic() - self.stamp > self.value('joint_state_timeout_sec'):
            raise ValueError('fresh six-axis position AND velocity /joint_states required')
        validate_joint_positions(self.positions, limits, label='current state')
        if stopped and any(abs(self.velocities[name]) > self.value('max_start_velocity_rad_s')
                           for name in ARM_JOINTS):
            raise ValueError('arm must be stopped before replay')
        return [self.positions[name] for name in ARM_JOINTS]

    def run_sequence(self):
        try:
            points, limits, speeds = compile_sequence(
                read_yaml(self.value('recorded_path_file')),
                read_yaml(self.value('hardware_config_file')),
                read_yaml(self.value('tools_config_file')))
            timeout = self.value('action_wait_timeout_sec')
            if self.cancelled.is_set():
                return
            if not self.client.wait_for_server(timeout_sec=timeout):
                raise ValueError('arm_controller action server unavailable')
            with self.lock:
                if self.cancelled.is_set():
                    return
                current = self.check_current(limits, stopped=True)
                if max(abs(a-b) for a, b in zip(current, points[0]['positions'])) > self.value('start_tolerance_rad'):
                    raise ValueError('not at reviewed start pose; automatic homing is prohibited')
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = list(ARM_JOINTS)
                elapsed = 0.05
                goal.trajectory.points = [JointTrajectoryPoint(
                    positions=current, velocities=[0.0]*6, time_from_start=duration(elapsed))]
                for point in points:
                    target = point['positions']
                    elapsed += segment_duration(current, target, speeds,
                                                self.value('max_joint_speed_rad_s'),
                                                self.value('min_segment_duration_sec'))
                    goal.trajectory.points.append(JointTrajectoryPoint(
                        positions=target, velocities=[0.0]*6, time_from_start=duration(elapsed)))
                    current = target
                goal.goal_tolerance = [JointTolerance(name=name, position=self.value('goal_tolerance_rad'))
                                       for name in ARM_JOINTS]
                goal.goal_time_tolerance = duration(timeout)
                self.active_limits = limits
                self.unresolved_goal = True
                # All points pass preflight before this sole command submission.
                future = self.client.send_goal_async(goal)
                future.add_done_callback(self.goal_response)
                self.publish('executing_recorded_path')
            handle = self.wait_future(future, timeout)
            if handle is None or not handle.accepted:
                raise ValueError('goal rejected, cancelled, or response timed out')
            result = self.wait_future(handle.get_result_async(), elapsed + timeout + 1.0)
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise ValueError('trajectory did not complete successfully')
            with self.lock:
                if self.cancelled.is_set():
                    return
                actual = self.check_current(limits)
                if max(abs(a-b) for a, b in zip(actual, points[-1]['positions'])) > self.value('goal_tolerance_rad'):
                    raise ValueError('final measured pose outside goal tolerance')
                self.publish('complete:trajectory_only')
        except Exception as exc:
            self.hold(str(exc))
        finally:
            with self.lock:
                self.busy = False

    def goal_response(self, future):
        with self.lock:
            try:
                handle = future.result()
                if handle is None or not handle.accepted:
                    self.unresolved_goal = False
                    return
                self.handle = handle
                handle.get_result_async().add_done_callback(self.goal_terminal)
                if self.cancelled.is_set():
                    handle.cancel_goal_async()
            except Exception as exc:
                # Unknown acceptance state: block reset until controller is inspected.
                self.hold(f'goal_response_error:{exc}')

    def goal_terminal(self, future):
        with self.lock:
            try:
                result = future.result()
                if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                         GoalStatus.STATUS_CANCELED):
                    return
                self.handle = None
                self.unresolved_goal = False
            except Exception as exc:
                self.hold(f'goal_result_error:{exc}')

    def wait_future(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout
        while not done.wait(0.05):
            if self.cancelled.is_set() or time.monotonic() >= deadline or not rclpy.ok():
                return None
        return future.result()

    def hold(self, reason):
        with self.lock:
            self.cancelled.set()
            if self.handle is not None:
                self.handle.cancel_goal_async()
            self.publish('hold:' + reason)

    def watchdog(self):
        with self.lock:
            if self.state == 'executing_recorded_path' and self.active_limits is not None:
                try:
                    self.check_current(self.active_limits)
                except ValueError as exc:
                    self.hold(str(exc))

    def cancel(self, msg):
        if msg.data:
            self.hold('operator_cancel')

    def hardware_fault(self, msg):
        with self.lock:
            self.fault = msg.data.strip()
            if self.fault:
                self.hold('hardware_fault:' + self.fault)

    def reset(self, _request, response):
        with self.lock:
            if not self.state.startswith('hold') or self.busy or self.unresolved_goal or self.fault:
                response.success, response.message = False, 'reset requires HOLD, idle worker, resolved goal, no fault'
            else:
                self.cancelled.clear()
                self.publish('idle')
                response.success, response.message = True, 'send 99 then 1 for a new request'
            return response


def main():
    rclpy.init()
    node = RecordedPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.hold('shutdown')
        if node.worker is not None:
            node.worker.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
