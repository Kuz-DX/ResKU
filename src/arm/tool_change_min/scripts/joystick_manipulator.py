#!/usr/bin/env python3
"""Single-joint jogging using tool_change_min limits and the arm action server.

Run instead of the automatic FSM/motion executor or MoveIt Servo. Each short
goal starts from fresh measured positions; goals are never queued or replaced.
"""
from __future__ import annotations

import math
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from tool_change_min.kinematics import ArmKinematics


class JoystickManipulator(Node):
    def __init__(self):
        super().__init__("joystick_manipulator")
        description = get_package_share_directory("tool_manipulator_description")
        bringup = get_package_share_directory("tool_manipulator_bringup")
        defaults = {
            "urdf_xacro": f"{description}/urdf/tool_manipulator.urdf.xacro",
            "hardware_yaml": f"{bringup}/config/hardware.yaml",
            "arm_action": "/arm_controller/follow_joint_trajectory",
            "joy_topic": "/joy", "joint_states_topic": "/joint_states",
            "focus_topic": "/control/active_target", "manage_focus": True,
            "focus_button": 9, "deadman_button": 4, "jog_axis": 4,
            "select_axis": 6, "invert_jog_axis": True,
            "joint_speed_rad_s": 0.08, "step_duration_s": 0.2,
            "joy_timeout_s": 0.25, "feedback_timeout_s": 0.5,
            "action_timeout_s": 2.0, "deadzone": 0.12,
            "limit_margin_rad": 0.01,
        }
        self.settings = {key: self.declare_parameter(key, value).value
                         for key, value in defaults.items()}
        p = self.settings
        for key in ("joint_speed_rad_s", "step_duration_s", "joy_timeout_s",
                    "feedback_timeout_s", "action_timeout_s"):
            if not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if p["action_timeout_s"] <= p["step_duration_s"]:
            raise ValueError("action_timeout_s must exceed step_duration_s")
        if not math.isfinite(p["deadzone"]) or not 0 <= p["deadzone"] < 1:
            raise ValueError("deadzone must be in [0, 1)")
        if not math.isfinite(p["limit_margin_rad"]) or p["limit_margin_rad"] < 0:
            raise ValueError("limit_margin_rad must be finite and nonnegative")
        for key in ("focus_button", "deadman_button", "jog_axis", "select_axis"):
            if p[key] < 0:
                raise ValueError(f"{key} must be nonnegative")
        self.model = ArmKinematics(p["urdf_xacro"], p["hardware_yaml"])
        self.names = list(self.model.joint_names)
        with open(p["hardware_yaml"], encoding="utf-8") as stream:
            hardware = yaml.safe_load(stream)
        self.speeds = []
        for name in self.names:
            speed = float(hardware["joints"][name]["velocity_limit_rad_s"])
            if not math.isfinite(speed) or speed <= 0:
                raise ValueError(f"invalid velocity limit for {name}")
            self.speeds.append(min(p["joint_speed_rad_s"], speed))
            low, high = self.model.limits[name]
            if low + p["limit_margin_rad"] >= high - p["limit_margin_rad"]:
                raise ValueError(f"limit margin consumes range of {name}")

        self.selected = 0
        self.focus = "drive"
        self.previous_focus = False
        self.selection_latched = False
        self.armed = False
        self.joy = None
        self.joy_at = self.feedback_at = self.completed_at = 0.0
        self.positions = None
        self.pending = False
        self.handle = None
        self.stop_requested = False
        self.cancel_sent = False
        self.deadline = 0.0
        self.fault = ""
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.focus_pub = (self.create_publisher(String, p["focus_topic"], qos)
                          if p["manage_focus"] else None)
        self.create_subscription(String, p["focus_topic"], self._focus, qos)
        self.create_subscription(Joy, p["joy_topic"], self._joy, qos_profile_sensor_data)
        self.create_subscription(JointState, p["joint_states_topic"], self._state,
                                 qos_profile_sensor_data)
        self.client = ActionClient(self, FollowJointTrajectory, p["arm_action"])
        self.create_timer(0.02, self._tick)
        if self.focus_pub:
            self.focus_pub.publish(String(data="drive"))
        self.get_logger().info(
            "Ready: Options selects arm/drive; release L1 to arm, D-pad selects "
            "joint, hold L1 + right stick to jog. Selected: " + self.names[0])

    def _focus(self, message):
        if self.focus != message.data:
            self.armed = False
            self._stop()
        self.focus = message.data

    def _joy(self, message):
        p = self.settings
        valid = (len(message.buttons) > max(p["focus_button"], p["deadman_button"])
                 and len(message.axes) > max(p["jog_axis"], p["select_axis"])
                 and all(math.isfinite(v) and abs(v) <= 1.0 for v in message.axes))
        if not valid:
            self.joy = None
            self.armed = False
            self._stop()
            return
        now = time.monotonic()
        if now - self.joy_at > p["joy_timeout_s"]:
            self.armed = False
        self.joy, self.joy_at = message, now
        pressed = bool(message.buttons[p["focus_button"]])
        if self.focus_pub and pressed and not self.previous_focus:
            target = "drive" if self.focus == "arm" else "arm"
            self._focus(String(data=target))
            self.focus_pub.publish(String(data=target))
            self.get_logger().info(f"Joystick focus: {target}")
        self.previous_focus = pressed
        deadman = bool(message.buttons[p["deadman_button"]])
        select = message.axes[p["select_axis"]]
        if not deadman:
            self._stop()
            self.armed = self.focus == "arm"
            if (self.armed and not self.pending and abs(select) >= 0.5
                    and not self.selection_latched):
                self.selected = (self.selected + (1 if select < 0 else -1)) % len(self.names)
                self.get_logger().info("Selected: " + self.names[self.selected])
        self.selection_latched = abs(select) >= 0.5
        if self._input() == 0:
            self._stop()

    def _state(self, message):
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise ValueError("malformed joint state")
            values = dict(zip(message.name, message.position))
            positions = [float(values[name]) for name in self.names]
            if not all(math.isfinite(v) for v in positions):
                raise ValueError("non-finite feedback")
            if not self.model.within_limits(positions, self.settings["limit_margin_rad"]):
                raise ValueError("feedback outside joint limits")
            self.positions, self.feedback_at = positions, time.monotonic()
        except (KeyError, ValueError, TypeError):
            self.positions = None
            self.armed = False
            self._stop()

    def _input(self):
        p = self.settings
        if (self.fault or not self.armed or self.focus != "arm" or self.joy is None
                or time.monotonic() - self.joy_at > p["joy_timeout_s"]
                or not self.joy.buttons[p["deadman_button"]]):
            return 0.0
        value = self.joy.axes[p["jog_axis"]]
        if abs(value) <= p["deadzone"]:
            return 0.0
        value = math.copysign((abs(value) - p["deadzone"]) / (1 - p["deadzone"]), value)
        return -value if p["invert_jog_axis"] else value

    def _fail(self, reason):
        if not self.fault:
            self.fault = reason
            self.get_logger().error(reason + "; restart after checking controller state")
        self.armed = False
        self._stop()

    def _stop(self):
        if not self.pending:
            return
        self.stop_requested = True
        if self.handle is not None and not self.cancel_sent:
            self.cancel_sent = True
            try:
                self.handle.cancel_goal_async().add_done_callback(self._cancelled)
            except Exception as exc:
                self._fail(f"cancel failed: {exc}")

    def _cancelled(self, future):
        try:
            if not future.result().goals_canceling and self.pending:
                self._fail("controller did not acknowledge cancellation")
        except Exception as exc:
            self._fail(f"cancel response failed: {exc}")

    def _accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self.pending = False
                self._fail("controller rejected jog goal")
                return
            self.handle.get_result_async().add_done_callback(self._result)
            if self.stop_requested or self._input() == 0:
                self._stop()
        except Exception as exc:
            self._fail(f"goal response failed: {exc}")

    def _result(self, future):
        stopped = self.stop_requested
        self.pending, self.handle = False, None
        self.completed_at = time.monotonic()
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_CANCELED and stopped:
                return
            if (result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
                self._fail(f"jog failed: status={result.status}, {result.result.error_string}")
        except Exception as exc:
            self._fail(f"result failed: {exc}")

    def _tick(self):
        now, p = time.monotonic(), self.settings
        fresh = self.positions is not None and now - self.feedback_at <= p["feedback_timeout_s"]
        if now - self.joy_at > p["joy_timeout_s"] or not fresh:
            self.armed = False
        value = self._input()
        if not value or not fresh:
            self._stop()
        if self.pending:
            if now > self.deadline:
                self._fail("action timed out (physical stop is not confirmed)")
            return
        if not value or not fresh or self.feedback_at <= self.completed_at:
            return
        if not self.client.server_is_ready():
            return
        target = list(self.positions)
        index = self.selected
        low, high = self.model.limits[self.names[index]]
        target[index] = min(high - p["limit_margin_rad"], max(low + p["limit_margin_rad"],
                            target[index] + value * self.speeds[index] * p["step_duration_s"]))
        if abs(target[index] - self.positions[index]) < 1e-9:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.names
        start = JointTrajectoryPoint(positions=list(self.positions))
        sec, nanosec = divmod(round(p["step_duration_s"] * 1e9), 1000000000)
        end = JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=sec, nanosec=nanosec))
        goal.trajectory.points = [start, end]
        self.pending, self.stop_requested, self.cancel_sent = True, False, False
        self.deadline = now + p["action_timeout_s"]
        try:
            self.client.send_goal_async(goal).add_done_callback(self._accepted)
        except Exception as exc:
            self._fail(f"goal send failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JoystickManipulator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._stop()
            # A lost process cannot guarantee cancellation; every goal is bounded.
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
