#!/usr/bin/env python3
"""Plan a MoveIt TCP-axis target received from a topic; optionally execute it."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import rclpy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (Constraints, DisplayTrajectory, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, OrientationConstraint,
                             PositionConstraint)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

source = Path(__file__).resolve().parents[2] / "tool_change_min"
if (source / "tool_change_min/kinematics.py").is_file():
    sys.path.insert(0, str(source))
try:
    from tool_change_min.kinematics import ArmKinematics, ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ArmKinematics, ARM_JOINTS


def normalized_quaternion(pose):
    values = [pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("orientation contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("orientation quaternion has zero length")
    return [value / norm for value in values]


def local_x_plane_tilt_deg(quaternion):
    """Undirected angle between the rotated local +X axis and world XY."""
    x, y, z, w = quaternion
    local_x_world_z = 2.0 * (x * z - w * y)
    return math.degrees(math.asin(min(1.0, abs(local_x_world_z))))


def rotated_local_x(quaternion):
    x, y, z, w = quaternion
    return np.asarray([
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    ])


def moveit_error_name(code):
    return {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            "START_STATE_VIOLATES_PATH_CONSTRAINTS",
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            "GOAL_VIOLATES_PATH_CONSTRAINTS",
        MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    }.get(code, "UNKNOWN")


class PoseGoalPlanner(Node):
    def __init__(self):
        super().__init__("arm_pose_goal_planner")
        self.group = self.declare_parameter("group_name", "arm").value
        self.target_link = self.declare_parameter("target_link", "tcp_link").value
        self.position_tolerance = float(
            self.declare_parameter("position_tolerance_m", 0.002).value)
        self.orientation_tolerance = float(
            self.declare_parameter("orientation_tolerance_rad", 0.02).value)
        self.free_roll_yaw = bool(
            self.declare_parameter("free_wrist_roll_yaw", True).value)
        self.axis_goal = bool(self.declare_parameter("use_axis_ik", True).value)
        self.execute_plan = bool(self.declare_parameter("execute_plan", False).value)
        package = Path(__file__).resolve().parents[1]
        urdf = str(self.declare_parameter(
            "urdf_xacro", str(package.parent /
                              "tool_manipulator_description/urdf/tool_manipulator.urdf.xacro")).value)
        hardware = str(self.declare_parameter(
            "hardware_config", str(package / "config/hardware.yaml")).value)
        self.model = ArmKinematics(urdf, hardware)
        srdf = Path(str(self.declare_parameter(
            "srdf", str(package.parent /
                        "tool_manipulator_moveit_config/config/tool_manipulator.srdf")).value))
        root = ET.parse(srdf).getroot()
        back = root.find("group_state[@group='arm'][@name='back']")
        if back is None:
            raise ValueError("SRDF has no arm/back seed")
        values = {joint.get("name"): float(joint.get("value"))
                  for joint in back.findall("joint")}
        self.back_seed = np.asarray([values[name] for name in ARM_JOINTS])
        self.allowed_time = float(
            self.declare_parameter("allowed_planning_time_sec", 8.0).value)
        self.attempts = int(self.declare_parameter("planning_attempts", 5).value)
        self.use_orientation = bool(
            self.declare_parameter("use_orientation_constraint", True).value)
        if (not 0.0 < self.position_tolerance <= 0.02
                or not 0.0 < self.orientation_tolerance <= 0.2
                or not 0.5 <= self.allowed_time <= 30.0
                or not 1 <= self.attempts <= 20):
            raise ValueError("invalid pose planning parameters")

        self.joint_state = None
        self.busy = False
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        # Node.executor is an rclpy-managed property. Do not shadow it with an
        # action client or Node.__init__ will try to treat the client as an
        # executor and call add_node() on it.
        self.trajectory_executor = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory")
        self.display = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 1)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(
            String, "/arm/pose_plan_status", status_qos)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            PoseStamped, "/arm/plan_pose_goal", self.on_goal, 10)
        mode = "PLAN+EXECUTE" if self.execute_plan else "PLAN ONLY"
        self.get_logger().info(f"{mode} pose input ready on /arm/plan_pose_goal")

    def report(self, state, reason, **extra):
        data = dict(state=state, reason=reason, **extra)
        self.status.publish(String(data=json.dumps(data)))
        # rclpy binds a logging call site to one severity. Keep distinct sites.
        if state == "failed":
            self.get_logger().error(reason)
        elif state == "blocked":
            self.get_logger().warning(reason)
        else:
            self.get_logger().info(reason)

    def on_joint_state(self, message):
        if (len(message.name) == len(message.position)
                and len(message.name) == len(set(message.name))
                and all(math.isfinite(value) for value in message.position)):
            self.joint_state = message

    def on_goal(self, message):
        if self.busy:
            self.report("blocked", "a pose plan is already in progress")
            return
        try:
            if message.header.frame_id not in ("arm_world", "base_actuator"):
                raise ValueError("goal frame must be arm_world or base_actuator")
            position = message.pose.position
            if not all(math.isfinite(value) for value in
                       (position.x, position.y, position.z)):
                raise ValueError("position contains a non-finite value")
            quaternion = normalized_quaternion(message.pose)
            if self.joint_state is None:
                raise RuntimeError("no valid /joint_states sample received")
            if not self.planner.service_is_ready():
                raise RuntimeError("/plan_kinematic_path is unavailable")
            if (self.execute_plan
                    and not self.trajectory_executor.server_is_ready()):
                raise RuntimeError("/execute_trajectory is unavailable")
            axis_solution = None
            if self.axis_goal:
                by_name = dict(zip(self.joint_state.name, self.joint_state.position))
                if any(name not in by_name for name in ARM_JOINTS):
                    raise RuntimeError("/joint_states does not contain the complete arm")
                seed = np.asarray([by_name[name] for name in ARM_JOINTS])
                errors = []
                for candidate in (seed, self.back_seed):
                    try:
                        axis_solution = self.model.solve_axis(
                            np.asarray([position.x, position.y, position.z]),
                            rotated_local_x(quaternion), np.asarray([1.0, 0.0, 0.0]),
                            candidate, margin_rad=0.02, max_iterations=500,
                            max_time_s=0.5, joint_step_rad=6.0)
                        break
                    except ValueError as exc:
                        errors.append(str(exc))
                if axis_solution is None:
                    raise ValueError("axis IK failed from current and back seeds: "
                                     + " | ".join(errors))
            request = self.make_request(message, quaternion, axis_solution)
        except (ValueError, RuntimeError) as exc:
            self.report("blocked", str(exc))
            return

        self.busy = True
        tilt = local_x_plane_tilt_deg(quaternion)
        self.report("planning", "MoveIt pose planning requested",
                    constraint_mode=("axis_ik_joint_goal" if self.axis_goal else
                                     "position_and_orientation"
                                     if self.use_orientation else "position_only"),
                    frame=message.header.frame_id,
                    xyz_m=[position.x, position.y, position.z],
                    wrist_to_world_xy_deg=tilt,
                    axis_goal_positions_rad=(None if axis_solution is None else
                                             axis_solution.tolist()))
        future = self.planner.call_async(request)
        context = dict(
            frame=message.header.frame_id,
            xyz_m=[position.x, position.y, position.z],
            requested_wrist_to_world_xy_deg=tilt)
        future.add_done_callback(lambda result: self.on_plan(result, context))

    def make_request(self, target, quaternion, axis_solution=None):
        request = MotionPlanRequest()
        request.group_name = self.group
        request.num_planning_attempts = self.attempts
        request.allowed_planning_time = self.allowed_time
        request.max_velocity_scaling_factor = 0.08
        request.max_acceleration_scaling_factor = 0.4
        request.start_state.joint_state = self.joint_state
        request.start_state.is_diff = True

        constraint = Constraints(name="tcp_pose_goal")
        if axis_solution is not None:
            for name, value in zip(ARM_JOINTS, axis_solution):
                constraint.joint_constraints.append(JointConstraint(
                    joint_name=name, position=float(value),
                    tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
            request.goal_constraints = [constraint]
            return GetMotionPlan.Request(motion_plan_request=request)
        position = PositionConstraint()
        position.header = target.header
        position.link_name = self.target_link
        position.constraint_region.primitives.append(SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[self.position_tolerance]))
        region_pose = Pose()
        region_pose.position = target.pose.position
        region_pose.orientation.w = 1.0
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0
        constraint.position_constraints.append(position)

        if self.use_orientation:
            orientation = OrientationConstraint()
            orientation.header = target.header
            orientation.link_name = self.target_link
            (orientation.orientation.x, orientation.orientation.y,
             orientation.orientation.z, orientation.orientation.w) = quaternion
            # The measured requirement fixes only the wrist direction's tilt
            # from world XY. Rotation about that direction and its world-Z
            # bearing remain unspecified and must not be arbitrarily pinned to
            # the quaternion's zero-roll/zero-yaw representatives.
            free = math.pi if self.free_roll_yaw else self.orientation_tolerance
            orientation.absolute_x_axis_tolerance = free
            orientation.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation.absolute_z_axis_tolerance = free
            orientation.weight = 1.0
            constraint.orientation_constraints.append(orientation)
        request.goal_constraints = [constraint]
        return GetMotionPlan.Request(motion_plan_request=request)

    def on_plan(self, future, context):
        try:
            result = future.result()
            if result is None:
                raise RuntimeError("MoveIt returned no response")
            response = result.motion_plan_response
            code = response.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                self.busy = False
                name = moveit_error_name(code)
                self.report(
                    "failed", f"MoveIt planning failed: {name} ({code})",
                    moveit_error_code=code, moveit_error_name=name,
                    planning_time_sec=response.planning_time, **context)
                return
            trajectory = response.trajectory.joint_trajectory
            if not trajectory.points:
                raise RuntimeError("MoveIt returned an empty trajectory")
            display = DisplayTrajectory()
            display.trajectory_start = response.trajectory_start
            display.trajectory.append(response.trajectory)
            self.display.publish(display)
            final = trajectory.points[-1]
            if self.execute_plan:
                self.report(
                    "executing", "collision-checked plan accepted for execution",
                    **context,
                    final_joint_names=list(trajectory.joint_names),
                    final_positions_rad=list(final.positions))
                goal = ExecuteTrajectory.Goal()
                goal.trajectory = response.trajectory
                sent = self.trajectory_executor.send_goal_async(goal)
                sent.add_done_callback(self.on_execute_goal)
                return
            self.busy = False
            self.report(
                "planned", "plan published on /display_planned_path; not executed",
                constraint_mode=("axis_ik_joint_goal" if self.axis_goal else
                                 "position_and_orientation"
                                 if self.use_orientation else "position_only"),
                planning_time_sec=response.planning_time,
                **context,
                final_joint_names=list(trajectory.joint_names),
                final_positions_rad=list(final.positions))
        except Exception as exc:  # rclpy futures may surface service exceptions here.
            self.busy = False
            self.report("failed", str(exc), **context)

    def on_execute_goal(self, future):
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("MoveIt rejected the trajectory execution goal")
            result = handle.get_result_async()
            result.add_done_callback(self.on_execute_result)
        except Exception as exc:
            self.busy = False
            self.report("failed", str(exc))

    def on_execute_result(self, future):
        self.busy = False
        try:
            wrapped = future.result()
            code = wrapped.result.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f"MoveIt trajectory execution failed with code {code}, "
                    f"action status {wrapped.status}")
            self.report("executed", "MoveIt trajectory execution completed")
        except Exception as exc:
            self.report("failed", str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
