#!/usr/bin/env python3
"""Plan, but never execute, a MoveIt TCP pose received from a ROS topic."""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (Constraints, DisplayTrajectory, MotionPlanRequest,
                             MoveItErrorCodes, OrientationConstraint,
                             PositionConstraint)
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


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


class PoseGoalPlanner(Node):
    def __init__(self):
        super().__init__("arm_pose_goal_planner")
        self.group = self.declare_parameter("group_name", "arm").value
        self.target_link = self.declare_parameter("target_link", "tcp_link").value
        self.position_tolerance = float(
            self.declare_parameter("position_tolerance_m", 0.002).value)
        self.orientation_tolerance = float(
            self.declare_parameter("orientation_tolerance_rad", 0.02).value)
        self.allowed_time = float(
            self.declare_parameter("allowed_planning_time_sec", 8.0).value)
        self.attempts = int(self.declare_parameter("planning_attempts", 5).value)
        if (not 0.0 < self.position_tolerance <= 0.02
                or not 0.0 < self.orientation_tolerance <= 0.2
                or not 0.5 <= self.allowed_time <= 30.0
                or not 1 <= self.attempts <= 20):
            raise ValueError("invalid pose planning parameters")

        self.joint_state = None
        self.busy = False
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
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
        self.get_logger().info(
            "Plan-only pose input ready on /arm/plan_pose_goal; no trajectory execution")

    def report(self, state, reason, **extra):
        data = dict(state=state, reason=reason, **extra)
        self.status.publish(String(data=json.dumps(data)))
        (self.get_logger().error if state == "failed" else self.get_logger().info)(
            reason)

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
            request = self.make_request(message, quaternion)
        except (ValueError, RuntimeError) as exc:
            self.report("blocked", str(exc))
            return

        self.busy = True
        tilt = local_x_plane_tilt_deg(quaternion)
        self.report("planning", "MoveIt pose planning requested",
                    frame=message.header.frame_id,
                    xyz_m=[position.x, position.y, position.z],
                    wrist_to_world_xy_deg=tilt)
        future = self.planner.call_async(request)
        future.add_done_callback(lambda result: self.on_plan(result, tilt))

    def make_request(self, target, quaternion):
        request = MotionPlanRequest()
        request.group_name = self.group
        request.num_planning_attempts = self.attempts
        request.allowed_planning_time = self.allowed_time
        request.max_velocity_scaling_factor = 0.08
        request.max_acceleration_scaling_factor = 0.4
        request.start_state.joint_state = self.joint_state
        request.start_state.is_diff = True

        constraint = Constraints(name="tcp_pose_goal")
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

        orientation = OrientationConstraint()
        orientation.header = target.header
        orientation.link_name = self.target_link
        (orientation.orientation.x, orientation.orientation.y,
         orientation.orientation.z, orientation.orientation.w) = quaternion
        orientation.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation.weight = 1.0
        constraint.orientation_constraints.append(orientation)
        request.goal_constraints = [constraint]
        return GetMotionPlan.Request(motion_plan_request=request)

    def on_plan(self, future, requested_tilt):
        self.busy = False
        try:
            result = future.result()
            if result is None:
                raise RuntimeError("MoveIt returned no response")
            response = result.motion_plan_response
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f"MoveIt planning failed with code {response.error_code.val}")
            trajectory = response.trajectory.joint_trajectory
            if not trajectory.points:
                raise RuntimeError("MoveIt returned an empty trajectory")
            display = DisplayTrajectory()
            display.trajectory_start = response.trajectory_start
            display.trajectory.append(response.trajectory)
            self.display.publish(display)
            final = trajectory.points[-1]
            self.report(
                "planned", "plan published on /display_planned_path; not executed",
                planning_time_sec=response.planning_time,
                requested_wrist_to_world_xy_deg=requested_tilt,
                final_joint_names=list(trajectory.joint_names),
                final_positions_rad=list(final.positions))
        except Exception as exc:  # rclpy futures may surface service exceptions here.
            self.report("failed", str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = PoseGoalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
