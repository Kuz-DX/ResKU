#!/usr/bin/env python3
"""Print joint coordinates on Enter and infer joint values from the TF tree.

For every movable URDF joint, this node reads the current world->parent and
world->child transforms.  It reconstructs parent->child, removes the joint's
static URDF origin, and projects the remaining motion onto the declared joint
axis.  This makes the reported ``q`` independent of the joint's mounting RPY.

Continuous-joint angles reconstructed from TF are necessarily wrapped to
[-pi, pi]; TF does not contain the number of completed revolutions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import sys
import threading
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros


EXTRA_FRAMES = ("cam_link",)


@dataclass(frozen=True)
class JointInfo:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


def _values(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected three values, got {text!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def _transform(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = xyz
    return result


def _inverse(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    result = np.eye(4)
    result[:3, :3] = rotation.T
    result[:3, 3] = -(rotation.T @ translation)
    return result


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    ))


def _tf_matrix(transform) -> np.ndarray:
    translation = transform.translation
    rotation = transform.rotation
    return _transform(
        np.array((translation.x, translation.y, translation.z), dtype=float),
        _quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _matrix_rpy(rotation: np.ndarray) -> np.ndarray:
    # URDF fixed-axis roll/pitch/yaw convention: R = Rz(yaw) Ry(pitch) Rx(roll).
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array((roll, pitch, yaw))


def _joint_value(joint: JointInfo, parent_to_child: np.ndarray) -> float | None:
    motion = _inverse(joint.origin) @ parent_to_child
    if joint.joint_type in ("revolute", "continuous"):
        rotation = motion[:3, :3]
        skew = np.array((
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        )) * 0.5
        sin_angle = float(joint.axis @ skew)
        cos_angle = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) * 0.5))
        return math.atan2(sin_angle, cos_angle)
    if joint.joint_type == "prismatic":
        return float(joint.axis @ motion[:3, 3])
    return None


def _parse_urdf(xml_text: str, include_fixed: bool) -> list[JointInfo]:
    root = ET.fromstring(xml_text)
    joints: list[JointInfo] = []
    for element in root.findall("joint"):
        joint_type = element.get("type", "")
        if joint_type == "fixed" and not include_fixed:
            continue
        parent = element.find("parent")
        child = element.find("child")
        if parent is None or child is None:
            continue
        origin_element = element.find("origin")
        xyz = _values(origin_element.get("xyz") if origin_element is not None else None, (0.0, 0.0, 0.0))
        rpy = _values(origin_element.get("rpy") if origin_element is not None else None, (0.0, 0.0, 0.0))
        axis_element = element.find("axis")
        axis = _values(axis_element.get("xyz") if axis_element is not None else None, (1.0, 0.0, 0.0))
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1.0e-12:
            raise ValueError(f"joint {element.get('name')} has a zero axis")
        joints.append(JointInfo(
            name=element.get("name", "unnamed_joint"),
            joint_type=joint_type,
            parent=parent.get("link", ""),
            child=child.get("link", ""),
            origin=_transform(xyz, _rpy_matrix(rpy)),
            axis=axis / axis_norm,
        ))
    return joints


class JointWorldCoordinates(Node):
    def __init__(self) -> None:
        super().__init__("joint_world_coordinates")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("include_fixed", False)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._joints: list[JointInfo] = []
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._print_requested = threading.Event()
        self._print_finished = threading.Event()

        description_qos = QoSProfile(depth=1)
        description_qos.reliability = ReliabilityPolicy.RELIABLE
        description_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._description_subscription = self.create_subscription(
            String, "/robot_description", self._description_callback, description_qos
        )

        description = str(self.get_parameter("robot_description").value)
        if description:
            self._load_description(description)
        self._request_timer = self.create_timer(0.05, self._handle_print_request)
        self._input_thread = threading.Thread(
            target=self._input_loop, name="joint-coordinate-input", daemon=True
        )
        self._input_thread.start()

    def _input_loop(self) -> None:
        while rclpy.ok():
            print("\n현재 좌표를 출력하려면 Enter를 누르세요 (종료: Ctrl+C): ", end="", flush=True)
            if sys.stdin.readline() == "":
                return
            self._print_finished.clear()
            self._print_requested.set()
            self._print_finished.wait()

    def _handle_print_request(self) -> None:
        if not self._print_requested.is_set():
            return
        self._print_requested.clear()
        try:
            self._print_coordinates()
        finally:
            self._print_finished.set()

    def _description_callback(self, message: String) -> None:
        if not self._joints:
            self._load_description(message.data)

    def _load_description(self, description: str) -> None:
        try:
            self._joints = _parse_urdf(
                description, bool(self.get_parameter("include_fixed").value)
            )
        except (ET.ParseError, ValueError) as error:
            self.get_logger().error(f"Failed to parse robot_description: {error}")
            return
        self.get_logger().info(
            f"Loaded {len(self._joints)} joints; reporting in {self._world_frame!r}"
        )

    def _lookup(self, target: str, source: str) -> np.ndarray:
        if target == source:
            return np.eye(4)
        stamped = self._tf_buffer.lookup_transform(
            target, source, Time(), timeout=Duration(seconds=0.05)
        )
        return _tf_matrix(stamped.transform)

    def _print_coordinates(self) -> None:
        if not self._joints:
            self.get_logger().warning("Waiting for robot_description")
            return

        lines = [
            f"Current joint coordinates (reference={self._world_frame}, xyz=m, rpy=deg):"
        ]
        missing: list[str] = []
        for joint in self._joints:
            try:
                world_to_parent = self._lookup(self._world_frame, joint.parent)
                world_to_child = self._lookup(self._world_frame, joint.child)
            except tf2_ros.TransformException:
                missing.append(joint.name)
                continue

            parent_to_child = _inverse(world_to_parent) @ world_to_child
            xyz_world = world_to_child[:3, 3]
            rpy_world_deg = np.degrees(_matrix_rpy(world_to_child[:3, :3]))
            xyz_parent = parent_to_child[:3, 3]
            value = _joint_value(joint, parent_to_child)
            if joint.joint_type in ("revolute", "continuous"):
                value_text = f"q={value:+.6f} rad ({math.degrees(value):+.3f} deg)"
            elif joint.joint_type == "prismatic":
                value_text = f"q={value:+.6f} m"
            else:
                value_text = "q=fixed"
            lines.append(
                f"  {joint.name:<24} {value_text:<32} "
                f"world_xyz=({xyz_world[0]:+.6f}, {xyz_world[1]:+.6f}, {xyz_world[2]:+.6f}) "
                f"world_rpy=({rpy_world_deg[0]:+.2f}, {rpy_world_deg[1]:+.2f}, {rpy_world_deg[2]:+.2f}) "
                f"parent_xyz=({xyz_parent[0]:+.6f}, {xyz_parent[1]:+.6f}, {xyz_parent[2]:+.6f})"
            )

        for frame in EXTRA_FRAMES:
            try:
                world_to_frame = self._lookup(self._world_frame, frame)
            except tf2_ros.TransformException:
                missing.append(frame)
                continue
            xyz_world = world_to_frame[:3, 3]
            rpy_world_deg = np.degrees(_matrix_rpy(world_to_frame[:3, :3]))
            lines.append(
                f"  {frame:<24} {'frame':<32} "
                f"world_xyz=({xyz_world[0]:+.6f}, {xyz_world[1]:+.6f}, {xyz_world[2]:+.6f}) "
                f"world_rpy=({rpy_world_deg[0]:+.2f}, {rpy_world_deg[1]:+.2f}, {rpy_world_deg[2]:+.2f})"
            )

        if len(lines) > 1:
            self.get_logger().info("\n" + "\n".join(lines))
        if missing:
            self.get_logger().warning("TF not ready for: " + ", ".join(missing))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointWorldCoordinates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
