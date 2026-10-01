"""URDF/Xacro-derived forward and damped-least-squares inverse kinematics.

The chain is read from the unmodified ``tool_manipulator.urdf.xacro`` between
``base_actuator`` and ``tcp_link``.  FK multiplies each URDF origin transform
then its joint-axis rotation.  IK minimizes the six-vector
``[p_target-p(q), log(R_target R(q)^T)]`` using a central-difference spatial
Jacobian and ``dq = J^T (J J^T + lambda^2 I)^-1 e``. A direction/distance
line uses ``ceil(abs(distance)/ik_step_m)`` segments. A pose-to-pose path
uses the larger position-distance and rotation-angle step counts; position is
linear and orientation follows the shortest SO(3) rotation. Every solved
segment is checked before the full path is returned.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import xacro
import yaml


ARM_JOINTS = (
    "base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
    "wrist_roll_joint", "wrist_yaw_joint",
)


def _vector(text: str | None, size: int = 3) -> np.ndarray:
    values = [float(v) for v in (text or "0 0 0").split()]
    if len(values) != size:
        raise ValueError(f"expected {size} values, got {text!r}")
    return np.asarray(values, dtype=float)


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    cross = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]],
                      [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


def _rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    return _rotation(np.array([0.0, 0.0, 1.0]), yaw) @ _rotation(
        np.array([0.0, 1.0, 0.0]), pitch) @ _rotation(np.array([1.0, 0.0, 0.0]), roll)


def _origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        transform[:3, :3] = _rpy(_vector(element.get("rpy")))
        transform[:3, 3] = _vector(element.get("xyz"))
    return transform


def _rotvec(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1.0e-10:
        return np.zeros(3)
    vector = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                       rotation[1, 0] - rotation[0, 1]])
    return angle * vector / (2.0 * math.sin(angle))


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    origin: np.ndarray
    axis: np.ndarray
    limit: tuple[float, float] | None
    parent: str
    child: str


class ArmKinematics:
    """Kinematic chain and limits generated directly from the shared URDF."""

    def __init__(self, urdf_xacro: str | Path, hardware_yaml: str | Path,
                 base_link: str = "base_actuator", tip_link: str = "tcp_link"):
        document = xacro.process_file(str(urdf_xacro), mappings={"use_mesh": "false", "tool_id": "0"})
        root = ET.fromstring(document.toxml())
        joints = []
        for element in root.findall("joint"):
            parent = element.find("parent")
            child = element.find("child")
            if parent is None or child is None:
                continue
            limit = element.find("limit")
            limits = None if limit is None else (float(limit.get("lower")), float(limit.get("upper")))
            axis_element = element.find("axis")
            joints.append(Joint(element.get("name", ""), element.get("type", ""),
                                _origin(element.find("origin")),
                                _vector(axis_element.get("xyz") if axis_element is not None else None),
                                limits, parent.get("link", ""), child.get("link", "")))
        by_parent: dict[str, list[Joint]] = {}
        for joint in joints:
            by_parent.setdefault(joint.parent, []).append(joint)

        def find_path(link: str, seen: set[str]) -> list[Joint] | None:
            if link == tip_link:
                return []
            for joint in by_parent.get(link, []):
                if joint.child in seen:
                    continue
                tail = find_path(joint.child, seen | {joint.child})
                if tail is not None:
                    return [joint] + tail
            return None

        chain = find_path(base_link, {base_link})
        if chain is None:
            raise ValueError(f"URDF has no chain from {base_link} to {tip_link}")
        self.chain = tuple(chain)
        self.joint_names = tuple(j.name for j in self.chain if j.joint_type in ("revolute", "continuous"))
        if self.joint_names != ARM_JOINTS:
            raise ValueError(f"unexpected arm joint order from URDF: {self.joint_names}")
        self.urdf_limits = {j.name: j.limit for j in self.chain if j.name in self.joint_names}
        with Path(hardware_yaml).open(encoding="utf-8") as stream:
            hardware = yaml.safe_load(stream)
        self.limits = self._checked_hardware_limits(hardware)
        self.zero_conventions = self._checked_zero_conventions(hardware)

    def _checked_hardware_limits(self, hardware: dict) -> dict[str, tuple[float, float]]:
        result = {}
        for name in self.joint_names:
            raw = hardware.get("joints", {}).get(name, {}).get("soft_limit_rad")
            if not isinstance(raw, list) or len(raw) != 2:
                raise ValueError(f"hardware.yaml missing {name}.soft_limit_rad")
            hardware_limit = (float(raw[0]), float(raw[1]))
            if self.urdf_limits[name] is None or not np.allclose(
                    hardware_limit, self.urdf_limits[name], rtol=0.0, atol=1.0e-12):
                raise ValueError(f"URDF/hardware limit mismatch for {name}: "
                                 f"URDF={self.urdf_limits[name]}, hardware={hardware_limit}")
            result[name] = hardware_limit
        return result

    def _checked_zero_conventions(self, hardware: dict) -> dict[str, tuple]:
        """Require the driver calibration that makes /joint_states URDF-q values.

        The URDF contains kinematic joint coordinates, not motor encoder
        offsets.  Conversion therefore remains the hardware-interface's sole
        responsibility; accepting an incomplete zero convention here would
        make those joint-state values unsafe to use for IK.
        """
        conventions = {}
        for name in self.joint_names:
            spec = hardware.get("joints", {}).get(name, {})
            vendor = spec.get("vendor")
            if vendor == "dynamixel":
                if not isinstance(spec.get("zero_raw"), int) or not isinstance(spec.get("raw_increases_ccw"), bool):
                    raise ValueError(f"hardware.yaml missing Dynamixel zero convention for {name}")
                conventions[name] = (vendor, spec["zero_raw"], spec["raw_increases_ccw"])
            elif vendor == "rmd":
                if not isinstance(spec.get("sign"), (int, float)) or not isinstance(spec.get("q_offset_rad"), (int, float)):
                    raise ValueError(f"hardware.yaml missing RMD zero convention for {name}")
                conventions[name] = (vendor, float(spec["sign"]), float(spec["q_offset_rad"]))
            else:
                raise ValueError(f"hardware.yaml has unsupported/missing vendor for {name}")
        return conventions

    def fk(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} joint positions")
        transform = np.eye(4)
        index = 0
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type in ("revolute", "continuous"):
                rotate = np.eye(4)
                rotate[:3, :3] = _rotation(joint.axis, q[index])
                transform = transform @ rotate
                index += 1
        return transform

    def pose_error(self, q: np.ndarray, target: np.ndarray) -> np.ndarray:
        current = self.fk(q)
        return np.concatenate((target[:3, 3] - current[:3, 3],
                               _rotvec(target[:3, :3] @ current[:3, :3].T)))

    def jacobian(self, q: np.ndarray, epsilon: float = 1.0e-5) -> np.ndarray:
        columns = []
        for index in range(len(q)):
            plus, minus = q.copy(), q.copy()
            plus[index] += epsilon
            minus[index] -= epsilon
            p_plus, p_minus = self.fk(plus), self.fk(minus)
            linear = (p_plus[:3, 3] - p_minus[:3, 3]) / (2.0 * epsilon)
            angular = _rotvec(p_plus[:3, :3] @ p_minus[:3, :3].T) / (2.0 * epsilon)
            columns.append(np.concatenate((linear, angular)))
        return np.column_stack(columns)

    def within_limits(self, q: np.ndarray, margin_rad: float = 0.0) -> bool:
        return all(self.limits[name][0] + margin_rad <= value <= self.limits[name][1] - margin_rad
                   for name, value in zip(self.joint_names, q))

    def solve(self, target: np.ndarray, seed: np.ndarray, margin_rad: float = 0.0,
              max_iterations: int = 250, damping: float = 0.015) -> np.ndarray:
        """Return a bounded IK solution or raise; no clipping can hide a limit violation."""
        q = np.asarray(seed, dtype=float).copy()
        if not self.within_limits(q, margin_rad):
            raise ValueError("IK seed violates soft limits")
        for _ in range(max_iterations):
            error = self.pose_error(q, target)
            if np.linalg.norm(error[:3]) < 1.0e-5 and np.linalg.norm(error[3:]) < 1.0e-4:
                return q
            jacobian = self.jacobian(q)
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + (damping * damping) * np.eye(6), error)
            norm = np.linalg.norm(delta)
            if norm > 0.12:
                delta *= 0.12 / norm
            candidate = q + delta
            if not self.within_limits(candidate, margin_rad):
                raise ValueError("IK iteration would violate soft limits")
            q = candidate
        raise ValueError("IK did not converge")

    def linear_path_to_pose(self, start: np.ndarray, target: np.ndarray, step_m: float,
                            orientation_step_rad: float,
                            margin_rad: float) -> list[np.ndarray]:
        """Solve a straight-position, shortest-rotation Cartesian pose path.

        The endpoint is included and the start point is not. Failure at any
        interpolation point raises, so callers can never execute a partial
        Cartesian trajectory.
        """
        start = np.asarray(start, dtype=float)
        target = np.asarray(target, dtype=float)
        if start.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} start joints")
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ValueError("target must be a finite 4x4 transform")
        if not math.isfinite(step_m) or step_m <= 0.0:
            raise ValueError("Cartesian position step must be finite and > 0")
        if not math.isfinite(orientation_step_rad) or orientation_step_rad <= 0.0:
            raise ValueError("Cartesian orientation step must be finite and > 0")

        initial = self.fk(start)
        translation = target[:3, 3] - initial[:3, 3]
        distance = float(np.linalg.norm(translation))
        rotation_vector = _rotvec(target[:3, :3] @ initial[:3, :3].T)
        angle = float(np.linalg.norm(rotation_vector))
        if distance <= 1.0e-12 and angle <= 1.0e-12:
            raise ValueError("Cartesian target equals the current TCP pose")
        position_steps = math.ceil(distance / step_m)
        orientation_steps = math.ceil(angle / orientation_step_rad)
        count = max(1, position_steps, orientation_steps)

        path, seed = [], start.copy()
        for index in range(1, count + 1):
            fraction = index / count
            waypoint = np.eye(4)
            waypoint[:3, 3] = initial[:3, 3] + fraction * translation
            if angle <= 1.0e-12:
                waypoint[:3, :3] = initial[:3, :3]
            else:
                waypoint[:3, :3] = (
                    _rotation(rotation_vector / angle, fraction * angle)
                    @ initial[:3, :3]
                )
            seed = self.solve(waypoint, seed, margin_rad)
            if not self.within_limits(seed, margin_rad):
                raise ValueError(f"pose-line IK point {index}/{count} violates soft limits")
            path.append(seed.copy())
        return path

    def linear_path(self, start: np.ndarray, direction_base: np.ndarray, distance_m: float,
                    step_m: float, margin_rad: float) -> list[np.ndarray]:
        """Solve a fixed-orientation Cartesian line; failure rejects the whole path."""
        direction = np.asarray(direction_base, dtype=float)
        norm = np.linalg.norm(direction)
        if not np.isfinite(norm) or norm == 0.0 or distance_m <= 0.0 or step_m <= 0.0:
            raise ValueError("line direction, distance and step must be finite and positive")
        count = max(1, math.ceil(distance_m / step_m))
        initial = self.fk(np.asarray(start, dtype=float))
        path, seed = [], np.asarray(start, dtype=float)
        for index in range(1, count + 1):
            target = initial.copy()
            target[:3, 3] += direction / norm * distance_m * index / count
            seed = self.solve(target, seed, margin_rad)
            if not self.within_limits(seed, margin_rad):
                raise ValueError(f"line IK point {index}/{count} violates soft limits")
            path.append(seed.copy())
        return path
