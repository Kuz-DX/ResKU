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
import time
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


def _bounded_damped_step(jacobian, error, lower, upper, damping):
    """Active-set box-constrained damped least squares for one IK iteration."""
    hessian = jacobian.T @ jacobian + damping**2 * np.eye(jacobian.shape[1])
    gradient = jacobian.T @ error
    step = np.zeros(jacobian.shape[1])
    active = np.zeros(len(step), dtype=int)  # -1 lower, +1 upper, 0 free
    for _ in range(80):
        free = active == 0
        candidate = step.copy()
        if np.any(free):
            candidate[free] = np.linalg.solve(
                hessian[np.ix_(free, free)],
                gradient[free] - hessian[np.ix_(free, ~free)] @ step[~free])
        direction = candidate - step
        fraction, hit, side = 1., None, 0
        for j in np.flatnonzero(free):
            if candidate[j] < lower[j]:
                value = (lower[j]-step[j])/direction[j]
                if value < fraction:
                    fraction, hit, side = value, j, -1
            elif candidate[j] > upper[j]:
                value = (upper[j]-step[j])/direction[j]
                if value < fraction:
                    fraction, hit, side = value, j, 1
        step += fraction * direction
        if hit is not None:
            step[hit] = lower[hit] if side < 0 else upper[hit]
            active[hit] = side
            continue
        derivative = hessian @ step - gradient
        violation = active * derivative
        if np.max(violation) <= 1e-10:
            return step
        active[int(np.argmax(violation))] = 0
    raise ValueError('axis IK bound search did not converge')


class ArmKinematics:
    """Kinematic chain and limits generated directly from the shared URDF."""

    def __init__(self, urdf_xacro: str | Path, hardware_yaml: str | Path,
                 base_link: str = "base_actuator", tip_link: str = "tcp_link",
                 description: str | None = None):
        if description is None:
            description = xacro.process_file(str(urdf_xacro), mappings={"use_mesh": "false", "tool_id": "0"}).toxml()
        root = ET.fromstring(description)
        joints = []
        for element in root.findall("joint"):
            parent = element.find("parent")
            child = element.find("child")
            if parent is None or child is None:
                continue
            limit = element.find("limit")
            limits = (None if limit is None or limit.get('lower') is None or limit.get('upper') is None
                      else (float(limit.get("lower")), float(limit.get("upper"))))
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

    def pose_and_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Compute FK and the spatial Jacobian in one chain traversal."""
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} joint positions")
        transform = np.eye(4)
        origins, axes = [], []
        index = 0
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type in ("revolute", "continuous"):
                origins.append(transform[:3, 3].copy())
                axes.append(transform[:3, :3] @ joint.axis)
                rotate = np.eye(4)
                rotate[:3, :3] = _rotation(joint.axis, q[index])
                transform = transform @ rotate
                index += 1
        tip = transform[:3, 3]
        jacobian = np.column_stack([
            np.concatenate((np.cross(axis, tip-origin), axis))
            for origin, axis in zip(origins, axes)])
        return transform, jacobian

    @staticmethod
    def _pose_error_from_pose(current: np.ndarray, target: np.ndarray) -> np.ndarray:
        return np.concatenate((target[:3, 3] - current[:3, 3],
                               _rotvec(target[:3, :3] @ current[:3, :3].T)))

    def pose_error(self, q: np.ndarray, target: np.ndarray) -> np.ndarray:
        return self._pose_error_from_pose(self.fk(q), target)

    def jacobian(self, q: np.ndarray, epsilon: float = 1.0e-5) -> np.ndarray:
        # Geometric Jacobian in the root frame. Include every fixed URDF
        # transform; each revolute axis is expressed before its own rotation.
        # epsilon remains accepted for compatibility with existing callers.
        return self.pose_and_jacobian(q)[1]

    def within_limits(self, q: np.ndarray, margin_rad: float = 0.0) -> bool:
        return (np.asarray(q).shape == (len(self.joint_names),) and np.all(np.isfinite(q)) and
                all(self.limits[name][0] + margin_rad <= value <= self.limits[name][1] - margin_rad
                   for name, value in zip(self.joint_names, q)))

    def solve(self, target: np.ndarray, seed: np.ndarray, margin_rad: float = 0.0,
              max_iterations: int = 250, damping: float = 0.015,
              max_time_s: float | None = None) -> np.ndarray:
        """Return a bounded IK solution or raise; no clipping can hide a limit violation."""
        q = np.asarray(seed, dtype=float).copy()
        if max_time_s is not None and (not math.isfinite(max_time_s) or max_time_s <= 0):
            raise ValueError('IK time budget must be finite and positive')
        deadline = math.inf if max_time_s is None else time.monotonic()+max_time_s
        if not self.within_limits(q, margin_rad):
            raise ValueError("IK seed violates soft limits")
        for _ in range(max_iterations):
            if time.monotonic() >= deadline:
                raise ValueError('IK time budget exceeded')
            current, jacobian = self.pose_and_jacobian(q)
            error = self._pose_error_from_pose(current, target)
            if np.linalg.norm(error[:3]) < 1.0e-5 and np.linalg.norm(error[3:]) < 1.0e-4:
                if time.monotonic() >= deadline:
                    raise ValueError('IK time budget exceeded')
                return q
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + (damping * damping) * np.eye(6), error)
            norm = np.linalg.norm(delta)
            if norm > 0.12:
                delta *= 0.12 / norm
            candidate = q + delta
            if not self.within_limits(candidate, margin_rad):
                violations = [
                    f'{name} candidate={value:.6f} allowed='
                    f'[{self.limits[name][0]+margin_rad:.6f}, '
                    f'{self.limits[name][1]-margin_rad:.6f}] rad'
                    for name, value in zip(self.joint_names, candidate)
                    if (not math.isfinite(value) or
                        not self.limits[name][0]+margin_rad <= value <=
                        self.limits[name][1]-margin_rad)]
                raise ValueError('IK iteration would violate soft limits: '
                                 + '; '.join(violations))
            q = candidate
        raise ValueError("IK did not converge")

    def solve_axis(self, position, direction, local_axis, seed, margin_rad=0.0,
                   max_iterations=100, max_time_s=.03, joint_step_rad=.03):
        """Constrain TCP position and a mounted axis; leave axial rotation free.

        Every iteration respects hardware margins and the seed-relative joint
        step bound. A joint at its bound remains available to move back inward.
        """
        position, direction, local_axis, seed = [np.asarray(v, dtype=float) for v in
                                               (position, direction, local_axis, seed)]
        if (any(v.shape != (3,) or not np.all(np.isfinite(v))
                for v in (position, direction, local_axis))
                or np.linalg.norm(direction) < 1e-9 or np.linalg.norm(local_axis) < 1e-9
                or not math.isfinite(max_time_s) or max_time_s <= 0
                or not math.isfinite(joint_step_rad) or joint_step_rad <= 0):
            raise ValueError('invalid axis IK input')
        if not self.within_limits(seed, margin_rad):
            raise ValueError('axis IK seed violates soft limits')
        direction = direction / np.linalg.norm(direction)
        local_axis = local_axis / np.linalg.norm(local_axis)
        lower = np.maximum([self.limits[j][0]+margin_rad for j in self.joint_names],
                           seed-joint_step_rad)
        upper = np.minimum([self.limits[j][1]-margin_rad for j in self.joint_names],
                           seed+joint_step_rad)
        deadline = time.monotonic()+max_time_s
        q = seed.copy()
        for _ in range(max_iterations):
            if time.monotonic() >= deadline:
                raise ValueError('axis IK time budget exceeded')
            pose, jacobian = self.pose_and_jacobian(q)
            axis = pose[:3, :3] @ local_axis
            error = np.r_[position-pose[:3, 3], direction-axis]
            if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
                if time.monotonic() >= deadline:
                    raise ValueError('axis IK time budget exceeded')
                return q
            jacobian[3:] = np.cross(jacobian[3:].T, axis).T
            delta = _bounded_damped_step(jacobian, error, lower-q, upper-q, .003)
            for backtrack in range(12):
                candidate = q + delta * 2.**(-backtrack)
                if not self.within_limits(candidate, margin_rad):
                    continue
                next_pose = self.fk(candidate)
                next_error = np.r_[position-next_pose[:3, 3],
                                   direction-next_pose[:3, :3] @ local_axis]
                if next_error @ next_error < error @ error:
                    q = candidate
                    break
            else:
                break
        pose = self.fk(q)
        angle = math.acos(float(np.clip(direction @ (pose[:3, :3] @ local_axis), -1., 1.)))
        active = [f'{name}={value:.6f}' for j, (name, value) in enumerate(zip(self.joint_names, q))
                  if min(value-lower[j], upper[j]-value) < 1e-6]
        raise ValueError('axis IK could not satisfy position/direction within joint bounds: '
                         f'position_error={np.linalg.norm(position-pose[:3, 3])*1000:.3f}mm, '
                         f'axis_error={angle:.6f}rad; active soft/step bounds={active}')

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
