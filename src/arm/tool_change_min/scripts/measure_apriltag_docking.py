#!/usr/bin/env python3
"""Record AprilTag docking accuracy and an optional tag-to-target transform.

The node consumes the existing JSON detection stream and CameraInfo; it never
commands robot motion. For each selected tag it solves PnP again from the four
published, sub-pixel-refined corners, the measured square size, camera matrix,
and distortion coefficients. OpenCV returns T_camera_tag. Reprojecting the
four object corners gives the pixel RMSE. Raw detector XYZ and corner-PnP XYZ
are logged separately with their difference, detection rate, decision margin,
hamming count, and image-stamp-to-receive latency.

When target_frame is non-empty, external TF supplies T_base_camera and
T_base_target at the image stamp. The measured docking offset is
T_tag_target = inv(T_base_camera*T_camera_tag)*T_base_target. Translation
statistics and a sign-consistent quaternion mean are written to the summary.
No TF is published. Sample count, physical tag size, and output CSV path are
mandatory; missing values terminate startup instead of being guessed.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


CSV_FIELDS = (
    "sample_index", "stamp_sec", "stamp_nanosec", "receive_time_ns",
    "latency_ms", "detected", "pnp_success", "tf_success", "frame_id",
    "tag_id", "hamming", "decision_margin", "center_u_px", "center_v_px",
    "raw_x_m", "raw_y_m", "raw_z_m", "pnp_x_m", "pnp_y_m", "pnp_z_m",
    "pnp_qx", "pnp_qy", "pnp_qz", "pnp_qw",
    "delta_x_m", "delta_y_m", "delta_z_m", "reprojection_rmse_px",
    "tag_target_x_m", "tag_target_y_m", "tag_target_z_m",
    "tag_target_qx", "tag_target_qy", "tag_target_qz", "tag_target_qw",
    "error",
)


def _matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.asarray([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not np.all(np.isfinite(quaternion)) or norm <= 1.0e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = quaternion / norm
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w), 2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w), 1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w), 2.0 * (y*z + x*w), 1.0 - 2.0 * (x*x + y*y)],
    ])


def _quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized ROS-order [x,y,z,w] quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    return quaternion / np.linalg.norm(quaternion)


def _transform_from_msg(message: TransformStamped) -> np.ndarray:
    transform = np.eye(4)
    translation = message.transform.translation
    rotation = message.transform.rotation
    transform[:3, 3] = [translation.x, translation.y, translation.z]
    transform[:3, :3] = _matrix_from_quaternion(
        rotation.x, rotation.y, rotation.z, rotation.w)
    return transform


def _quaternion_mean(quaternions: list[np.ndarray]) -> np.ndarray:
    reference = np.asarray(quaternions[0], dtype=float)
    aligned = []
    for quaternion in quaternions:
        value = np.asarray(quaternion, dtype=float)
        aligned.append(-value if float(np.dot(value, reference)) < 0.0 else value)
    accumulator = sum(np.outer(value, value) for value in aligned)
    values, vectors = np.linalg.eigh(accumulator)
    result = vectors[:, int(np.argmax(values))]
    if float(np.dot(result, reference)) < 0.0:
        result = -result
    return result / np.linalg.norm(result)


def _stats(vectors: list[np.ndarray]) -> dict[str, Any] | None:
    if not vectors:
        return None
    values = np.asarray(vectors, dtype=float)
    return {
        "count": int(len(values)),
        "mean": np.mean(values, axis=0).tolist(),
        "std": np.std(values, axis=0).tolist(),
        "min": np.min(values, axis=0).tolist(),
        "max": np.max(values, axis=0).tolist(),
        "peak_to_peak": np.ptp(values, axis=0).tolist(),
    }


def _scalar_stats(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    array = np.asarray(values, dtype=float)
    return {
        "count": int(len(array)),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        "p95": float(np.percentile(array, 95.0)),
    }


class AprilTagDockingMeasurement(Node):
    """Read-only measurement node for tag-1 vision and docking offset."""

    def __init__(self):
        super().__init__("apriltag_docking_measurement")
        self.declare_parameter("centers_topic", "/arm/apriltag/centers")
        self.declare_parameter("camera_info_topic", "/arm/camera/color/camera_info")
        self.declare_parameter("tag_id", 1)
        self.declare_parameter("tag_size_m", 0.0)
        self.declare_parameter("sample_count", 0)
        self.declare_parameter("output_csv", "")
        self.declare_parameter("base_frame", "base_actuator")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("tf_timeout_s", 0.05)
        self.declare_parameter("reference_camera_position_m", [])

        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        output_csv_value = str(self.get_parameter("output_csv").value).strip()
        self.output_csv = Path(output_csv_value) if output_csv_value else None
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        reference = list(self.get_parameter("reference_camera_position_m").value)

        missing = []
        if self.tag_size_m <= 0.0 or not math.isfinite(self.tag_size_m):
            missing.append("tag_size_m (> 0)")
        if self.sample_count <= 0:
            missing.append("sample_count (> 0)")
        if self.output_csv is None:
            missing.append("output_csv")
        if self.tf_timeout_s <= 0.0 or not math.isfinite(self.tf_timeout_s):
            missing.append("tf_timeout_s (> 0)")
        if reference and (
                len(reference) != 3
                or not all(math.isfinite(float(value)) for value in reference)):
            missing.append("reference_camera_position_m ([] or finite [x,y,z])")
        if missing:
            raise ValueError("missing/invalid measurement parameters: " + ", ".join(missing))
        self.reference = None if not reference else np.asarray(reference, dtype=float)

        assert self.output_csv is not None
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_stream = self.output_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_stream, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.summary_path = self.output_csv.with_suffix(".summary.json")

        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_info: dict[str, Any] | None = None
        self.total_frames = 0
        self.detected_frames = 0
        self.pnp_frames = 0
        self.tf_frames = 0
        self.finished = False
        self.raw_positions: list[np.ndarray] = []
        self.pnp_positions: list[np.ndarray] = []
        self.position_deltas: list[np.ndarray] = []
        self.reference_errors: list[np.ndarray] = []
        self.reprojection_errors: list[float] = []
        self.latencies_ms: list[float] = []
        self.decision_margins: list[float] = []
        self.tag_target_positions: list[np.ndarray] = []
        self.tag_target_quaternions: list[np.ndarray] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter("centers_topic").value),
            self._centers_callback, 10)
        self.get_logger().info(
            f"measurement armed: tag_id={self.tag_id}, samples={self.sample_count}, "
            f"output={self.output_csv}, target_frame={self.target_frame or '<disabled>'}")

    def _camera_info_callback(self, message: CameraInfo):
        matrix = np.asarray(message.k, dtype=float).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().warning("rejected CameraInfo with invalid focal length")
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(message.d, dtype=float)
        self.camera_info = {
            "frame_id": message.header.frame_id,
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "k": matrix.reshape(-1).tolist(),
            "d": self.distortion.tolist(),
        }

    def _solve_pnp(self, detection: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float]:
        if self.camera_matrix is None or self.distortion is None:
            raise ValueError("CameraInfo has not been received")
        corners = detection.get("corners_px")
        if not isinstance(corners, list) or len(corners) != 4:
            raise ValueError("detection does not contain four corners_px")
        image_points = np.asarray(
            [[float(corner["u"]), float(corner["v"])] for corner in corners],
            dtype=np.float64)
        half = self.tag_size_m / 2.0
        object_points = np.asarray([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float64)
        success, rotation_vector, translation = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            raise ValueError("OpenCV solvePnP failed")
        rotation, _ = cv2.Rodrigues(rotation_vector)
        translation = np.asarray(translation, dtype=float).reshape(3)
        if translation[2] <= 0.0:
            raise ValueError("PnP returned a tag behind the camera")
        projected, _ = cv2.projectPoints(
            object_points, rotation_vector, translation,
            self.camera_matrix, self.distortion)
        residual = image_points - projected.reshape(4, 2)
        rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        return rotation, translation, rmse

    def _tag_target(self, frame_id: str, stamp: Time,
                    camera_to_tag: np.ndarray) -> np.ndarray:
        base_to_camera = _transform_from_msg(self.tf_buffer.lookup_transform(
            self.base_frame, frame_id, stamp,
            timeout=Duration(seconds=self.tf_timeout_s)))
        base_to_target = _transform_from_msg(self.tf_buffer.lookup_transform(
            self.base_frame, self.target_frame, stamp,
            timeout=Duration(seconds=self.tf_timeout_s)))
        return np.linalg.inv(base_to_camera @ camera_to_tag) @ base_to_target

    def _centers_callback(self, message: String):
        if self.finished:
            return
        if self.camera_matrix is None:
            self.get_logger().warning(
                "waiting for CameraInfo; measurement sample not counted")
            return
        receive_ns = int(self.get_clock().now().nanoseconds)
        row = {field: "" for field in CSV_FIELDS}
        row.update({
            "sample_index": self.total_frames,
            "receive_time_ns": receive_ns,
            "tag_id": self.tag_id,
            "detected": 0,
            "pnp_success": 0,
            "tf_success": 0,
        })
        try:
            payload = json.loads(message.data)
            stamp_sec = int(payload["stamp"]["sec"])
            stamp_nanosec = int(payload["stamp"]["nanosec"])
            stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
            latency_ms = (receive_ns - stamp_ns) / 1.0e6
            frame_id = str(payload.get("frame_id", ""))
            row.update({
                "stamp_sec": stamp_sec, "stamp_nanosec": stamp_nanosec,
                "latency_ms": latency_ms, "frame_id": frame_id,
            })
            if math.isfinite(latency_ms) and latency_ms >= 0.0:
                self.latencies_ms.append(latency_ms)

            detection = next((
                item for item in payload.get("detections", [])
                if int(item.get("id", -1)) == self.tag_id), None)
            if detection is not None:
                self.detected_frames += 1
                row["detected"] = 1
                row["hamming"] = int(detection.get("hamming", -1))
                margin = float(detection.get("decision_margin", float("nan")))
                row["decision_margin"] = margin
                if math.isfinite(margin):
                    self.decision_margins.append(margin)
                center = detection.get("center_px", {})
                row["center_u_px"] = center.get("u", "")
                row["center_v_px"] = center.get("v", "")

                raw_data = detection.get("position_camera_cm")
                raw_position = None
                if isinstance(raw_data, dict):
                    raw_position = np.asarray([
                        float(raw_data["x"]), float(raw_data["y"]),
                        float(raw_data["z"])
                    ]) / 100.0
                    self.raw_positions.append(raw_position)
                    row["raw_x_m"], row["raw_y_m"], row["raw_z_m"] = raw_position

                rotation, translation, rmse = self._solve_pnp(detection)
                quaternion = _quaternion_from_matrix(rotation)
                self.pnp_frames += 1
                self.pnp_positions.append(translation)
                self.reprojection_errors.append(rmse)
                row.update({
                    "pnp_success": 1,
                    "pnp_x_m": translation[0], "pnp_y_m": translation[1],
                    "pnp_z_m": translation[2],
                    "pnp_qx": quaternion[0], "pnp_qy": quaternion[1],
                    "pnp_qz": quaternion[2], "pnp_qw": quaternion[3],
                    "reprojection_rmse_px": rmse,
                })
                if raw_position is not None:
                    delta = translation - raw_position
                    self.position_deltas.append(delta)
                    row["delta_x_m"], row["delta_y_m"], row["delta_z_m"] = delta
                if self.reference is not None:
                    self.reference_errors.append(translation - self.reference)

                if self.target_frame:
                    camera_to_tag = np.eye(4)
                    camera_to_tag[:3, :3] = rotation
                    camera_to_tag[:3, 3] = translation
                    tag_to_target = self._tag_target(
                        frame_id, Time(nanoseconds=stamp_ns), camera_to_tag)
                    target_quaternion = _quaternion_from_matrix(tag_to_target[:3, :3])
                    self.tf_frames += 1
                    self.tag_target_positions.append(tag_to_target[:3, 3].copy())
                    self.tag_target_quaternions.append(target_quaternion)
                    row.update({
                        "tf_success": 1,
                        "tag_target_x_m": tag_to_target[0, 3],
                        "tag_target_y_m": tag_to_target[1, 3],
                        "tag_target_z_m": tag_to_target[2, 3],
                        "tag_target_qx": target_quaternion[0],
                        "tag_target_qy": target_quaternion[1],
                        "tag_target_qz": target_quaternion[2],
                        "tag_target_qw": target_quaternion[3],
                    })
        except (KeyError, TypeError, ValueError, cv2.error, TransformException) as exc:
            row["error"] = str(exc)

        self.writer.writerow(row)
        self.csv_stream.flush()
        self.total_frames += 1
        if self.total_frames >= self.sample_count:
            self.finish()
            rclpy.shutdown()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        quaternion_mean = (
            None if not self.tag_target_quaternions
            else _quaternion_mean(self.tag_target_quaternions).tolist())
        orientation_deviation = []
        if quaternion_mean is not None:
            mean = np.asarray(quaternion_mean, dtype=float)
            orientation_deviation = [
                2.0 * math.acos(float(np.clip(abs(np.dot(value, mean)), 0.0, 1.0)))
                for value in self.tag_target_quaternions
            ]
        summary = {
            "tag_id": self.tag_id,
            "tag_size_m": self.tag_size_m,
            "total_frames": self.total_frames,
            "detected_frames": self.detected_frames,
            "detection_rate": (
                0.0 if self.total_frames == 0
                else self.detected_frames / self.total_frames),
            "pnp_success_frames": self.pnp_frames,
            "tf_success_frames": self.tf_frames,
            "camera_info": self.camera_info,
            "raw_position_camera_m": _stats(self.raw_positions),
            "corner_pnp_position_camera_m": _stats(self.pnp_positions),
            "corner_pnp_minus_raw_m": _stats(self.position_deltas),
            "corner_pnp_minus_reference_m": _stats(self.reference_errors),
            "reprojection_rmse_px": _scalar_stats(self.reprojection_errors),
            "capture_to_receive_latency_ms": _scalar_stats(self.latencies_ms),
            "decision_margin": _scalar_stats(self.decision_margins),
            "tag_to_target": {
                "base_frame": self.base_frame,
                "target_frame": self.target_frame or None,
                "translation_m": _stats(self.tag_target_positions),
                "orientation_xyzw_mean": quaternion_mean,
                "orientation_deviation_rad": _scalar_stats(orientation_deviation),
            },
        }
        self.summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        self.csv_stream.close()
        self.get_logger().info(
            f"measurement complete: detected={self.detected_frames}/{self.total_frames}, "
            f"pnp={self.pnp_frames}, tf={self.tf_frames}, summary={self.summary_path}")

    def destroy_node(self):
        self.finish()
        return super().destroy_node()


def main():
    rclpy.init()
    try:
        node = AprilTagDockingMeasurement()
    except Exception as exc:
        rclpy.logging.get_logger("apriltag_docking_measurement").error(
            f"configuration invalid; exiting: {exc}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
