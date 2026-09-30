#!/usr/bin/env python3
"""Validate existing AprilTag JSON and aligned depth before it reaches motion."""
from __future__ import annotations

import json
import math
import struct
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


class TagPoseFilter(Node):
    """The vision package remains unchanged; this node owns docking admission."""
    def __init__(self) -> None:
        super().__init__('tag_pose_filter')
        for name, default in (
            ('tools_config_file', ''),
            ('source_topic', '/arm/apriltag/centers'),
            ('depth_topic', '/arm/camera/aligned_depth_to_color/image_raw'),
            ('selected_tool_id_topic', '/selected_tool_id'),
            ('valid_pose_topic', '/tag_pose_valid'),
            ('status_topic', '/tag_status'),
            ('expected_frame_id', ''),
            ('max_age_sec', 0.25),
            # Every physical gate deliberately has no guessed production default.
            ('min_decision_margin', -1.0),
            ('depth_range_m', [-1.0, -1.0]),
            ('max_depth_age_sec', -1.0),
            ('max_depth_pnp_error_m', -1.0),
            ('depth_window_radius_px', 2),
        ):
            self.declare_parameter(name, default)
        self.tools = self._load_tools(self.get_parameter('tools_config_file').value)
        self.requested_tool_id = 0
        self.latest_depth: Image | None = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(String, self.get_parameter('valid_pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, latched)
        self.create_subscription(String, self.get_parameter('source_topic').value, self._source_cb, 20)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self._depth_cb, 10)
        self.create_subscription(Int32, self.get_parameter('selected_tool_id_topic').value, self._tool_cb, 10)
        self._status('idle:no_tool_selected')

    @staticmethod
    def _load_tools(filename: str) -> dict[int, dict]:
        if not filename:
            raise RuntimeError('tools_config_file is required')
        try:
            with Path(filename).open(encoding='utf-8') as stream:
                return {int(key): value for key, value in (yaml.safe_load(stream) or {}).get('tools', {}).items()}
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(f'cannot read tools_config_file: {exc}') from exc

    def _tool_cb(self, msg: Int32) -> None:
        self.requested_tool_id = int(msg.data)
        self._status('idle:no_tool_selected' if not self.requested_tool_id else f'awaiting_tag:tool_{self.requested_tool_id}')

    def _depth_cb(self, msg: Image) -> None:
        # Store only the newest aligned-depth frame. The tag callback verifies its
        # timestamp against the RGB tag image before using any pixel.
        self.latest_depth = msg

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _gate_configured(self) -> bool:
        margin = float(self.get_parameter('min_decision_margin').value)
        depth_range = [float(value) for value in self.get_parameter('depth_range_m').value]
        max_depth_age = float(self.get_parameter('max_depth_age_sec').value)
        max_depth_error = float(self.get_parameter('max_depth_pnp_error_m').value)
        return (margin >= 0.0 and len(depth_range) == 2 and 0.0 < depth_range[0] < depth_range[1]
                and max_depth_age > 0.0 and max_depth_error >= 0.0)

    def _depth_at(self, source_sec: float, center: dict) -> tuple[float | None, str]:
        image = self.latest_depth
        if image is None:
            return None, 'waiting:aligned_depth_unavailable'
        age = abs(self._stamp_sec(image.header.stamp) - source_sec)
        if age > float(self.get_parameter('max_depth_age_sec').value):
            return None, f'rejected:aligned_depth_stale:{age:.3f}s'
        try:
            u, v = int(round(float(center['u']))), int(round(float(center['v'])))
        except (KeyError, TypeError, ValueError):
            return None, 'rejected:tag_center_unavailable'
        encoding = image.encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            bytes_per_pixel, scale, value_format = 2, 0.001, 'H'
        elif encoding == '32fc1':
            bytes_per_pixel, scale, value_format = 4, 1.0, 'f'
        else:
            return None, f'rejected:unsupported_depth_encoding:{image.encoding}'
        radius = int(self.get_parameter('depth_window_radius_px').value)
        if radius < 0 or u < radius or v < radius or u + radius >= image.width or v + radius >= image.height:
            return None, 'rejected:tag_center_outside_depth_image'
        endian = '>' if image.is_bigendian else '<'
        samples = []
        for y in range(v - radius, v + radius + 1):
            for x in range(u - radius, u + radius + 1):
                offset = y * image.step + x * bytes_per_pixel
                if offset + bytes_per_pixel > len(image.data):
                    return None, 'rejected:invalid_depth_image_layout'
                raw = struct.unpack_from(endian + value_format, image.data, offset)[0]
                depth_m = float(raw) * scale
                if math.isfinite(depth_m) and depth_m > 0.0:
                    samples.append(depth_m)
        if not samples:
            return None, 'rejected:no_valid_depth_at_tag'
        samples.sort()
        return samples[len(samples) // 2], ''

    def _source_cb(self, msg: String) -> None:
        if self.requested_tool_id <= 0:
            return
        spec = self.tools.get(self.requested_tool_id)
        if not spec:
            self._status(f'rejected:unknown_tool:{self.requested_tool_id}')
            return
        if not spec.get('enabled', False):
            self._status(f'rejected:tool_disabled:{self.requested_tool_id}')
            return
        tag_id = spec.get('tag_id')
        if not isinstance(tag_id, int):
            self._status(f'rejected:tag_id_unconfigured:tool_{self.requested_tool_id}')
            return
        if not self._gate_configured():
            self._status('rejected:tag_quality_or_depth_gate_unconfigured')
            return
        try:
            source = json.loads(msg.data)
            stamp = source['stamp']
            source_sec = float(stamp['sec']) + float(stamp['nanosec']) * 1e-9
            frame_id = str(source['frame_id'])
            detection = next(item for item in source.get('detections', []) if int(item.get('id', -1)) == tag_id)
            position_cm = detection['position_camera_cm']
            center_px = detection['center_px']
            xyz_m = [float(position_cm[axis]) / 100.0 for axis in ('x', 'y', 'z')]
            margin = float(detection['decision_margin'])
        except (KeyError, StopIteration, TypeError, ValueError, json.JSONDecodeError):
            self._status(f'waiting:tag_{tag_id}_not_valid')
            return
        depth_m, depth_status = self._depth_at(source_sec, center_px)
        if depth_m is None:
            self._status(depth_status)
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - source_sec
        distance = math.sqrt(sum(value * value for value in xyz_m))
        pnp_z_m = xyz_m[2]
        if not frame_id:
            self._status('rejected:invalid_tag_pose')
        elif frame_id != self.get_parameter('expected_frame_id').value:
            self._status(f'rejected:unexpected_tag_frame:{frame_id}')
        elif not all(math.isfinite(value) for value in xyz_m):
            self._status('rejected:invalid_tag_pose')
        elif age < -0.05 or age > float(self.get_parameter('max_age_sec').value):
            self._status(f'rejected:stale_tag_pose:{age:.3f}s')
        elif margin < float(self.get_parameter('min_decision_margin').value):
            self._status(f'rejected:decision_margin:{margin:.2f}')
        elif not self.get_parameter('depth_range_m').value[0] <= distance <= self.get_parameter('depth_range_m').value[1]:
            self._status(f'rejected:pnp_distance:{distance:.3f}m')
        elif abs(depth_m - pnp_z_m) > float(self.get_parameter('max_depth_pnp_error_m').value):
            self._status(f'rejected:depth_pnp_disagreement:{depth_m:.3f}/{pnp_z_m:.3f}m')
        else:
            # id is normalized to tool_id for the legacy motion executor. The
            # physical tag ID and the independent aligned-depth check are kept.
            self.pose_pub.publish(String(data=json.dumps({
                'stamp': stamp, 'frame_id': frame_id,
                'detections': [{
                    'id': self.requested_tool_id,
                    'source_tag_id': tag_id,
                    'position_camera_cm': position_cm,
                    'decision_margin': margin,
                    'aligned_depth_m': depth_m,
                    'pnp_z_m': pnp_z_m,
                }],
            }, separators=(',', ':'))))
            self._status(f'valid:tool_{self.requested_tool_id}:tag_{tag_id}:depth={depth_m:.3f}m')

    def _status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = TagPoseFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
