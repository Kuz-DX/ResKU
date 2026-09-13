#!/usr/bin/env python3
"""Publish a ROS 2 JPEG CompressedImage topic to an RTSP server.

The ROS callback forwards the JPEG payload to GStreamer without decoding it in
Python. GStreamer decodes JPEG, encodes browser-compatible H.264, and publishes
the stream to MediaMTX through RTSP/TCP.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def _gst_quote(value: str) -> str:
    """Quote a string used as a GStreamer launch property."""
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


class CompressedImageRtspBridge(Node):
    """Bridge one JPEG CompressedImage topic to one MediaMTX RTSP path."""

    def __init__(self) -> None:
        super().__init__("compressed_image_rtsp_bridge")

        self.declare_parameter("image_topic", "/side/left/image_raw/compressed")
        self.declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/sub1")
        self.declare_parameter("fps", 15)
        self.declare_parameter("bitrate_kbps", 1500)
        self.declare_parameter("keyframe_interval", 15)
        self.declare_parameter("restart_delay_sec", 5.0)
        self.declare_parameter("stats_interval_sec", 10.0)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._rtsp_url = str(self.get_parameter("rtsp_url").value)
        self._fps = int(self.get_parameter("fps").value)
        self._bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)
        self._keyframe_interval = int(self.get_parameter("keyframe_interval").value)
        self._restart_delay_sec = float(self.get_parameter("restart_delay_sec").value)
        self._stats_interval_sec = float(self.get_parameter("stats_interval_sec").value)
        self._validate_parameters()

        Gst.init(None)
        self._require_gstreamer_elements()

        self._pipeline: Optional[Gst.Pipeline] = None
        self._appsrc: Optional[Gst.Element] = None
        self._restart_at = 0.0
        self._received_frames = 0
        self._published_frames = 0
        self._rejected_frames = 0
        self._last_stats_at = time.monotonic()
        self._last_received_frames = 0
        self._last_published_frames = 0
        self._last_bad_format_log_at = 0.0

        # [갱신, 2026-09-05] BEST_EFFORT로 구독한다. DDS 매칭 규칙상 BEST_EFFORT
        # 구독자는 RELIABLE 발행자(usb_cam, 기본 realsense)와 BEST_EFFORT 발행자
        # (SENSOR_DATA QoS로 띄운 realsense, summer_supply debug_image) 모두와
        # 연결되지만, 이전의 RELIABLE 구독은 BEST_EFFORT 발행자와 매칭이 안 돼
        # 팔 카메라(/arm/camera/...)가 UI에 아예 안 뜨는 원인이었다.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            CompressedImage,
            self._image_topic,
            self._on_image,
            qos,
        )
        self._health_timer = self.create_timer(0.25, self._poll_pipeline)

        self._start_pipeline()
        self.get_logger().info(
            f"ROS JPEG -> RTSP bridge ready: {self._image_topic} -> {self._rtsp_url}"
        )

    def _validate_parameters(self) -> None:
        if not self._image_topic.startswith("/"):
            raise ValueError("image_topic must be an absolute ROS topic")
        if not self._rtsp_url.startswith(("rtsp://", "rtsps://")):
            raise ValueError("rtsp_url must start with rtsp:// or rtsps://")
        if self._fps <= 0:
            raise ValueError("fps must be greater than zero")
        if self._bitrate_kbps <= 0:
            raise ValueError("bitrate_kbps must be greater than zero")
        if self._keyframe_interval <= 0:
            raise ValueError("keyframe_interval must be greater than zero")
        if not math.isfinite(self._restart_delay_sec) or self._restart_delay_sec <= 0:
            raise ValueError("restart_delay_sec must be a positive finite number")
        if not math.isfinite(self._stats_interval_sec) or self._stats_interval_sec <= 0:
            raise ValueError("stats_interval_sec must be a positive finite number")

    def _require_gstreamer_elements(self) -> None:
        required = (
            "appsrc",
            "queue",
            "jpegdec",
            "videoconvert",
            "x264enc",
            "h264parse",
            "rtspclientsink",
        )
        missing = [name for name in required if Gst.ElementFactory.find(name) is None]
        if missing:
            raise RuntimeError(
                "Missing GStreamer elements: "
                + ", ".join(missing)
                + ". Install gstreamer1.0-rtsp, gstreamer1.0-plugins-good, "
                "and gstreamer1.0-plugins-ugly."
            )

    def _pipeline_description(self) -> str:
        return (
            "appsrc name=jpeg_source is-live=true format=time do-timestamp=true "
            "block=false max-buffers=2 leaky-type=downstream "
            f"caps=image/jpeg,framerate={self._fps}/1 "
            "! queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            "leaky=downstream "
            "! jpegdec "
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! x264enc tune=zerolatency speed-preset=veryfast "
            f"bitrate={self._bitrate_kbps} key-int-max={self._keyframe_interval} "
            "bframes=0 byte-stream=true aud=true "
            "! video/x-h264,profile=baseline "
            "! h264parse config-interval=-1 "
            f"! rtspclientsink protocols=tcp location={_gst_quote(self._rtsp_url)}"
        )

    def _start_pipeline(self) -> None:
        if self._pipeline is not None:
            return
        try:
            pipeline = Gst.parse_launch(self._pipeline_description())
            appsrc = pipeline.get_by_name("jpeg_source")
            if appsrc is None:
                raise RuntimeError("GStreamer appsrc was not created")
            result = pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                pipeline.set_state(Gst.State.NULL)
                raise RuntimeError("GStreamer pipeline failed to enter PLAYING state")
            self._pipeline = pipeline
            self._appsrc = appsrc
            self._restart_at = 0.0
            self.get_logger().info(f"RTSP publisher starting: {self._rtsp_url}")
        except Exception as exc:
            self.get_logger().error(f"Cannot start RTSP publisher: {exc}")
            self._pipeline = None
            self._appsrc = None
            self._restart_at = time.monotonic() + self._restart_delay_sec

    def _stop_pipeline(self) -> None:
        pipeline = self._pipeline
        self._appsrc = None
        self._pipeline = None
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)

    def _schedule_restart(self, reason: str) -> None:
        self.get_logger().warning(
            f"RTSP publisher stopped ({reason}); retrying in "
            f"{self._restart_delay_sec:.1f}s"
        )
        self._stop_pipeline()
        self._restart_at = time.monotonic() + self._restart_delay_sec

    def _on_image(self, message: CompressedImage) -> None:
        self._received_frames += 1
        image_format = message.format.lower()
        if "jpeg" not in image_format and "jpg" not in image_format and "mjpg" not in image_format:
            self._rejected_frames += 1
            now = time.monotonic()
            if now - self._last_bad_format_log_at >= 5.0:
                self.get_logger().warning(
                    f"Ignoring non-JPEG CompressedImage format: {message.format!r}"
                )
                self._last_bad_format_log_at = now
            return

        appsrc = self._appsrc
        if appsrc is None or not message.data:
            self._rejected_frames += 1
            return

        payload = bytes(message.data)
        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        buffer.duration = Gst.util_uint64_scale_int(1, Gst.SECOND, self._fps)
        flow = appsrc.emit("push-buffer", buffer)
        if flow == Gst.FlowReturn.OK:
            self._published_frames += 1
        else:
            self._rejected_frames += 1
            self._schedule_restart(f"appsrc returned {flow.value_nick}")

    def _poll_pipeline(self) -> None:
        now = time.monotonic()
        pipeline = self._pipeline
        if pipeline is None:
            if now >= self._restart_at:
                self._start_pipeline()
        else:
            bus = pipeline.get_bus()
            while True:
                message = bus.pop_filtered(
                    Gst.MessageType.ERROR | Gst.MessageType.EOS | Gst.MessageType.WARNING
                )
                if message is None:
                    break
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = f"{error.message}; {debug}" if debug else error.message
                    self._schedule_restart(detail)
                    break
                if message.type == Gst.MessageType.EOS:
                    self._schedule_restart("unexpected end of stream")
                    break
                warning, debug = message.parse_warning()
                detail = f"{warning.message}; {debug}" if debug else warning.message
                self.get_logger().warning(f"GStreamer warning: {detail}")

        if now - self._last_stats_at >= self._stats_interval_sec:
            elapsed = now - self._last_stats_at
            received_rate = (self._received_frames - self._last_received_frames) / elapsed
            published_rate = (self._published_frames - self._last_published_frames) / elapsed
            self.get_logger().info(
                f"frames: ROS {received_rate:.1f} fps, RTSP {published_rate:.1f} fps, "
                f"rejected total {self._rejected_frames}"
            )
            self._last_stats_at = now
            self._last_received_frames = self._received_frames
            self._last_published_frames = self._published_frames

    def close(self) -> None:
        self._stop_pipeline()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CompressedImageRtspBridge] = None
    try:
        node = CompressedImageRtspBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
