#!/usr/bin/env python3
"""Compare Ultralytics and direct OpenVINO detections frame by frame."""

from __future__ import annotations

import argparse
import json
import sys
from time import monotonic
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator

import cv2
import numpy as np

from cputest.ov_person_detector import Detection, OpenVINOPersonDetector


DEFAULT_TOPICS = {
    '/drive/camera/color/image_raw/compressed': 'drive',
    '/arm/camera/color/image_raw/compressed': 'arm',
    '/side/left/image_raw/compressed': 'left',
    '/side/right/image_raw/compressed': 'right',
}
IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


@dataclass
class Metrics:
    """Accumulate matching quality for a camera or the full run."""

    frames: int = 0
    count_matches: int = 0
    ious: list[float] = field(default_factory=list)
    confidence_differences: list[float] = field(default_factory=list)
    unmatched_boxes: int = 0

    def summary(self) -> dict[str, float | int]:
        count_rate = self.count_matches / self.frames if self.frames else 0.0
        no_boxes_and_no_mismatches = (
            self.frames > 0 and not self.ious and self.unmatched_boxes == 0
        )
        return {
            'frames': self.frames,
            'box_count_match_rate': count_rate,
            'iou_mean': (
                float(np.mean(self.ious)) if self.ious
                else 1.0 if no_boxes_and_no_mismatches else 0.0
            ),
            'iou_min': (
                float(np.min(self.ious)) if self.ious
                else 1.0 if no_boxes_and_no_mismatches else 0.0
            ),
            'confidence_diff_mean': (
                float(np.mean(self.confidence_differences))
                if self.confidence_differences else 0.0
            ),
            'confidence_diff_max': (
                float(np.max(self.confidence_differences))
                if self.confidence_differences else 0.0
            ),
            'matched_boxes': len(self.ious),
            'unmatched_boxes': self.unmatched_boxes,
        }


class PTDetector:
    """Local Ultralytics adapter using the original node's predict arguments."""

    def __init__(self, path: Path, confidence: float, image_size: int) -> None:
        from ultralytics import YOLO

        if not path.is_file():
            raise FileNotFoundError(f'PT model not found: {path}')
        self.model = YOLO(str(path), task='detect')
        self.confidence = confidence
        self.image_size = image_size

    def predict(self, image: np.ndarray) -> list[Detection]:
        detections: list[Detection] = []
        results = self.model.predict(
            source=image,
            conf=self.confidence,
            imgsz=self.image_size,
            device='cpu',
            verbose=False,
        )
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = str(result.names.get(class_id, f'class_{class_id}'))
                if label.lower() not in {'person', 'pedestrian'}:
                    continue
                detections.append(
                    Detection(
                        xyxy=tuple(float(value) for value in box.xyxy[0].tolist()),
                        confidence=float(box.conf[0]),
                        class_id=class_id,
                    )
                )
        return detections


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--bag', type=Path, help='rosbag2 directory')
    inputs.add_argument('--images', type=Path, help='image directory')
    inputs.add_argument(
        '--live', action='store_true', help='capture directly from live ROS topics'
    )
    parser.add_argument('--pt', type=Path, required=True, help='source PT model')
    parser.add_argument('--ov', type=Path, required=True, help='OpenVINO folder or XML')
    parser.add_argument('--conf', type=float, default=0.5)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--max-frames', type=int, default=0, help='0 means all frames')
    parser.add_argument(
        '--capture-timeout',
        type=float,
        default=30.0,
        help='seconds allowed for --live capture',
    )
    parser.add_argument('--out', type=Path, default=Path('backend_comparison'))
    parser.add_argument('--drive-topic', default='/drive/camera/color/image_raw/compressed')
    parser.add_argument('--arm-topic', default='/arm/camera/color/image_raw/compressed')
    parser.add_argument('--left-topic', default='/side/left/image_raw/compressed')
    parser.add_argument('--right-topic', default='/side/right/image_raw/compressed')
    return parser.parse_args()


def image_frames(folder: Path) -> Iterator[tuple[str, str, np.ndarray]]:
    if not folder.is_dir():
        raise FileNotFoundError(f'Image folder not found: {folder}')
    paths = sorted(path for path in folder.rglob('*') if path.suffix.lower() in IMAGE_SUFFIXES)
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            print(f'warning: cannot decode {path}', file=sys.stderr)
            continue
        camera = path.parent.name.lower()
        if camera not in {'drive', 'arm', 'left', 'right'}:
            camera = 'drive'
        yield camera, path.stem, image


def bag_frames(
    bag: Path, topic_map: dict[str, str]
) -> Iterator[tuple[str, str, np.ndarray]]:
    """Read selected CompressedImage topics without starting a ROS node."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage
    except ImportError as exc:
        raise RuntimeError('rosbag2_py, rclpy and sensor_msgs are required for --bag.') from exc
    if not bag.exists():
        raise FileNotFoundError(f'rosbag2 path not found: {bag}')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = [topic for topic in topic_map if available.get(topic) == 'sensor_msgs/msg/CompressedImage']
    if not selected:
        raise ValueError(f'No requested CompressedImage topics found. Available: {sorted(available)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    counters: defaultdict[str, int] = defaultdict(int)
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        message = deserialize_message(serialized, CompressedImage)
        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            print(f'warning: cannot decode {topic} at {timestamp}', file=sys.stderr)
            continue
        camera = topic_map[topic]
        counters[camera] += 1
        yield camera, f'{timestamp}_{counters[camera]:06d}', image


def live_frames(
    topic_map: dict[str, str], max_frames: int, timeout_s: float
) -> Iterator[tuple[str, str, np.ndarray]]:
    """Capture compressed frames from live ROS topics before comparison."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
    except ImportError as exc:
        raise RuntimeError('rclpy and sensor_msgs are required for --live.') from exc
    if max_frames <= 0:
        raise ValueError('--live requires --max-frames greater than zero.')
    if timeout_s <= 0.0:
        raise ValueError('--capture-timeout must be greater than zero.')

    captured: list[tuple[str, str, bytes]] = []
    counters: defaultdict[str, int] = defaultdict(int)
    rclpy.init(args=[])
    node = Node('compare_backends_live_capture')
    subscriptions = []

    def callback(message: CompressedImage, camera: str) -> None:
        if len(captured) >= max_frames:
            return
        counters[camera] += 1
        stamp = f'{message.header.stamp.sec}_{message.header.stamp.nanosec:09d}'
        captured.append((camera, f'{stamp}_{counters[camera]:06d}', bytes(message.data)))

    try:
        for topic, camera in topic_map.items():
            subscriptions.append(
                node.create_subscription(
                    CompressedImage,
                    topic,
                    lambda message, name=camera: callback(message, name),
                    qos_profile_sensor_data,
                )
            )
        print(
            f'Capturing {max_frames} live frames from {len(topic_map)} topics '
            f'(timeout {timeout_s:.1f}s)...',
            flush=True,
        )
        deadline = monotonic() + timeout_s
        while len(captured) < max_frames and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not captured:
        raise RuntimeError(
            'No live frames received. Check camera nodes, topic names, and QoS.'
        )
    print(f'Captured {len(captured)} frames; starting backend comparison.', flush=True)
    for camera, frame_name, compressed in captured:
        image = cv2.imdecode(np.frombuffer(compressed, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            print(f'warning: cannot decode live frame {camera}/{frame_name}', file=sys.stderr)
            continue
        yield camera, frame_name, image


def box_iou(first: Detection, second: Detection) -> float:
    ax1, ay1, ax2, ay2 = first.xyxy
    bx1, by1, bx2, by2 = second.xyxy
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def greedy_matches(
    first: list[Detection], second: list[Detection]
) -> list[tuple[int, int, float]]:
    candidates = sorted(
        (
            (box_iou(a, b), first_index, second_index)
            for first_index, a in enumerate(first)
            for second_index, b in enumerate(second)
        ),
        reverse=True,
    )
    used_first: set[int] = set()
    used_second: set[int] = set()
    matches: list[tuple[int, int, float]] = []
    for iou, first_index, second_index in candidates:
        if first_index in used_first or second_index in used_second:
            continue
        used_first.add(first_index)
        used_second.add(second_index)
        matches.append((first_index, second_index, iou))
    return matches


def update_metrics(
    metrics: Iterable[Metrics], first: list[Detection], second: list[Detection]
) -> bool:
    matches = greedy_matches(first, second)
    count_match = len(first) == len(second)
    for item in metrics:
        item.frames += 1
        item.count_matches += int(count_match)
        item.unmatched_boxes += len(first) + len(second) - 2 * len(matches)
        for first_index, second_index, iou in matches:
            item.ious.append(iou)
            item.confidence_differences.append(
                abs(first[first_index].confidence - second[second_index].confidence)
            )
    return count_match and all(iou >= 0.99 for _, _, iou in matches)


def draw_mismatch(
    output: Path,
    image: np.ndarray,
    pt_detections: list[Detection],
    ov_detections: list[Detection],
) -> None:
    canvas = image.copy()
    for prefix, detections, color in (
        ('PT', pt_detections, (255, 0, 0)),
        ('OV', ov_detections, (0, 0, 255)),
    ):
        for detection in detections:
            x1, y1, x2, y2 = (int(round(value)) for value in detection.xyxy)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                canvas,
                f'{prefix} {detection.confidence:.3f}',
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )
    cv2.imwrite(str(output), canvas)


def print_table(summaries: dict[str, dict[str, float | int]]) -> None:
    print('camera | frames | count match | IoU mean | IoU min | conf mean | conf max | unmatched')
    for camera, row in summaries.items():
        print(
            f'{camera:<6} | {row["frames"]:6d} | {100.0 * row["box_count_match_rate"]:10.2f}% | '
            f'{row["iou_mean"]:8.5f} | {row["iou_min"]:7.5f} | '
            f'{row["confidence_diff_mean"]:9.6f} | {row["confidence_diff_max"]:8.6f} | '
            f'{row["unmatched_boxes"]:9d}'
        )


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.conf <= 1.0:
        raise ValueError('--conf must be between 0 and 1.')
    args.out.mkdir(parents=True, exist_ok=True)
    pt = PTDetector(args.pt.expanduser().resolve(), args.conf, args.imgsz)
    ov_detector = OpenVINOPersonDetector(
        args.ov,
        confidence_threshold=args.conf,
        infer_size=args.imgsz,
        device='CPU',
        perf_hint='LATENCY',
        num_requests=1,
    )
    if args.images:
        frames = image_frames(args.images.expanduser().resolve())
    else:
        topic_map = {
            args.drive_topic: 'drive',
            args.arm_topic: 'arm',
            args.left_topic: 'left',
            args.right_topic: 'right',
        }
        if args.live:
            frames = live_frames(topic_map, args.max_frames, args.capture_timeout)
        else:
            frames = bag_frames(args.bag.expanduser().resolve(), topic_map)

    total = Metrics()
    by_camera: defaultdict[str, Metrics] = defaultdict(Metrics)
    mismatch_count = 0
    processed = 0
    for camera, frame_name, image in frames:
        if args.max_frames > 0 and processed >= args.max_frames:
            break
        pt_detections = pt.predict(image)
        ov_detections = ov_detector.predict(image)
        agrees = update_metrics(
            (total, by_camera[camera]), pt_detections, ov_detections
        )
        if not agrees and mismatch_count < 50:
            safe_name = ''.join(ch if ch.isalnum() or ch in '-_' else '_' for ch in frame_name)
            draw_mismatch(
                args.out / f'mismatch_{mismatch_count + 1:03d}_{camera}_{safe_name}.jpg',
                image,
                pt_detections,
                ov_detections,
            )
            mismatch_count += 1
        processed += 1

    summaries = {
        camera: by_camera[camera].summary() for camera in sorted(by_camera)
    }
    summaries['total'] = total.summary()
    total_summary = summaries['total']
    passed = (
        total_summary['frames'] > 0
        and total_summary['box_count_match_rate'] >= 0.99
        and total_summary['iou_mean'] >= 0.99
    )
    report = {
        'result': 'PASS' if passed else 'FAIL',
        'criteria': {'box_count_match_rate': 0.99, 'iou_mean': 0.99},
        'openvino_precision': ov_detector.precision,
        'pt_model': str(args.pt.expanduser().resolve()),
        'openvino_model': str(ov_detector.model_path),
        'saved_mismatches': mismatch_count,
        'cameras': summaries,
    }
    (args.out / 'summary.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print_table(summaries)
    print(f'OpenVINO precision: {ov_detector.precision}')
    print(f'{report["result"]}: count match >=99% and mean IoU >=0.99')
    print(f'Summary: {args.out / "summary.json"}')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
