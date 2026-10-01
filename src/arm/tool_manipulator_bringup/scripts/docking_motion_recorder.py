#!/usr/bin/env python3
"""Read-only recorder for taught docking motion from /joint_states.

Commands: start <segment>, mark <label>, stop, q.
This node subscribes only; it never publishes motion commands or opens hardware.
"""
from __future__ import annotations

import argparse
import json
import threading
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = (
    'base_joint', 'shoulder_joint', 'elbow_joint',
    'wrist_pitch_joint', 'wrist_roll_joint', 'wrist_yaw_joint',
)


class DockingMotionRecorder(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('docking_motion_recorder')
        self._lock = threading.Lock()
        self._recording = False
        self._segment: str | None = None
        self._latest_sample: dict | None = None
        self._samples: list[dict] = []
        self._events: list[dict] = []
        self.create_subscription(JointState, topic, self._joint_state_cb, 50)
        self.topic = topic

    def _joint_state_cb(self, message: JointState) -> None:
        stamp = {
            'sec': int(message.header.stamp.sec),
            'nanosec': int(message.header.stamp.nanosec),
        }
        positions = dict(zip(message.name, map(float, message.position)))
        velocities = dict(zip(message.name, map(float, message.velocity)))
        efforts = dict(zip(message.name, map(float, message.effort)))
        with self._lock:
            sample = {
                'stamp': stamp,
                'frame_id': message.header.frame_id,
                'positions_rad': positions,
                'velocities_rad_s': velocities,
                'efforts': efforts,
            }
            self._latest_sample = sample
            if self._recording:
                self._samples.append({**sample, 'segment': self._segment})

    def start_segment(self, name: str) -> None:
        with self._lock:
            self._segment = name
            self._recording = True
            self._events.append(self._event('start', name))
        print(f'Recording segment: {name}')

    def mark(self, label: str) -> None:
        with self._lock:
            self._events.append(self._event('mark', label))
        print(f'Marked: {label}')

    def stop_segment(self) -> None:
        with self._lock:
            was_recording = self._recording
            segment = self._segment
            self._recording = False
            self._events.append(self._event('stop', segment))
            count = sum(sample['segment'] == segment for sample in self._samples)
        if was_recording:
            print(f'Stopped segment {segment}: {count} samples')
        else:
            print('Recorder is already stopped.')

    def snapshot(self) -> dict:
        with self._lock:
            taught_path = []
            for event in self._events:
                positions = event.get('positions_rad')
                if event.get('event') != 'mark' or not isinstance(positions, dict):
                    continue
                if not all(joint in positions for joint in ARM_JOINTS):
                    continue
                taught_path.append({
                    'name': event['label'],
                    'positions_rad': [float(positions[joint]) for joint in ARM_JOINTS],
                })
            return {
                'format': 'tool_manipulator_docking_motion/v1',
                'recorded_at_utc': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
                'joint_state_topic': self.topic,
                'samples': list(self._samples),
                'events': list(self._events),
                'joint_names': list(ARM_JOINTS),
                'taught_joint_path': taught_path,
            }

    def save(self, output: Path) -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(self.snapshot(), indent=2, ensure_ascii=False, allow_nan=False) + '\n',
            encoding='utf-8',
        )
        print(f'Saved {output}')
        taught_path = self.snapshot()['taught_joint_path']
        if taught_path:
            print('\nPaste this under tools.1.lock.joint_path in tools.yaml:')
            print(yaml.safe_dump({'joint_path': taught_path}, sort_keys=False).rstrip())

    def _event(self, kind: str, label: str | None) -> dict:
        latest = self._latest_sample or {}
        return {
            'event': kind,
            'label': label,
            'sample_index': len(self._samples),
            'stamp': latest.get('stamp'),
            'positions_rad': latest.get('positions_rad'),
        }


class DockingLogicV2:
    """Reserved for the second docking-logic proposal; not wired to hardware."""

    def on_recorded_motion(self, recording: dict) -> None:
        # TODO(v2): define how recorded encoder samples become a docking plan.
        raise NotImplementedError('Docking logic v2 has not been specified.')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Record taught docking motion from JointState.')
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--output', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output or Path.cwd() / (
        'docking_motion_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '.json'
    )
    rclpy.init()
    node = DockingMotionRecorder(args.topic)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print('Read-only encoder-calibrated JointState recorder.')
    print('For drill: start drill_lock | mark unlock | mark lock_step1 | mark lock | stop | q')
    try:
        while rclpy.ok():
            try:
                words = input('motion> ').strip().split(maxsplit=1)
            except EOFError:
                break
            if not words:
                continue
            if words[0] in {'q', 'quit', 'exit'}:
                break
            if words[0] == 'start' and len(words) == 2:
                node.start_segment(words[1])
            elif words[0] == 'mark' and len(words) == 2:
                node.mark(words[1])
            elif words[0] == 'stop' and len(words) == 1:
                node.stop_segment()
            else:
                print('Invalid command. Use start <segment>, mark <label>, stop, or q.')
    finally:
        node.stop_segment()
        node.save(output)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
