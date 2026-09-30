#!/usr/bin/env python3
"""Interactive, read-only RMD zero/sign/soft-limit teaching helper.

Requires rmd_joint_state_bridge.  It never opens CAN and never sends a motor
command: all values come from the bridge's published raw angle/encoder topics.
"""
from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINTS = ('shoulder_joint', 'elbow_joint', 'wrist_pitch_joint')


class RmdCalibrationCapture(Node):
    def __init__(self):
        super().__init__('capture_rmd_calibration')
        self.lock = threading.Lock()
        self.raw_deg: list[float] | None = None
        self.encoder: list[float] | None = None
        self.calibration = {name: {'sign': None, 'q_offset_rad': None, 'limits': {}} for name in JOINTS}
        self.create_subscription(Float64MultiArray, '/arm/rmd_raw_angle_deg', self._raw_cb, 10)
        self.create_subscription(Float64MultiArray, '/arm/rmd_encoder', self._encoder_cb, 10)

    def _raw_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 3:
            with self.lock:
                self.raw_deg = list(msg.data)

    def _encoder_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 3:
            with self.lock:
                self.encoder = list(msg.data)

    def snapshot(self) -> list[float] | None:
        with self.lock:
            return None if self.raw_deg is None else list(self.raw_deg)

    def print_snapshot(self) -> None:
        raw = self.snapshot()
        if raw is None:
            print('아직 /arm/rmd_raw_angle_deg 수신 전입니다.')
            return
        with self.lock:
            encoder = None if self.encoder is None else list(self.encoder)
        for index, name in enumerate(JOINTS):
            encoder_text = '?' if encoder is None else f'{encoder[index]:.0f}'
            print(f'{name}: raw_deg={raw[index]:.6f}, raw_encoder={encoder_text}')

    def zero(self, name: str, sign: int) -> None:
        raw = self.snapshot()
        if raw is None:
            print('raw data가 없습니다.')
            return
        index = JOINTS.index(name)
        # joint_rad = sign * (raw_rad - q_offset). Desired joint=0 -> q_offset=raw_rad.
        offset = math.radians(raw[index])
        self.calibration[name]['sign'] = sign
        self.calibration[name]['q_offset_rad'] = offset
        print(f'{name}: sign={sign:+d}, q_offset_rad={offset:.12f}  # raw {raw[index]:.6f} deg에서 URDF 0')

    def limit(self, name: str, side: str) -> None:
        raw = self.snapshot()
        item = self.calibration[name]
        if raw is None or item['sign'] is None or item['q_offset_rad'] is None:
            print(f'{name}: 먼저 zero {name} + 또는 zero {name} - 를 실행하세요.')
            return
        index = JOINTS.index(name)
        joint_rad = item['sign'] * (math.radians(raw[index]) - item['q_offset_rad'])
        item['limits'][side] = joint_rad
        print(f'{name}: {side}={joint_rad:.12f} rad ({math.degrees(joint_rad):.3f} deg)')

    def show(self) -> None:
        print('\n# hardware.yaml에 복사 (속도·전류 보호값은 별도 실측 후 입력)')
        for name in JOINTS:
            item = self.calibration[name]
            lo, hi = item['limits'].get('min'), item['limits'].get('max')
            print(f'{name}:')
            print(f"  sign: {item['sign']}")
            print(f"  q_offset_rad: {item['q_offset_rad']}")
            print(f'  soft_limit_rad: [{lo}, {hi}]')


def main() -> None:
    rclpy.init()
    node = RmdCalibrationCapture()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    print('RMD read-only calibration. 먼저 bridge를 실행하세요.')
    print('명령: snap | zero <shoulder|elbow|wrist_pitch> <+|-> | limit <joint> <min|max> | show | q')
    try:
        while rclpy.ok():
            words = input('rmd> ').strip().split()
            if not words:
                continue
            if words[0] in ('q', 'quit', 'exit'):
                break
            if words[0] == 'snap' and len(words) == 1:
                node.print_snapshot()
            elif words[0] == 'zero' and len(words) == 3 and words[1] in ('shoulder', 'elbow', 'wrist_pitch') and words[2] in ('+', '-'):
                node.zero(f'{words[1]}_joint', 1 if words[2] == '+' else -1)
            elif words[0] == 'limit' and len(words) == 3 and words[1] in ('shoulder', 'elbow', 'wrist_pitch') and words[2] in ('min', 'max'):
                node.limit(f'{words[1]}_joint', words[2])
            elif words[0] == 'show' and len(words) == 1:
                node.show()
            else:
                print('잘못된 명령입니다.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
