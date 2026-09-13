#!/usr/bin/env python3
"""
compare_ekf_vs_direct.py

같은 rosbag(mcap/db3)에서 EKF(/odometry/filtered)와 direct_odom(/direct_odom)
두 nav_msgs/Odometry 궤적을 읽어 같은 그래프에 overlay해서 비교하는
오프라인 분석 스크립트. ROS 노드가 아니라 python3로 직접 실행하는 도구.

전제: 두 topic이 이미 같은 bag에 기록돼 있어야 한다. direct_odom_node는
robot_localization을 fork/수정하지 않고 별도 topic(/direct_odom)에
발행하므로, reduced_odom_bringup.launch.py + direct_odom.launch.py를 동시에 띄운 채로
재생/실주행한 bag이면 두 topic이 함께 들어있다.

사용법:
    python3 compare_ekf_vs_direct.py <bag_dir> \\
        [--ekf-topic /odometry/filtered] [--direct-topic /direct_odom] \\
        [--out compare.png]
"""
import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message


def yaw_from_quat(q):
    # REP-103, ZYX 오일러의 yaw만 필요.
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_odom_topic(bag_dir, topic_name):
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in topic_types:
        raise SystemExit(
            f"'{topic_name}'이 bag에 없습니다. bag에 있는 topic: "
            f"{sorted(topic_types.keys())}")
    msg_type = get_message(topic_types[topic_name])

    t0 = None
    out = {'t': [], 'x': [], 'y': [], 'z': [], 'yaw': []}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        hdr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t = hdr_ns * 1e-9
        if t0 is None:
            t0 = t
        out['t'].append(t - t0)
        p = msg.pose.pose.position
        out['x'].append(p.x)
        out['y'].append(p.y)
        out['z'].append(p.z)
        out['yaw'].append(yaw_from_quat(msg.pose.pose.orientation))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--ekf-topic', default='/odometry/filtered')
    ap.add_argument('--direct-topic', default='/direct_odom')
    ap.add_argument('--out', default='compare.png')
    args = ap.parse_args()

    ekf = read_odom_topic(args.bag_dir, args.ekf_topic)
    direct = read_odom_topic(args.bag_dir, args.direct_topic)
    print(f'EKF({args.ekf_topic}): {len(ekf["t"])} msgs, '
          f'direct({args.direct_topic}): {len(direct["t"])} msgs')

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    ax = axes[0][0]
    ax.plot(ekf['x'], ekf['y'], label='EKF', alpha=0.8)
    ax.plot(direct['x'], direct['y'], label='direct_odom', alpha=0.8)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('XY trajectory')
    ax.axis('equal'); ax.legend(); ax.grid(True)

    for ax, key, title in [
        (axes[0][1], 'x', 'x vs t'),
        (axes[0][2], 'y', 'y vs t'),
        (axes[1][0], 'z', 'z vs t'),
        (axes[1][1], 'yaw', 'yaw vs t'),
    ]:
        ax.plot(ekf['t'], ekf[key], label='EKF', alpha=0.8)
        ax.plot(direct['t'], direct[key], label='direct_odom', alpha=0.8)
        ax.set_xlabel('t [s]'); ax.set_ylabel(key); ax.set_title(title)
        ax.legend(); ax.grid(True)

    axes[1][2].axis('off')

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f'saved: {args.out}')


if __name__ == '__main__':
    main()
