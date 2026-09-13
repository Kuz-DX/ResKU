#!/usr/bin/env python3
"""extract_track_from_bag.py -- [테스트용] manual 주행 중 녹화한 bag(/path,
/tf, /tf_static)에서 실제 트랙 모양을 odom 고정 좌표계 폴리라인으로
재구성해서 JSON으로 저장.

왜 이 스크립트가 필요한가 (bag을 그냥 /path만 재생하면 안 되는 이유):
    /path는 camera_link 기준(로봇 상대좌표)이라, 그 자체로는 "트랙이 월드
    어디에 있는지"를 담고 있지 않다 -- 매 순간 "로봇 바로 앞이 이렇게
    생겼다"만 말해준다. path_relay_node는 이걸 odom으로 바꿀 때 그 메시지의
    timestamp 시점 TF를 찾아서 변환하는데, bag 속 /path의 timestamp는
    manual 주행 때의 과거 시각이라 재생 시점의 라이브 TF 버퍼엔 그 시각이
    아예 없다 -- TF lookup이 실패하고 path_relay_node는 그대로 정지한다.
    설령 timestamp를 지금 시각으로 새로 찍어도, 그 순간 로봇의 실제 위치는
    (자율주행 속도 프로파일이 manual과 다르므로) 원래 그 /path가 찍힌
    시점의 로봇 위치와 다를 수밖에 없어 "로봇 기준 앞"이라는 의미 자체가
    깨진다.

    그래서 이 스크립트는 bag 안에서 /path 각 메시지를 "그 메시지 자신의
    시점" 기준으로 odom 좌표계에 미리 다 변환해서 하나의 고정된 트랙
    폴리라인으로 합쳐둔다. 이렇게 만든 결과를 bag_track_path_publisher.py가
    (synthetic_track_loop_path.py와 동일한 패턴으로) 로봇의 "현재" 위치에
    재-앵커링해서 라이브로 발행하면, 로봇 상대좌표 문제 자체가 없어진다.

TF lookup 정밀도에 대해: 각 /path 메시지를 변환할 때 그 메시지의 정확한
timestamp에 맞는 TF(interpolation)를 찾는 대신 그 시점까지 먹인 것 중
"가장 최신" TF를 쓴다(rclpy.time.Time() = latest 관례). 트랙 재구성이
목적이라 이 정도 시간 오차(길어야 한두 TF 주기)는 무시할 수준이고, 대신
bracket 데이터(앞뒤로 둘러싼 TF) 부족으로 인한 extrapolation 에러를
원천적으로 피할 수 있어 훨씬 단순하고 안정적이다.

겹침 제거: /path는 15Hz로 매번 "로봇 바로 앞 몇 m" 구간 전체(포즈 여러 개)를
다시 보낸다. 메시지 하나의 포즈를 전부 쓰면 연속된 메시지끼리 lookahead
구간이 크게 겹쳐서, 이어붙였을 때 지그재그가 쌓여 총 길이가 실제보다
훨씬 부풀어 오른다(실측: 28초 bag이 1.3km로 계산되는 버그를 실제로 겪고
고침). 그래서 메시지당 "로봇에 가장 가까운 점(poses[0])" 딱 하나만
breadcrumb으로 쓴다 -- 로봇이 실제로 전진한 궤적만 자연스럽게 단조 증가로
쌓이고, lookahead 겹침 문제 자체가 없어진다. 그 뒤 시간순으로 훑으면서
직전에 남긴 breadcrumb과 min_spacing_m 이상 떨어진 것만 남겨 정지/저속
구간의 중복도 정리한다.

사용:
    python3 extract_track_from_bag.py manual_run_01
    python3 extract_track_from_bag.py manual_run_01 -o manual_run_01_track.json --min-spacing-m 0.1
"""
import argparse
import json
import math

from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer


def tangent_yaws(points):
    """연속된 (x,y) 점들의 atan2(dy,dx)로 각 점의 접선 방향을 근사한다.
    synthetic_track_loop_path.py의 동일 함수와 같은 패턴(독립 스크립트라
    공용 모듈 분리는 하지 않음)."""
    n = len(points)
    if n <= 1:
        return [0.0] * n
    yaws = []
    for i in range(n - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        yaws.append(math.atan2(dy, dx))
    yaws.append(yaws[-1])
    return yaws


def _transform_point(x, y, transform) -> tuple:
    """geometry_msgs/TransformStamped로 (x,y)를 변환. tf2_geometry_msgs의
    do_transform_pose API 불확실성을 피하려고 쿼터니언에서 yaw만 뽑아
    수동으로 회전+평행이동한다(synthetic_track_loop_path.py의 _rotate()와
    동일 원리, 로봇은 2D 평면 위에서만 움직이므로 z/roll/pitch 무시해도
    무방)."""
    q = transform.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    return (tx + x * c - y * s, ty + x * s + y * c)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_path', help='ros2 bag record로 만든 bag 디렉토리')
    parser.add_argument('-o', '--output', default=None,
                         help='출력 json 경로 (기본: <bag_path>_track.json)')
    parser.add_argument('--path-topic', default='/path')
    parser.add_argument('--target-frame', default='odom')
    parser.add_argument('--min-spacing-m', type=float, default=0.1,
                         help='이 거리 이상 떨어진 점만 남김 (중복 제거)')
    args = parser.parse_args()

    output = args.output or (args.bag_path.rstrip('/') + '_track.json')

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_path, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.path_topic not in topic_types:
        raise SystemExit(
            f"bag에 '{args.path_topic}' 토픽이 없음 -- "
            f"'ros2 bag record -o <name> /path /odometry/filtered /tf /tf_static'로 "
            f"녹화했는지 확인")
    # /tf, /tf_static 둘 다 필수는 아님 -- 녹화 방식에 따라 odom->base_link가
    # dynamic(/tf, 실제 EKF 주행)일 수도 static(/tf_static, mock/고정 pose
    # 테스트)일 수도 있다. 있는 것만 버퍼에 먹이고, 트리가 실제로 안 이어지면
    # (아래 lookup_transform 단계에서) 메시지별로 skip 카운트로 드러난다.
    tf_topics = [t for t in ('/tf', '/tf_static') if t in topic_types]
    if not tf_topics:
        raise SystemExit("bag에 /tf, /tf_static 둘 다 없음 -- TF 없이는 재구성 불가")
    needed = {args.path_topic, *tf_topics}

    msg_classes = {t: get_message(topic_types[t]) for t in needed}

    # 아주 넉넉한 cache_time -- bag 전체 길이 내내 과거 TF가 안 지워지게.
    # (기본 10초짜리 캐시로는 그보다 긴 bag에서 앞부분 TF가 밀려나 사라짐)
    buffer = Buffer(cache_time=Duration(seconds=86400))

    raw_points = []
    total_path_msgs = 0
    skipped = 0

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic not in needed:
            continue
        msg = deserialize_message(data, msg_classes[topic])

        if topic in ('/tf', '/tf_static'):
            for tr in msg.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(tr, 'bag_extract')
                else:
                    buffer.set_transform(tr, 'bag_extract')
            continue

        total_path_msgs += 1
        if not msg.poses:
            continue
        try:
            tf = buffer.lookup_transform(args.target_frame, msg.header.frame_id, Time())
        except Exception:
            skipped += 1
            continue

        # 메시지당 로봇에 가장 가까운 점 하나만 (위 겹침 제거 설명 참고).
        nearest_pose = msg.poses[0]
        raw_points.append(
            _transform_point(nearest_pose.pose.position.x, nearest_pose.pose.position.y, tf))

    print(f"/path 메시지 {total_path_msgs}개 중 {skipped}개는 TF 없어서 스킵")
    print(f"변환된 원본 점 {len(raw_points)}개")

    dedup = []
    for x, y in raw_points:
        if not dedup or math.hypot(x - dedup[-1][0], y - dedup[-1][1]) >= args.min_spacing_m:
            dedup.append((x, y))

    if len(dedup) < 2:
        raise SystemExit("재구성된 점이 너무 적음 -- bag에 /path가 제대로 기록됐는지 확인")

    yaws = tangent_yaws(dedup)
    track = [[x, y, yaw] for (x, y), yaw in zip(dedup, yaws)]

    with open(output, 'w') as f:
        json.dump({'frame_id': args.target_frame, 'points': track}, f)

    length_m = sum(
        math.hypot(track[i + 1][0] - track[i][0], track[i + 1][1] - track[i][1])
        for i in range(len(track) - 1))
    print(f"재구성 완료: {len(track)}점, 총 길이 약 {length_m:.1f}m -> {output}")


if __name__ == '__main__':
    main()
