"""Read-only camera/TF preflight. TF existence does not certify physical calibration."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from visual_landmark.calibration import camera_model
from visual_landmark.visual_localization_node import tf_matrix
from visual_landmark.localization import valid_transform, pose_fields


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--output', required=True)
    options, ros_args = parser.parse_known_args(args)
    if not 0 < options.seconds <= 60:
        parser.error('seconds must be in (0,60]')
    rclpy.init(args=ros_args)
    node = Node('vpr_preflight')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {}
    topics = dict(rear_rgb=('/rear/camera/image_raw/compressed', CompressedImage),
                  rear_info=('/rear/camera/camera_info', CameraInfo),
                  arm_rgb=('/arm/camera/color/image_raw/compressed', CompressedImage),
                  arm_info=('/arm/camera/color/camera_info', CameraInfo),
                  arm_depth=('/arm/camera/aligned_depth_to_color/image_raw', Image))
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, key=key: samples.__setitem__(key, msg), qos_profile_sensor_data)
        for key, (topic, kind) in topics.items()]
    result = dict(ready_for_observation=False, checks={}, transforms={},
                  extrinsic_status='TF lookup only; physical mounting/optical calibration not certified',
                  correction_enabled=None)
    try:
        end = time.monotonic()+options.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        for key, (topic, _) in topics.items():
            msg = samples.get(key)
            if msg is None:
                result['checks'][key] = dict(ok=False, topic=topic, reason='no messages')
                continue
            age = (node.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
            result['checks'][key] = dict(ok=0 <= age <= .5, topic=topic,
                                         frame=msg.header.frame_id, age_s=age)
        for camera in ['rear', 'arm']:
            try:
                rgb, ci = samples[camera+'_rgb'], samples[camera+'_info']
                image = cv2.imdecode(np.frombuffer(rgb.data, np.uint8), cv2.IMREAD_GRAYSCALE)
                if image is None or ci.header.frame_id != rgb.header.frame_id:
                    raise ValueError('decode/frame mismatch')
                camera_model(ci.width, ci.height, ci.k, ci.d, ci.distortion_model, image.shape)
                result['checks'][camera+'_calibration'] = dict(ok=True, resolution=[ci.width,ci.height],
                                                               model=ci.distortion_model, k=list(map(float, ci.k)), d=list(map(float, ci.d)))
                for parent in ['base_link', 'odom']:
                    tf = tf_matrix(buffer.lookup_transform(parent, rgb.header.frame_id,
                                                           Time.from_msg(rgb.header.stamp)))
                    if not valid_transform(tf):
                        raise ValueError('invalid transform')
                    result['transforms'][parent+'<-'+rgb.header.frame_id] = dict(matrix=tf.tolist(), **pose_fields(tf))
                result['checks'][camera+'_tf'] = dict(ok=True)
            except Exception as exc:
                result['checks'][camera+'_geometry'] = dict(ok=False, reason=str(exc))
        try:
            depth, color, ci = samples['arm_depth'], samples['arm_rgb'], samples['arm_info']
            if (depth.header.frame_id != color.header.frame_id or depth.encoding not in {'16UC1','32FC1'}
                    or (depth.width,depth.height)!=(ci.width,ci.height)):
                raise ValueError('depth alignment frame/encoding/size mismatch')
            skew = abs((Time.from_msg(depth.header.stamp)-Time.from_msg(color.header.stamp)).nanoseconds*1e-9)
            result['checks']['aligned_depth'] = dict(ok=skew <= .04, latest_sample_skew_s=skew,
                note='latest-sample check, not a measurement of synchronizer pairing success or depth accuracy')
        except Exception as exc:
            result['checks']['aligned_depth'] = dict(ok=False, reason=str(exc))
        result['rear_image_intrinsic_ready'] = all(result['checks'].get(k, {}).get('ok', False) for k in ['rear_rgb','rear_info','rear_calibration'])
        result['ready_for_observation'] = all(check['ok'] for check in result['checks'].values())
        Path(options.output).write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
