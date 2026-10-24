"""ROS camera_info YAML parsing with strict supported-model/size validation."""
from pathlib import Path
import numpy as np
import yaml


def camera_model(width, height, k, d, model, image_shape=None):
    k, d = np.asarray(k, dtype=float).reshape(3, 3), np.asarray(d, dtype=float).ravel()
    if model not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('supported distortion models: plumb_bob, rational_polynomial; fisheye/equidistant unsupported')
    if min(width, height) <= 0 or (image_shape is not None and tuple(image_shape[:2]) != (height, width)):
        raise ValueError('image/calibration resolution mismatch')
    if not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('invalid or missing K/D')
    if not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-6:
        raise ValueError('unsupported camera matrix')
    if (model == 'plumb_bob' and len(d) not in (4, 5)) or (model == 'rational_polynomial' and len(d) not in (8, 12, 14)):
        raise ValueError('distortion coefficient count inconsistent with model')
    return k, d


def load_calibration(path):
    data = yaml.safe_load(Path(path).read_text())
    width, height = int(data['image_width']), int(data['image_height'])
    k, d = camera_model(width, height, data['camera_matrix']['data'],
                        data['distortion_coefficients']['data'], data['distortion_model'])
    return data, k, d
