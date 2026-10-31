"""Pure tool-2 sequencing and calibration checks, independent of ROS."""
import math
from tool_ids import SUPPLY_BOX, tool_label

STAGES = ('approach', 'grasp', 'return_path', 'ready', 'person',
          'align', 'preplace', 'descend', 'release', 'complete')


class PickPlaceSequence:
    def __init__(self):
        self.stage = 'approach'

    def complete(self, expected):
        if expected != self.stage or self.stage in ('complete', 'failed'):
            raise ValueError('unexpected or duplicate stage completion')
        self.stage = STAGES[STAGES.index(self.stage)+1]
        return self.stage

    def fail(self):
        self.stage = 'failed'


def checked_config(data):
    """Do not invent gripper calibration, floor frame or placement geometry."""
    if data.get('enabled') is not True:
        raise ValueError(f'{tool_label(SUPPLY_BOX)} pick/place disabled; measure and enable its configuration first')
    for key in ('floor_frame', 'person_ground_frame', 'person_topic', 'person_camera_info_topic'):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f'missing {key}')
    ranges = dict(close_rad=(-2.351592547829, 2.839398438376),
                  open_rad=(-2.351592547829, 2.839398438376),
                  ee_speed_rad_s=(0., .12),
                  preplace_height_m=(0., .3), person_clearance_m=(0., 5.),
                  max_detection_age_s=(0., .5), min_confidence=(0., 1.),
                  stable_angle_rad=(0., .1), acquire_timeout_s=(0., 30.),
                  sequence_timeout_s=(0., 1800.), linear_speed_m_s=(0., .02))
    mode = data.get('placement_mode')
    if mode == 'radius':
        ranges['place_radius_m'] = (0., 1.)
    elif mode == 'chassis_edge':
        if not isinstance(data.get('chassis_frame'), str) or not data['chassis_frame']:
            raise ValueError('chassis_frame required')
        clearance = data.get('edge_clearance_m')
        if type(clearance) not in (int, float) or not math.isfinite(clearance) or not 0 <= clearance <= .3:
            raise ValueError('edge_clearance_m must be in [0, 0.3]')
    else:
        raise ValueError('placement_mode must be radius or chassis_edge')
    for key, (low, high) in ranges.items():
        value = data.get(key)
        if type(value) not in (float, int) or not math.isfinite(value) or not low < value <= high:
            raise ValueError(f'{key} must be measured/configured in ({low}, {high}]')
    if data['close_rad'] == data['open_rad']:
        raise ValueError('open and close targets must differ')
    if type(data.get('stable_samples')) is not int or not 3 <= data['stable_samples'] <= 100:
        raise ValueError('stable_samples must be 3..100')
    for key, size in (('tcp_to_object_bottom_m', 3), ('place_orientation_xyzw', 4)):
        value = data.get(key)
        if (not isinstance(value, list) or len(value) != size
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
            raise ValueError(f'{key} requires {size} measured finite values')
    if sum(x*x for x in data['place_orientation_xyzw']) < 1e-12:
        raise ValueError('place orientation quaternion must be nonzero')
    return dict(data)
