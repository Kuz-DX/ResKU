#!/usr/bin/env python3
"""Opt-in tool2 pick/place; the mode manager remains the sole motor gateway."""
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from tool_docking_coordinator import ToolDockingCoordinator
from tool_docking_core import transform
from tool_trajectory_joy import ARM_JOINTS, reverse_path, execution_plan
from tool2_sequence import PickPlaceSequence, checked_config
from person_place_geometry import person_on_floor, placement_targets, aligned_base


class Tool2PickPlaceCoordinator(ToolDockingCoordinator):
    def __init__(self):
        super().__init__()
        self.pick_config = checked_config(yaml.safe_load(Path(self.declare_parameter(
            'pick_place_config', '').value).read_text()))
        if self.pick_config['placement_mode'] == 'chassis_edge':
            robot = ET.fromstring(self.get_parameter('robot_description').value)
            link = next((link for link in robot.findall('link')
                         if link.get('name') == self.pick_config['chassis_frame']), None)
            box = None if link is None else link.find('collision/geometry/box')
            if box is None:
                raise ValueError('chassis requires a box collision geometry')
            collision_origin = link.find('collision/origin')
            if collision_origin is not None and any(float(v) != 0. for name in ('xyz', 'rpy')
                    for v in collision_origin.get(name, '0 0 0').split()):
                raise ValueError('chassis collision box must be centred on chassis frame')
            size = np.asarray([float(v) for v in box.get('size').split()])
            if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
                raise ValueError('invalid chassis box size')
            self.pick_config['chassis_half_extents_m'] = (size[:2]/2).tolist()
        ready_file = Path(self.declare_parameter('pick_place_ready_yaml', '').value)
        self.pick_ready = np.asarray(yaml.safe_load(ready_file.read_text())['tcp_ready']['positions_rad'])
        if not self.model.within_limits(self.pick_ready, self.margin):
            raise ValueError('invalid pick/place TCP ready target')
        if self.paths[2].get('after_path') is not None:
            raise ValueError('tool2 after_path conflicts with pick/place coordinator')
        self.person_camera = None
        self.create_subscription(CameraInfo, self.pick_config['person_camera_info_topic'],
                                 self.on_person_camera, qos_profile_sensor_data)
        self.create_subscription(String, self.pick_config['person_topic'],
                                 self.on_person, qos_profile_sensor_data)

    def reset(self):
        super().reset()
        self.pick_sequence = None
        self.person_count = 0
        self.person_stamp = -1
        self.person_last_received = -math.inf
        self.person_sample = None

    def begin(self, path_id, planned_path=None):
        if path_id == 2 and self.pick_sequence is None:
            self.pick_sequence = PickPlaceSequence()
            self.pick_started = time.monotonic()
            # Returning along the taught route requires the complete approach.
            self.use_nearest_entry = False
        super().begin(path_id, planned_path)

    def begin_failed(self, reason):
        if self.pick_sequence is not None:
            self.fail(reason)
        else:
            super().begin_failed(reason)

    def fail(self, reason):
        if self.pick_sequence is not None:
            self.pick_sequence.fail()
            if self.cancel.service_is_ready():
                self.cancel.call_async(Trigger.Request())
        super().fail(reason)

    def on_person_camera(self, msg):
        matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if (np.isfinite(matrix).all() and matrix[0, 0] > 0 and matrix[1, 1] > 0
                and np.isfinite(msg.d).all() and msg.distortion_model in ('plumb_bob', 'rational_polynomial')):
            self.person_camera = (msg.header.frame_id, matrix, np.asarray(msg.d), msg.width, msg.height)
        else:
            self.person_camera = None

    def tf_matrix(self, parent, child, stamp):
        value = self.tf_buffer.lookup_transform(parent, child, stamp,
                                                timeout=Duration(seconds=.05)).transform
        p, q = value.translation, value.rotation
        return transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def on_person(self, msg):
        if self.pick_sequence is None or self.pick_sequence.stage not in ('person', 'align', 'preplace', 'descend', 'release'):
            return
        try:
            payload = json.loads(msg.data)
            stamp = Time(seconds=int(payload['stamp']['sec']), nanoseconds=int(payload['stamp']['nanosec']))
            age = (self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
            if not 0 <= age < self.pick_config['max_detection_age_s'] or stamp.nanoseconds <= self.person_stamp:
                raise ValueError('stale/non-increasing person observation')
            self.person_stamp = stamp.nanoseconds
            if self.person_camera is None:
                raise ValueError('person CameraInfo unavailable')
            frame, matrix, distortion, width, height = self.person_camera
            if payload['frame_id'] != frame or payload['image_size'] != dict(width=width, height=height):
                raise ValueError('person camera frame/size mismatch')
            floor = self.pick_config['floor_frame']
            ground = self.pick_config['person_ground_frame']
            ground_camera = self.tf_matrix(ground, frame, stamp)
            floor_ground = self.tf_matrix(floor, ground, stamp)
            floor_base = self.tf_matrix(floor, self.base_frame, stamp)
            floor_chassis = (self.tf_matrix(floor, self.pick_config['chassis_frame'], stamp)
                             if self.pick_config['placement_mode'] == 'chassis_edge' else None)
            if np.dot(floor_base[:3, 2], [0., 0., 1.]) < .999:
                raise ValueError('floor and base vertical axes must agree')
            if (self.pick_sequence.stage != 'person'
                    and not np.allclose(floor_base, self.pick_floor_base, atol=.002, rtol=0.)):
                raise ValueError('robot base moved after placement target was locked')
            person_ground = person_on_floor(payload, matrix, distortion, ground_camera,
                                            self.pick_config['min_confidence'])
            person = (floor_ground @ np.r_[person_ground, 1.])[:3]
            direction = person[:2]-floor_base[:2, 3]
            bearing = math.atan2(direction[1], direction[0])
            if time.monotonic()-self.person_last_received > self.pick_config['max_detection_age_s']:
                self.person_count = 0
            if self.person_sample is not None:
                difference = bearing-self.person_sample[2]
                if abs(math.atan2(math.sin(difference), math.cos(difference))) > self.pick_config['stable_angle_rad']:
                    self.person_count = 0
            # Validate stand-off before accepting the observation.
            placement_targets(person, floor_base, self.pick_config, floor_chassis)
            self.person_sample = (person, floor_base, bearing, floor_chassis)
            self.person_count += 1
            self.person_last_received = time.monotonic()
        except (ValueError, TypeError, KeyError, IndexError, cv2.error, TransformException) as exc:
            self.person_count = 0
            self.person_sample = None
            self.report('person_wait', str(exc))

    def current_q(self):
        return np.asarray([self.full_state[j] for j in ARM_JOINTS])

    def joint_move(self, goal):
        current = self.current_q()
        duration = max(.5, float(np.max(np.abs(goal-current)/.025)))
        self.begin(2, dict(start=current, waypoints=[goal], durations=[duration],
                           total=duration, start_error=0.))

    def path_completed(self):
        if self.pick_sequence is None:
            super().path_completed()
            return
        stage = self.pick_sequence.complete(self.pick_sequence.stage)
        self.report('pick_place', stage)
        if stage in ('grasp', 'release'):
            current = self.current_q()
            if stage == 'release':
                error = self.model.pose_error(current, self.pick_final)
                if np.linalg.norm(error[:3]) > .002 or np.linalg.norm(error[3:]) > .02:
                    raise ValueError('release blocked: final placement pose not reached')
            self.begin(2, dict(start=current, waypoints=[current], durations=[.5],
                               total=.5, start_error=0.,
                               ee_target_rad=self.pick_config['close_rad' if stage == 'grasp' else 'open_rad'],
                               ee_speed_rad_s=self.pick_config['ee_speed_rad_s']))
        elif stage == 'return_path':
            self.begin(2, execution_plan(reverse_path(self.paths[2]), self.current_q(),
                                         self.start_tolerance, self.max_start_delta, self.approach_speed))
        elif stage == 'ready':
            self.joint_move(self.pick_ready)
        elif stage == 'person':
            self.stage = 'pick_person'
            self.pick_hold_epoch = self.status['epoch']
            self.pick_wait_started = time.monotonic()
            self.person_stamp = -1
            self.person_count = 0
            self.person_sample = None
        elif stage == 'preplace':
            goal = self.model.solve(self.pick_preplace, self.current_q(), self.margin, max_time_s=.03)
            # MoveIt plans the transit to preplace; descent is a separate line.
            path = dict(start=goal, waypoints=[goal], durations=[.5], total=.5)
            self.entry_path = path
            self.request_entry_plan(2, path, self.current_q(),
                                    float(np.max(np.abs(goal-self.current_q()))), self.status)
        elif stage == 'descend':
            self.pick_descent_origin = self.model.fk(self.current_q())
            if (np.linalg.norm(self.pick_descent_origin[:3, 3]-self.pick_preplace[:3, 3]) > .002
                    or np.linalg.norm(self.model.pose_error(self.current_q(), self.pick_preplace)[3:]) > .02):
                raise ValueError('preplace feedback outside pose tolerance')
            self.pick_plan = dict(start=self.current_q(), waypoints=[], durations=[], total=0.,
                                  start_error=0., cartesian_line=True)
            self.pick_hold_epoch = self.status['epoch']
            self.pick_steps = max(1, math.ceil(self.pick_config['preplace_height_m']/.002))
            self.stage = 'pick_descent_plan'
        elif stage == 'complete':
            self.report('completed', 'tool2 release target reached; remain HOLD')
            self.reset()

    def tick(self):
        if self.pick_sequence is None:
            super().tick()
            return
        try:
            now = time.monotonic()
            if now-self.pick_started > self.pick_config['sequence_timeout_s']:
                raise RuntimeError('tool2 sequence timed out')
            if self.pick_sequence.stage in ('align', 'preplace', 'descend', 'release'):
                if (self.person_sample is None
                        or now-self.person_last_received > self.pick_config['max_detection_age_s']):
                    raise RuntimeError('person observation lost/unsafe during placement')
            if self.stage in ('pick_person', 'pick_descent_plan'):
                self.healthy_hold(now)
                if self.status['epoch'] != self.pick_hold_epoch:
                    raise RuntimeError('pick/place ownership changed or cancelled')
                if self.stage == 'pick_person':
                    if now-self.pick_wait_started > self.pick_config['acquire_timeout_s']:
                        raise RuntimeError('person acquisition timeout')
                    if (self.person_count < self.pick_config['stable_samples']
                            or now-self.person_last_received > self.pick_config['max_detection_age_s']):
                        return
                    person, floor_base, _, floor_chassis = self.person_sample
                    preplace, final = placement_targets(person, floor_base, self.pick_config, floor_chassis)
                    base_floor = np.linalg.inv(floor_base)
                    self.pick_floor_base = floor_base.copy()
                    self.pick_preplace, self.pick_final = base_floor @ preplace, base_floor @ final
                    person_base = (base_floor @ np.r_[person, 1.])[:3]
                    self.pick_sequence.complete('person')
                    self.joint_move(aligned_base(self.model, self.current_q(), person_base, self.margin))
                else:
                    if np.max(np.abs(self.current_q()-self.pick_plan['start'])) > .005:
                        raise RuntimeError('arm moved during descent planning')
                    index = len(self.pick_plan['waypoints'])+1
                    target = self.pick_final.copy()
                    target[:3, 3] = (self.pick_preplace[:3, 3]+index/self.pick_steps*
                                     (self.pick_final[:3, 3]-self.pick_preplace[:3, 3]))
                    seed = self.pick_plan['waypoints'][-1] if self.pick_plan['waypoints'] else self.current_q()
                    q = self.model.solve(target, seed, self.margin, max_time_s=.03)
                    if np.max(np.abs(q-seed)) > .03:
                        raise ValueError('descent IK joint step exceeds limit')
                    duration = max(.02, self.pick_config['preplace_height_m']/self.pick_steps/
                                   self.pick_config['linear_speed_m_s'], float(np.max(np.abs(q-seed)))/.025)
                    self.pick_plan['waypoints'].append(q)
                    self.pick_plan['durations'].append(duration)
                    self.pick_plan['total'] += duration
                    if index == self.pick_steps:
                        self.begin(2, self.pick_plan)
                return
            super().tick()
        except (ValueError, RuntimeError, KeyError, TransformException, np.linalg.LinAlgError) as exc:
            self.fail(str(exc))


def main():
    rclpy.init()
    node = None
    try:
        node = Tool2PickPlaceCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.stage != 'idle':
                node.fail('pick/place coordinator shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
