"""Button-requested angular alignment; manual translation after completion."""
import json
import math
import threading
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity
from tf2_ros import Buffer, TransformException, TransformListener

from manual_control_common import JOINTS
from manual_docking_core import DockingAssist, load_config, plan_alignment, trajectory_position
from tool_docking_core import transform, tag_pose, tag_normal_from_camera_side
from tool_trajectory_joy import ArmKinematics


class ManualDocking:
    def __init__(self, node, config_file):
        self.node = node
        self.core = DockingAssist(load_config(config_file))
        self.ready = False
        self.initialize_alignment()
        self.camera = None
        self.camera_frame = node.declare_parameter(
            'docking_camera_frame', 'arm_camera_color_optical_frame').value
        self.base_frame = node.params['root_link']
        if not self.camera_frame:
            raise ValueError('docking_camera_frame must not be empty')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.create_subscription(CameraInfo, node.declare_parameter(
            'docking_camera_info_topic', '/arm/camera/color/camera_info').value,
            self.on_camera, qos_profile_sensor_data)
        node.create_subscription(String, node.declare_parameter(
            'docking_centers_topic', '/arm/apriltag/centers').value, self.on_tag, 1)
        node.create_service(Trigger, '/manual/docking/cancel', self.on_cancel)
        node.get_logger().warning(
            'Manual docking uses ASSUMED/unverified tuning and tag-to-TCP transforms from ' + config_file)

    def initialize_alignment(self):
        n = self.node
        self.model = ArmKinematics('', n.params['hardware_config'],
                                   n.params['root_link'], n.params['controlled_link'],
                                   description=n.params['robot_description'])
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='manual_alignment')
        self.cancel_event = threading.Event()
        self.future = self.collision_future = self.plan = None
        self.collision_client = n.create_client(GetStateValidity, '/check_state_validity')
        self.stage = 'idle'
        self.request_wrist_yaw = None
        self.plan_started = 0.

    def clear_plan(self):
        self.cancel_event.set()
        for future in (self.future, self.collision_future):
            if future is not None and not future.done():
                future.cancel()
        self.future = self.collision_future = self.plan = None
        self.stage = 'idle'
        self.request_wrist_yaw = None

    def shutdown(self):
        self.clear_plan()
        self.executor.shutdown(wait=False, cancel_futures=True)

    @property
    def selected(self):
        return self.core.selected

    def cancel(self, reason):
        self.clear_plan()
        self.core.cancel(reason)
        self.ready = False

    def on_cancel(self, request, response):
        self.node.stop('docking assist cancelled by service', self.node.last_dt)
        self.cancel('cancelled by service')
        response.success, response.message = True, 'Docking assist cancelled; neutral required'
        return response

    def select(self, tool):
        n = self.node
        if self.selected and self.core.tool == tool and not self.core.monitoring:
            n.stop('docking assist cancelled by operator', n.last_dt)
            return
        if not self.core.settings['tools'][tool].get('alignment_enabled', False):
            n.get_logger().warning(f'tool{tool} alignment rejected: tag/insertion-axis geometry unconfirmed')
            return
        blockers = [name for name, blocked in (
            ('controller not ready', not n.controller_ready),
            ('focus is not arm', n.focus != 'arm'),
            ('neutral controls required', n.neutral_required),
            ('hardware fault', n.hardware_fault), ('fault', n.fault),
            ('startup recovery active', n.startup_recovering),
            ('entry alignment incomplete', not n.entry_alignment_complete)) if blocked]
        now = n.now_seconds()
        if not n.fresh_state(now):
            blockers.append('joint feedback stale or missing')
        elif not n.valid_measured(now):
            blockers.append('joint feedback outside limits or non-finite')
        if blockers:
            n.get_logger().warning('Docking assist selection rejected: ' + '; '.join(blockers))
            return
        self.clear_plan()
        n.stop('angular alignment request', n.last_dt)
        # Capture actual ID3 feedback at the accepted button edge, not when
        # tag acquisition or asynchronous planning later completes.
        self.request_wrist_yaw = float(n.measured()[5]) if tool == 1 else 0.
        self.core.begin(tool, n.now_seconds())
        self.raw_base_tag = None
        self.tcp_camera_rotation = None
        # Also reject delayed frames captured before the selection event.
        self.selection_stamp_ns = n.get_clock().now().nanoseconds
        self.ready = False
        n.get_logger().info(f'tool{tool} alignment: release buttons; DXL reference first, then RMD-only pitch; waiting for tag')

    def handle_joy(self, request, message, l2):
        n = self.node
        if message.buttons[3] and (self.selected or request in (0, 1)):
            self.cancel('manual HOME requested')
            return l2
        if l2 and sum(message.buttons[i] for i in (0, 1, 2)) > 1:
            n.stop('ambiguous docking selection', n.last_dt)
            return True
        if request in (0, 1):
            n.start_tool_path(request)
            return True
        if not self.selected:
            return False
        if request == 2:
            self.cancel('manual tool2 path requested')
        # Ordinary buttons, including O and HOME, keep their manual meanings.
        return False

    def stamp(self, data):
        stamp = data['stamp']
        if (type(stamp['sec']) is not int or type(stamp['nanosec']) is not int or
                stamp['sec'] < 0 or not 0 <= stamp['nanosec'] < 1000000000):
            raise ValueError('invalid tag timestamp')
        ns = stamp['sec']*1000000000 + stamp['nanosec']
        if ns <= self.selection_stamp_ns:
            raise ValueError('tag image predates selection')
        age = (self.node.get_clock().now().nanoseconds-ns)*1e-9
        if not 0 <= age < self.core.cfg['tag_max_age_s']:
            raise ValueError('stale/future tag timestamp')
        return ns, age

    def on_camera(self, message):
        self.camera = None
        matrix, distortion = np.asarray(message.k).reshape(3, 3), np.asarray(message.d)
        if (message.header.frame_id == self.camera_frame and
                message.distortion_model in ('plumb_bob', 'rational_polynomial') and
                matrix[0, 0] > 0 and matrix[1, 1] > 0 and
                np.isfinite(matrix).all() and np.isfinite(distortion).all() and
                len(distortion) in (0, 4, 5, 8, 12, 14)):
            self.camera = matrix, distortion

    def on_tag(self, message):
        if not self.selected:
            return
        try:
            data = json.loads(message.data)
            stamp_ns, age = self.stamp(data)
            if self.camera is None or data['frame_id'] != self.camera_frame:
                raise ValueError('missing CameraInfo or optical frame mismatch')
            item = self.core.settings['tools'][self.core.tool]
            detections = [d for d in data['detections']
                          if d['id'] == item['tag_id'] and d.get('family') == item['tag_family']]
            if len(detections) != 1:
                raise ValueError('selected tool tag missing or duplicated')
            # Non-blocking lookup at IMAGE time; never stall the 50 Hz watchdog
            # in this single-threaded node waiting for camera TF.
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time(nanoseconds=stamp_ns)).transform
            p, q = tf.translation, tf.rotation
            base_camera = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            reference = (None if getattr(self, 'raw_base_tag', None) is None else
                         np.linalg.inv(base_camera) @ self.raw_base_tag)
            camera_tag = tag_pose(detections[0], *self.camera,
                                  {**self.core.settings['vision'], **item}, reference=reference)
            age = (self.node.get_clock().now().nanoseconds-stamp_ns)*1e-9
            base_tag = base_camera @ camera_tag
            raw_base_tag = base_tag.copy()
            normal, _ = tag_normal_from_camera_side(base_tag, base_camera)
            if float(normal @ base_tag[:3, 2]) < 0:
                base_tag[:3, 1:3] *= -1.
            if item.get('alignment_reference') == 'camera_plane':
                relation = self.tf_buffer.lookup_transform(
                    self.node.params['controlled_link'], self.camera_frame,
                    Time(nanoseconds=stamp_ns)).transform.rotation
                self.tcp_camera_rotation = transform([0., 0., 0.],
                    [relation.x, relation.y, relation.z, relation.w])[:3, :3]
            self.core.observe(base_tag, stamp_ns, self.node.now_seconds(), age)
            self.raw_base_tag = raw_base_tag
        except (ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError,
                TransformException, cv2.error, np.linalg.LinAlgError) as exc:
            self.core.reject(str(exc))

    def fail(self, reason, dt):
        self.node.stop('angular alignment failed: ' + reason, dt)
        self.clear_plan()
        self.core.fail(reason)
        self.ready = False
        self.node.get_logger().warning('ALIGNMENT_FAILED: ' + reason)

    def run(self, now, dt):
        n = self.node
        try:
            measured = n.measured()
            pose = self.model.fk(measured)
            if self.core.monitoring:
                self.core.monitor(pose)  # Actual joint_states, never the command target.
                self.ready = self.core.state == 'ALIGNMENT_OK'
                if n.path_l2_held or n.neutral_required:
                    self.hold_until_neutral(dt)
                else:
                    n.run_operator(now, dt)
                return
            if n.path_l2_held or any(n.joy.buttons[i] for i in (0, 1, 2, 3)):
                n.publish_ee_velocity(0.)
                n.hold()
                if now-self.core.started > self.core.cfg['acquire_timeout_s']:
                    self.fail('selection buttons held too long', dt)
                return
            # Wait for both selection buttons to release, regardless of release order.
            # Once released, manual motion cancels rather than competing with alignment.
            if not n.neutral():
                self.fail('manual input; release controls and request again', dt)
                return
            if n.neutral_required:
                self.hold_until_neutral(dt)
                return
            if not self.core.capture(pose, now, measured[0]):
                n.publish_ee_velocity(0.)
                n.status = 'TAG_NOT_DETECTED: waiting for stable observations'
                n.hold()
                return
            if now >= self.core.expires:
                self.fail('tag input timeout during requested alignment', dt)
                return
            if self.stage != 'idle' and not 0 <= now-n.state_times.get('ee_joint', -math.inf) <= n.params['joint_state_timeout_sec']:
                self.fail('EE feedback timeout during alignment', dt)
                return
            if self.stage == 'idle':
                if not self.collision_client.service_is_ready():
                    self.fail('collision-check service unavailable', dt)
                    return
                ee = n.positions.get('ee_joint')
                if ee is None or not math.isfinite(ee) or now-n.state_times.get('ee_joint', -math.inf) > n.params['joint_state_timeout_sec']:
                    self.fail('fresh EE position required for collision validation', dt)
                    return
                self.held_ee = ee
                self.fixed_state = dict(n.positions)
                cfg = dict(self.core.settings['alignment'], max_angle_rad=self.core.cfg['max_target_angle_rad'])
                if self.core.tool == 1:
                    if getattr(self, 'request_wrist_yaw', None) is None:
                        self.fail('missing ID3 angle captured at alignment request', dt)
                        return
                    cfg['wrist_yaw_rad'] = self.request_wrist_yaw
                if self.core.settings['tools'][self.core.tool].get('alignment_reference') == 'camera_plane':
                    if getattr(self, 'tcp_camera_rotation', None) is None:
                        self.fail('TCP-to-camera optical TF required for camera alignment', dt)
                        return
                    cfg['tcp_camera_rotation'] = self.tcp_camera_rotation.copy()
                self.cancel_event = threading.Event()
                self.plan_started = now
                self.future = self.executor.submit(plan_alignment, self.model, measured.copy(),
                    self.core.target.copy(), cfg,
                    np.minimum(n.speeds, self.core.cfg['joint_speed_rad_s']),
                    n.params['limit_margin_rad'], self.cancel_event)
                self.stage = 'planning'
            if self.stage in ('planning', 'checking'):
                n.publish_ee_velocity(0.)
                n.hold()
                if now-self.plan_started > self.core.settings['alignment']['planning_timeout_s']:
                    self.fail('planning/collision-check timeout', dt)
                    return
                if self.stage == 'planning':
                    if not self.future.done():
                        return
                    self.plan = self.future.result()
                    self.core.target = self.plan['target'].copy()
                    self.stage, self.check_index = 'checking', 0
                # ASSUMED/reused: 0.005 rad path settling, 0.01 rad EE-change guard;
                # these are controller tolerances, not measured hardware accuracy.
                if np.max(np.abs(measured-self.plan['start'])) > .005 or abs(n.positions['ee_joint']-self.held_ee) > .01:
                    self.fail('robot moved after planning snapshot', dt)
                    return
                if self.collision_future is not None:
                    if not self.collision_future.done():
                        return
                    result = self.collision_future.result()
                    if result is None or not result.valid:
                        contacts = [] if result is None else getattr(result, 'contacts', [])
                        pairs = sorted({f'{c.contact_body_1}/{c.contact_body_2}' for c in contacts})
                        self.fail('alignment path collision/invalid state: ' + ', '.join(pairs), dt)
                        return
                    self.check_index += 1
                    self.collision_future = None
                if self.check_index < len(self.plan['samples']):
                    state = dict(self.fixed_state)
                    state.update(zip(JOINTS, self.plan['samples'][self.check_index]))
                    request = GetStateValidity.Request()
                    request.robot_state.is_diff = True
                    request.robot_state.joint_state.name = list(state)
                    request.robot_state.joint_state.position = [float(v) for v in state.values()]
                    self.collision_future = self.collision_client.call_async(request)
                    n.status = f'ALIGNMENT_READY: collision sample {self.check_index+1}/{len(self.plan["samples"])}'
                    return
                self.stage, self.elapsed = 'executing', 0.
                self.dxl_prepared = False
                self.deadline = now+sum(self.plan['durations'])+self.core.settings['alignment']['feedback_timeout_s']
                self.core.state, self.core.reason = 'ALIGNING', 'DXL reference then RMD pitch alignment'
            if self.stage == 'executing':
                if now > self.deadline or not self.collision_client.service_is_ready():
                    self.fail('execution timeout or collision service lost', dt)
                    return
                if abs(n.positions['ee_joint']-self.held_ee) > .01:
                    self.fail('EE moved during alignment', dt)
                    return
                prep_end = self.plan['durations'][0]
                # A timed command endpoint is not measured hardware arrival.
                # Wait under the existing deadline while holding the validated reference.
                if not self.dxl_prepared:
                    if (self.elapsed >= prep_end
                            and np.max(np.abs(measured-self.plan['reference'])) <= .005
                            and np.linalg.norm(pose[:3, 3]-self.core.target[:3, 3])
                            <= self.core.settings['alignment']['position_tolerance_m']):
                        self.dxl_prepared = True
                    else:
                        self.elapsed = min(prep_end, self.elapsed+dt)
                if self.dxl_prepared:
                    self.elapsed = min(sum(self.plan['durations']), self.elapsed+dt)
                candidate = trajectory_position(self.plan, self.elapsed)
                if np.max(np.abs(candidate-measured)) > n.params['max_target_lead_rad']:
                    self.fail('alignment following error', dt)
                    return
                # DXL preparation may translate TCP; RMD-only phase must hold its new position.
                rmd_phase = self.dxl_prepared
                if rmd_phase and np.max(np.abs(measured[[0, 4, 5]]-self.plan['reference'][[0, 4, 5]])) > .005:
                    self.fail('DXL reference not reached or moved during RMD alignment', dt)
                    return
                if rmd_phase and np.linalg.norm(pose[:3, 3]-self.core.target[:3, 3]) > self.core.settings['alignment']['position_tolerance_m']:
                    self.fail('measured TCP drift during RMD alignment', dt)
                    return
                n.status = 'ALIGNING: ' + ('RMD-only pitch, DXL held' if rmd_phase else
                    'waiting for measured DXL preparation' if self.elapsed >= prep_end else
                    'DXL reference preparation')
                n.apply_motion(candidate, 0., 'angular_alignment', dt)
                self.core.monitor(pose)
                if (self.elapsed >= sum(self.plan['durations']) and
                        np.max(np.abs(measured-self.plan['points'][-1])) <= .005 and
                        self.core.errors['angle_rad'] <= self.core.settings['alignment']['angle_tolerance_rad']):
                    self.core.monitoring = True
                    self.core.monitor(pose)
                    self.ready = True
                    n.neutral_required = True
                    self.stage = 'monitoring'
                    label = ('CAMERA_ALIGNMENT_OK (insertion angle unverified)' if
                             self.core.settings['tools'][self.core.tool].get('alignment_reference') == 'camera_plane'
                             else 'ALIGNMENT_OK')
                    n.get_logger().info(label + ': measured error within +/-1 degree; manual docking only')
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            self.fail(str(exc), dt)

    def hold_until_neutral(self, dt):
        n = self.node
        n.publish_ee_velocity(0.)
        if n.stop_target is None:
            n.stop_target = n.measured()
        n.target += np.clip(n.stop_target-n.target, -n.speeds*dt, n.speeds*dt)
        if (not n.path_l2_held and n.neutral() and
                not any(n.joy.buttons[i] for i in (0, 1, 2, 3)) and
                np.allclose(n.target, n.stop_target, atol=1e-9)):
            n.neutral_required = False
            n.stop_target = None
        n.status = 'angular alignment: release selection and neutralize controls'
        n.hold()

    def snapshot(self):
        return dict(**self.core.snapshot(), ready=self.ready, stage=self.stage,
                    input_mapping='manual', automatic_position_correction=False,
                    alignment_reference=(None if self.core.tool is None else
                        self.core.settings['tools'][self.core.tool].get('alignment_reference')),
                    angle_error_deg=(None if 'angle_rad' not in self.core.errors else
                        math.degrees(self.core.errors['angle_rad'])))
