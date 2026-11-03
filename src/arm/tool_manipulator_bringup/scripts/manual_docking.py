"""AprilTag input and operator-only docking adapter for standalone manual.py."""
import json
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from manual_control_common import JOINTS, bounded_step
from manual_docking_core import DockingAssist, load_config, joint_velocity
from tool_docking_core import transform, tag_pose


class ManualDocking:
    def __init__(self, node, config_file):
        self.node = node
        self.core = DockingAssist(load_config(config_file))
        self.ready = False
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

    @property
    def selected(self):
        return self.core.selected

    def cancel(self, reason):
        self.core.cancel(reason)
        self.ready = False

    def on_cancel(self, request, response):
        self.node.stop('docking assist cancelled by service', self.node.last_dt)
        self.cancel('cancelled by service')
        response.success, response.message = True, 'Docking assist cancelled; neutral required'
        return response

    def select(self, tool):
        n = self.node
        if self.selected and self.core.tool == tool:
            n.stop('docking assist cancelled by operator', n.last_dt)
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
        n.stop('docking assist selection', n.last_dt)
        self.core.begin(tool, n.now_seconds())
        # Also reject delayed frames captured before the selection event.
        self.selection_stamp_ns = n.get_clock().now().nanoseconds
        self.ready = False
        n.get_logger().info(f'tool{tool} assist: release selection buttons and neutralize sticks; waiting for tag')

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
            reference = (None if self.core.base_tag is None else
                         np.linalg.inv(base_camera) @ self.core.base_tag)
            camera_tag = tag_pose(detections[0], *self.camera,
                                  {**self.core.settings['vision'], **item}, reference=reference)
            age = (self.node.get_clock().now().nanoseconds-stamp_ns)*1e-9
            self.core.observe(base_camera @ camera_tag, stamp_ns, self.node.now_seconds(), age)
        except (ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError,
                TransformException, cv2.error, np.linalg.LinAlgError) as exc:
            self.core.reject(str(exc))

    def run(self, now, dt):
        """Supervise tag freshness, then use the SAME operator path as manual."""
        n = self.node
        try:
            measured_pose, _ = n.chain.pose_and_jacobian(n.positions)
            active = self.core.check(measured_pose, now)
            if n.path_l2_held or n.neutral_required:
                n.publish_ee_velocity(0.)
                if n.stop_target is None:
                    n.stop_target = n.measured()
                n.target += np.clip(n.stop_target-n.target, -n.speeds*dt, n.speeds*dt)
                if (not n.path_l2_held and n.neutral() and
                        not any(n.joy.buttons[i] for i in (0, 1, 2, 3)) and
                        np.allclose(n.target, n.stop_target, atol=1e-9)):
                    n.neutral_required = False
                    n.stop_target = None
                self.ready = active and not n.neutral_required
                n.status = 'docking assist: release selection and neutralize manual controls'
                n.hold()
                return
            self.ready = active
            n.run_operator(now, dt)
        except (ValueError, np.linalg.LinAlgError) as exc:
            n.stop('docking assist stopped: ' + str(exc), dt)

    def adjust(self, manual_candidate, measured, now, dt):
        """Add tag correction to the already-mapped/bounded manual position step.

        Explicit DXL, O-layer or R2 pitch-hold inputs take priority and pass through
        exactly. The right-stick RMD coordination receives assistance only when
        its ACTUAL Cartesian velocity approaches the dock, regardless of axis sign.
        """
        n = self.node
        if self.core.state != 'active':
            n.status = 'manual control; acquiring docking tag'
            return manual_candidate
        explicit = (n.pitch_hold_requested() or n.button('ee_enable_button') or n.button('base_ccw_button') or
                    n.button('base_cw_button') or n.axis('wrist_roll_axis') or
                    n.axis('wrist_yaw_axis'))
        if explicit or not n.axis('forward_axis'):
            self.core.reason = 'original manual mapping; explicit input priority'
            return manual_candidate
        current, linear, angular = n.chain.pose_and_jacobian(
            dict(zip(JOINTS, n.target)), include_angular=True, all_joints=True)
        jacobian = np.vstack((linear, angular))
        manual_velocity = (manual_candidate-n.target)/dt
        manual_twist = jacobian @ manual_velocity
        twist = self.core.assist_twist(current, manual_twist, now, dt)
        if np.max(np.abs(twist-manual_twist)) < 1e-12:
            return manual_candidate
        velocity = joint_velocity(jacobian, twist, n.speeds, self.core.cfg,
                                  self.core.target[:3, 2], manual_velocity)
        margin = n.params['limit_margin_rad']
        if np.any(((measured <= n.lower+margin) & (velocity < 0)) |
                  ((measured >= n.upper-margin) & (velocity > 0))):
            velocity[:] = 0.
        candidate = bounded_step(n.target, velocity, n.lower, n.upper, dt, margin)
        if np.max(np.abs(candidate-measured)) > n.params['max_target_lead_rad']:
            candidate = n.target.copy()
        # Include finite FK curvature, not only the differential IK constraint.
        normal = self.core.target[:3, 2]
        remaining = float((self.core.target[:3, 3]-current[:3, 3]) @ normal)
        for _ in range(12):
            next_pose, _ = n.chain.pose_and_jacobian(dict(zip(JOINTS, candidate)))
            advance = float((next_pose[:3, 3]-current[:3, 3]) @ normal)
            allowed = max(0., float(twist[:3] @ normal)*dt)
            if advance <= min(allowed, max(0., remaining-self.core.cfg['axial_tolerance_m'])) + 1e-9:
                break
            candidate = (candidate+n.target)*.5
        else:
            candidate = n.target.copy()
        n.status = 'manual control + AprilTag assist: ' + self.core.reason
        return candidate

    def snapshot(self):
        return dict(**self.core.snapshot(), ready=self.ready,
                    assumed_parameters=True,
                    input_mapping='manual')
