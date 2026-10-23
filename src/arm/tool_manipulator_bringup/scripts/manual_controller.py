#!/usr/bin/env python3
"""Operator-guided RMD coordination inspired by dolbotZ MANUAL_EE.

Only shoulder/elbow/wrist pitch participate in translation-only differential
IK. This is not six-dimensional TCP pose control or collision path planning.
Position targets go directly to a dedicated ForwardCommandController.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointJog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String

JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_roll_joint', 'wrist_yaw_joint')
RMD = JOINTS[1:4]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(norm) or norm == 0:
        raise ValueError('joint axis must be finite and nonzero')
    x, y, z = axis / norm
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


class ArmChain:
    """FK uses every movable joint; the Jacobian contains only the RMDs."""
    def __init__(self, description, root='base_actuator', tip='tcp_link'):
        by_child = {}
        for element in ET.fromstring(description).findall('joint'):
            if element.find('mimic') is not None:
                raise ValueError('mimic joints are not supported by this arm model')
            origin = element.find('origin')
            attrs = {} if origin is None else origin.attrib
            xyz = np.array([float(v) for v in attrs.get('xyz', '0 0 0').split()])
            rpy = [float(v) for v in attrs.get('rpy', '0 0 0').split()]
            if xyz.shape != (3,) or len(rpy) != 3 or not np.all(np.isfinite([*xyz, *rpy])):
                raise ValueError('invalid URDF origin')
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (rotation([0, 0, 1], rpy[2]) @
                                  rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0]))
            axis = element.find('axis')
            axis = np.array([float(v) for v in (
                '1 0 0' if axis is None else axis.get('xyz', '1 0 0')).split()])
            if axis.shape != (3,):
                raise ValueError('invalid URDF axis')
            rotation(axis, 0.)
            by_child[element.find('child').get('link')] = (
                element.get('name'), element.get('type'),
                element.find('parent').get('link'), transform, axis / np.linalg.norm(axis))
        chain, visited = [], set()
        while tip != root:
            if tip in visited or tip not in by_child:
                raise ValueError('missing or cyclic URDF chain')
            visited.add(tip)
            joint = by_child[tip]
            chain.append(joint)
            tip = joint[2]
        self.chain = list(reversed(chain))
        if tuple(j[0] for j in self.chain if j[0] in RMD) != RMD:
            raise ValueError('controlled link must be downstream of all three RMD joints')
        self.required = tuple(j[0] for j in self.chain if j[1] != 'fixed')
        if set(self.required) - set(JOINTS):
            raise ValueError('chain contains unsupported movable joints')

    def pose_and_jacobian(self, positions):
        transform, frames = np.eye(4), []
        for name, kind, _, origin, axis in self.chain:
            transform = transform @ origin
            if name in RMD:
                frames.append((transform[:3, 3].copy(), transform[:3, :3] @ axis))
            if kind in ('revolute', 'continuous'):
                transform[:3, :3] = transform[:3, :3] @ rotation(axis, positions[name])
            elif kind == 'prismatic':
                transform[:3, 3] += transform[:3, :3] @ axis * positions[name]
            elif kind != 'fixed':
                raise ValueError(f'unsupported joint type: {kind}')
        point = transform[:3, 3]
        return transform, np.column_stack([np.cross(axis, point-origin) for origin, axis in frames])


def coordinated_velocity(jacobian, requested, damping, speed_limits):
    """Damped translation IK, uniformly scaled to preserve joint coordination."""
    jacobian = np.asarray(jacobian, dtype=float)
    requested = np.asarray(requested, dtype=float)
    speed_limits = np.asarray(speed_limits, dtype=float)
    if (jacobian.shape != (3, 3) or requested.shape != (3,) or
            speed_limits.shape != (3,) or not np.all(np.isfinite(jacobian)) or
            not np.all(np.isfinite(requested)) or not np.all(np.isfinite(speed_limits)) or
            np.any(speed_limits <= 0) or not math.isfinite(damping) or damping <= 0):
        raise ValueError('invalid differential IK inputs')
    velocity = jacobian.T @ np.linalg.solve(
        jacobian @ jacobian.T + damping*damping*np.eye(3), requested)
    return velocity / max(1., float(np.max(np.abs(velocity) / speed_limits)))


def bounded_step(target, velocity, lower, upper, dt, margin):
    """Scale the whole vector at a boundary; allow inward motion in the margin."""
    delta = np.asarray(velocity) * dt
    scale = 1.
    for q, d, lo, hi in zip(target, delta, lower+margin, upper-margin):
        if d > 0:
            scale = min(scale, max(0., (hi-q)/d))
        elif d < 0:
            scale = min(scale, max(0., (lo-q)/d))
    return np.asarray(target) + scale*delta


class ManualController(Node):
    node_name = 'manual_controller'
    requires_model = True
    extra_parameters = {}
    limit_groups = (slice(1, 4), slice(0, 1), slice(4, 5), slice(5, 6))

    def __init__(self):
        super().__init__(self.node_name)
        share = Path(get_package_share_directory('tool_manipulator_bringup'))
        defaults = {
            'hardware_config': str(share/'config/hardware.yaml'),
            'robot_description': '', 'root_link': 'base_actuator', 'controlled_link': 'tcp_link',
            'command_topic': '/manual_position_controller/commands', 'joy_topic': '/joy',
            'servo_mode': False, 'joint_jog_topic': '/servo_node/delta_joint_cmds',
            'ee_velocity_command_topic': '/ee_velocity_controller/commands',
            'joint_state_topic': '/joint_states', 'publish_rate_hz': 50.,
            'linear_speed_mps': 0.03, 'joint_speed_rad_s': 0.12, 'damping': 0.05,
            'ee_velocity_rad_s': 0.12,
            'joy_timeout_sec': 0.25, 'joint_state_timeout_sec': 0.5,
            'max_target_lead_rad': 0.10, 'limit_margin_rad': 0.02, 'deadzone': 0.12,
            'forward_axis': 4, 'invert_forward': False,
            'wrist_yaw_axis': 0, 'invert_wrist_yaw': True,
            'wrist_roll_axis': 6, 'invert_wrist_roll': False,
            'ee_velocity_axis': 7, 'invert_ee_velocity': False, 'ee_enable_button': 1,
            'base_ccw_button': 4, 'base_cw_button': 5,
            'focus_button': 9, 'manage_focus': True, 'focus_topic': '/control/active_target',
        }
        defaults.update(self.extra_parameters)
        self.params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        for name in ('publish_rate_hz', 'linear_speed_mps', 'joint_speed_rad_s', 'damping',
                     'ee_velocity_rad_s',
                     'joy_timeout_sec', 'joint_state_timeout_sec', 'max_target_lead_rad'):
            if not math.isfinite(self.params[name]) or self.params[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('forward_axis', 'wrist_yaw_axis', 'wrist_roll_axis', 'ee_velocity_axis',
                     'base_ccw_button', 'base_cw_button', 'ee_enable_button', 'focus_button',
                     *self.extra_parameters):
            if self.params[name] < 0:
                raise ValueError(f'{name} must be nonnegative')
        if not 0 <= self.params['deadzone'] < 1:
            raise ValueError('deadzone must be in [0, 1)')
        margin = self.params['limit_margin_rad']
        if not math.isfinite(margin) or margin < 0:
            raise ValueError('limit_margin_rad must be finite and nonnegative')
        hardware = yaml.safe_load(Path(self.params['hardware_config']).read_text())
        self.command_diagnostics = hardware.get('buses', {}).get('dynamixel_u2d2', {}).get('position_diagnostics', False)
        config = hardware['joints']
        self.lower = np.array([config[n]['soft_limit_rad'][0] for n in JOINTS], dtype=float)
        self.upper = np.array([config[n]['soft_limit_rad'][1] for n in JOINTS], dtype=float)
        self.speeds = np.minimum(self.params['joint_speed_rad_s'],
                                 [config[n]['velocity_limit_rad_s'] for n in JOINTS])
        if (not np.all(np.isfinite([self.lower, self.upper, self.speeds])) or
                np.any(self.upper-self.lower <= 2*margin) or np.any(self.speeds <= 0)):
            raise ValueError('invalid hardware limits')
        self.chain = None
        self.positions, self.state_times = {}, {}
        self.target, self.joy, self.joy_time = None, None, None
        self._jog_active = False
        self.focus, self.previous_focus, self.fault = 'drive', False, False
        self.last_direction_warning = -math.inf
        self.last_time = self.now_seconds()
        self.publisher = self.create_publisher(Float64MultiArray, self.params['command_topic'], 1)
        self.jog_publisher = (self.create_publisher(JointJog, self.params['joint_jog_topic'], 10)
                              if self.params['servo_mode'] else None)
        self.ee_velocity_publisher = self.create_publisher(
            Float64MultiArray, self.params['ee_velocity_command_topic'], 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.focus_publisher = self.create_publisher(String, self.params['focus_topic'], qos)
        self.create_subscription(String, self.params['focus_topic'], self.on_focus, qos)
        if not self.requires_model:
            pass
        elif self.params['robot_description']:
            self.on_description(String(data=self.params['robot_description']))
        else:
            self.create_subscription(String, '/robot_description', self.on_description, qos)
        self.create_subscription(JointState, self.params['joint_state_topic'], self.on_state, 10)
        self.create_subscription(Joy, self.params['joy_topic'], self.on_joy, 10)
        self.create_timer(1/self.params['publish_rate_hz'], self.on_timer)
        if self.params['manage_focus']:
            self.focus_publisher.publish(String(data='drive'))
        self.get_logger().info('ready to activate: press Options to switch drive -> arm')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_description(self, message):
        try:
            chain = ArmChain(message.data, self.params['root_link'], self.params['controlled_link'])
        except (ValueError, ET.ParseError, KeyError, TypeError) as exc:
            self.chain = None
            self.hold()
            self.target = None
            self.get_logger().error(f'Invalid arm model: {exc}')
            return
        self.hold()
        self.chain, self.target = chain, None

    def on_state(self, message):
        now = self.now_seconds()
        for name, q in zip(message.name, message.position):
            if name in JOINTS:
                if math.isfinite(q):
                    self.positions[name], self.state_times[name] = q, now
                else:
                    self.state_times.pop(name, None)

    def on_focus(self, message):
        if message.data != self.focus:
            self.hold()
            self.publish_ee_velocity(0.)
            self.target = None
            # Explicit focus cycling is required after a following-error stop.
            if message.data != 'arm':
                self.fault = False
        self.focus = message.data

    def button(self, name):
        i = self.params[name]
        return bool(self.joy is not None and i < len(self.joy.buttons) and self.joy.buttons[i])

    def axis(self, name):
        i = self.params[name]
        value = self.joy.axes[i] if self.joy is not None and i < len(self.joy.axes) else 0.
        if not math.isfinite(value) or abs(value) < self.params['deadzone']:
            return 0.
        return float(np.clip(value, -1., 1.))

    def on_joy(self, message):
        self.joy, self.joy_time = message, self.now_seconds()
        pressed = self.button('focus_button')
        if self.params['manage_focus'] and pressed and not self.previous_focus:
            target = 'drive' if self.focus == 'arm' else 'arm'
            self.on_focus(String(data=target))
            self.focus_publisher.publish(String(data=target))
            self.get_logger().info(f'Options selected control mode: {target}')
        self.previous_focus = pressed

    def hold(self):
        if self.jog_publisher is not None:
            self.publish_jog(np.zeros(len(JOINTS)))
            return
        if self.target is not None:
            self.publisher.publish(Float64MultiArray(data=self.target.tolist()))

    def publish_jog(self, velocity):
        moving = bool(np.any(np.abs(velocity) > 1e-9))
        if not moving and not getattr(self, '_jog_active', False):
            return
        self._jog_active = moving
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.params['root_link']
        message.joint_names = list(JOINTS)
        message.velocities = [float(value) for value in velocity]
        message.duration = 1 / self.params['publish_rate_hz']
        self.jog_publisher.publish(message)

    def publish_ee_velocity(self, velocity):
        self.ee_velocity_publisher.publish(Float64MultiArray(data=[float(velocity)]))

    def ee_velocity(self):
        """O must remain held; D-pad up is clockwise and down counterclockwise."""
        if not self.button('ee_enable_button'):
            return 0.
        value = self.axis('ee_velocity_axis')
        direction = -1 if self.params['invert_ee_velocity'] else 1
        return self.params['ee_velocity_rad_s'] * value * direction

    def on_timer(self):
        now = self.now_seconds()
        dt = now-self.last_time
        self.last_time = now
        fresh_state = all(0 <= now-self.state_times.get(n, -math.inf) <=
                          self.params['joint_state_timeout_sec'] for n in JOINTS)
        fresh_joy = self.joy_time is not None and 0 <= now-self.joy_time <= self.params['joy_timeout_sec']
        if ((self.requires_model and self.chain is None) or self.focus != 'arm' or not fresh_state or
                not fresh_joy or self.fault or dt <= 0 or dt > self.params['joint_state_timeout_sec']):
            self.publish_ee_velocity(0.)
            self.hold()
            if not fresh_state or not fresh_joy or dt <= 0:
                self.target = None
            return
        measured = np.array([self.positions[n] for n in JOINTS])
        if np.any(measured < self.lower) or np.any(measured > self.upper):
            self.publish_ee_velocity(0.)
            self.hold()
            self.fault = True
            self.get_logger().error('Joint feedback outside hardware limits; use reviewed recovery before manual control.')
            return
        if self.target is None:
            self.target = measured.copy()
        if np.max(np.abs(self.target-measured)) > self.params['max_target_lead_rad']:
            self.publish_ee_velocity(0.)
            self.hold()
            self.fault = True
            self.get_logger().error('Following error: holding last target. Inspect arm, then cycle focus to reset.')
            return
        self.publish_ee_velocity(self.ee_velocity())
        if self.jog_publisher is not None:
            velocity = self.command_velocity(measured)
            for index, value in enumerate(velocity):
                if ((value > 0 and measured[index] >= self.upper[index]-self.params['limit_margin_rad']) or
                        (value < 0 and measured[index] <= self.lower[index]+self.params['limit_margin_rad'])):
                    velocity[index] = 0
            if getattr(self, 'command_diagnostics', False) and np.any(np.abs(velocity) > 1e-9):
                if now - getattr(self, '_last_command_log', -math.inf) >= 1.:
                    self._last_command_log = now
                    self.get_logger().info(
                        'DIAG_JOG ' + ' '.join(f'{name}={v:.6f}' for name, v in zip(JOINTS, velocity))
                        + ' rad/s -> /servo_node/delta_joint_cmds')
            self.publish_jog(velocity)
            self.target = measured.copy()
            return
        dt = min(dt, 1/self.params['publish_rate_hz'])
        velocity = self.command_velocity(measured)
        candidate = self.target.copy()
        for indexes in self.limit_groups:
            candidate[indexes] = bounded_step(self.target[indexes], velocity[indexes],
                self.lower[indexes], self.upper[indexes], dt, self.params['limit_margin_rad'])
        lead = np.max(np.abs(candidate-measured))
        if lead <= self.params['max_target_lead_rad']:
            self.target = candidate
        self.hold()

    def command_velocity(self, measured):
        """RMD differential IK; separate mode overrides only this mapping."""
        now = self.now_seconds()
        forward = self.axis('forward_axis') * (-1 if self.params['invert_forward'] else 1)
        requested = np.array([math.cos(measured[0])*forward, math.sin(measured[0])*forward, 0.])
        requested *= self.params['linear_speed_mps'] / max(1., np.linalg.norm(requested))
        _, jacobian = self.chain.pose_and_jacobian(self.positions)
        velocity = np.zeros(6)
        velocity[1:4] = coordinated_velocity(jacobian, requested, self.params['damping'], self.speeds[1:4])
        if (np.linalg.norm(requested) > 1e-6 and
                np.linalg.norm(jacobian @ velocity[1:4]) < .1*np.linalg.norm(requested) and
                now-self.last_direction_warning > 2.):
            self.get_logger().warning('Requested direction is poorly reachable at this posture; '
                                      'try the forward/back input to bend the arm first.')
            self.last_direction_warning = now
        velocity[0] = self.speeds[0]*(self.button('base_ccw_button')-self.button('base_cw_button'))
        roll = self.axis('wrist_roll_axis')
        velocity[4] = self.speeds[4]*roll*(-1 if self.params['invert_wrist_roll'] else 1)
        # Left-stick horizontal follows the same arm command path as the other
        # five joints: Servo JointJog in coordinated mode, bounded position
        # target integration in separate mode.
        yaw = self.axis('wrist_yaw_axis')
        velocity[5] = self.speeds[5]*yaw*(-1 if self.params['invert_wrist_yaw'] else 1)
        return velocity


def main(controller_type=ManualController):
    rclpy.init()
    node = None
    try:
        node = controller_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.hold()
                node.publish_ee_velocity(0.)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
