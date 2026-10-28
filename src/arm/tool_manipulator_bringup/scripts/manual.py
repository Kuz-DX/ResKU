#!/usr/bin/env python3
"""Main supervised position control without Servo or collision checking."""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String

from manual_control_common import JOINTS, ManualController, bounded_step, vertical_pitch_velocity


# Joint-space radians: DXL base/roll/yaw = 0; RMD shoulder/elbow/pitch = +85/+87/-89 deg.
STOP_POSE_RAD = (0., math.radians(85.), math.radians(87.), math.radians(-89.), 0., 0.)


def position_limit_violations(positions, lower, upper):
    """Describe finite measured-limit violations without clipping or moving."""
    violations = []
    for name, lo, hi in zip(JOINTS, lower, upper):
        value = positions.get(name)
        if value is None or not math.isfinite(value):
            continue
        excess = value-hi if value > hi else value-lo if value < lo else 0.
        if excess:
            violations.append(dict(joint=name, measured_rad=float(value), lower_rad=float(lo),
                                   upper_rad=float(hi), excess_rad=float(excess),
                                   excess_deg=math.degrees(float(excess))))
    return violations


class Manual(ManualController):
    node_name = 'manual'
    collision_guarded = False
    local_tool_paths = True
    # Legacy Servo-mode on_timer gates only the joint at a directional limit.
    # Keep the same DLS velocity coupling, but do not freeze all three RMDs
    # because one position target reaches its boundary or lead budget.
    limit_groups = tuple(slice(i, i+1) for i in range(6))
    extra_parameters = {
        'ee_velocity_axis': 6,
        'wrist_pitch_axis': 7,
        'vertical_axis': 1,
        'stop_pose_button': 3,  # Linux PlayStation square
        'startup_recovery_enabled': True,
        'startup_recovery_margin_rad': math.radians(2.),
        'startup_recovery_speed_rad_s': .03,
        'startup_recovery_max_excursion_rad': math.radians(5.),
        'startup_recovery_timeout_sec': 15.,
        'entry_alignment_speed_rad_s': .09,
        'entry_alignment_timeout_margin_sec': 15.,
    }

    def __init__(self):
        # The base constructor calls the overridden hold() while loading URDF.
        self.controller_ready = False
        self.phase = 'waiting'
        self.status = 'waiting for controller and feedback'
        self.stop_target = None
        self.stop_pose_active = False
        self.previous_stop_pose = False
        self.path_combos = [False, False, False]
        self.path_points = None
        self.path_index = 0
        self.path_l2_held = False
        self.hardware_fault = False
        self.neutral_required = True
        self.controller_stamp = -math.inf
        self.controllers = {}
        self.list_future = self.switch_future = self.parameter_future = None
        self.parameters_verified = False
        self.switch_started = None
        self.last_poll = -math.inf
        self.last_dt = .02
        self.activation_blockers = []
        self.last_activation_warning = -math.inf
        self.startup_goal = None
        self.startup_recovering = False
        self.startup_recovery_started = None
        self.startup_complete = False
        self.entry_alignment_goal = None
        self.entry_alignment_started = None
        self.entry_alignment_deadline = None
        self.entry_alignment_complete = False
        self.rmd_limit_blocked_joints = []
        self.rmd_lead_blocked_joints = []
        self.rmd_requested_velocity = np.zeros(3)
        self.rmd_applied_velocity = np.zeros(3)
        super().__init__()
        for name in ('startup_recovery_margin_rad', 'startup_recovery_speed_rad_s',
                     'startup_recovery_max_excursion_rad', 'startup_recovery_timeout_sec'):
            if not math.isfinite(self.params[name]) or self.params[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if (self.params['startup_recovery_margin_rad'] < self.params['limit_margin_rad'] or
                np.any(2*self.params['startup_recovery_margin_rad'] >= self.upper[1:4]-self.lower[1:4])):
            raise ValueError('startup recovery margin must fit RMD limits and cover manual margin')
        for name in ('entry_alignment_speed_rad_s', 'entry_alignment_timeout_margin_sec'):
            if not math.isfinite(self.params[name]) or self.params[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if np.any(self.lower[[0, 4, 5]] + self.params['limit_margin_rad'] > 0) or np.any(
                self.upper[[0, 4, 5]] - self.params['limit_margin_rad'] < 0):
            raise ValueError('DXL zero must fit manual limits')
        if self.params['servo_mode']:
            raise ValueError('manual requires servo_mode:=false')
        if self.params['command_topic'] != '/manual_position_controller/commands':
            raise ValueError('manual only supports manual_position_controller')
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.parameter_client = self.create_client(GetParameters, '/manual_position_controller/get_parameters')
        self.diagnostics = self.create_publisher(String, '/manual/diagnostics', 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/control/hardware_fault', self.on_hardware_fault, qos)
        collision_notice = ('Collision checking required. ' if self.collision_guarded else 'No collision checking. ')
        if self.params['startup_recovery_enabled']:
            self.get_logger().warning(collision_notice + 'Automatic startup RMD recovery to 2-degree inset; '
                                      'manual input stays disabled until recovery completes. '
                                      'No automatic home or grasp.')
        else:
            self.get_logger().warning(collision_notice + 'Automatic startup recovery is disabled.')

    def now_seconds(self):
        # Watchdogs must continue to expire if ROS /clock stalls or jumps.
        return time.monotonic()

    def fresh_state(self, now):
        return all(0 <= now-self.state_times.get(n, -math.inf) <=
                   self.params['joint_state_timeout_sec'] for n in JOINTS)

    def measured(self):
        return np.array([self.positions[n] for n in JOINTS])

    def valid_measured(self, now):
        return (self.fresh_state(now) and
                np.all(self.measured() >= self.lower) and np.all(self.measured() <= self.upper))

    def startup_feedback_allowed(self, now):
        if not self.fresh_state(now):
            return False
        if not self.params.get('startup_recovery_enabled', False):
            return self.valid_measured(now)
        measured = self.measured()
        allowance = np.zeros(6)
        allowance[1:4] = self.params['startup_recovery_max_excursion_rad']
        # Only the three RMDs may start slightly outside their soft limits.
        # Dynamixel calibration, nearest-turn and limits are not changed.
        return bool(np.all(measured >= self.lower-allowance) and np.all(measured <= self.upper+allowance))

    def make_startup_goal(self, measured):
        goal = measured.copy()
        if self.params.get('startup_recovery_enabled', False):
            margin = self.params['startup_recovery_margin_rad']
            goal[1:4] = np.clip(measured[1:4], self.lower[1:4]+margin, self.upper[1:4]-margin)
        return goal

    def on_state(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            self.state_times.clear()
            return
        super().on_state(message)

    def on_hardware_fault(self, message):
        if message.data.strip():
            self.hardware_fault = True
            self.fault = True
            self.status = 'hardware fault: ' + message.data
            self.neutral_required = True
            self.publish_ee_velocity(0.)
            self.hold()

    def on_focus(self, message):
        if message.data != self.focus:
            self.publish_ee_velocity(0.)
            self.stop_pose_active = False
            self.stop_target = None
            self.neutral_required = True
            # Hardware/controller failures require a reviewed full restart.
            if message.data != 'arm' and not self.hardware_fault and self.phase != 'failed':
                self.fault = False
            if message.data != 'arm':
                self.entry_alignment_started = None
                self.entry_alignment_deadline = None
        self.focus = message.data

    def on_joy(self, message):
        axes = ('forward_axis', 'wrist_yaw_axis', 'wrist_roll_axis', 'ee_velocity_axis', 'wrist_pitch_axis', 'vertical_axis')
        buttons = ('base_ccw_button', 'base_cw_button', 'ee_enable_button', 'focus_button', 'stop_pose_button')
        if (len(message.axes) <= max(self.params[n] for n in axes) or
                len(message.buttons) <= max(self.params[n] for n in buttons) or
                not all(math.isfinite(v) for v in message.axes)):
            self.joy, self.joy_time = None, None
            self.path_points = None
            self.path_l2_held = False
            self.path_combos = [False, False, False]
            self.neutral_required = True
            return
        super().on_joy(message)
        l2 = len(message.axes) > 2 and message.axes[2] < -.5
        # Same Linux PlayStation mapping as ik_manual: triangle=tool0,
        # square=tool1, X=tool2. Managed mode delegates all three to its producer.
        from tool_trajectory_joy import requested_path
        request, combos = requested_path(message.axes, message.buttons, self.path_combos)
        self.path_combos = combos
        self.path_l2_held = l2
        if self.path_points is not None and not l2:
            self.path_points = None
            self.neutral_required = True
        if (self.local_tool_paths and request is not None and
                self.path_points is None and not self.stop_pose_active):
            self.start_tool_path(request)
        if l2:
            self.previous_stop_pose = bool(message.buttons[3])
            return
        pressed = self.button('stop_pose_button')
        if (pressed and not self.previous_stop_pose and self.controller_ready and
                self.focus == 'arm' and not self.neutral_required and
                not self.hardware_fault and not self.fault and
                not getattr(self, 'startup_recovering', False)):
            goal = np.array(STOP_POSE_RAD)
            margin = self.params['limit_margin_rad']
            if np.any(goal < self.lower+margin) or np.any(goal > self.upper-margin):
                self.get_logger().error('Stop_pose rejected: requested joint angles outside manual limits')
            else:
                self.stop_pose_active = True
        self.previous_stop_pose = pressed

    def start_tool_path(self, path_id):
        if not self.local_tool_paths:
            return
        if (not self.controller_ready or self.focus != 'arm' or self.neutral_required or
                self.hardware_fault or self.fault or self.startup_recovering or
                not self.entry_alignment_complete or not self.valid_measured(self.now_seconds())):
            self.get_logger().warning(f'Tool{path_id} path blocked: ready arm focus, neutral controls and fresh feedback required')
            return
        path_file = Path(__file__).resolve().parents[1]/'config'/f'tool{path_id}_trajectory.yaml'
        try:
            from types import SimpleNamespace
            from tool_trajectory_joy import load_path
            model = SimpleNamespace(within_limits=lambda q, margin: bool(
                np.all(q >= self.lower+margin) and np.all(q <= self.upper-margin)))
            path = load_path(path_file, model, self.params['limit_margin_rad'])
            if path is None:
                raise ValueError('path disabled')
            if path['after_path'] is not None:
                raise ValueError('close/return sequence requires ik_manual.launch.py')
            points = [path['start'], *path['waypoints']]
            if np.max(np.abs(self.measured()-points[0])) > .10:
                raise ValueError('start differs from measured position by more than 0.10 rad')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Tool{path_id} path rejected: {exc}')
            return
        self.path_points = points
        self.path_index = 0
        self.path_durations = [.02, *path['durations']]
        self.path_segment_index = None
        self.stop_target = None
        self.get_logger().warning(f'Tool{path_id} path started; keep L2 held')

    def run_tool_path(self, dt):
        goal = self.path_points[self.path_index]
        if self.path_segment_index != self.path_index:
            self.path_segment_index = self.path_index
            duration = max(self.path_durations[self.path_index],
                           float(np.max(np.abs(goal-self.target)/self.speeds)))
            self.path_segment_speeds = np.abs(goal-self.target)/max(duration, .02)
            self.path_deadline = self.now_seconds()+duration+30.
        if self.now_seconds() > self.path_deadline:
            self.get_logger().error(f'Tool path point {self.path_index+1} feedback timeout')
            self.stop('tool path feedback timeout', dt)
            return
        candidate = self.target + np.clip(goal-self.target,
            -self.path_segment_speeds*dt, self.path_segment_speeds*dt)
        # Pace every joint together when one falls behind; retain the taught line.
        if np.any(np.abs(candidate-self.measured()) > self.params['max_target_lead_rad']):
            candidate = self.target.copy()
        self.status = f'tool path point {self.path_index+1}/{len(self.path_points)}'
        self.apply_motion(candidate, 0., 'tool_path', dt, goal=goal)
        if (np.allclose(self.target, goal, atol=1e-9, rtol=0.) and
                np.max(np.abs(self.measured()-goal)) <= .005):
            self.path_index += 1
            if self.path_index == len(self.path_points):
                self.path_points = None
                self.neutral_required = True
                self.status = 'tool path reached'

    def run_stop_pose(self, dt):
        """Ramp to the exact pose with existing speed and feedback lead limits."""
        goal = np.array(STOP_POSE_RAD)
        candidate = self.target + np.clip(goal-self.target, -self.speeds*dt, self.speeds*dt)
        allowed = np.abs(candidate-self.measured()) <= self.params['max_target_lead_rad']
        candidate = np.where(allowed, candidate, self.target)
        self.status = 'aligning Stop_pose'
        self.apply_motion(candidate, 0., 'stop_pose', dt, goal=goal)
        if (np.allclose(self.target, goal, atol=1e-9, rtol=0.) and
                np.max(np.abs(self.measured()-goal)) <= .005):
            self.stop_pose_active = False
            self.neutral_required = True
            self.status = 'Stop_pose reached'

    def neutral(self):
        return (all(self.axis(n) == 0. for n in
                    ('forward_axis', 'wrist_roll_axis', 'wrist_yaw_axis', 'ee_velocity_axis',
                     'wrist_pitch_axis', 'vertical_axis')) and
                not any(self.button(n) for n in ('base_ccw_button', 'base_cw_button', 'ee_enable_button', 'stop_pose_button')))

    def command_velocity(self, measured):
        self.pitch_vertical_active = False
        if self.button('ee_enable_button'):
            # O is an exclusive tool/pitch layer. Simultaneous pitch requests
            # favor explicit wrist adjustment; release D-pad to translate.
            velocity = np.zeros(6)
            pitch = self.axis('wrist_pitch_axis')
            vertical = self.axis('vertical_axis')
            if pitch:
                velocity[3] = self.speeds[3]*pitch
            elif vertical:
                self.pitch_vertical_active = True
                positions = dict(zip(JOINTS, self.target if self.target is not None else measured))
                velocity[1:4] = vertical_pitch_velocity(
                    self.chain, positions, vertical*self.params['linear_speed_mps'],
                    self.params['damping'], self.speeds[1:4])
            self.rmd_requested_velocity = velocity[1:4].copy()
            return velocity
        # Reuse the existing ResKU FK/DLS, direction and mapping only when the
        # bend input is nonzero. No FK/Jacobian/IK for a Dynamixel-only input.
        if self.axis('forward_axis') != 0.:
            velocity = super().command_velocity(measured)
            self.rmd_requested_velocity = velocity[1:4].copy()
            self.rmd_limit_blocked_joints = []
            for index in range(1, 4):
                # Match the original manual_controller.py Servo branch's
                # directional measured-position gate. Target limits are also
                # enforced per joint when integrating below.
                if ((velocity[index] > 0 and measured[index] >= self.upper[index]-self.params['limit_margin_rad']) or
                        (velocity[index] < 0 and measured[index] <= self.lower[index]+self.params['limit_margin_rad'])):
                    velocity[index] = 0.
                    self.rmd_limit_blocked_joints.append(JOINTS[index])
            return velocity
        velocity = np.zeros(6)
        velocity[0] = self.speeds[0]*(self.button('base_ccw_button')-self.button('base_cw_button'))
        velocity[4] = self.speeds[4]*self.axis('wrist_roll_axis')*(-1 if self.params['invert_wrist_roll'] else 1)
        velocity[5] = self.speeds[5]*self.axis('wrist_yaw_axis')*(-1 if self.params['invert_wrist_yaw'] else 1)
        return velocity

    def fail_controller(self, reason):
        self.phase, self.status = 'failed', reason
        self.controller_ready = False
        self.fault = True
        self.neutral_required = True
        self.publish_ee_velocity(0.)
        self.get_logger().error(reason + '; movement blocked, restart after inspection.')

    def report_activation_blockers(self, now):
        """Make every startup gate visible; never silently wait on real limits."""
        blockers = []
        if not self.parameters_verified:
            blockers.append('waiting for verified controller joints/position interface')
        if self.controllers.get('manual_position_controller') != 'inactive':
            blockers.append('manual_position_controller is not inactive')
        if self.controllers.get('ee_velocity_controller') != 'active':
            blockers.append('ee_velocity_controller is not active')
        if now-self.controller_stamp > .75:
            blockers.append('controller state unavailable or stale')
        stale = [n for n in JOINTS if not 0 <= now-self.state_times.get(n, -math.inf) <=
                 self.params['joint_state_timeout_sec']]
        if stale:
            blockers.append('missing/stale joint feedback: ' + ', '.join(stale))
        for violation in position_limit_violations(self.positions, self.lower, self.upper):
            recoverable = (self.params.get('startup_recovery_enabled', False) and
                           violation['joint'] in JOINTS[1:4] and
                           abs(violation['excess_rad']) <= self.params['startup_recovery_max_excursion_rad'])
            if not recoverable:
                blockers.append('{joint}={measured_rad:.6f} rad outside '
                                '[{lower_rad:.6f}, {upper_rad:.6f}], excess={excess_deg:+.3f} deg'.format(**violation))
        if self.chain is None:
            blockers.append('robot model unavailable')
        if self.hardware_fault:
            blockers.append('hardware fault latched')
        if self.publisher.get_subscription_count() == 0:
            blockers.append('position command subscriber unavailable')
        if not self.switch_client.service_is_ready():
            blockers.append('controller switch service unavailable')
        self.activation_blockers = blockers
        if blockers:
            self.status = 'activation blocked: ' + '; '.join(blockers)
            if now-getattr(self, 'last_activation_warning', -math.inf) >= 2.:
                self.last_activation_warning = now
                self.get_logger().warning(self.status +
                    '. No arm commands are sent. Check initial pose/calibration before recovery; '
                    'Options cannot override this block.')

    def poll_controller(self, now):
        if self.phase == 'failed':
            return
        for attr, client in (('list_future', self.list_client), ('parameter_future', self.parameter_client)):
            future = getattr(self, attr)
            if future is not None and future.done():
                setattr(self, attr, None)
                try:
                    result = future.result()
                    if attr == 'list_future':
                        self.controllers = {c.name: c.state for c in result.controller}
                        self.controller_stamp = now
                        manual = next((c for c in result.controller if c.name == 'manual_position_controller'), None)
                        if manual and manual.type != 'forward_command_controller/ForwardCommandController':
                            self.fail_controller('Unexpected manual controller type')
                        if self.controllers.get('arm_controller') == 'active':
                            self.fail_controller('arm_controller active in Servo-free path')
                        if self.phase == 'ready' and self.controllers.get('manual_position_controller') != 'active':
                            self.fail_controller('manual_position_controller lost activation')
                        if self.phase == 'ready' and self.controllers.get('ee_velocity_controller') != 'active':
                            self.fail_controller('ee_velocity_controller lost activation')
                    else:
                        values = result.values
                        self.parameters_verified = (len(values) == 2 and
                            list(values[0].string_array_value) == list(JOINTS) and
                            values[1].string_value == 'position')
                        if not self.parameters_verified:
                            self.fail_controller('Manual controller joint order/interface mismatch')
                except Exception as exc:
                    self.fail_controller('Controller service error: ' + str(exc))
        if self.switch_future is not None and self.switch_future.done():
            try:
                if not self.switch_future.result().ok:
                    self.fail_controller('Strict controller activation failed')
                elif self.phase != 'failed':
                    self.phase = 'confirming'
                    # Require a list response requested after switching.
                    if self.list_future is not None:
                        self.list_future.cancel()
                        self.list_future = None
                    self.controller_stamp = -math.inf
            except Exception as exc:
                self.fail_controller('Controller activation error: ' + str(exc))
            self.switch_future = None
        if self.phase == 'failed':
            return
        if self.switch_started is not None and self.phase != 'ready' and now-self.switch_started > 5.:
            self.fail_controller('Controller activation timed out')
            return
        if self.phase == 'ready' and now-self.controller_stamp > .75:
            self.fail_controller('Controller state watchdog expired')
            return
        if now-self.last_poll >= .2:
            self.last_poll = now
            if self.list_future is None and self.list_client.service_is_ready():
                self.list_future = self.list_client.call_async(ListControllers.Request())
            if (not self.parameters_verified and self.parameter_future is None and
                    self.parameter_client.service_is_ready()):
                self.parameter_future = self.parameter_client.call_async(
                    GetParameters.Request(names=['joints', 'interface_name']))
        if self.phase == 'waiting' and self.controllers.get('manual_position_controller') == 'active':
            self.fail_controller('Manual controller must start inactive for measured-position seeding')
            return
        if self.phase == 'waiting':
            self.report_activation_blockers(now)
        if (self.phase == 'waiting' and self.parameters_verified and
                self.controllers.get('manual_position_controller') == 'inactive' and
                self.controllers.get('ee_velocity_controller') == 'active' and
                now-self.controller_stamp <= .75 and self.startup_feedback_allowed(now) and
                self.chain is not None and not self.hardware_fault and
                self.publisher.get_subscription_count() > 0 and self.switch_client.service_is_ready()):
            self.target = self.measured()
            self.startup_goal = self.make_startup_goal(self.target)
            self.startup_recovering = not np.allclose(self.target, self.startup_goal, atol=1e-9, rtol=0.)
            self.startup_complete = not self.startup_recovering
            self.publisher.publish(Float64MultiArray(data=self.target.tolist()))
            self.switch_started = now
            self.phase = 'switching'
            self.activation_blockers = []
            self.status = 'activating position controller with measured hold'
            if self.startup_recovering:
                details = ', '.join(f'{n}: {q:.6f} -> {g:.6f} rad' for n, q, g in
                                    zip(JOINTS[1:4], self.target[1:4], self.startup_goal[1:4]) if abs(g-q) > 1e-9)
                self.get_logger().warning('Startup RMD recovery planned: ' + details +
                    '. Joystick movement commands are ignored until recovery completes. '
                    'Only inward limited position increments will be sent.')
            self.switch_future = self.switch_client.call_async(SwitchController.Request(
                activate_controllers=['manual_position_controller'],
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=3)))
        if (self.phase == 'confirming' and now-self.controller_stamp <= .75 and
                self.controllers.get('manual_position_controller') == 'active' and
                self.controllers.get('ee_velocity_controller') == 'active'):
            self.phase = 'ready'
            self.controller_ready = True
            self.activation_blockers = []
            self.neutral_required = True
            self.get_logger().info('Position controller active with measured hold. ' +
                ('Startup RMD recovery must finish before manual input.' if self.startup_recovering else
                 'Neutralize controls; Options selects arm.'))

    def run_startup_recovery(self, now, dt):
        """Inward-only direct position ramp, once per startup, without any IK."""
        self.publish_ee_velocity(0.)
        self.neutral_required = True
        if (self.startup_recovery_started is not None and
                now-self.startup_recovery_started > self.params['startup_recovery_timeout_sec']):
            self.fail_controller('Startup recovery timed out; inspect feedback/tracking before restart')
            return
        if not self.startup_feedback_allowed(now):
            if self.fresh_state(now):
                self.fail_controller('Startup recovery feedback outside permitted recovery envelope')
            else:
                self.status = 'startup recovery paused: missing feedback'
                self.hold()
            return
        measured = self.measured()
        if np.max(np.abs(self.target-measured)) > self.params['max_target_lead_rad']:
            self.fail_controller('Startup recovery following error')
            return
        # This once-per-launch recovery is the explicitly requested startup
        # sequence, like the legacy recovery that runs before joy_node starts.
        # Joystick input cannot modify its target; normal teleoperation below
        # still requires fresh input, arm focus and neutral acknowledgement.
        if self.startup_recovery_started is None:
            self.startup_recovery_started = now
        self.status = 'startup RMD recovery: inward to 2-degree inset'
        step = np.minimum(self.speeds[1:4], self.params['startup_recovery_speed_rad_s'])*dt
        candidate = self.target.copy()
        candidate[1:4] += np.clip(self.startup_goal[1:4]-self.target[1:4], -step, step)
        if np.max(np.abs(candidate[1:4]-measured[1:4])) <= self.params['max_target_lead_rad']:
            self.apply_motion(candidate, 0., 'startup', dt, goal=self.startup_goal)
        else:
            self.hold()
        if (np.allclose(self.target[1:4], self.startup_goal[1:4], atol=1e-9, rtol=0.) and
                self.valid_measured(now) and
                np.max(np.abs(measured[1:4]-self.startup_goal[1:4])) <= .005):
            self.startup_recovering = False
            self.startup_complete = True
            self.stop_target = None
            self.status = 'startup RMD recovery verified'
            self.get_logger().info('Startup RMD recovery verified from joint feedback; '
                                   'Options selects arm and starts one-time DXL zero alignment.')

    def run_entry_alignment(self, now, dt):
        """On first arm focus, move only the three DXL joints to joint zero."""
        self.publish_ee_velocity(0.)
        self.neutral_required = True
        if not self.valid_measured(now):
            self.fail_controller('DXL entry alignment requires valid joint feedback')
            return
        measured = self.measured()
        indexes = [0, 4, 5]
        if self.entry_alignment_goal is None:
            self.target = measured.copy()
            self.entry_alignment_goal = measured.copy()
            self.entry_alignment_goal[indexes] = 0.
            self.get_logger().info('First arm entry: aligning DXL base/roll/yaw to zero')
        if self.entry_alignment_started is None:
            self.entry_alignment_started = now
            speeds = np.minimum(self.speeds[indexes], self.params['entry_alignment_speed_rad_s'])
            self.entry_alignment_deadline = (now + np.max(np.abs(measured[indexes])/speeds)
                                             + self.params['entry_alignment_timeout_margin_sec'])
        if now > self.entry_alignment_deadline:
            self.fail_controller('DXL entry alignment timed out; inspect feedback')
            return
        if np.max(np.abs(self.target-measured)) > self.params['max_target_lead_rad']:
            self.fail_controller('DXL entry alignment following error')
            return
        step = np.minimum(self.speeds[indexes], self.params['entry_alignment_speed_rad_s'])*dt
        candidate = self.target.copy()
        candidate[indexes] += np.clip(-self.target[indexes], -step, step)
        if np.max(np.abs(candidate[indexes]-measured[indexes])) <= self.params['max_target_lead_rad']:
            self.apply_motion(candidate, 0., 'entry_alignment', dt, goal=self.entry_alignment_goal)
        else:
            self.hold()
        self.status = 'aligning DXL base/roll/yaw to zero'
        if (np.max(np.abs(self.target[indexes])) <= 1e-9 and
                np.max(np.abs(measured[indexes])) <= .005):
            self.entry_alignment_complete = True
            self.entry_alignment_goal = None
            self.entry_alignment_started = None
            self.entry_alignment_deadline = None
            self.status = 'DXL entry alignment verified'
            self.get_logger().info('DXL entry alignment verified; neutralize controls for manual input')

    def apply_motion(self, candidate, ee_velocity, source, dt, goal=None):
        """Direct path used by the managed producer; main manual overrides the gate."""
        self.target = candidate
        self.publish_ee_velocity(ee_velocity)
        self.hold()

    def hold(self):
        # Only publish after explicit initialization; never send a default zero.
        # On loss of controller ownership, block the entire arm command stream.
        if self.phase in ('switching', 'confirming', 'ready') and self.target is not None:
            self.publisher.publish(Float64MultiArray(data=self.target.tolist()))

    def stop(self, reason, dt):
        self.stop_pose_active = False
        self.path_points = None
        self.publish_ee_velocity(0.)
        self.neutral_required = True
        if self.status != reason:
            self.stop_target = None
        self.status = reason
        if self.target is not None and self.valid_measured(self.now_seconds()):
            if self.stop_target is None:
                self.stop_target = self.measured()
            # Remove residual target lead without an instantaneous target jump.
            self.target += np.clip(self.stop_target-self.target, -self.speeds*dt, self.speeds*dt)
        self.hold()

    def emit_diagnostics(self, dt):
        measured = [self.positions.get(n) for n in JOINTS]
        target = None if self.target is None else self.target.tolist()
        data = dict(monotonic_sec=self.now_seconds(), dt_sec=dt, joints=JOINTS,
                    collision=getattr(self, 'collision_status', None),
                    status=self.status, phase=self.phase, focus=self.focus, fault=self.fault,
                    activation_blockers=getattr(self, 'activation_blockers', []),
                    position_limit_violations=position_limit_violations(self.positions, self.lower, self.upper),
                    startup_recovering=getattr(self, 'startup_recovering', False),
                    startup_complete=getattr(self, 'startup_complete', False),
                    entry_alignment_complete=getattr(self, 'entry_alignment_complete', False),
                    startup_goal_rad=None if getattr(self, 'startup_goal', None) is None else self.startup_goal.tolist(),
                    rmd_requested_velocity_rad_s=getattr(self, 'rmd_requested_velocity', np.zeros(3)).tolist(),
                    rmd_applied_velocity_rad_s=getattr(self, 'rmd_applied_velocity', np.zeros(3)).tolist(),
                    rmd_limit_blocked_joints=getattr(self, 'rmd_limit_blocked_joints', []),
                    rmd_lead_blocked_joints=getattr(self, 'rmd_lead_blocked_joints', []),
                    controller_ready=self.controller_ready, controllers=self.controllers,
                    measured_rad=measured, target_rad=target,
                    error_rad=None if target is None else [
                        None if q is None else t-q for t, q in zip(target, measured)],
                    joy_axes=None if self.joy is None else list(self.joy.axes),
                    joy_buttons=None if self.joy is None else list(self.joy.buttons))
        self.diagnostics.publish(String(data=json.dumps(data, allow_nan=False)))

    def on_timer(self):
        now = self.now_seconds()
        elapsed = now-self.last_time
        self.last_time = now
        dt = min(max(elapsed, 0.), 1/self.params['publish_rate_hz'])
        self.last_dt = dt
        self.pitch_vertical_active = False
        self.rmd_requested_velocity = np.zeros(3)
        self.rmd_applied_velocity = np.zeros(3)
        self.rmd_limit_blocked_joints = []
        self.rmd_lead_blocked_joints = []
        self.poll_controller(now)
        fresh_joy = self.joy_time is not None and 0 <= now-self.joy_time <= self.params['joy_timeout_sec']
        if not self.controller_ready:
            self.publish_ee_velocity(0.)
            # Re-seed continuously while inactive/activating, including after
            # the controller's on_activate may have cleared its input buffer.
            if self.phase in ('switching', 'confirming') and self.startup_feedback_allowed(now):
                # Keep the captured seed fixed when outside limits: repeated
                # noisy measurements must not request outward recovery motion.
                if not getattr(self, 'startup_recovering', False):
                    self.target = self.measured()
                self.hold()
        elif self.hardware_fault or self.fault:
            self.stop('latched fault', dt)
        elif getattr(self, 'startup_recovering', False):
            if elapsed <= 0 or elapsed > self.params['joint_state_timeout_sec']:
                self.status = 'startup recovery paused: timer discontinuity'
                self.publish_ee_velocity(0.)
                self.hold()
            else:
                self.run_startup_recovery(now, dt)
        elif self.focus == 'arm' and not getattr(self, 'entry_alignment_complete', False):
            if elapsed <= 0 or elapsed > self.params['joint_state_timeout_sec']:
                self.status = 'DXL entry alignment paused: timer discontinuity'
                self.publish_ee_velocity(0.)
                self.hold()
            else:
                self.run_entry_alignment(now, dt)
        elif not self.fresh_state(now):
            self.stop('feedback timeout', dt)
        elif not self.valid_measured(now):
            self.fault = True
            self.stop('feedback outside hardware limits', dt)
        elif elapsed <= 0 or elapsed > self.params['joint_state_timeout_sec']:
            self.stop('timer discontinuity', dt)
        elif self.target is not None and np.max(np.abs(self.target-self.measured())) > self.params['max_target_lead_rad']:
            self.fault = True
            self.get_logger().error('Following error; hold capture. Inspect arm and cycle focus.')
            self.stop('following error', dt)
        elif not fresh_joy or self.focus != 'arm' or self.chain is None:
            self.path_points = None
            self.stop('input timeout' if not fresh_joy else 'input disabled', dt)
        elif self.path_points is not None and self.path_l2_held:
            self.run_tool_path(dt)
        elif self.path_l2_held:
            self.stop('L2 path selection', dt)
        elif self.neutral_required:
            self.stop('neutral controls required', dt)
            if self.neutral() and (self.stop_target is None or np.allclose(self.target, self.stop_target, atol=1e-9)):
                self.neutral_required = False
                self.stop_target = None
        elif getattr(self, 'stop_pose_active', False):
            self.run_stop_pose(dt)
        else:
            self.status = 'operator control'
            self.stop_target = None
            measured = self.measured()
            velocity = self.command_velocity(measured)
            candidate = self.target.copy()
            groups = ((slice(1, 4), slice(0, 1), slice(4, 5), slice(5, 6))
                      if self.pitch_vertical_active else self.limit_groups)
            for indexes in groups:
                step = bounded_step(self.target[indexes], velocity[indexes], self.lower[indexes],
                                    self.upper[indexes], dt, self.params['limit_margin_rad'])
                index = indexes.start
                if index in (1, 2, 3) and not np.allclose(
                        step-self.target[indexes], velocity[indexes]*dt, atol=1e-12, rtol=0.):
                    if JOINTS[index] not in self.rmd_limit_blocked_joints:
                        self.rmd_limit_blocked_joints.extend(JOINTS[indexes] if self.pitch_vertical_active else [JOINTS[index]])
                # Vertical motion shares limits and lead budget across all RMDs
                # to preserve pitch. Legacy bending retains per-joint gates.
                if np.max(np.abs(step-measured[indexes])) <= self.params['max_target_lead_rad']:
                    candidate[indexes] = step
                elif index in (1, 2, 3) and abs(velocity[index]) > 1e-12:
                    self.rmd_lead_blocked_joints.extend(JOINTS[indexes] if self.pitch_vertical_active else [JOINTS[index]])
            if dt > 0:
                self.rmd_applied_velocity = (candidate[1:4]-self.target[1:4])/dt
            self.apply_motion(candidate, self.ee_velocity(), 'operator', dt)
        self.emit_diagnostics(elapsed)

    def shutdown_controller(self):
        self.publish_ee_velocity(0.)
        self.stop('shutdown', 1/self.params['publish_rate_hz'])
        if self.switch_client.service_is_ready() and self.phase != 'waiting':
            future = self.switch_client.call_async(SwitchController.Request(
                deactivate_controllers=['manual_position_controller'],
                strictness=SwitchController.Request.BEST_EFFORT, timeout=Duration(sec=1)))
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)


def check_exclusive():
    """Read-only graph check, before this launch opens any hardware devices."""
    node = rclpy.create_node('manual_preflight')
    try:
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        conflicts = set(node.get_node_names()) & {
            'controller_manager', 'manual_controller', 'manual_controller_sep',
            'manual', 'collision_scene', 'servo_node', 'joint_joy_teleop', 'tcp_joy_teleop'}
        if conflicts:
            node.get_logger().error('Stop the existing arm stack first: ' + ', '.join(sorted(conflicts)))
            return 1
        return 0
    finally:
        node.destroy_node()


def main(node_type=Manual):
    # Keep ROS context alive through the bounded hold/deactivation cleanup.
    from rclpy.signals import SignalHandlerOptions
    import signal
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    try:
        if '--check-exclusive' in sys.argv:
            return check_exclusive()
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Do not let timer callbacks integrate while cleanup spins services.
            for timer in node.timers:
                timer.cancel()
            node.hardware_fault = True
            node.shutdown_controller()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
