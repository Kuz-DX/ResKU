#!/usr/bin/env python3
"""Single command gateway for manual, TCP IK results and joint waypoint paths."""
import json
import signal
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers, SwitchController
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory

from arm_mode_core import ModeCore
from arm_pose_safety import load_soft_limits

JOINTS = ('base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_roll_joint', 'wrist_yaw_joint')


class ControlModeManager(Node):
    def __init__(self):
        super().__init__('arm_control_mode_manager')
        default = Path(get_package_share_directory('tool_manipulator_bringup'))/'config/hardware.yaml'
        path = Path(self.declare_parameter('hardware_config', str(default)).value)
        self.focus_arm_mode = str(self.declare_parameter('focus_arm_mode', '').value)
        if self.focus_arm_mode not in ('', 'manual', 'tcp_ik'):
            raise ValueError('focus_arm_mode must be empty, manual, or tcp_ik')
        self.focus_activation_pending = False
        limits = load_soft_limits(path, JOINTS)
        config = yaml.safe_load(path.read_text())
        speeds = [min(.12, float(config['joints'][j]['velocity_limit_rad_s'])/2) for j in JOINTS]
        # Tool paths use a conservative command acceleration; manual/TCP keep
        # their existing direct bounded response.
        # Loaded shoulder motion is deliberately much gentler than the distal
        # printed links. Order matches JOINTS.
        path_speeds = [.08, .04, .06, .06, .08, .08]
        path_accelerations = [.08, .03, .06, .06, .08, .08]
        # Limit acceleration slew as well as acceleration itself.  The loaded
        # shoulder gets the gentlest S-curve to avoid exciting printed links.
        path_jerks = [.30, .04, .15, .15, .30, .30]
        path_feedback_leads = [.04, .015, .025, .025, .04, .04]
        tcp_accelerations = [float(self.declare_parameter(
            'tcp_acceleration_rad_s2', .25).value)]*len(JOINTS)
        self.core = ModeCore([limits[j] for j in JOINTS], speeds,
                             startup_recovery_enabled=True,
                             path_accelerations=path_accelerations,
                             tcp_accelerations=tcp_accelerations,
                             joint_names=JOINTS, path_speeds=path_speeds,
                             path_feedback_leads=path_feedback_leads,
                             path_jerks=path_jerks)
        self.core.entry_alignment_enabled = True
        self.core.entry_alignment_complete = False
        self.core.tcp_entry_required = True
        self.ee = self.create_publisher(Float64MultiArray, '/ee_velocity_controller/commands', 1)
        self.position = self.create_publisher(Float64MultiArray, '/manual_position_controller/commands', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/arm/control_mode', qos)
        self.status_pub = self.create_publisher(String, '/arm/control_status', qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 1)
        self.create_subscription(String, '/control/active_target', self.on_focus, qos)
        self.hardware_fault = False
        self.create_subscription(String, '/control/hardware_fault', self.on_fault, qos)
        self.create_subscription(String, '/arm/rmd_diagnostics', self.on_rmd_diagnostic, 10)
        self.create_subscription(String, '/arm/tcp_joy/entry_status', self.on_tcp_entry, 1)
        for source in ('manual', 'tcp_ik', 'tool_auto'):
            self.create_subscription(String, '/arm/input/'+source,
                                     lambda msg, source=source: self.on_command(source, msg), 1)
        for mode in ('manual', 'tcp_ik', 'tool_auto', 'hold'):
            self.create_service(Trigger, '/arm/set_mode/'+mode,
                                lambda req, res, mode=mode: self.set_mode(mode, res))
        self.create_service(Trigger, '/arm/cancel', lambda req,res: self.set_mode('hold', res))
        self.create_service(Trigger, '/arm/reset_fault', self.reset_fault)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_future = self.switch_future = None
        self.controllers = {}
        self.controller_time = self.last_poll = -float('inf')
        self.last_tick = time.monotonic()
        self.activation_started = None
        self.activated = False
        self.last_error = ''
        self.last_report = None
        self.last_path_clamp_warning = -float('inf')
        self.path_clamp_warning_rad = float(self.declare_parameter(
            'path_clamp_warning_rad', .001).value)
        if not 0 < self.path_clamp_warning_rad <= .05:
            raise ValueError('path_clamp_warning_rad must be in (0, 0.05]')
        self.activation_blockers = []
        # Match the 100 Hz ros2_control update rate so the position stream does
        # not alternate between new and repeated 20 ms samples.
        self.create_timer(.01, self.tick)
        self.publish_status()

    def on_state(self, msg):
        if (len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity)
                or len(set(msg.name)) != len(msg.name)):
            self.core.fault('malformed joint feedback')
            return
        p = dict(zip(msg.name, msg.position))
        v = dict(zip(msg.name, msg.velocity))
        if 'ee_joint' in p and 'ee_joint' in v:
            self.core.ee_feedback(p['ee_joint'], v['ee_joint'], time.monotonic())
        if all(j in p and j in v for j in JOINTS):
            self.core.feedback([p[j] for j in JOINTS], [v[j] for j in JOINTS], time.monotonic())
        else:
            self.core.fault('incomplete arm joint feedback')

    def on_focus(self, msg):
        previous = self.core.focus
        self.core.focus = msg.data
        if msg.data == 'arm':
            if previous != 'arm' and self.focus_arm_mode:
                self.focus_activation_pending = True
        else:
            self.focus_activation_pending = False
            self.core.entry_alignment_started = None
            self.core.entry_alignment_deadline = None
        if msg.data != 'arm' and self.core.mode != 'fault' and self.core.startup_complete:
            self.core.request('hold', time.monotonic())

    def on_tcp_entry(self, msg):
        try:
            data = json.loads(msg.data)
            if data['epoch'] != self.core.epoch:
                return
            if (type(data['safe']) is not bool or not self.core.valid(data['positions'])
                    or not isinstance(data['reason'], str)):
                raise ValueError('invalid TCP entry assessment')
            self.core.tcp_entry = dict(data, time=time.monotonic())
        except (ValueError, TypeError, KeyError):
            self.core.tcp_entry = None

    def activate_pending_focus(self, now):
        c = self.core
        if not self.focus_activation_pending:
            return
        if c.mode == 'fault':
            self.focus_activation_pending = False
            return
        if (c.focus == 'arm' and c.mode == 'hold' and not c.transitioning and
                c.ready and c.startup_complete and c.entry_alignment_complete and c.fresh(now)):
            try:
                c.request(self.focus_arm_mode, now)
            except ValueError as exc:
                c.reason = str(exc)
            self.focus_activation_pending = False

    def on_fault(self, msg):
        if msg.data.strip() and msg.data.lower() not in ('ok', 'false', '0', 'none'):
            self.hardware_fault = True
            self.core.fault('hardware fault: '+msg.data)

    def on_rmd_diagnostic(self, msg):
        """Stop an automatic path as soon as an RMD target is being relaxed."""
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('current_protection') is not True:
                return
            joint = data.get('joint', 'unknown RMD joint')
            current = data.get('current_a')
            detail = (f'{joint} at {float(current):.3f} A'
                      if isinstance(current, (int, float)) and not isinstance(current, bool)
                      else str(joint))
            if self.core.mode == 'tool_auto' and self.core.execution_id is not None:
                self.core.abort_path_to_hold(
                    'RMD current protection active: '+detail, time.monotonic())
        except (ValueError, TypeError, OverflowError):
            # Diagnostics are advisory JSON. Hardware faults retain their own
            # latched /control/hardware_fault channel.
            return

    def set_mode(self, mode, res):
        self.focus_activation_pending = False
        try:
            self.core.request(mode, time.monotonic())
            res.success, res.message = True, 'accepted; wait for control_status.transitioning=false'
        except ValueError as exc:
            res.success, res.message = False, str(exc)
        self.publish_status()
        return res

    def reset_fault(self, req, res):
        try:
            if self.hardware_fault:
                raise ValueError('hardware fault requires inspection and full restart')
            self.core.reset(time.monotonic())
            res.success, res.message = True, 'reset to hold'
        except ValueError as exc:
            res.success, res.message = False, str(exc)
        return res

    def on_command(self, source, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('command must be a JSON object')
            if (data.get('fault') and self.core.mode == source and
                    data.get('epoch') == self.core.epoch):
                self.core.fault(source+' producer fault')
                return
            if source == 'tcp_ik' and data.get('pause') is True:
                accepted = self.core.pause_tcp(data['epoch'], time.monotonic())
            elif source == 'tcp_ik' and data.get('stop') is True:
                accepted = self.core.stop_tcp(data['epoch'], time.monotonic())
            elif source == 'tool_auto':
                if data.get('cancel') is True:
                    accepted = self.core.cancel_path(data['epoch'], data['execution_id'], time.monotonic())
                elif 'ee_target_rad' in data:
                    accepted = self.core.close_ee(data['epoch'], data['execution_id'],
                                                  data['ee_target_rad'], time.monotonic())
                else:
                    accepted = self.core.path(data['epoch'], data['execution_id'], data['waypoints'],
                                              time.monotonic(), durations=data.get('durations'),
                                              stop_at_waypoints=data.get('stop_at_waypoints'))
            else:
                accepted = self.core.command(source, data['epoch'], data['positions'],
                                             data.get('ee_velocity', 0.), time.monotonic())
            self.last_error = '' if accepted else 'rejected stale or inactive source: '+source
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.last_error = 'invalid '+source+' command: '+str(exc)

    def poll(self, now):
        if self.list_future is not None and self.list_future.done():
            try:
                result = self.list_future.result()
                self.controllers = {c.name: c.state for c in result.controller}
                self.controller_time = now
                for c in result.controller:
                    if c.name == 'manual_position_controller' and c.state == 'active':
                        expected = {j+'/position' for j in JOINTS}
                        if set(c.claimed_interfaces) != expected:
                            self.core.fault('position controller interface mismatch')
                    if c.name == 'arm_controller' and c.state == 'active':
                        self.core.fault('trajectory controller competes with mode manager')
            except Exception as exc:
                self.core.fault('controller query failed: '+str(exc))
            self.list_future = None
        if now-self.last_poll > .2 and self.list_future is None and self.list_client.service_is_ready():
            self.last_poll = now
            self.list_future = self.list_client.call_async(ListControllers.Request())
        fresh = now-self.controller_time < .75
        blockers = []
        if not fresh:
            blockers.append('controller state missing/stale')
        if self.controllers.get('manual_position_controller') != 'inactive':
            blockers.append('position controller must be inactive before startup')
        if self.controllers.get('ee_velocity_controller') != 'active':
            blockers.append('EE velocity controller is not active')
        if not self.core.fresh(now):
            blockers.append('complete position/velocity feedback missing/stale')
        elif max(map(abs, self.core.velocity)) > .02:
            blockers.append('waiting for stopped feedback (max velocity > 0.02 rad/s)')
        if not self.switch_client.service_is_ready():
            blockers.append('switch_controller service unavailable')
        if self.position.get_subscription_count() == 0:
            blockers.append('position command subscriber not discovered')
        self.activation_blockers = blockers if self.activation_started is None else []
        active = (fresh and self.controllers.get('manual_position_controller') == 'active'
                  and self.controllers.get('ee_velocity_controller') == 'active')
        if self.switch_future is not None:
            if self.switch_future.done():
                try:
                    if not self.switch_future.result().ok:
                        self.core.fault('strict activation failed')
                except Exception as exc:
                    self.core.fault('activation failed: '+str(exc))
                self.switch_future = None
            elif not active and now-self.activation_started > 5.:
                self.core.fault('activation timeout')
        if not self.activated and active:
            if self.activation_started is None:
                self.core.fault('position controller must start inactive')
            else:
                self.activated = True
        if (self.activation_started is None and self.core.mode != 'fault' and fresh
                and self.controllers.get('manual_position_controller') == 'inactive'
                and self.controllers.get('ee_velocity_controller') == 'active'
                and self.core.fresh(now) and max(map(abs, self.core.velocity)) <= .02
                and self.switch_client.service_is_ready()
                and self.position.get_subscription_count() > 0):
            self.core.begin_startup(now)
            # Match legacy manual: seed measured position BEFORE asking for activation.
            self.position.publish(Float64MultiArray(data=self.core.target))
            self.activation_started = now
            self.switch_future = self.switch_client.call_async(SwitchController.Request(
                activate_controllers=['manual_position_controller'],
                strictness=SwitchController.Request.STRICT, timeout=Duration(sec=3)))
        self.core.ready = self.activated and active
        if self.activation_started is not None and not self.activated and now-self.activation_started > 5.:
            self.core.fault('activation confirmation timeout')
        if self.activated and not active:
            self.core.fault('controller inactive or state watchdog expired')

    def publish_status(self):
        c = self.core
        now = time.monotonic()
        if c.mode == 'fault':
            blocker = c.reason
        elif not c.fresh(now):
            blocker = 'waiting for complete fresh joint feedback'
        elif not c.ready:
            blocker = '; '.join(self.activation_blockers) or 'waiting for controller activation confirmation'
        elif not c.startup_complete:
            blocker = c.reason or 'startup recovery pending'
        elif c.focus == 'arm' and not c.entry_alignment_complete:
            blocker = c.reason or 'DXL entry alignment pending'
        elif c.focus != 'arm':
            blocker = 'select arm with Options'
        elif c.transitioning:
            blocker = c.reason
        elif c.mode == 'hold':
            if c.reason:
                blocker = c.reason
            elif self.focus_activation_pending and self.focus_arm_mode:
                blocker = 'HOLD: activating '+self.focus_arm_mode
            elif self.focus_arm_mode:
                blocker = ('HOLD: toggle drive -> arm or request /arm/set_mode/' +
                           self.focus_arm_mode)
            else:
                blocker = 'HOLD: request an arm control mode'
        else:
            blocker = ''
        report = (c.mode, c.ready, c.startup_complete, blocker)
        if report != self.last_report:
            self.get_logger().info('mode=%s ready=%s startup_complete=%s: %s' %
                                   (*report[:3], blocker or 'input enabled'))
            self.last_report = report
        self.mode_pub.publish(String(data=c.mode))
        self.status_pub.publish(String(data=json.dumps(dict(
            mode=c.mode, requested=c.requested, transitioning=c.transitioning,
            focus=c.focus, blocker=blocker,
            feedback_age_sec=now-c.state_time if c.position is not None else None,
            positions_rad=c.position, velocities_rad_s=c.velocity,
            epoch=c.epoch, ready=c.ready, reason=c.reason, controllers=self.controllers,
            startup_recovering=c.startup_recovering, startup_complete=c.startup_complete,
            entry_alignment_complete=c.entry_alignment_complete,
            path_speed_limits_rad_s=c.path_speed_limits,
            startup_goal_rad=c.startup_goal,
            focus_arm_mode=self.focus_arm_mode,
            focus_activation_pending=self.focus_activation_pending,
            activation_blockers=self.activation_blockers,
            controller_age_sec=now-self.controller_time if self.controllers else None,
            command_subscribers=self.position.get_subscription_count(),
            last_execution_id=c.last_execution_id, last_execution_result=c.last_execution_result,
            execution_id=c.execution_id, remaining_waypoints=len(c.queue), last_error=self.last_error,
            path_reference_error_rad=c.path_reference_error,
            path_command_error_rad=c.path_command_error,
            path_clamp_amount_rad=c.path_clamp_amount,
            path_goal_error_rad=c.path_goal_error))))

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        self.poll(now)
        self.activate_pending_focus(now)
        # Seed both sides of activation; ForwardCommandController clears its buffer on activation.
        if (not self.activated and self.activation_started is not None and
                self.core.fresh(now) and self.core.mode != 'fault'):
            self.position.publish(Float64MultiArray(data=self.core.target))
        target = self.core.tick(now, dt)
        if (self.core.mode == 'tool_auto' and
                self.core.path_clamp_amount >= self.path_clamp_warning_rad and
                now-self.last_path_clamp_warning >= 1.):
            self.get_logger().warning(
                'tool path feedback clamp active: reference_error=%.6f command_error=%.6f '
                'clamp=%.6f goal_remaining=%.6f rad' % (
                    self.core.path_reference_error, self.core.path_command_error,
                    self.core.path_clamp_amount, self.core.path_goal_error))
            self.last_path_clamp_warning = now
        if target is not None:
            self.position.publish(Float64MultiArray(data=target))
        enabled = (self.core.ready and not self.core.transitioning and
                   (self.core.mode in ('manual', 'tcp_ik') or
                    (self.core.mode == 'tool_auto' and self.core.ee_goal is not None)))
        self.ee.publish(Float64MultiArray(data=[self.core.ee if enabled else 0.]))
        self.publish_status()


def main():
    rclpy.init()
    node = ControlModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.ee.publish(Float64MultiArray(data=[0.]))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
