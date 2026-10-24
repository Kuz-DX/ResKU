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
        limits = load_soft_limits(path, JOINTS)
        config = yaml.safe_load(path.read_text())
        speeds = [min(.12, float(config['joints'][j]['velocity_limit_rad_s'])/2) for j in JOINTS]
        self.core = ModeCore([limits[j] for j in JOINTS], speeds, startup_recovery_enabled=True)
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
        self.activation_blockers = []
        self.create_timer(.02, self.tick)
        self.publish_status()

    def on_state(self, msg):
        p = dict(zip(msg.name, msg.position))
        v = dict(zip(msg.name, msg.velocity))
        if all(j in p and j in v for j in JOINTS):
            self.core.feedback([p[j] for j in JOINTS], [v[j] for j in JOINTS], time.monotonic())

    def on_focus(self, msg):
        self.core.focus = msg.data
        if msg.data != 'arm' and self.core.mode != 'fault' and self.core.startup_complete:
            self.core.request('hold', time.monotonic())

    def on_fault(self, msg):
        if msg.data.strip() and msg.data.lower() not in ('ok', 'false', '0', 'none'):
            self.hardware_fault = True
            self.core.fault('hardware fault: '+msg.data)

    def set_mode(self, mode, res):
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
            if source == 'tool_auto':
                accepted = self.core.path(data['epoch'], data['execution_id'], data['waypoints'], time.monotonic())
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
            elif now-self.activation_started > 5.:
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
        elif c.focus != 'arm':
            blocker = 'select arm with Options'
        elif c.transitioning:
            blocker = c.reason
        elif c.mode == 'hold':
            blocker = 'HOLD: request /arm/set_mode/manual to enable joystick control'
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
            startup_goal_rad=c.startup_goal,
            activation_blockers=self.activation_blockers,
            controller_age_sec=now-self.controller_time if self.controllers else None,
            command_subscribers=self.position.get_subscription_count(),
            execution_id=c.execution_id, remaining_waypoints=len(c.queue), last_error=self.last_error))))

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        self.poll(now)
        # Seed both sides of activation; ForwardCommandController clears its buffer on activation.
        if (not self.activated and self.activation_started is not None and
                self.core.fresh(now) and self.core.mode != 'fault'):
            self.position.publish(Float64MultiArray(data=self.core.target))
        target = self.core.tick(now, dt)
        if target is not None:
            self.position.publish(Float64MultiArray(data=target))
        enabled = self.core.ready and self.core.mode in ('manual', 'tcp_ik') and not self.core.transitioning
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
