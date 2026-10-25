#!/usr/bin/env python3
"""Deadman Cartesian joystick producer for the managed TCP_IK mode."""
import json
import math
from pathlib import Path
import sys
import time

# Source launch support without building; installed packages use their normal import.
source = Path(__file__).resolve().parents[2]/'tool_change_min'
if (source/'tool_change_min/kinematics.py').is_file():
    sys.path.insert(0, str(source))

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetStateValidity
try:
    from tool_change_min.kinematics import ArmKinematics, ARM_JOINTS
except ImportError:
    from arm_shared_kinematics import ArmKinematics, ARM_JOINTS
from tcp_joy_core import TcpStepper, joy_twist, LimitBlocked, SafetyError, TcpMetrics


class TcpJoy(Node):
    def __init__(self):
        super().__init__('tcp_joy')
        def param(name, default):
            return self.declare_parameter(name, default).value
        description = param('robot_description', '')
        hardware = param('hardware_config', '')
        base = param('root_link', 'base_actuator')
        tip = param('controlled_link', 'tcp_link')
        self.model = ArmKinematics('', hardware, base, tip, description=description)
        config = yaml.safe_load(Path(hardware).read_text())
        self.stepper = TcpStepper(self.model,
            [float(config['joints'][j]['velocity_limit_rad_s'])/2 for j in ARM_JOINTS],
            linear=float(param('linear_speed_mps', .02)), angular=float(param('angular_speed_rad_s', .15)),
            margin=float(param('limit_margin_rad', .02)), lead=float(param('max_lead_rad', .08)),
            iterations=int(param('ik_iterations', 30)), budget=float(param('ik_budget_sec', .030)))
        self.pub = self.create_publisher(String, '/arm/input/tcp_ik', 1)
        self.diag = self.create_publisher(String, '/arm/tcp_joy/status', 1)
        self.entry_pub = self.create_publisher(String, '/arm/tcp_joy/entry_status', 1)
        self.entry_pending = None
        self.check = self.create_client(GetStateValidity, '/check_state_validity')
        self.cancel = self.create_client(Trigger, '/arm/cancel')
        self.status, self.epoch = {}, None
        self.status_time = self.state_time = self.joy_time = -math.inf
        self.measured = None
        self.full_state = None
        self.deadman = False
        self.rearmed = False
        self.twist = np.zeros(6)
        self.pending = None
        self.failures = 0
        self.blocked = False
        self.reason = 'waiting for TCP_IK mode'
        self.last_failure_reason = ''
        self.last_failure_time = -math.inf
        self.last_ik_duration_ms = None
        self.metrics = TcpMetrics()
        self.last_logged_state = None
        self.last_time = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/arm/control_status', self.on_status, qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, 1)
        self.create_subscription(Joy, '/joy', self.on_joy, 1)
        self.create_timer(.02, self.tick)

    def discard(self):
        if self.pending:
            for future in self.pending['futures']:
                if not future.done():
                    future.cancel()
        self.pending = None

    def assess_entry(self, now):
        """Read-only assessment in HOLD; never requests motion or changes mode."""
        s = self.status
        eligible = (now-self.status_time < .3 and s.get('mode') == 'hold'
                    and s.get('ready') and s.get('startup_complete')
                    and now-self.state_time < .3 and self.measured is not None)
        if not eligible:
            if self.entry_pending:
                self.entry_pending['future'].cancel()
            self.entry_pending = None
            return

        def report(safe, reason, q):
            self.entry_pub.publish(String(data=json.dumps(dict(
                epoch=s.get('epoch'), safe=safe, reason=reason, positions=q.tolist()))))

        q = self.measured.copy()
        try:
            # Extra 0.001 covers the manager's snapshot comparison tolerance.
            if not self.model.within_limits(q, .101):
                raise ValueError('TCP entry requires 0.10 rad margin; remain HOLD, assess tcp_ready')
            jac = self.model.jacobian(q).copy()
            jac[:3] /= .2
            cond = np.linalg.cond(jac)
            if not math.isfinite(cond) or cond > 20.:
                raise ValueError('TCP entry condition exceeds 20; remain HOLD, assess tcp_ready')
            if not self.check.service_is_ready():
                raise ValueError('TCP entry collision service unavailable; remain HOLD')
            if self.entry_pending:
                p = self.entry_pending
                if (p['epoch'] != s.get('epoch') or np.max(np.abs(q-p['q'])) > .001
                        or self.full_state.keys() != p['full_state'].keys()
                        or any(abs(v-p['full_state'][j]) > .001
                               for j, v in self.full_state.items())):
                    raise ValueError('TCP entry snapshot changed; recheck required')
                if now-p['started'] > .15:
                    raise ValueError('TCP entry collision timeout; remain HOLD')
                if not p['future'].done():
                    return
                response = p['future'].result()
                self.entry_pending = None
                if response is None or not response.valid:
                    raise ValueError('TCP entry collision/state invalid; remain HOLD')
                report(True, 'TCP entry current pose verified', p['q'])
                return
            req = GetStateValidity.Request()
            req.robot_state.joint_state.name = list(self.full_state)
            req.robot_state.joint_state.position = list(self.full_state.values())
            req.robot_state.is_diff = True
            self.entry_pending = dict(future=self.check.call_async(req), q=q,
                                      epoch=s.get('epoch'), started=now,
                                      full_state=dict(self.full_state))
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            if self.entry_pending:
                self.entry_pending['future'].cancel()
            self.entry_pending = None
            report(False, str(exc), q)

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                raise ValueError('manager status must be an object')
            self.status = status
            self.status_time = time.monotonic()
        except ValueError:
            self.status = {}

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        valid = (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                 and all(j in values for j in ARM_JOINTS) and np.all(np.isfinite(msg.position)))
        if not valid:
            self.state_time = -math.inf
            self.safety_stop('invalid or incomplete joint feedback')
            return
        self.measured = np.array([values[j] for j in ARM_JOINTS])
        self.full_state = values
        self.state_time = time.monotonic()

    def safety_stop(self, reason, fault=True):
        """Block immediately and use the manager's existing FAULT/HOLD gateway."""
        self.discard()
        if self.blocked:
            return
        self.blocked = True
        self.rearmed = False
        self.reason = ('FAULT requested: ' if fault else 'HOLD requested: ')+reason
        self.last_failure_reason = reason
        self.last_failure_time = time.monotonic()
        self.metrics.counts['safety_stops'] += 1
        self.metrics.failures[reason] += 1
        if self.epoch is not None:
            if fault:
                self.pub.publish(String(data=json.dumps(dict(epoch=self.epoch, fault=True))))
            elif self.cancel.service_is_ready():
                self.cancel.call_async(Trigger.Request())
        # No heartbeat after stopping. The manager also has independent
        # feedback/controller watchdogs if a request cannot be delivered.

    def on_joy(self, msg):
        self.deadman, self.twist = joy_twist(msg.axes, msg.buttons)
        self.joy_time = time.monotonic()
        if not self.deadman:
            self.discard()
            self.rearmed = True

    def fail(self, reason):
        self.discard()
        self.reason = reason
        self.last_failure_reason = reason
        self.last_failure_time = time.monotonic()
        self.failures += 1
        self.metrics.counts['general_failures'] += 1
        self.metrics.failures[reason] += 1
        if self.failures >= 5:
            self.blocked = True
            self.reason = 'HOLD requested after five failures: '+reason
            if self.cancel.service_is_ready():
                self.cancel.call_async(Trigger.Request())
            # Stop heartbeat as a second stop path: manager command watchdog -> HOLD.

    def publish(self):
        if self.stepper.q is not None and not self.blocked:
            self.pub.publish(String(data=json.dumps(dict(epoch=self.epoch,
                positions=self.stepper.q.tolist(), ee_velocity=0.))))

    def submit_collision(self, q, pose, now):
        if not self.check.service_is_ready():
            raise ValueError('MoveIt /check_state_validity unavailable')
        # Check both measured->target and command->target ramps, including endpoints.
        samples = []
        for start in (self.measured, self.stepper.q):
            count = max(1, math.ceil(float(np.max(np.abs(q-start)))/.005))
            samples.extend(start+(q-start)*i/count for i in range(count+1))
        futures = []
        seen = set()
        for sample in samples:
            key = tuple(map(float, sample))
            if key in seen:
                continue
            seen.add(key)
            req = GetStateValidity.Request()
            # Empty group checks the entire robot, including tool and environment.
            values = dict(self.full_state)
            values.update(zip(ARM_JOINTS, map(float, sample)))
            req.robot_state.joint_state.name = list(values)
            req.robot_state.joint_state.position = list(values.values())
            req.robot_state.is_diff = True
            futures.append(self.check.call_async(req))
        self.pending = dict(q=q, pose=pose, futures=futures, started=time.monotonic(),
                            measured=self.measured.copy(), epoch=self.epoch,
                            full_state=dict(self.full_state))

    def tick(self):
        callback_started = time.perf_counter()
        now = time.monotonic()
        dt, self.last_time = now-self.last_time, now
        self.metrics.observe('interval', dt*1000.)
        s = self.status
        self.assess_entry(now)
        active = (now-self.status_time < .3 and s.get('mode') == 'tcp_ik' and
                  s.get('focus') == 'arm' and s.get('ready') and not s.get('transitioning'))
        if not active:
            self.discard()
            self.epoch = None
            self.rearmed = False
            self.reason = (('blocked after failure: '+self.last_failure_reason)
                           if self.blocked and self.last_failure_reason else
                           'waiting for active TCP_IK mode')
        else:
            if self.epoch != s.get('epoch'):
                self.discard()
                self.epoch = s.get('epoch')
                self.rearmed = False
                self.blocked = False
                self.failures = 0
                self.last_failure_reason = ''
                self.last_failure_time = -math.inf
                self.stepper.q = self.stepper.pose = None
            if now-self.state_time >= .3 or self.measured is None:
                self.safety_stop('joint feedback stale')
            elif not self.blocked:
                try:
                    if self.stepper.q is None:
                        self.stepper.reset(self.measured)
                    if np.max(np.abs(self.stepper.q-self.measured)) > self.stepper.lead:
                        self.safety_stop('feedback lead exceeded', fault=False)
                    elif not self.check.service_is_ready():
                        self.discard()
                        self.rearmed = False
                        self.reason = 'waiting for MoveIt collision service; release L1 when ready'
                    elif now-self.joy_time >= .25 or not self.deadman or not self.rearmed:
                        self.discard()
                        if now-self.joy_time >= .25:
                            self.rearmed = False
                        self.reason = 'deadman released / release L1 to rearm'
                    elif not 0 < dt <= .1:
                        self.fail('timer discontinuity')
                    elif self.pending:
                        p = self.pending
                        if p['epoch'] != self.epoch:
                            self.discard()
                        elif now-p['started'] > .15:
                            self.fail('collision check timeout')
                        elif all(f.done() for f in p['futures']):
                            responses = [f.result() for f in p['futures']]
                            invalid = [r for r in responses if r is None or not r.valid]
                            if invalid:
                                pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2)))
                                                for r in invalid if r is not None
                                                for c in r.contacts})
                                detail = ', '.join(a+'<->'+b for a, b in pairs[:6])
                                self.safety_stop('self/environment collision' +
                                          (': '+detail if detail else ''))
                            elif np.max(np.abs(self.measured-p['measured'])) > .01:
                                self.fail('feedback changed during collision check')
                            elif (self.full_state.keys() != p['full_state'].keys() or
                                  any(abs(v-p['full_state'][j]) > .001
                                      for j, v in self.full_state.items() if j not in ARM_JOINTS)):
                                self.fail('non-arm feedback changed during collision check')
                            elif np.max(np.abs(p['q']-self.measured)) > self.stepper.lead:
                                self.safety_stop('feedback lead exceeded', fault=False)
                            else:
                                self.stepper.commit(p['q'], p['pose'])
                                self.pending = None
                                self.failures = 0
                                self.metrics.counts['accepted_commands'] += 1
                                self.reason = 'TCP incremental command accepted'
                    elif np.any(self.twist):
                        # No catch-up integration while IK/collision service is busy.
                        started = time.monotonic()
                        self.metrics.counts['ik_attempts'] += 1
                        try:
                            q, pose = self.stepper.candidate(
                                self.measured, self.twist, min(dt, .02))
                        finally:
                            self.last_ik_duration_ms = (time.monotonic()-started)*1000.
                            self.metrics.observe('ik', self.last_ik_duration_ms)
                            if self.stepper.last_jacobian_duration_ms is not None:
                                self.metrics.observe('jacobian', self.stepper.last_jacobian_duration_ms)
                        self.submit_collision(q, pose, now)
                    self.publish()
                except LimitBlocked as exc:
                    self.metrics.counts['limit_blocked'] += 1
                    self.discard()
                    self.reason = str(exc)
                    # An infeasible jog direction is not a producer failure.
                    # Keep both the validated joint/pose target and any existing
                    # failure count; reversing the stick may produce a valid step.
                    self.publish()
                except SafetyError as exc:
                    self.safety_stop(str(exc), fault=exc.fault)
                except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
                    self.fail(str(exc))
                    self.publish()
        self.diag.publish(String(data=json.dumps(dict(reason=self.reason, failures=self.failures,
            deadman=self.deadman, rearmed=self.rearmed, pending=self.pending is not None,
            condition=self.stepper.condition if math.isfinite(self.stepper.condition) else None,
            singularity_scale=self.stepper.scale, velocity_scale=self.stepper.velocity_scale,
            ik_duration_ms=self.last_ik_duration_ms, ik_budget_ms=self.stepper.budget*1000.,
            performance=self.metrics.snapshot(),
            last_failure=self.last_failure_reason,
            last_failure_age_sec=(now-self.last_failure_time
                                  if math.isfinite(self.last_failure_time) else None)))))
        logged = (self.reason, self.failures, self.deadman, self.rearmed)
        if logged != self.last_logged_state:
            self.get_logger().info(
                'reason=%s failures=%d deadman=%s rearmed=%s mode=%s focus=%s' %
                (self.reason, self.failures, self.deadman, self.rearmed,
                 s.get('mode', 'missing'), s.get('focus', 'missing')))
            self.last_logged_state = logged
        self.metrics.observe('callback', (time.perf_counter()-callback_started)*1000.)


def main():
    rclpy.init()
    node = TcpJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.discard()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
