#!/usr/bin/env python3
"""Explicit ROS-wire integration test. No hardware/controller/action execution.

Generated project services are unavailable without building. Only their client
TYPES are substituted by Trigger; all arm-call boundaries are overridden and
never called on the wire. Production FSM constructor, request/sequence, Arduino
callbacks and safety methods execute unchanged. This is not an arm-motion test.
Run alone in ROS_DOMAIN_ID=187, ROS_LOCALHOST_ONLY=1.
"""
import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

if os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Require isolated ROS_DOMAIN_ID=187 and ROS_LOCALHOST_ONLY=1')

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
source = PACKAGE/'scripts/tool_change_fsm.py'
tree = ast.parse(source.read_text())
tree.body = [n for n in tree.body if not (
    isinstance(n, ast.ImportFrom) and n.module == 'tool_change_min.srv')]
class MockArmService:
    Request = SimpleNamespace


namespace = dict(__name__='changer_mock_source', ExecuteTrajectory=MockArmService,
                 LinearMoveToPose=MockArmService, MoveNamedPose=MockArmService)
exec(compile(ast.fix_missing_locations(tree), str(source), 'exec'), namespace)
Base = namespace['ToolChangeFsm']


class Fsm(Base):
    def create_client(self, service_type, *args, **kwargs):
        return super().create_client(Trigger if service_type is MockArmService else service_type,
                                     *args, **kwargs)

    def __init__(self):
        self.moves = []
        super().__init__()

    def _named(self, state, pose, timeout):
        if self.cancelled or self.hardware_error:
            raise RuntimeError('mock motion blocked by production safety state')
        self.moves.append((state, time.monotonic()))
        self._publish(state)

    def _cartesian_target(self, state, target, timeout):
        self._publish(state)
        self.docking_goal = [0.]*6
        self.moves.append(('DOCKED_MOCK', time.monotonic()))


class Peer(Node):
    def __init__(self):
        super().__init__('changer_mock_peer')
        self.scenario = 'success'
        self.feedback_enabled = True
        self.drift = False
        self.active = True
        self.commands = []
        self.statuses = []
        self.scheduled = []
        self.cancel_future = None
        self.fault = self.create_publisher(String, '/control/hardware_fault',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.feedback = self.create_publisher(JointState, '/joint_states', 10)
        self.response = self.create_publisher(String, '/tool_changer/status', 10)
        self.request = self.create_publisher(Int32, '/selected_tool_id', 10)
        self.cancel = self.create_client(Trigger, '/tool_change/cancel')
        self.create_subscription(String, '/tool_changer/command', self.command, 10)
        self.create_subscription(String, '/tool_change/status',
                                 lambda msg: self.statuses.append(msg.data), 10)
        self.create_service(ListControllers, '/controller_manager/list_controllers', self.controllers)
        self.create_timer(.02, self.tick)

    def controllers(self, req, res):
        res.controller = [ControllerState(name='arm_controller',
                           state='active' if self.active else 'inactive')]
        return res

    def tick(self):
        if self.feedback_enabled:
            self.feedback.publish(JointState(name=NAMES, position=[.1 if self.drift else 0.]*6,
                                               velocity=[0.]*6))
        now = time.monotonic()
        due = [v for v in self.scheduled if v[0] <= now]
        self.scheduled = [v for v in self.scheduled if v[0] > now]
        for _, msg in due:
            self.response.publish(String(data=json.dumps(msg)))

    def command(self, msg):
        d = json.loads(msg.data)
        self.commands.append((d, time.monotonic()))
        done = 'UNLOCKED' if d['command'] == 'UNLOCK' else 'LOCKED'
        if d['command'] == 'STOP':
            # A late completion after STOP must not restart the sequence.
            self.response.publish(String(data=json.dumps(dict(id=d['id'], state='LOCKED'))))
            return
        if self.scenario in ('success', 'unlock', 'wrong_then_success'):
            self.response.publish(String(data='malformed JSON'))
            self.response.publish(String(data=json.dumps(dict(id='old', state=done))))
            self.response.publish(String(data=json.dumps(dict(id=d['id'], state='MOVING'))))
            self.response.publish(String(data=json.dumps(dict(id=d['id'],
                state='LOCKED' if done == 'UNLOCKED' else 'UNLOCKED'))))
            response = dict(id=d['id'], state=done)
            self.scheduled.extend([(time.monotonic()+.15, response), (time.monotonic()+.18, response)])
        elif self.scenario == 'error':
            self.response.publish(String(data=json.dumps(dict(id=d['id'], state='ERROR', detail='mock jam'))))
        elif self.scenario == 'cancel':
            self.cancel_future = self.cancel.call_async(Trigger.Request())
        elif self.scenario == 'drift':
            self.drift = True
        elif self.scenario == 'feedback_loss':
            self.feedback_enabled = False
        elif self.scenario == 'controller_loss':
            self.active = False
        elif self.scenario == 'hardware_fault':
            self.fault.publish(String(data='mock fault'))
        # timeout: deliberately no response


NAMES = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
         'wrist_roll_joint', 'wrist_yaw_joint']


def wait_for(predicate, seconds=4.):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('mock condition timed out')


def main():
    config = yaml.safe_load((PACKAGE/'config/poses.yaml').read_text())
    def fill(value):
        if value is None: return 1.
        if isinstance(value, dict): return {k: fill(v) for k, v in value.items()}
        if isinstance(value, list): return [fill(v) for v in value]
        return value
    config = fill(config)
    config['development']['stop_after_state'] = 'done'
    config['tool_changer'].update(enabled=True, docking_timeout_s=2.)
    config['timeouts_s']['lock'] = .8
    report = []
    with tempfile.TemporaryDirectory(prefix='changer_mock_') as tmp:
        share = Path(tmp)
        (share/'config').mkdir()
        (share/'config/poses.yaml').write_text(yaml.safe_dump(config))
        namespace['get_package_share_directory'] = lambda name: str(share)
        rclpy.init()
        peer = Peer()
        fsm = Fsm()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(peer)
        executor.add_node(fsm)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            wait_for(lambda: fsm.controller_active and fsm.feedback_snapshot is not None
                     and peer.request.get_subscription_count() > 0
                     and fsm.changer_pub.get_subscription_count() > 0)
            allowed = {'tool_change_fsm', 'changer_mock_peer'}
            assert set(peer.get_node_names()) <= allowed, 'foreign nodes in test domain'
            for scenario in ('success', 'unlock', 'wrong_then_success', 'timeout', 'error',
                             'cancel', 'drift', 'feedback_loss', 'controller_loss', 'hardware_fault'):
                peer.scenario = scenario
                peer.feedback_enabled, peer.drift, peer.active = True, False, True
                peer.commands.clear()
                peer.statuses.clear()
                peer.scheduled.clear()
                fsm.moves.clear()
                fsm.config['tool_changer']['command'] = 'UNLOCK' if scenario == 'unlock' else 'LOCK'
                wait_for(lambda: fsm.controller_active and fsm.feedback_snapshot is not None
                         and time.monotonic()-fsm.feedback_snapshot[0] < .1
                         and fsm.feedback_snapshot[1] == [0.]*6)
                peer.request.publish(Int32(data=1))
                wait_for(lambda: bool(peer.commands))
                success = scenario in ('success', 'unlock', 'wrong_then_success')
                wait_for(lambda: ('DONE' if success else 'HOLD') in peer.statuses)
                time.sleep(.25)  # allow duplicate/late completion to arrive
                commands = [c[0]['command'] for c in peer.commands]
                moves = [m[0] for m in fsm.moves]
                if success:
                    assert commands == ['UNLOCK' if scenario == 'unlock' else 'LOCK']
                    assert moves[-2:] == ['RETURN_DOCKING_WAIT', 'RETURN_HOME']
                    retreat = next(t for name, t in fsm.moves if name == 'RETURN_DOCKING_WAIT')
                    assert retreat-peer.commands[0][1] >= .14, 'retreated before correct completion'
                else:
                    assert commands == ['LOCK', 'STOP'], commands
                    assert peer.commands[0][0]['id'] == peer.commands[1][0]['id']
                    assert not any(m.startswith('RETURN_') for m in moves), moves
                if scenario == 'cancel':
                    wait_for(lambda: peer.cancel_future.done())
                    assert peer.cancel_future.result().success
                report.append(dict(scenario=scenario, result='PASS', commands=commands,
                                   moves=moves, final_state=fsm.state))
                print('PASS '+scenario, flush=True)
            Path('/tmp/changer_ros_mock_results.json').write_text(json.dumps(dict(
                scope='actual ROS topics/services; mocked arm/IK/controller state',
                domain=187, no_hardware=True, results=report), indent=2))
        finally:
            executor.shutdown(timeout_sec=3.)
            thread.join(timeout=3.)
            fsm.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
