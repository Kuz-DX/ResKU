#!/usr/bin/env python3
"""Release every arm motor while retaining ros2_control joint-state feedback.

This transition is deliberately one-way. Restart the arm stack to leave teach
mode so every controller is reseeded from measured positions before torque is
enabled again.
"""
import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from dynamixel_interfaces.msg import DynamixelState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


COMMAND_CONTROLLERS = {
    'manual_position_controller', 'arm_controller',
    'ee_velocity_controller', 'ee_controller',
}
ARM_JOINTS = {
    'base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
    'wrist_roll_joint', 'wrist_yaw_joint', 'ee_joint',
}


def active_command_controllers(controllers):
    return sorted(c.name for c in controllers
                  if c.state == 'active' and c.name in COMMAND_CONTROLLERS)


class TeachMode(Node):
    def __init__(self):
        super().__init__('enter_arm_teach_mode')
        self.list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.dxl_torque = self.create_client(
            SetBool, '/dynamixel_hardware_interface/set_dxl_torque')
        self.feedback_names = set()
        self.feedback_time = -float('inf')
        self.dxl_torque_off = False
        self.dxl_state_time = -float('inf')
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(DynamixelState,
                                 '/dynamixel_hardware_interface/dxl_state',
                                 self.on_dxl_state, 10)

    def on_state(self, message):
        if len(message.name) == len(message.position):
            self.feedback_names = set(message.name)
            self.feedback_time = time.monotonic()

    def on_dxl_state(self, message):
        self.dxl_torque_off = (len(message.id) == 4
                               and len(message.torque_state) == 4
                               and not any(message.torque_state))
        self.dxl_state_time = time.monotonic()

    def call(self, client, request, timeout, label):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{label} service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{label} timed out')
        return future.result()

    def enter(self, timeout):
        listed = self.call(self.list_client, ListControllers.Request(), timeout,
                           'controller list')
        active = active_command_controllers(listed.controller)
        if active:
            request = SwitchController.Request()
            request.deactivate_controllers = active
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = max(1, int(timeout))
            response = self.call(self.switch_client, request, timeout,
                                 'controller deactivation')
            if not response.ok:
                raise RuntimeError('strict controller deactivation was rejected')
        torque = self.call(self.dxl_torque, SetBool.Request(data=False), timeout,
                           'Dynamixel torque-off')
        if not torque.success:
            raise RuntimeError('Dynamixel torque-off rejected: '+torque.message)

        deadline = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            now = time.monotonic()
            if (ARM_JOINTS <= self.feedback_names and now-self.feedback_time < .5
                    and self.dxl_torque_off and now-self.dxl_state_time < .5):
                return active
        raise RuntimeError('fresh arm feedback or verified Dynamixel torque-off unavailable')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=5.)
    parser.add_argument('--yes', action='store_true',
                        help='required acknowledgement that the arm is physically supported')
    args = parser.parse_args()
    if not args.yes:
        parser.error('support the arm against gravity, then pass --yes')
    if not 1. <= args.timeout <= 30.:
        parser.error('--timeout must be in [1, 30] seconds')
    print('WARNING: releasing every arm motor. Physically support the arm.', flush=True)
    rclpy.init()
    node = TeachMode()
    try:
        stopped = node.enter(args.timeout)
        print('TEACH MODE ACTIVE: motor commands stopped; joint feedback remains active.')
        print('Stopped controllers: '+(', '.join(stopped) if stopped else 'none'))
        print('Do not re-enable torque in this process. Restart the arm launch to exit safely.')
        return 0
    except RuntimeError as exc:
        print('Failed to enter teach mode: '+str(exc), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
