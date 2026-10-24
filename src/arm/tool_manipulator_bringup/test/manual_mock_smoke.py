#!/usr/bin/env python3
"""Exercise an already-running mock-only launch; refuse real URDF plugins.

Run on a dedicated ROS_DOMAIN_ID with manual.launch.py (mock default).
This is an explicit integration test, not part of ordinary pytest collection.
"""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def main(expect_recovery=False):
    rclpy.init()
    node = rclpy.create_node('manual_mock_smoke')
    samples = []
    node.create_subscription(String, '/manual/diagnostics',
                             lambda m: samples.append(json.loads(m.data)), 100)
    publisher = node.create_publisher(Joy, '/joy', 10)
    def spin(seconds, message=None):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if message is not None:
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.015)
    def call(client, request):
        assert client.wait_for_service(timeout_sec=8.), 'missing service'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.)
        assert future.done(), 'service timeout'
        return future.result()
    def joy(axis=None, value=0., buttons=()):
        message = Joy(axes=[0.]*8, buttons=[0]*11)
        if axis is not None:
            message.axes[axis] = value
        for button in buttons:
            message.buttons[button] = 1
        return message
    def target():
        return np.array(samples[-1]['target_rad'])
    results = []
    try:
        config = call(node.create_client(GetParameters, '/controller_manager/get_parameters'),
                      GetParameters.Request(names=['robot_description']))
        description = config.values[0].string_value
        assert 'mock_components/GenericSystem' in description, 'REFUSING non-mock hardware'
        assert 'rmd_hardware_interface/' not in description and 'DynamixelHardware' not in description
        deadline = time.monotonic()+20.
        while time.monotonic() < deadline and (not samples or not samples[-1]['controller_ready'] or
                                                not samples[-1].get('startup_complete', False)):
            spin(.1, joy())
        assert samples and samples[-1]['controller_ready'], 'controller not ready'
        assert samples[-1]['controllers'].get('arm_controller') != 'active'
        assert 'servo_node' not in node.get_node_names()
        results.append('PASS: mock-only model, position controller active, no Servo or active JTC')
        if expect_recovery:
            import yaml
            import xml.etree.ElementTree as ET
            names = samples[-1]['joints']
            original = np.array([float(ET.fromstring(description).find(
                f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']").text)
                for name in names])
            hardware = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())['joints']
            expected = original.copy()
            expected[1:4] = np.clip(original[1:4],
                [hardware[n]['soft_limit_rad'][0]+np.deg2rad(2.) for n in names[1:4]],
                [hardware[n]['soft_limit_rad'][1]-np.deg2rad(2.) for n in names[1:4]])
            np.testing.assert_allclose(target(), expected, atol=1e-9)
            np.testing.assert_allclose(samples[-1]['measured_rad'], expected, atol=.005)
            recovery = [np.array(s['target_rad']) for s in samples if s['target_rad'] is not None]
            assert any(s.get('startup_recovering') for s in samples)
            for q in recovery:
                np.testing.assert_allclose(q[[0, 4, 5]], original[[0, 4, 5]], atol=1e-12)
            for a, b in zip(recovery, recovery[1:]):
                assert np.max(np.abs(b-a)) <= .0006+1e-8, 'startup position jump'
            results.append('PASS: actual storage-pose feedback recovered to nearest RMD 2-degree inset; '
                           'DXL unchanged; per-sample increment <= 0.0006 rad')
            print('\n'.join(results), flush=True)
            return
        # Options rising edge, then neutral acknowledgement.
        spin(.08, joy(buttons=(9,)))
        spin(.15, joy())
        assert samples[-1]['focus'] == 'arm'
        for index, axis, buttons, sign in ((0, None, (4,), 1.), (0, None, (5,), -1.),
                                           (4, 6, (), 1.), (4, 6, (), -1.),
                                           (5, 0, (), 1.), (5, 0, (), -1.)):
            spin(.08, joy())
            before = target()
            spin(.22, joy(axis, sign, buttons))
            after = target()
            direction = -sign if index == 5 else sign
            assert direction*(after[index]-before[index]) > .008, (index, before, after)
            np.testing.assert_allclose((after-before)[[j for j in range(6) if j != index]], 0., atol=1e-9)
            spin(.08, joy())
            np.testing.assert_allclose(samples[-1]['measured_rad'], after, atol=.003)
            results.append(f'PASS: DXL joint index {index}, direction {direction:+g}, isolated target and mock tracking')
        before = target()
        spin(.24, joy(4, 1.))
        bend = target()-before
        assert np.linalg.norm(bend[1:4]) > .001
        np.testing.assert_allclose(bend[[0, 4, 5]], 0., atol=1e-9)
        spin(.1, joy())
        before = target()
        spin(.24, joy(4, -1.))
        reverse = target()-before
        assert np.dot(bend[1:4], reverse[1:4]) < 0
        results.append('PASS: actual ResKU URDF RMD bend/reverse, unchanged DXL targets')
        spin(.1, joy())
        before = target()
        combined = joy(4, 1., (4,))
        combined.axes[0], combined.axes[6] = 1., 1.
        spin(.24, combined)
        delta = target()-before
        assert delta[0] > .008 and delta[4] > .008 and delta[5] < -.008
        assert np.linalg.norm(delta[1:4]) > .001
        results.append('PASS: RMD and all DXL simultaneous inputs')
        spin(.1, joy())
        before = target()
        spin(.18, joy())
        np.testing.assert_allclose(target(), before, atol=1e-9)
        spin(.15, joy(7, 1., (1,)))
        np.testing.assert_allclose(target(), before, atol=1e-9)
        spin(.1, joy())
        results.append('PASS: neutral hold and separate EE input leave arm targets unchanged')
        spin(.45)
        assert samples[-1]['status'] == 'input timeout'
        before = target()
        spin(.12, joy(0, 1.))
        np.testing.assert_allclose(target(), before, atol=.003)
        spin(.15, joy())
        assert samples[-1]['status'] == 'operator control'
        results.append('PASS: input disconnect and neutral-before-resume')
        call(node.create_client(SwitchController, '/controller_manager/switch_controller'),
             SwitchController.Request(deactivate_controllers=['manual_position_controller'],
                                      strictness=2, timeout=Duration(sec=2)))
        spin(.45, joy())
        assert samples[-1]['phase'] == 'failed' and not samples[-1]['controller_ready']
        results.append('PASS: runtime controller deactivation blocks movement')
        periods = [s['dt_sec'] for s in samples if s['dt_sec'] > 0]
        results.append(f'Observed timer median {np.median(periods):.6f} s; samples={len(samples)}')
        print('\n'.join(results), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch-mock', action='store_true')
    parser.add_argument('--mock-initial-positions', default='')
    parser.add_argument('--expect-startup-recovery', action='store_true')
    args = parser.parse_args()
    process = None
    log = None
    try:
        if args.launch_mock:
            log = open('/tmp/resku_manual_mock_' + os.environ.get('ROS_DOMAIN_ID', '0') + '.log', 'w')
            command = ['ros2', 'launch', 'tool_manipulator_bringup', 'manual.launch.py',
                       'real_hardware:=false', 'launch_joy:=false']
            if args.mock_initial_positions:
                command.append('mock_initial_positions_file:=' + str(Path(args.mock_initial_positions).resolve(strict=True)))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        main(args.expect_startup_recovery)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.)
        if log is not None:
            log.close()
