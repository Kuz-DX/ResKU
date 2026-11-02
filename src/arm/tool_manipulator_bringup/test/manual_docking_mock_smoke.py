#!/usr/bin/env python3
"""Launch exact manual.launch.py with mock hardware and synthetic BASE poses.

No camera, image generation, detector, PnP or recognition accuracy test. Run in
an isolated ROS_DOMAIN_ID. Refuse any non-GenericSystem hardware description.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from manual_control_common import ArmChain, JOINTS
from manual_docking_core import load_config


def quaternion(matrix):
    vector = cv2.Rodrigues(matrix)[0].reshape(3)
    angle = float(np.linalg.norm(vector))
    return ([0., 0., 0., 1.] if angle < 1e-12 else
            [*(vector/angle*np.sin(angle/2)), float(np.cos(angle/2))])


def joy(button=None, forward=0., axis=None, value=0.):
    msg = Joy(axes=[0., 0., 1., 0., forward, 1., 0., 0.], buttons=[0]*11)
    if button is not None:
        msg.buttons[button] = 1
    if axis is not None:
        msg.axes[axis] = value
    return msg


def main():
    rclpy.init()
    node = rclpy.create_node('manual_docking_mock_smoke')
    samples = []
    ee_commands = []
    node.create_subscription(String, '/manual/diagnostics',
                             lambda msg: samples.append(json.loads(msg.data)), 100)
    node.create_subscription(Float64MultiArray, '/ee_velocity_controller/commands',
                             lambda msg: ee_commands.append(list(msg.data)), 100)
    joy_pub = node.create_publisher(Joy, '/joy', 10)
    tag_pub = node.create_publisher(String, '/manual/docking/mock_tag_pose', 1)
    settings = load_config(ROOT/'config/manual_docking_assist.yaml')
    fixture = None
    results = []

    def spin(seconds, message=None, publish_tag=True, wrong_id=False, stale=False):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            if message is not None:
                message.header.stamp = stamp
                joy_pub.publish(message)
            if publish_tag and fixture is not None:
                data = dict(fixture, stamp=dict(sec=stamp.sec-(1 if stale else 0), nanosec=stamp.nanosec))
                if wrong_id:
                    data['id'] = 1-data['id']
                tag_pub.publish(String(data=json.dumps(data)))
            rclpy.spin_once(node, timeout_sec=.01)

    def wait_for(predicate, timeout=10., message=None):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            spin(.03, joy() if message is None else message)
            if samples and predicate(samples[-1]):
                return
        raise AssertionError('condition timeout; last status: '+json.dumps(samples[-1] if samples else {}))

    def pose():
        return chain.pose_and_jacobian(dict(zip(JOINTS, samples[-1]['measured_rad'])))[0]

    def select(tool):
        combo = joy(button=2 if tool == 0 else 1, axis=2, value=-1.)
        spin(.10, combo, publish_tag=False)

    def make_fixture(tool, lateral, axial):
        # ASSUMED / 임의값: a stationary rack target placed relative to the
        # current virtual TCP. No physical rack/camera extrinsics are invented.
        current = pose()
        goal = current.copy()
        goal[:3, 3] += current[:3, :3] @ np.array([lateral, 0., axial])
        tag = goal @ np.linalg.inv(settings['tools'][tool]['tag_to_tcp'])
        data = dict(synthetic=True, frame_id='base_actuator', id=tool, family='tag36h11',
                    xyz_m=tag[:3, 3].tolist(), xyzw=quaternion(tag[:3, :3]))
        return data, current, goal

    try:
        client = node.create_client(GetParameters, '/controller_manager/get_parameters')
        assert client.wait_for_service(timeout_sec=30.), 'controller manager missing'
        future = client.call_async(GetParameters.Request(names=['robot_description']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        assert future.done() and future.result() is not None
        description = future.result().values[0].string_value
        import xml.etree.ElementTree as ET
        plugins = [p.text for p in ET.fromstring(description).findall('ros2_control/hardware/plugin')]
        assert plugins and set(plugins) == {'mock_components/GenericSystem'}, 'REFUSING physical hardware'
        chain = ArmChain(description)
        wait_for(lambda s: s['controller_ready'] and s['startup_complete'])
        spin(.1, joy(button=9))
        wait_for(lambda s: s['entry_alignment_complete'] and not s['fault'] and s['status'] == 'operator control')
        assert 'apriltag_compressed_node' not in node.get_node_names()
        results.append('PASS: exact manual.launch.py, GenericSystem, no camera/detector')

        # Existing manual input remains functional before the selection event.
        before = np.array(samples[-1]['target_rad'])
        spin(.25, joy(axis=0, value=.5))
        assert abs(samples[-1]['target_rad'][5]-before[5]) > .005
        spin(.15, joy())
        results.append('PASS: ordinary manual wrist jog before assist')

        fixture, initial, goal = make_fixture(0, .008, .040)
        before = np.array(samples[-1]['target_rad'])
        select(0)
        spin(.25, joy(), wrong_id=True)
        assert samples[-1]['docking_assist']['state'] == 'acquiring'
        spin(.20, joy(), stale=True)
        np.testing.assert_allclose(samples[-1]['target_rad'], before, atol=.003)
        wait_for(lambda s: s['docking_assist']['ready'])
        stationary = np.array(samples[-1]['target_rad'])
        spin(.25, joy())
        np.testing.assert_array_equal(samples[-1]['target_rad'], stationary)
        results.append('PASS: tool0 selection rejects wrong/stale tag; fresh samples + neutral required; neutral holds')
        for axis, button, joint in ((0, None, 5), (6, None, 4),
                                    (None, 4, 0), (7, 1, 3)):
            before = np.array(samples[-1]['target_rad'])
            spin(.16, joy(button=button, axis=axis, value=.5))
            delta = np.array(samples[-1]['target_rad'])-before
            assert delta[joint] > .003, (axis, button, delta)
            np.testing.assert_allclose(np.delete(delta, joint), 0., atol=1e-10)
            spin(.08, joy())
        before = np.array(samples[-1]['target_rad'])
        ee_start = len(ee_commands)
        spin(.2, joy(button=1, axis=6, value=1.))
        np.testing.assert_array_equal(samples[-1]['target_rad'], before)
        assert any(command[0] > 0 for command in ee_commands[ee_start:])
        spin(.1, joy())
        assert samples[-1]['docking_assist']['input_mapping'] == 'manual'
        results.append('PASS: active assist preserves base, wrist yaw/roll, O+pitch and O+EE mappings')
        # Capture a fresh stationary synthetic target after the direct manual
        # mapping checks, using explicit toggle-off/on rather than a tag jump.
        select(0)
        spin(.15, joy())
        fixture, initial, goal = make_fixture(0, .008, .040)
        select(0)
        wait_for(lambda s: s['docking_assist']['ready'])
        forward = joy(forward=-1.)  # Original manual forward-axis inversion.
        wait_for(lambda s: (s['docking_assist'].get('axial_m', 1.) <= .0011), timeout=35., message=forward)
        position = pose()[:3, 3]
        axial = float((goal[:3, 3]-position) @ goal[:3, 2])
        lateral = np.linalg.norm((goal[:3, 3]-position)-axial*goal[:3, 2])
        assert .0009 <= axial <= .0015 and lateral < .005
        results.append(f'PASS: tool0 original manual approach + tag correction, lateral {lateral:.6f} m, axial clearance {axial:.6f} m')
        spin(.2, joy())
        hold = np.array(samples[-1]['target_rad'])
        spin(.3, joy())
        np.testing.assert_array_equal(samples[-1]['target_rad'], hold)
        spin(.35, joy(forward=1.))
        assert float((goal[:3, 3]-pose()[:3, 3]) @ goal[:3, 2]) > axial+.001
        results.append('PASS: operator retreat and neutral hold at dock reference')

        spin(.4, forward, publish_tag=False)
        assert samples[-1]['docking_assist']['state'] == 'idle'
        spin(.2, forward)
        hold = np.array(samples[-1]['target_rad'])
        spin(.2, forward)
        np.testing.assert_allclose(samples[-1]['target_rad'], hold, atol=1e-10)
        results.append('PASS: tag loss stops; returning tag with held stick does not restart')
        spin(.2, joy())

        fixture, initial, goal = make_fixture(1, .006, .030)
        select(1)
        wait_for(lambda s: s['docking_assist']['ready'])
        spin(2., forward)
        assert samples[-1]['docking_assist']['tool'] == 1
        assert np.linalg.norm(pose()[:3, 3]-initial[:3, 3]) > .002
        spin(.1, joy(button=3))
        assert samples[-1]['docking_assist']['state'] == 'idle'
        assert 'HOME' in samples[-1]['status']
        results.append('PASS: tool1 assistance; square keeps original HOME behavior and clears assist')
        spin(.08, joy())
        spin(.1, joy(button=3))  # Existing second-square cancels HOME.
        spin(.2, joy())

        fixture, _, _ = make_fixture(1, 0., .030)
        select(1)
        wait_for(lambda s: s['docking_assist']['ready'])
        spin(.4, message=None)
        assert samples[-1]['docking_assist']['state'] == 'idle'
        assert samples[-1]['status'] == 'input timeout'
        results.append('PASS: joystick loss stops assistance')
        assert not any(s['fault'] for s in samples), 'unexpected control fault'
        report = dict(results=results, diagnostic_samples=len(samples),
                      camera_recognition_evaluated=False, hardware='mock_components/GenericSystem',
                      assumed_initial_joints_rad=[0., .4, .8, -.7, 0., 0.],
                      assumed_tcp_offsets_m=dict(tool0=[.008, 0., .040], tool1=[.006, 0., .030]),
                      max_observed_target_tracking_error_rad=max(
                          max(abs(e) for e in s['error_rad']) for s in samples if s['error_rad']))
        Path('/tmp/resku_manual_docking_mock_report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # An explicit mock launch is unconditional; this script never connects to
    # the default real-hardware launch. The ROS domain must be isolated by caller.
    log_path = '/tmp/resku_manual_docking_mock_'+os.environ.get('ROS_DOMAIN_ID', '0')+'.log'
    with open(log_path, 'w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'tool_manipulator_bringup', 'manual.launch.py',
            'real_hardware:=false', 'launch_joy:=false', 'docking_mock_input:=true',
            'mock_initial_positions_file:='+str(ROOT/'test/manual_docking_initial_positions.yaml')],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            main()
        finally:
            if process.poll() is None:
                # Signal launch once; it forwards SIGINT to its children.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
            print('Mock launch log: '+log_path)
