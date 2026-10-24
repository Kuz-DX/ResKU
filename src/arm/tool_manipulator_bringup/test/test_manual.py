"""Control isolation and fault/activation checks; no hardware access."""
from concurrent.futures import Future
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from manual import Manual, STOP_POSE_RAD, position_limit_violations
from manual_control_common import JOINTS, ArmChain, bounded_step, coordinated_velocity
from test_manual_control_common import harness as base_harness


def harness():
    node = Manual.__new__(Manual)
    node.__dict__.update(base_harness().__dict__)
    node.phase, node.status = 'ready', 'operator control'
    node.controller_ready, node.hardware_fault = True, False
    node.stop_target, node.neutral_required = None, False
    node.stop_pose_active, node.previous_stop_pose = False, False
    node.last_dt = .02
    node.diagnostics = Mock()
    node.joy = Joy(axes=[0.]*8, buttons=[0]*11)
    node.params.update(forward_axis=4, wrist_yaw_axis=0, wrist_roll_axis=6,
                       ee_velocity_axis=6, wrist_pitch_axis=7, vertical_axis=1, base_ccw_button=4, base_cw_button=5,
                       ee_enable_button=1, focus_button=9, stop_pose_button=3, deadzone=.12, manage_focus=True)
    node.poll_controller = Mock()
    node.axis = lambda name: 0.
    node.list_client = Mock()
    node.switch_client = Mock()
    node.parameter_client = Mock()
    node.list_future = node.switch_future = node.parameter_future = None
    node.controller_stamp = 1.
    node.last_poll = 1.
    node.switch_started = None
    node.controllers = {'manual_position_controller': 'active', 'ee_velocity_controller': 'active'}
    node.parameters_verified = True
    node.publisher.get_subscription_count = Mock(return_value=1)
    return node


def tick(node):
    node.last_time = .98
    node.on_timer()


@pytest.mark.parametrize('name,index', [('base', 0), ('wrist_roll_axis', 4), ('wrist_yaw_axis', 5)])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_dxl_bidirectional_accumulation_independence_no_fk(name, index, sign):
    node = harness()
    node.chain.pose_and_jacobian = Mock(side_effect=AssertionError('DXL must not use FK'))
    node.axis = lambda key: sign if key == name else 0.
    node.button = lambda key: (name == 'base' and key == ('base_ccw_button' if sign > 0 else 'base_cw_button'))
    before = node.target.copy()
    for _ in range(10):
        tick(node)
    delta = node.target-before
    expected = np.zeros(6)
    expected[index] = sign*.024*(-1 if index == 5 else 1)
    np.testing.assert_allclose(delta, expected, atol=1e-12)
    node.axis, node.button = lambda _: 0., lambda _: False
    before = node.target.copy()
    tick(node)
    np.testing.assert_array_equal(node.target, before)


def test_rmd_forward_reverse_neutral_and_simultaneous_dxl():
    velocities = []
    for sign in (-1., 1.):
        node = harness()
        node.axis = lambda key: sign if key == 'forward_axis' else 0.
        velocities.append(node.command_velocity(node.measured()))
    np.testing.assert_allclose(velocities[0], -velocities[1])
    np.testing.assert_array_equal(velocities[1][[0, 4, 5]], [0.]*3)
    node = harness()
    node.axis = lambda key: 1. if key in ('forward_axis', 'wrist_roll_axis', 'wrist_yaw_axis') else 0.
    node.button = lambda key: key == 'base_ccw_button'
    combined = node.command_velocity(node.measured())
    np.testing.assert_allclose(combined[1:4], velocities[1][1:4])
    before = node.target.copy()
    tick(node)
    assert np.all(np.abs(node.target-before) <= node.speeds*.02+1e-12)
    assert np.all(np.abs(node.target-before) > 0)
    node.axis, node.button = lambda _: 0., lambda _: False
    node.chain.pose_and_jacobian = Mock(side_effect=AssertionError('neutral IK'))
    before = node.target.copy()
    tick(node)
    np.testing.assert_array_equal(node.target, before)


@pytest.mark.parametrize('index', [0, 4, 5])
def test_dxl_limit_blocks_only_that_joint(index):
    node = harness()
    node.target[index] = node.upper[index]-.02
    node.positions[JOINTS[index]] = node.target[index]
    node.axis = lambda key: -1. if key == 'wrist_yaw_axis' else (1. if key == 'wrist_roll_axis' else 0.)
    node.button = lambda key: key == 'base_ccw_button'
    before = node.target.copy()
    tick(node)
    assert node.target[index] == before[index]
    for other in {0, 4, 5}-{index}:
        assert node.target[other] > before[other]


@pytest.mark.parametrize('reason', ['joy', 'state', 'fault', 'focus', 'following', 'timer'])
def test_stop_blocks_input_and_zeros_ee(reason):
    node = harness()
    node.axis = lambda _: 1.
    node.button = lambda _: True
    if reason == 'joy':
        node.joy_time = .1
    elif reason == 'state':
        node.state_times['elbow_joint'] = .1
    elif reason == 'fault':
        node.on_hardware_fault(String(data='CAN disconnected'))
    elif reason == 'focus':
        node.focus = 'drive'
    elif reason == 'following':
        node.target[1] += .15
    else:
        node.last_time = .1
    before = node.target.copy()
    node.on_timer()
    assert node.neutral_required
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]
    assert np.all(np.abs(node.target-before) <= node.speeds*.02+1e-12)
    if reason in ('fault', 'following'):
        assert node.fault


def test_timeout_hold_removes_lead_gradually_and_needs_neutral_rearm():
    node = harness()
    node.target[0] += .05
    node.joy_time = .1
    previous = node.target.copy()
    tick(node)
    assert node.target[0] < previous[0]
    assert abs(node.target[0]-previous[0]) <= .0024+1e-12
    node.joy_time = 1.
    node.axis = lambda _: 1.
    for _ in range(25):
        tick(node)
    np.testing.assert_allclose(node.target, node.measured())
    assert node.neutral_required
    node.axis = lambda _: 0.
    tick(node)
    assert not node.neutral_required


def test_malformed_joy_feedback_and_hardware_fault_cannot_be_cleared_by_focus():
    node = harness()
    node.on_joy(Joy(axes=[float('nan')]*8, buttons=[0]*11))
    assert node.joy_time is None
    node.on_state(JointState(name=['base_joint'], position=[]))
    assert node.state_times == {}
    node.on_hardware_fault(String(data='fault'))
    node.on_focus(String(data='drive'))
    assert node.fault and node.hardware_fault


def finished(value):
    future = Future()
    future.set_result(value)
    return future


def test_controller_failure_and_lost_state_block_all_arm_publication():
    for kind in ('switch', 'watchdog', 'arm_conflict', 'inactive', 'ee_inactive'):
        node = harness()
        node.poll_controller = Manual.poll_controller.__get__(node)
        if kind == 'switch':
            node.phase = 'switching'
            node.switch_future = finished(SimpleNamespace(ok=False))
        elif kind == 'watchdog':
            node.controller_stamp = 0.
        elif kind == 'ee_inactive':
            node.list_future = finished(SimpleNamespace(controller=[SimpleNamespace(
                name='manual_position_controller', state='active',
                type='forward_command_controller/ForwardCommandController'), SimpleNamespace(
                name='ee_velocity_controller', state='inactive',
                type='forward_command_controller/ForwardCommandController')]))
        else:
            node.list_future = finished(SimpleNamespace(controller=[SimpleNamespace(
                name='arm_controller' if kind == 'arm_conflict' else 'manual_position_controller',
                state='active' if kind == 'arm_conflict' else 'inactive',
                type='forward_command_controller/ForwardCommandController')]))
        tick(node)
        assert node.phase == 'failed'
        assert not node.controller_ready
        node.publisher.publish.assert_not_called()


def test_activation_seeds_nonzero_measured_before_switch_and_requires_confirmation():
    node = harness()
    node.poll_controller = Manual.poll_controller.__get__(node)
    node.phase, node.controller_ready = 'waiting', False
    node.controllers['manual_position_controller'] = 'inactive'
    node.target = None
    node.axis = lambda key: 1. if key == 'forward_axis' else 0.
    future = Future()
    def activate(request):
        np.testing.assert_array_equal(node.publisher.publish.call_args.args[0].data, node.measured())
        assert request.activate_controllers == ['manual_position_controller']
        assert request.strictness == request.STRICT
        return future
    node.switch_client.call_async = Mock(side_effect=activate)
    tick(node)
    assert node.phase == 'switching' and not node.controller_ready
    np.testing.assert_array_equal(node.target, node.measured())
    future.set_result(SimpleNamespace(ok=True))
    tick(node)
    assert node.phase == 'confirming' and not node.controller_ready
    node.list_future = finished(SimpleNamespace(controller=[SimpleNamespace(
        name='manual_position_controller', state='active',
        type='forward_command_controller/ForwardCommandController'), SimpleNamespace(
            name='ee_velocity_controller', state='active',
            type='forward_command_controller/ForwardCommandController')]))
    tick(node)
    assert node.controller_ready and node.neutral_required


def test_wrong_joint_order_prevents_activation():
    node = harness()
    node.poll_controller = Manual.poll_controller.__get__(node)
    node.phase, node.controller_ready = 'waiting', False
    node.parameters_verified = False
    node.parameter_future = finished(SimpleNamespace(values=[
        SimpleNamespace(string_array_value=list(reversed(JOINTS))), SimpleNamespace(string_value='position')]))
    tick(node)
    assert node.phase == 'failed'
    node.switch_client.call_async.assert_not_called()


def test_reported_real_startup_pose_is_blocked_with_exact_joint_reasons():
    # Read-only snapshot from the user's stalled real launch. No mock motion
    # or replacement of calibrated hardware limits is needed to reproduce it.
    import yaml
    hardware = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())
    node = harness()
    node.lower = np.array([hardware['joints'][n]['soft_limit_rad'][0] for n in JOINTS])
    node.upper = np.array([hardware['joints'][n]['soft_limit_rad'][1] for n in JOINTS])
    node.positions = dict(zip(JOINTS, [-.016873788666949, 1.5502014362950414,
        1.5362387490158649, -1.5997687231358142, .0322135965453918, -.027611654181734835]))
    node.phase, node.controller_ready, node.target = 'waiting', False, None
    node.controllers['manual_position_controller'] = 'inactive'
    node.poll_controller = Manual.poll_controller.__get__(node)
    tick(node)
    assert not node.controller_ready and node.phase == 'waiting'
    assert node.target is None
    node.switch_client.call_async.assert_not_called()
    node.publisher.publish.assert_not_called()
    assert 'shoulder_joint=1.550201' in node.status
    assert 'wrist_pitch_joint=-1.599769' in node.status
    assert 'excess=+1.841 deg' in node.status
    assert 'excess=-0.421 deg' in node.status
    node.get_logger().warning.assert_called_once()
    import json
    diagnostic = json.loads(node.diagnostics.publish.call_args.args[0].data)
    assert len(diagnostic['position_limit_violations']) == 2
    assert diagnostic['activation_blockers']


def test_healthy_pose_has_no_measured_limit_violation():
    node = harness()
    assert position_limit_violations(node.positions, node.lower, node.upper) == []


def storage_pose_harness():
    import math
    import yaml
    node = harness()
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hardware.yaml').read_text())['joints']
    node.lower = np.array([config[n]['soft_limit_rad'][0] for n in JOINTS])
    node.upper = np.array([config[n]['soft_limit_rad'][1] for n in JOINTS])
    node.positions = dict(zip(JOINTS, [-.016873788666949, 1.5502014362950414,
        1.5362387490158649, -1.5997687231358142, .0322135965453918, -.027611654181734835]))
    node.target = node.measured()
    node.params.update(startup_recovery_enabled=True, startup_recovery_margin_rad=math.radians(2.),
                       startup_recovery_speed_rad_s=.03, startup_recovery_max_excursion_rad=math.radians(5.),
                       startup_recovery_timeout_sec=15.)
    node.startup_goal = node.make_startup_goal(node.target)
    node.startup_recovering = True
    node.startup_complete = False
    node.startup_recovery_started = None
    return node


def test_storage_pose_recovery_reaches_two_degree_inset_without_dxl_or_ik():
    node = storage_pose_harness()
    original = node.target.copy()
    node.chain.pose_and_jacobian = Mock(side_effect=AssertionError('startup must not use IK'))
    node.focus = 'drive'
    now = 1.
    for _ in range(250):
        now += .02
        node.now_seconds = lambda: now
        node.joy_time = None  # startup must finish even before joystick connection
        node.state_times = dict.fromkeys(JOINTS, now)
        node.last_time = now-.02
        before = node.target.copy()
        node.on_timer()
        delta = node.target-before
        assert np.all(np.abs(delta[1:4]) <= .03*.02+1e-12)
        assert np.all(delta[1:4]*(node.startup_goal[1:4]-before[1:4]) >= -1e-12)
        np.testing.assert_array_equal(node.target[[0, 4, 5]], original[[0, 4, 5]])
        node.positions = dict(zip(JOINTS, node.target))  # ideal hardware feedback
        if node.startup_complete:
            break
    assert node.startup_complete and not node.startup_recovering
    expected = np.clip(original[1:4], node.lower[1:4]+np.deg2rad(2.), node.upper[1:4]-np.deg2rad(2.))
    np.testing.assert_allclose(node.target[1:4], expected, atol=1e-12)
    assert node.valid_measured(now)
    assert node.neutral_required


@pytest.mark.parametrize('blocked', ['dxl_outside', 'rmd_far_outside'])
def test_storage_pose_recovery_startup_gates(blocked):
    node = storage_pose_harness()
    node.poll_controller = Manual.poll_controller.__get__(node)
    node.phase, node.controller_ready, node.target = 'waiting', False, None
    node.controllers['manual_position_controller'] = 'inactive'
    if blocked == 'dxl_outside':
        node.positions['base_joint'] = node.upper[0]+.001
    else:
        node.positions['shoulder_joint'] = node.upper[1]+.2
    tick(node)
    node.switch_client.call_async.assert_not_called()
    node.publisher.publish.assert_not_called()
    assert node.phase == 'waiting' and node.activation_blockers


def test_storage_pose_activation_seeds_real_measurement_without_boundary_jump():
    node = storage_pose_harness()
    node.poll_controller = Manual.poll_controller.__get__(node)
    node.phase, node.controller_ready, node.target = 'waiting', False, None
    node.controllers['manual_position_controller'] = 'inactive'
    node.switch_client.call_async = Mock(return_value=Future())
    tick(node)
    assert node.phase == 'switching' and node.startup_recovering
    np.testing.assert_array_equal(node.publisher.publish.call_args.args[0].data, node.measured())
    assert not node.valid_measured(1.) and node.startup_feedback_allowed(1.)
    assert node.target[1] > node.upper[1]  # never snap to boundary on activation


def test_recovery_timeout_blocks_and_does_not_return_to_automatic_recovery_after_success():
    node = storage_pose_harness()
    node.startup_recovery_started = -20.
    tick(node)
    assert node.phase == 'failed'
    assert not node.controller_ready
    node = storage_pose_harness()
    node.startup_recovering, node.startup_complete = False, True
    tick(node)
    assert node.fault
    assert not node.startup_recovering


@pytest.mark.parametrize('reason', ['feedback_timeout', 'hardware_fault'])
def test_running_startup_recovery_pauses_without_new_position_increments(reason):
    node = storage_pose_harness()
    if reason == 'feedback_timeout':
        node.state_times['shoulder_joint'] = .1
    else:
        node.on_hardware_fault(String(data='test fault'))
    before = node.target.copy()
    tick(node)
    np.testing.assert_array_equal(node.target, before)
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]
    assert not node.startup_complete


@pytest.mark.parametrize('joy_state', ['absent', 'stale', 'deflected'])
def test_startup_activation_and_recovery_do_not_wait_for_or_follow_joystick(joy_state):
    node = storage_pose_harness()
    if joy_state == 'absent':
        node.joy_time = None
    elif joy_state == 'stale':
        node.joy_time = .1
    else:
        node.axis = lambda _: 1.
        node.button = lambda _: True
    node.poll_controller = Manual.poll_controller.__get__(node)
    node.phase, node.controller_ready, node.target = 'waiting', False, None
    node.controllers['manual_position_controller'] = 'inactive'
    node.switch_client.call_async = Mock(return_value=Future())
    tick(node)
    assert node.phase == 'switching' and node.startup_recovering
    assert not node.activation_blockers
    node.phase, node.controller_ready = 'ready', True
    node.poll_controller = Mock()
    node.command_velocity = Mock(side_effect=AssertionError('joystick mapping during startup'))
    before = node.target.copy()
    tick(node)
    assert np.any(np.abs(node.target[1:4]-before[1:4]) > 0)
    np.testing.assert_array_equal(node.target[[0, 4, 5]], before[[0, 4, 5]])
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]


@pytest.mark.parametrize('joy_state', ['absent', 'stale', 'deflected'])
def test_manual_input_stays_blocked_after_startup_until_fresh_neutral_joystick(joy_state):
    node = storage_pose_harness()
    node.target = node.startup_goal.copy()
    node.positions = dict(zip(JOINTS, node.target))
    node.startup_recovering, node.startup_complete = False, True
    node.neutral_required = True
    node.focus = 'arm'
    if joy_state == 'absent':
        node.joy_time = None
    elif joy_state == 'stale':
        node.joy_time = .1
    else:
        node.axis = lambda _: 1.
        node.button = lambda _: True
    before = node.target.copy()
    node.command_velocity = Mock(side_effect=AssertionError('manual motion before neutral rearm'))
    tick(node)
    np.testing.assert_array_equal(node.target, before)
    assert node.neutral_required
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]


@pytest.fixture(scope='module')
def resku_chain():
    # Expand local URDF files only. No ROS node, driver, launch or hardware.
    from moveit_configs_utils import MoveItConfigsBuilder
    model = MoveItConfigsBuilder('tool_manipulator',
                                package_name='tool_manipulator_moveit_config').to_moveit_configs()
    return ArmChain(model.robot_description['robot_description'])


@pytest.mark.parametrize('direction', [-1., 1.])
def test_rmd_storage_pose_no_longer_freezes_all_axes_at_first_boundary(resku_chain, direction):
    node = storage_pose_harness()
    node.chain = resku_chain
    node.target = node.startup_goal.copy()
    node.positions = dict(zip(JOINTS, node.target))
    node.startup_recovering, node.startup_complete = False, True
    node.neutral_required = False
    node.axis = lambda name: direction if name == 'forward_axis' else 0.
    start = node.target.copy()
    baseline = start.copy()
    baseline_tail = []
    now = 1.
    for frame in range(500):
        now += .02
        # Reproduce the removed whole-RMD scaling using the identical model,
        # direction, limits and ideal position feedback, isolating that policy.
        _, jacobian = resku_chain.pose_and_jacobian(dict(zip(JOINTS, baseline)))
        requested = .03*direction*np.array([np.cos(baseline[0]), np.sin(baseline[0]), 0.])
        velocity = coordinated_velocity(jacobian, requested, .05, node.speeds[1:4])
        baseline[1:4] = bounded_step(baseline[1:4], velocity, node.lower[1:4], node.upper[1:4], .02, .02)
        if frame >= 400:
            baseline_tail.append(baseline.copy())
        node.now_seconds = lambda: now
        node.joy_time = now
        node.state_times = dict.fromkeys(JOINTS, now)
        node.last_time = now-.02
        before = node.target.copy()
        node.on_timer()
        assert not node.fault
        assert np.all(np.abs(node.target-before) <= node.speeds*.02+1e-12)
        assert np.all(node.target >= node.lower+.02-1e-12)
        assert np.all(node.target <= node.upper-.02+1e-12)
        np.testing.assert_array_equal(node.target[[0, 4, 5]], start[[0, 4, 5]])
        node.positions = dict(zip(JOINTS, node.target))  # numerical feedback only
    np.testing.assert_allclose(baseline_tail[-1], baseline_tail[0], atol=1e-10)
    assert np.max(np.abs(baseline[1:4]-start[1:4])) < .05
    assert np.max(np.abs(node.target[1:4]-start[1:4])) > .1
    print(f'RMD {direction:+g}: old grouped delta={baseline[1:4]-start[1:4]}, '
          f'new delta={node.target[1:4]-start[1:4]}')


@pytest.mark.parametrize('index', [1, 2, 3])
def test_rmd_limit_keeps_other_rmd_and_dxl_inputs_running(index):
    node = harness()
    node.target[index] = node.upper[index]-.02
    node.positions[JOINTS[index]] = node.target[index]
    node.command_velocity = Mock(return_value=np.array([.12, .1, .1, .1, .12, -.12]))
    before = node.target.copy()
    tick(node)
    assert node.target[index] == before[index]
    for other in {1, 2, 3}-{index}:
        assert node.target[other] > before[other]
    assert np.all(np.abs(node.target[[0, 4, 5]]-before[[0, 4, 5]]) > 0)
    assert JOINTS[index] in node.rmd_limit_blocked_joints


def test_rmd_directional_gate_matches_legacy_servo_branch_and_allows_reverse():
    from manual_control_common import ManualController
    node = harness()
    node.axis = lambda name: 1. if name == 'forward_axis' else 0.
    measured = node.measured()
    measured[1], measured[2] = node.upper[1]-.02, node.lower[2]+.02
    with patch.object(ManualController, 'command_velocity', return_value=np.array([0., .1, -.1, .05, 0., 0.])):
        result = node.command_velocity(measured)
    np.testing.assert_allclose(result[1:4], [0., 0., .05])
    assert node.rmd_limit_blocked_joints == ['shoulder_joint', 'elbow_joint']
    with patch.object(ManualController, 'command_velocity', return_value=np.array([0., -.1, .1, -.05, 0., 0.])):
        result = node.command_velocity(measured)
    np.testing.assert_allclose(result[1:4], [-.1, .1, -.05])


def test_one_rmd_lead_budget_cannot_freeze_the_other_two():
    node = harness()
    node.target[1] += .099
    node.command_velocity = Mock(return_value=np.array([0., .1, .1, .1, 0., 0.]))
    before = node.target.copy()
    tick(node)
    assert node.target[1] == before[1]
    assert node.target[2] > before[2] and node.target[3] > before[3]
    assert node.rmd_lead_blocked_joints == ['shoulder_joint']


def test_square_ramps_exact_stop_pose_without_ik_and_stops_on_timeout():
    node = harness()
    node.chain.pose_and_jacobian = Mock(side_effect=AssertionError('pose uses joint space'))
    node.button = lambda name: bool(node.joy.buttons[node.params[name]])
    node.previous_focus = False
    node.on_joy(Joy(axes=[0.]*8, buttons=[0, 0, 0, 1]+[0]*7))
    assert node.stop_pose_active
    for _ in range(2000):
        before = node.target.copy()
        tick(node)
        assert np.all(np.abs(node.target-before) <= node.speeds*.02+1e-12)
        node.positions = dict(zip(JOINTS, node.target))
        if not node.stop_pose_active:
            break
    np.testing.assert_allclose(node.target, STOP_POSE_RAD, atol=1e-12)
    assert node.neutral_required
    node = harness()
    node.stop_pose_active = True
    node.joy_time = .1
    tick(node)
    assert not node.stop_pose_active


def test_square_accepts_stop_pose_within_real_hardware_limits():
    node = storage_pose_harness()
    node.startup_recovering = False
    node.button = lambda name: bool(node.joy.buttons[node.params[name]])
    node.previous_focus = False
    node.on_joy(Joy(axes=[0.]*8, buttons=[0, 0, 0, 1]+[0]*7))
    assert node.stop_pose_active
    assert STOP_POSE_RAD[1] == pytest.approx(np.deg2rad(85.))
    assert STOP_POSE_RAD[2] == pytest.approx(np.deg2rad(87.))
    assert STOP_POSE_RAD[3] == pytest.approx(np.deg2rad(-89.))
    node.get_logger().error.assert_not_called()


@pytest.mark.parametrize('sign', [-1., 1.])
def test_circle_layer_real_joy_mapping(sign):
    from manual_control_common import ManualController
    node = harness()
    node.axis = ManualController.axis.__get__(node)
    node.button = ManualController.button.__get__(node)
    node.joy.buttons[1] = 1
    node.joy.axes[6] = sign
    np.testing.assert_array_equal(node.command_velocity(node.measured()), np.zeros(6))
    assert node.ee_velocity() == pytest.approx(sign*.12)
    node.joy.axes[6] = 0.
    node.joy.axes[7] = sign
    velocity = node.command_velocity(node.measured())
    np.testing.assert_allclose(velocity, [0., 0., 0., sign*.12, 0., 0.])
    assert node.ee_velocity() == 0.
    node.joy.axes[7] = 0.
    node.joy.axes[1] = sign
    velocity = node.command_velocity(node.measured())
    _, linear, angular = node.chain.pose_and_jacobian(node.positions, include_angular=True)
    np.testing.assert_allclose(angular @ velocity[1:4], 0., atol=1e-12)
    np.testing.assert_allclose((linear @ velocity[1:4])[:2], 0., atol=1e-12)
    assert sign*(linear @ velocity[1:4])[2] > 0.
    assert not node.neutral()
    node.joy.buttons[1] = 0
    np.testing.assert_array_equal(node.command_velocity(node.measured()), np.zeros(6))


def test_pitch_adjust_then_vertical_keeps_adjusted_orientation():
    node = harness()
    node.button = lambda name: name == 'ee_enable_button'
    node.axis = lambda name: 1. if name == 'wrist_pitch_axis' else 0.
    for _ in range(20):
        tick(node)
        node.positions = dict(zip(JOINTS, node.target))
    initial, _ = node.chain.pose_and_jacobian(node.positions)
    node.axis = lambda name: 1. if name == 'vertical_axis' else 0.
    for _ in range(100):
        tick(node)
        node.positions = dict(zip(JOINTS, node.target))
    final, _ = node.chain.pose_and_jacobian(node.positions)
    np.testing.assert_allclose(final[:3, :3], initial[:3, :3], atol=1e-10)
    np.testing.assert_allclose(final[:2, 3], initial[:2, 3], atol=1e-4)
    assert final[2, 3] > initial[2, 3]+.001


@pytest.mark.parametrize('block', ['limit', 'lead'])
def test_vertical_constraint_survives_joint_limits_and_lead(block):
    node = harness()
    node.button = lambda name: name == 'ee_enable_button'
    node.axis = lambda name: 1. if name == 'vertical_axis' else 0.
    velocity = node.command_velocity(node.measured())
    index = 1+int(np.argmax(np.abs(velocity[1:4])))
    sign = np.sign(velocity[index])
    if block == 'limit':
        if sign > 0:
            node.upper[index] = node.target[index]+.02
        else:
            node.lower[index] = node.target[index]-.02
    else:
        node.positions[JOINTS[index]] = node.target[index]-sign*.0999
    before = node.target.copy()
    tick(node)
    np.testing.assert_array_equal(node.target, before)


def test_vertical_singularity_stops_and_adjustment_has_priority():
    node = harness()
    node.button = lambda name: name == 'ee_enable_button'
    node.axis = lambda name: 1. if name in ('wrist_pitch_axis', 'vertical_axis', 'forward_axis', 'wrist_roll_axis') else 0.
    np.testing.assert_allclose(node.command_velocity(node.measured()), [0., 0., 0., .12, 0., 0.])
    from manual_control_common import vertical_pitch_velocity
    chain = Mock()
    chain.pose_and_jacobian.return_value = (np.eye(4), np.zeros((3, 3)), np.ones((3, 3)))
    np.testing.assert_allclose(vertical_pitch_velocity(chain, {}, .03, .05, np.ones(3)), 0.)


@pytest.mark.parametrize('base', [0., .7])
@pytest.mark.parametrize('direction', [-1., 1.])
def test_vertical_with_actual_robot_model(base, direction):
    import xacro
    from manual_control_common import vertical_pitch_velocity
    path = Path(__file__).resolve().parents[2]/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro'
    chain = ArmChain(xacro.process_file(str(path)).toxml())
    q = np.array([base, -.4, .7, -.2, .3, -.2])
    initial, _ = chain.pose_and_jacobian(dict(zip(JOINTS, q)))
    for _ in range(100):
        velocity = vertical_pitch_velocity(chain, dict(zip(JOINTS, q)), direction*.03, .05, np.full(3, .12))
        assert np.max(np.abs(velocity)) <= .12+1e-12
        q[1:4] += .02*velocity
    final, _ = chain.pose_and_jacobian(dict(zip(JOINTS, q)))
    np.testing.assert_allclose(final[:3, :3], initial[:3, :3], atol=1e-10)
    np.testing.assert_allclose(final[:2, 3], initial[:2, 3], atol=1e-4)
    assert direction*(final[2, 3]-initial[2, 3]) > .001
