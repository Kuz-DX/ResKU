"""Synthetic geometry/control tests. No camera, detector or recognition scoring."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Joy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from manual_control_common import JOINTS, ArmChain, rotation
from manual_docking_core import DockingAssist, load_config, joint_velocity
from manual_docking import ManualDocking
from test_manual import harness, tick


@pytest.fixture
def settings():
    return load_config(ROOT/'config/manual_docking_assist.yaml')


def acquire(core, target, tool=0, now=1.):
    core.begin(tool, now-.1)
    tag = target @ np.linalg.inv(core.settings['tools'][tool]['tag_to_tcp'])
    for i in range(core.cfg['stable_samples']):
        core.observe(tag, i+1, now+i*.01, 0.)
    return tag


@pytest.fixture(scope='module')
def chain():
    import xacro
    return ArmChain(xacro.process_file(str(
        ROOT.parent/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro')).toxml())


@pytest.mark.parametrize('tool', [0, 1])
def test_neutral_is_exact_hold_and_forward_is_operator_only(settings, tool):
    core = DockingAssist(settings)
    current, target = np.eye(4), np.eye(4)
    target[:3, 3] = [.01, -.008, .05]  # ASSUMED synthetic dock, metres.
    target[:3, :3] = rotation([0, 0, 1], .1)
    acquire(core, target, tool)
    np.testing.assert_array_equal(core.assist_twist(current, np.zeros(6), 1.03, .02), np.zeros(6))
    velocity = core.assist_twist(current, [0, 0, .01, 0, 0, 0], 1.04, .02)
    assert abs(velocity[2]) < 1e-12  # alignment gate, not automatic insertion
    assert velocity[0] > 0 and velocity[1] < 0 and velocity[5] > 0
    assert np.linalg.norm(velocity[:2]) <= core.cfg['max_lateral_correction_m_s']+1e-12
    assert np.linalg.norm(velocity[3:]) <= core.cfg['max_angular_correction_rad_s']+1e-12
    retreat = core.assist_twist(current, [0, 0, -.01, 0, 0, 0], 1.05, .02)
    assert retreat[2] < 0
    np.testing.assert_array_equal(retreat[[0, 1, 3, 4, 5]], np.zeros(5))


def test_manual_root_frame_velocity_is_equivariant(settings):
    current, target = np.eye(4), np.eye(4)
    target[:3, 3] = [0., 0., .05]
    core = DockingAssist(settings)
    acquire(core, target)
    expected = core.assist_twist(current, [.002, .004, .006, 0, 0, 0], 1.03, .02)
    world = np.eye(4)
    world[:3, :3] = rotation([1., 2., 3.], .7)
    world[:3, 3] = [.3, -.1, .2]  # ASSUMED rigid world change, tests frame invariance.
    acquire(core, world @ target)
    actual = core.assist_twist(world @ current, np.r_[world[:3, :3] @ [.002, .004, .006], [0., 0., 0.]], 1.03, .02)
    np.testing.assert_allclose(actual[:3], world[:3, :3] @ expected[:3], atol=1e-12)
    np.testing.assert_allclose(actual[3:], world[:3, :3] @ expected[3:], atol=1e-12)


def test_forward_slowdown_stop_plane_and_oblique_approach_gate(settings):
    current = np.eye(4)
    core = DockingAssist(settings)
    speeds = []
    for distance in (.05, .01, .001, 0., -.002):
        target = np.eye(4)
        target[2, 3] = distance
        acquire(core, target)
        speeds.append(core.assist_twist(current, [0, 0, .01, 0, 0, 0], 1.03, .02)[2])
    assert speeds[0] > speeds[1] > 0
    np.testing.assert_allclose(speeds[2:], 0., atol=1e-12)
    target[:3, 3] = [.02, 0., .05]
    acquire(core, target)
    velocity = core.assist_twist(current, [-.01, 0, .01, 0, 0, 0], 1.03, .02)
    assert 0 < velocity[0] <= .003 and abs(velocity[2]) < 1e-12


@pytest.mark.parametrize('failure', ['old', 'future', 'duplicate', 'jump', 'nan'])
def test_observation_validation_never_refreshes_bad_input(settings, failure):
    core = DockingAssist(settings)
    target = np.eye(4)
    target[2, 3] = .05
    tag = acquire(core, target)
    assert core.check(np.eye(4), 1.03)
    deadline = core.expires
    stamp, age = 10, 0.
    if failure == 'old': age = .3
    if failure == 'future': age = -.001
    if failure == 'duplicate': stamp = core.last_stamp
    if failure == 'jump': tag[0, 3] += .02
    if failure == 'nan': tag[0, 3] = float('nan')
    with pytest.raises(ValueError):
        core.observe(tag, stamp, 1.04, age)
    assert core.expires == deadline
    with pytest.raises(ValueError, match='timeout'):
        core.check(np.eye(4), 1.5)
    with pytest.raises(ValueError, match='timeout'):
        core.observe(target @ np.linalg.inv(settings['tools'][0]['tag_to_tcp']), 11, 1.5, 0.)


def test_acquisition_requires_distinct_stable_frames_and_nearby_target(settings):
    core = DockingAssist(settings)
    core.begin(0, 1.)
    assert not core.check(np.eye(4), 1.1)
    with pytest.raises(ValueError, match='acquisition timeout'):
        core.check(np.eye(4), 6.)
    target = np.eye(4)
    target[2, 3] = .3
    acquire(core, target)
    with pytest.raises(ValueError, match='outside assist envelope'):
        core.check(np.eye(4), 1.03)


def test_all_six_jacobian_columns_against_finite_differences(chain):
    # ASSUMED test posture inside the current hardware limits.
    q = np.array([.2, .4, .8, -.7, .25, -.3])
    pose, linear, angular = chain.pose_and_jacobian(dict(zip(JOINTS, q)), True, True)
    for i in range(6):
        moved = q.copy()
        moved[i] += 1e-6
        next_pose, _ = chain.pose_and_jacobian(dict(zip(JOINTS, moved)))
        np.testing.assert_allclose((next_pose[:3, 3]-pose[:3, 3])/1e-6, linear[:, i], atol=1e-6)
        # cv2.Rodrigues rounds very small rotations to zero; use skew differential.
        dr = (next_pose[:3, :3] @ pose[:3, :3].T-np.eye(3))/1e-6
        np.testing.assert_allclose([dr[2, 1], dr[0, 2], dr[1, 0]], angular[:, i], atol=1e-6)


def test_ik_caps_singularities_and_zero_unrequested_axial_motion(settings, chain):
    _, linear, angular = chain.pose_and_jacobian(
        dict(zip(JOINTS, [.2, .4, .8, -.7, .25, -.3])), True, True)
    jacobian = np.vstack((linear, angular))
    normal = np.array([0., 0., 1.])
    twist = np.array([.003, -.002, 0., .01, -.02, .03])
    v = joint_velocity(jacobian, twist, np.full(6, .04), settings['control'], normal)
    assert np.max(np.abs(v)) <= .04+1e-12
    assert abs((linear @ v) @ normal) < 1e-12
    np.testing.assert_array_equal(joint_velocity(np.zeros((6, 6)), twist,
        np.full(6, .04), settings['control'], normal), np.zeros(6))


def adapter(settings, chain):
    node = harness()
    node.chain = chain
    from manual_control_common import ManualController
    node.axis = ManualController.axis.__get__(node)
    node.button = ManualController.button.__get__(node)
    node.params.update(invert_forward=True, invert_wrist_yaw=False)
    node.startup_recovering = False
    node.target = np.array([0., .4, .8, -.7, 0., 0.])
    node.positions = dict(zip(JOINTS, node.target))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000000000))
    dock = ManualDocking.__new__(ManualDocking)
    dock.node, dock.core = node, DockingAssist(settings)
    dock.ready, dock.mock_input = False, True
    dock.base_frame = 'base_actuator'
    dock.selection_stamp_ns = 0
    node.docking = dock
    return node, dock


def joy(combo=None, forward=0.):
    axes, buttons = [0.]*8, [0]*11
    axes[2], axes[5], axes[4] = 1., 1., forward
    if combo is not None:
        axes[2] = -1.
        buttons[combo] = 1
    return Joy(axes=axes, buttons=buttons)


@pytest.mark.parametrize('button,tool', [(2, 0), (1, 1)])
def test_combo_no_path_button_release_neutral_and_cancel(settings, chain, button, tool):
    node, dock = adapter(settings, chain)
    with patch('tool_trajectory_joy.load_path', side_effect=AssertionError('no stored path')):
        node.on_joy(joy(button))
        node.on_joy(joy(button))
    assert dock.core.tool == tool and dock.selected and node.path_points is None
    current, _ = chain.pose_and_jacobian(node.positions)
    target = current.copy()
    target[:3, 3] += current[:3, 2]*.05
    acquire(dock.core, target, tool)
    before = node.target.copy()
    dock.run(1.03, .02)
    assert not dock.ready
    node.on_joy(joy(forward=1.))
    dock.run(1.04, .02)
    assert not dock.ready
    node.on_joy(joy())
    dock.run(1.05, .02)
    assert dock.ready
    np.testing.assert_array_equal(node.target, before)
    # Same combination cancels and requires neutral before resuming.
    node.on_joy(joy(button))
    assert not dock.selected and node.neutral_required
    assert list(node.ee_velocity_publisher.publish.call_args.args[0].data) == [0.]


@pytest.mark.parametrize('event', ['focus', 'joy_loss', 'feedback_loss', 'fault', 'tag_loss'])
def test_interruptions_cancel_and_require_explicit_reselection(settings, chain, event):
    node, dock = adapter(settings, chain)
    current, _ = chain.pose_and_jacobian(node.positions)
    target = current.copy()
    target[:3, 3] += current[:3, 2]*.05
    acquire(dock.core, target)
    dock.core.check(current, 1.03)
    dock.ready = True
    if event == 'focus':
        node.on_focus(String(data='drive'))
    elif event == 'joy_loss':
        node.joy_time = None
        tick(node)
    elif event == 'feedback_loss':
        node.state_times = {}
        tick(node)
    elif event == 'fault':
        node.on_hardware_fault(String(data='motor: fault'))
        tick(node)
    else:
        dock.run(2., .02)
    assert not dock.selected and node.neutral_required


@pytest.mark.parametrize('field,value', [('id', 1), ('frame_id', 'wrong'), ('synthetic', False)])
def test_mock_input_rejects_wrong_identity_or_frame(settings, chain, field, value):
    node, dock = adapter(settings, chain)
    dock.core.begin(0, .9)
    data = dict(stamp=dict(sec=1, nanosec=0), synthetic=True, id=0, family='tag36h11',
                frame_id='base_actuator', xyz_m=[0., 0., .1], xyzw=[0., 0., 0., 1.])
    data[field] = value
    dock.on_mock(String(data=json.dumps(data)))
    assert dock.core.target is None


def test_mock_input_refuses_real_hardware_description(settings):
    n = Mock()
    n.params = dict(root_link='base_actuator', robot_description=(
        '<robot><ros2_control><hardware><plugin>rmd_hardware_interface/RMD</plugin>'
        '</hardware></ros2_control></robot>'))
    n.declare_parameter.side_effect = lambda name, value: SimpleNamespace(value=value)
    with pytest.raises(ValueError, match='GenericSystem'):
        ManualDocking(n, str(ROOT/'config/manual_docking_assist.yaml'), mock_input=True)
    n.create_subscription.assert_not_called()


def test_live_adapter_requires_calibration_and_does_not_use_latest_tf(settings, chain):
    # Stub the existing pose solver; this exercises wiring, NOT tag recognition.
    node, dock = adapter(settings, chain)
    dock.mock_input = False
    dock.camera_frame = 'camera_optical'
    dock.camera = (np.eye(3), np.zeros(5))
    dock.tf_buffer = Mock()
    dock.tf_buffer.lookup_transform.return_value = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    dock.core.begin(0, .9)
    message = String(data=json.dumps(dict(stamp=dict(sec=1, nanosec=0),
        frame_id='camera_optical', detections=[dict(id=0, family='tag36h11')])))
    with patch('manual_docking.tag_pose', return_value=np.eye(4)) as solver:
        dock.on_tag(message)
        assert dock.core.stable == 1
        solver.assert_called_once()
        args = dock.tf_buffer.lookup_transform.call_args.args
        assert args[:2] == ('base_actuator', 'camera_optical')
        assert args[2].nanoseconds == 1000000000
        dock.camera = None
        dock.on_tag(message)
        assert solver.call_count == 1


def test_invalid_config_rejected_before_control(tmp_path, settings):
    import yaml
    data = yaml.safe_load((ROOT/'config/manual_docking_assist.yaml').read_text())
    data['control']['assist_full_speed_m_s'] = float('nan')
    path = tmp_path/'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='assist_full_speed_m_s'):
        load_config(path)


def test_cancel_has_priority_over_selection_and_multiple_faces_never_select(settings, chain):
    node, dock = adapter(settings, chain)
    msg = joy(combo=2)
    msg.buttons[3] = 1
    node.on_joy(msg)
    assert not dock.selected and not node.stop_pose_active
    node.path_combos = [False]*3
    node.neutral_required = False
    msg.buttons[3], msg.buttons[1] = 0, 1
    node.on_joy(msg)
    assert not dock.selected and node.path_points is None


@pytest.mark.parametrize('phase', ['acquiring', 'active'])
@pytest.mark.parametrize('axis,buttons', [
    (0, ()), (6, ()), (None, (4,)), (None, (5,)),
    (1, ()), (7, ()), (3, ()),  # Unassigned inputs must stay unassigned.
    (6, (1,)), (7, (1,)), (1, (1,)),  # Original O/EE/pitch/vertical layer.
    (None, (1,)), (None, ()),
])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_original_manual_mapping_and_ee_are_identical_with_assist(
        settings, chain, phase, axis, buttons, sign):
    plain, _ = adapter(settings, chain)
    plain.docking = None
    assisted, dock = adapter(settings, chain)
    pose, _ = chain.pose_and_jacobian(assisted.positions)
    goal = pose.copy()
    goal[:3, 3] += pose[:3, :3] @ [.008, 0., .04]
    if phase == 'active':
        acquire(dock.core, goal)
        assert dock.core.check(pose, 1.03)
    else:
        dock.core.begin(0, 1.)
    message = joy()
    if axis is not None:
        message.axes[axis] = sign
    for button in buttons:
        message.buttons[button] = 1
    for node in (plain, assisted):
        node.now_seconds = lambda: 1.03
        node.on_joy(message)
        tick(node)
    np.testing.assert_array_equal(assisted.target, plain.target)
    assert list(assisted.ee_velocity_publisher.publish.call_args.args[0].data) == list(
        plain.ee_velocity_publisher.publish.call_args.args[0].data)
    assert dock.snapshot()['input_mapping'] == 'manual'


def test_square_preserves_home_and_cancels_assist(settings, chain):
    node, dock = adapter(settings, chain)
    pose, _ = chain.pose_and_jacobian(node.positions)
    acquire(dock.core, pose)
    node.startup_goal = node.target.copy()
    message = joy()
    message.buttons[3] = 1
    node.on_joy(message)
    assert not dock.selected
    assert node.stop_pose_active
    np.testing.assert_array_equal(node.home_goals[1], node.startup_goal)


@pytest.mark.parametrize('axis_sign', [-1., 1.])
def test_existing_rmd_direction_determines_approach_without_stick_reinterpretation(settings, chain, axis_sign):
    node, dock = adapter(settings, chain)
    pose, linear, angular = chain.pose_and_jacobian(node.positions, True, True)
    goal = pose.copy()
    goal[:3, 3] += pose[:3, :3] @ [.008, 0., .04]
    acquire(dock.core, goal)
    node.joy = joy(forward=axis_sign)
    baseline = node.command_velocity(node.measured())
    expected = np.vstack((linear, angular)) @ baseline
    # These are original joint velocities; no alternative TCP joystick mapping.
    with patch.object(dock.core, 'assist_twist', wraps=dock.core.assist_twist) as assist:
        node.now_seconds = lambda: 1.03
        node.joy_time = 1.03
        tick(node)
        np.testing.assert_allclose(assist.call_args.args[1], expected, atol=1e-12)
    actual = (node.target-np.array([0., .4, .8, -.7, 0., 0.]))/.02
    if expected[:3] @ goal[:3, 2] < 0:
        np.testing.assert_allclose(actual, baseline, atol=1e-12)  # Retreat stays manual.
    else:
        assert not np.allclose(actual, baseline)
        assert abs(float(linear @ actual @ goal[:3, 2])) < 1e-6  # Gated approach.


def test_rmd_approach_passes_through_while_acquiring_and_during_explicit_override(settings, chain):
    for acquire_tag, override in ((False, False), (True, True)):
        plain, _ = adapter(settings, chain)
        plain.docking = None
        node, dock = adapter(settings, chain)
        if acquire_tag:
            pose, _ = chain.pose_and_jacobian(node.positions)
            goal = pose.copy()
            goal[:3, 3] += pose[:3, 2]*.04
            acquire(dock.core, goal)
        else:
            dock.core.begin(0, 1.)
        msg = joy(forward=-1.)
        if override:
            msg.buttons[4] = 1
        for item in (plain, node):
            item.now_seconds = lambda: 1.03
            item.on_joy(msg)
            tick(item)
        np.testing.assert_array_equal(node.target, plain.target)


def test_original_manual_approach_with_tag_correction_closed_loop(settings, chain):
    node, dock = adapter(settings, chain)
    initial, _ = chain.pose_and_jacobian(node.positions)
    goal = initial.copy()
    # ASSUMED synthetic offset; initial TCP Z is oblique to the existing
    # root-radial manual translation. This must not drift sideways indefinitely.
    goal[:3, 3] += initial[:3, :3] @ [.008, 0., .040]
    tag = acquire(dock.core, goal)
    node.joy = joy(forward=-1.)  # Existing invert_forward=True, not a TCP mapping.
    now = 1.03
    for index in range(1500):
        now += .02
        node.now_seconds = lambda: now
        node.last_time, node.joy_time = now-.02, now
        node.state_times = dict.fromkeys(JOINTS, now)
        dock.core.observe(tag, index+10, now, 0.)
        before = node.target.copy()
        node.on_timer()
        assert dock.selected and not node.fault
        assert np.max(np.abs(node.target-before)) <= settings['control']['joint_speed_rad_s']*.02+1e-12
        node.positions = dict(zip(JOINTS, node.target))
        pose, _ = chain.pose_and_jacobian(node.positions)
        error = initial[:3, :3].T @ (goal[:3, 3]-pose[:3, 3])
        assert error[2] >= .001-2e-6
        assert np.linalg.norm(error[:2]) < .015
    assert .000998 <= error[2] <= .0012
    assert np.linalg.norm(error[:2]) < .001
    before = node.target.copy()
    node.joy = joy()
    dock.run(now+.01, .01)
    np.testing.assert_array_equal(node.target, before)


