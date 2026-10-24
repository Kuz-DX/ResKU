"""Offline Cartesian increment and command gating regressions."""
from pathlib import Path
import sys
from unittest.mock import Mock, patch
import numpy as np
import pytest

ARM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ARM/'tool_change_min'))
sys.path.insert(0, str(ARM/'tool_manipulator_bringup/scripts'))
from tcp_joy_core import TcpStepper, joy_twist
from tcp_joy import TcpJoy
from tool_change_min.kinematics import ArmKinematics
from manual_control_common import ArmChain
import xacro
import yaml


@pytest.fixture
def model():
    return ArmKinematics(ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro',
                         ARM/'tool_manipulator_bringup/config/hardware.yaml')


def test_deadman_modifier_and_vector_speed_bound():
    buttons = [0]*13
    assert not joy_twist([1.]*8, buttons)[0]
    buttons[4] = 1
    held, twist = joy_twist([1.]*8, buttons)
    assert held and np.linalg.norm(twist[:3]) == pytest.approx(1.)
    np.testing.assert_array_equal(twist[3:], 0.)
    buttons[5] = 1
    _, twist = joy_twist([1.]*8, buttons)
    np.testing.assert_array_equal(twist[:3], 0.)
    assert np.linalg.norm(twist[3:]) == pytest.approx(1.)
    assert not joy_twist([float('nan')]*8, buttons)[0]


def test_actual_urdf_fk_matches_manual_chain(model):
    xml = xacro.process_file(str(ARM/'tool_manipulator_description/urdf/tool_manipulator.urdf.xacro')).toxml()
    manual = ArmChain(xml)
    for q in (np.zeros(6), np.array([.1, -.4, .7, -.2, .3, -.2])):
        fk, jac = manual.pose_and_jacobian(dict(zip(model.joint_names, q)))
        np.testing.assert_allclose(model.fk(q), fk, atol=1e-12)
        np.testing.assert_allclose(model.jacobian(q)[:3, 1:4], jac, atol=1e-7)


def fake_stepper():
    model = Mock()
    model.within_limits.return_value = True
    model.fk.return_value = np.eye(4)
    model.jacobian.return_value = np.diag([.2]*3+[1.]*3)
    model.solve.return_value = np.ones(6)*.001
    step = TcpStepper(model, [.12]*6)
    step.reset(np.zeros(6))
    return step


def test_increment_only_commits_after_collision_acceptance():
    s = fake_stepper()
    q, target = s.candidate(np.zeros(6), np.array([1.,0,0,0,0,0]), .02)
    assert target[0,3] == pytest.approx(.0004)
    assert s.pose[0,3] == 0.
    s.commit(q,target)
    _, target2 = s.candidate(q, np.array([1.,0,0,0,0,0]), .02)
    assert target2[0,3] == pytest.approx(.0008)
    # The accumulated target pose and its matching previous command solution
    # form the next IK seed; live feedback is enforced separately by lead checks.
    np.testing.assert_array_equal(s.model.solve.call_args.args[1], s.q)


@pytest.mark.parametrize('kind', ['speed', 'lead', 'singularity', 'ik'])
def test_failed_candidate_preserves_last_target(kind):
    s = fake_stepper()
    measured = np.zeros(6)
    if kind == 'speed':
        s.model.solve.return_value = np.ones(6)*.1
    if kind == 'lead':
        measured[:] = -.09
    if kind == 'singularity':
        s.model.jacobian.return_value = np.zeros((6,6))
    if kind == 'ik':
        s.model.solve.side_effect = ValueError('no solution')
    with pytest.raises(ValueError):
        s.candidate(measured, np.ones(6), .02)
    np.testing.assert_array_equal(s.q, np.zeros(6))
    np.testing.assert_array_equal(s.pose, np.eye(4))


def test_ik_time_and_iteration_budgets(model):
    q = np.zeros(6)
    target = model.fk(q)
    target[0,3] += .01
    with pytest.raises(ValueError, match='time budget'):
        model.solve(target, q, max_time_s=1e-12)
    with pytest.raises(ValueError, match='converge'):
        model.solve(target, q, max_iterations=0)


def test_deadman_release_discards_pending_checks():
    node = Mock()
    node.on_joy = TcpJoy.on_joy.__get__(node)
    node.on_joy(Mock(axes=[0.]*8, buttons=[0]*13))
    node.discard.assert_called_once()
    assert not node.deadman and node.rearmed
    np.testing.assert_array_equal(node.twist, np.zeros(6))


def test_repeated_failure_requests_hold_and_stops_heartbeat():
    node = Mock(failures=0, blocked=False, last_failure_reason='',
                last_failure_time=-float('inf'))
    for _ in range(5):
        TcpJoy.fail(node, 'collision service failed')
    assert node.blocked
    assert node.last_failure_reason == 'collision service failed'
    assert np.isfinite(node.last_failure_time)
    node.cancel.call_async.assert_called_once()


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1., 1.])
def test_real_feedback_all_tcp_directions_obey_joint_speed(model, axis, sign):
    measured = np.array([-.0046019424, 1.4837043012, 1.5070917825,
                         -1.5568336395, .0582912699, .0582912699])
    step = TcpStepper(model, [.12]*6)
    step.reset(measured)
    twist = np.zeros(6)
    twist[axis] = sign
    # Freeze wall clock here to test numerical convergence deterministically;
    # the separate deadline regression checks time exhaustion.
    with patch('time.monotonic', return_value=10.):
        q, pose = step.candidate(measured, twist, .02)
    assert np.max(np.abs(q-measured)) > 1e-6
    assert np.all(np.abs(q-measured) <= .12*.02+1e-9)
    assert model.within_limits(q, .02)
    error = model.pose_error(q, pose)
    assert np.linalg.norm(error[:3]) < 1e-5
    assert np.linalg.norm(error[3:]) < 1e-4
    np.testing.assert_array_equal(step.q, measured)


def test_wait_for_collision_server_keeps_hold_heartbeat_without_failures():
    node = Mock()
    node.status = dict(mode='tcp_ik', focus='arm', ready=True,
                       transitioning=False, epoch='test')
    node.status_time = node.state_time = node.last_time = 10.
    node.epoch = 'test'
    node.measured = np.zeros(6)
    node.stepper = fake_stepper()
    node.blocked = False
    node.failures = 0
    node.last_failure_reason = ''
    node.last_failure_time = -float('inf')
    node.last_ik_duration_ms = None
    node.pending = None
    node.deadman = node.rearmed = True
    node.check.service_is_ready.return_value = False
    with patch('time.monotonic', return_value=10.02):
        TcpJoy.tick(node)
    assert not node.rearmed
    node.fail.assert_not_called()
    node.publish.assert_called_once()
    assert 'waiting for MoveIt' in node.reason


def test_tcp_mock_initial_pose_is_inside_margin_and_nonsingular(model):
    values = yaml.safe_load((ARM/'tool_manipulator_bringup/config/tcp_mock_initial_positions.yaml').read_text())
    q = np.array([values['initial_positions'][j] for j in model.joint_names])
    assert model.within_limits(q, .02)
    jac = model.jacobian(q)
    jac[:3] /= .2
    assert np.linalg.cond(jac) < 20.


def test_candidate_total_deadline_preserves_previous_goal():
    step = fake_stepper()
    with patch('time.monotonic', side_effect=[10., 10.02]):
        with pytest.raises(ValueError, match='time budget'):
            step.candidate(np.zeros(6), np.array([1., 0, 0, 0, 0, 0]), .02)
    step.model.solve.assert_not_called()
    np.testing.assert_array_equal(step.q, np.zeros(6))


def test_gripper_zero_and_internal_allowed_collisions_stay_consistent():
    hardware = yaml.safe_load((ARM/'tool_manipulator_bringup/config/hardware.yaml').read_text())
    assert hardware['joints']['ee_joint']['zero_raw'] == 1992
    text = (ARM/'tool_manipulator_moveit_config/config/tool_manipulator.srdf').read_text()
    expected = {
        ('rack_left', 'ee_actuator'), ('tool_spurgear', 'ee_actuator'),
        ('rack_right', 'wrist_link_yaw'), ('rack_left', 'tool_spurgear'),
        ('rack_right', 'tool_spurgear'), ('rack_left', 'wrist_link_yaw'),
    }
    import xml.etree.ElementTree as ET
    root = ET.fromstring(text)
    allowed = {tuple(sorted((x.get('link1'), x.get('link2'))))
               for x in root.findall('disable_collisions')}
    assert {tuple(sorted(pair)) for pair in expected} <= allowed
