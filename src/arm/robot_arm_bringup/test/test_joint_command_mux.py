"""Regression tests for coordinated hold behavior."""

import importlib.util
from pathlib import Path

from sensor_msgs.msg import Joy
from rclpy.time import Time
from std_msgs.msg import String


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'joint_command_mux.py'
SPEC = importlib.util.spec_from_file_location('joint_command_mux', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Logger:
    def error(self, _message):
        pass

    def info(self, _message):
        pass


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Clock:
    def now(self):
        return Time(seconds=1.0)


def make_mux():
    mux = object.__new__(MODULE.JointCommandMux)
    mux.joint_names = [
        'base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint',
        'gripper_joint']
    mux.gripper_joint = 'gripper_joint'
    mux.lower = [-3.0] * 5
    mux.upper = [3.0] * 5
    mux.positions = {name: float(index) / 10.0
                     for index, name in enumerate(mux.joint_names)}
    mux.hold_target = None
    mux.off_hold_target = None
    mux.output = [1.0] * 5
    mux.home_hold_target = None
    mux.home_phase = None
    mux.home_phase_target = {}
    mux.home_pending_phases = []
    mux.home_phase_started = None
    mux.home_in_tolerance_since = None
    mux.home_joint_names = [
        'shoulder_joint', 'elbow_joint', 'wrist_joint']
    mux.home_positions = [-0.55851, 1.58825, 1.51844]
    mux.home_values = dict(zip(mux.home_joint_names, mux.home_positions))
    mux.hold_joint_names = [
        'base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']
    mux.hold_positions = [0.0, -0.76044, 1.63830, 1.53290]
    mux.hold_values = dict(zip(mux.hold_joint_names, mux.hold_positions))
    mux.hold_button = 3
    mux.home_base_position = 0.0
    mux.home_shoulder_clearance_position = 0.78539816339
    mux.home_wrist_stage1_position = 1.308996938996
    mux.home_branch_deadband = 0.03
    mux.home_move_threshold = 0.03
    mux.home_tolerance = 0.02
    mux.home_base_tolerance = 0.05
    mux.home_dwell_time = 0.3
    mux.home_phase_timeout = 60.0
    mux.sequence_kind = None
    mux.sequence_gripper_hold = None
    mux.manual_ee_ready_pub = Publisher()
    mux.get_logger = lambda: Logger()
    mux.get_clock = lambda: Clock()
    return mux


def configure_following_error(mux):
    mux.mode = 'MANUAL_EE'
    mux.protective_stop = False
    mux.tracking_fault = False
    mux.following_error_limits = [0.2, 0.2, 0.2, 0.2, 0.4]
    mux.following_error_timeout = 1.0
    mux.following_error_started = {}
    mux.manual_command = None
    mux.manual_stamp = None
    mux.ee_command = None
    mux.ee_stamp = None
    mux.gripper_command = None
    mux.gripper_stamp = None


def test_gripper_following_error_does_not_latch_whole_arm():
    mux = make_mux()
    configure_following_error(mux)
    command = mux.measured_positions()
    command[-1] += 1.0

    assert not mux.update_following_error(command, Time(seconds=1.0))
    assert not mux.update_following_error(command, Time(seconds=3.0))
    assert not mux.tracking_fault
    assert 'gripper_joint' not in mux.following_error_started


def test_arm_joint_following_error_still_latches():
    mux = make_mux()
    configure_following_error(mux)
    command = mux.measured_positions()
    command[1] += 1.0

    assert not mux.update_following_error(command, Time(seconds=1.0))
    assert mux.update_following_error(command, Time(seconds=2.1))
    assert mux.tracking_fault


def test_off_hold_is_first_feedback_snapshot_not_live_measurement():
    mux = make_mux()
    mux.capture_off_hold_target()
    expected = list(mux.off_hold_target)

    mux.positions['shoulder_joint'] = -1.0
    mux.positions['elbow_joint'] = -1.2

    assert mux.off_hold_target == expected
    assert mux.output == expected


def test_off_hold_clamps_initial_feedback_to_command_limits():
    mux = make_mux()
    mux.lower[1] = -1.4818
    mux.positions['shoulder_joint'] = -1.481803

    mux.capture_off_hold_target()

    assert mux.off_hold_target[1] == -1.4818
    assert mux.output[1] == -1.4818


def test_hold_target_is_a_snapshot_not_a_moving_measurement():
    mux = make_mux()
    mux.latch_hold_target('test')
    expected = list(mux.hold_target)

    mux.positions['shoulder_joint'] = 2.0
    mux.latch_hold_target('second request')

    assert mux.hold_target == expected
    assert mux.output == expected


def test_hardware_fault_latches_stop_and_clears_operator_commands():
    mux = make_mux()
    mux.hardware_fault = None
    mux.protective_stop = False
    mux.home_phase = 'shoulder'
    mux.manual_command = {'shoulder_joint': 1.0}
    mux.manual_stamp = object()
    mux.ee_command = {'shoulder_joint': 1.0}
    mux.ee_stamp = object()
    mux.gripper_command = 1.0
    mux.gripper_stamp = object()

    mux.on_hardware_fault(String(data='wrist_joint: feedback timeout'))

    assert mux.hardware_fault.startswith('wrist_joint:')
    assert mux.protective_stop
    assert mux.hold_target is not None
    assert mux.manual_command is None
    assert mux.ee_command is None
    assert mux.gripper_command is None


def test_positive_shoulder_branch_starts_with_clearance():
    mux = make_mux()
    mux.positions['shoulder_joint'] = 0.5

    mux.start_home_sequence(Time(seconds=1.0))

    assert mux.home_phase == 'shoulder_clearance'
    assert mux.home_phase_target == {
        'shoulder_joint': mux.home_shoulder_clearance_position}
    assert [phase for phase, _ in mux.home_pending_phases] == [
        'wrist_stage1', 'elbow_home', 'base_home', 'shoulder_home', 'wrist_home']


def test_nonpositive_shoulder_branch_starts_with_wrist():
    mux = make_mux()
    mux.positions['shoulder_joint'] = -0.2

    mux.start_home_sequence(Time(seconds=1.0))

    assert mux.home_phase == 'wrist_home'
    assert mux.home_phase_target == {'wrist_joint': 1.51844}
    assert [phase for phase, _ in mux.home_pending_phases] == [
        'elbow_home', 'shoulder_home', 'base_home']


def test_home_phase_holds_non_target_joints_at_stage_snapshot():
    mux = make_mux()
    mux.home_hold_target = [0.4, 0.1, -0.2, -0.3, 0.7]
    mux.home_phase_target = {'wrist_joint': 1.51844}

    target = mux.home_target(mux.measured_positions())

    assert target == [0.4, 0.1, -0.2, 1.51844, 0.7]


def test_home_sequence_skips_joints_already_near_targets():
    mux = make_mux()
    mux.positions.update({
        'base_joint': 0.0,
        'shoulder_joint': mux.home_shoulder_clearance_position,
        'elbow_joint': mux.home_values['elbow_joint'],
        'wrist_joint': mux.home_values['wrist_joint'],
    })

    mux.start_home_sequence(Time(seconds=1.0))

    # Wrist is already beyond stage 1 and at final home, so it must not move
    # backward. Clearance, wrist, elbow and base phases are skipped here.
    assert mux.home_phase == 'shoulder_home'
    assert mux.home_phase_target == {
        'shoulder_joint': mux.home_values['shoulder_joint']}


def test_positive_branch_holds_wrist_when_already_above_stage1():
    mux = make_mux()
    mux.positions['shoulder_joint'] = 0.5
    mux.positions['wrist_joint'] = mux.home_wrist_stage1_position + 0.1

    mux.start_home_sequence(Time(seconds=1.0))

    assert mux.home_phase == 'shoulder_clearance'
    assert mux.home_phase_target == {
        'shoulder_joint': mux.home_shoulder_clearance_position}
    assert mux.home_hold_target[3] == mux.positions['wrist_joint']
    assert [phase for phase, _ in mux.home_pending_phases] == [
        'elbow_home', 'base_home', 'shoulder_home', 'wrist_home']


def test_arm_home_moves_only_joint_with_large_error():
    mux = make_mux()
    mux.positions['elbow_joint'] = mux.home_values['elbow_joint'] - 0.01
    mux.positions['wrist_joint'] = mux.home_values['wrist_joint'] - 0.5
    mux.home_pending_phases = [('arm_home', {
        'elbow_joint': mux.home_values['elbow_joint'],
        'wrist_joint': mux.home_values['wrist_joint'],
    })]

    mux.start_next_home_phase(Time(seconds=1.0))

    assert mux.home_phase == 'arm_home'
    assert mux.home_phase_target == {
        'wrist_joint': mux.home_values['wrist_joint']}


def test_home_phase_requires_tolerance_dwell_before_advancing():
    mux = make_mux()
    mux.home_phase = 'wrist_home'
    mux.home_phase_target = {'wrist_joint': mux.home_values['wrist_joint']}
    mux.home_hold_target = mux.measured_positions()
    mux.home_phase_started = Time(seconds=0.0)
    mux.home_pending_phases = [
        ('elbow_home', {'elbow_joint': mux.home_values['elbow_joint']})]
    mux.positions['wrist_joint'] = mux.home_values['wrist_joint']

    mux.update_home_sequence(Time(seconds=1.0))
    assert mux.home_phase == 'wrist_home'

    mux.update_home_sequence(Time(seconds=1.31))
    assert mux.home_phase == 'elbow_home'


def test_home_phase_uses_relaxed_tolerance_only_for_base():
    mux = make_mux()
    mux.home_phase = 'hold_pose'
    mux.home_phase_target = {'base_joint': 0.0, 'wrist_joint': 1.0}
    mux.home_hold_target = mux.measured_positions()
    mux.home_phase_started = Time(seconds=0.0)
    mux.positions['base_joint'] = 0.04
    mux.positions['wrist_joint'] = 1.03

    mux.update_home_sequence(Time(seconds=1.0))
    assert mux.home_in_tolerance_since is None

    mux.positions['wrist_joint'] = 1.01
    mux.update_home_sequence(Time(seconds=1.1))
    assert mux.home_in_tolerance_since == Time(seconds=1.1)


def test_home_phase_timeout_aborts_and_keeps_manual_ee_blocked():
    mux = make_mux()
    mux.home_phase = 'wrist_home'
    mux.home_phase_target = {'wrist_joint': mux.home_values['wrist_joint']}
    mux.home_hold_target = mux.measured_positions()
    mux.home_phase_started = Time(seconds=0.0)

    mux.update_home_sequence(Time(seconds=61.0))

    assert mux.home_phase is None
    assert mux.manual_ee_ready_pub.messages[-1].data is False


def test_manual_modes_do_not_home_automatically_on_entry():
    assert 'MANUAL_EE' not in MODULE.JointCommandMux.HOME_ENTRY_MODES
    assert 'MANUAL_100' not in MODULE.JointCommandMux.HOME_ENTRY_MODES


def test_circle_starts_manual_ee_home_only_on_rising_edge():
    mux = make_mux()
    mux.home_button = 1
    mux.last_joy_buttons = []
    mux.mode = 'MANUAL_EE'
    mux.home_phase = None
    mux.protective_stop = False
    mux.hardware_fault = None
    mux.ee_command = {'base_joint': 0.0}
    mux.ee_stamp = object()
    mux.gripper_command = 0.5
    mux.gripper_stamp = object()

    mux.on_joy(Joy(buttons=[0, 1]))

    assert mux.home_phase == 'shoulder_clearance'
    assert mux.ee_command is None
    assert mux.gripper_command is None
    assert mux.manual_ee_ready_pub.messages[-1].data is False

    mux.home_phase = None
    mux.on_joy(Joy(buttons=[0, 1]))
    assert mux.home_phase is None


def test_square_starts_hold_pose_and_preserves_gripper_snapshot():
    mux = make_mux()
    mux.home_button = 1
    mux.hold_button = 3
    mux.last_joy_buttons = []
    mux.mode = 'MANUAL_EE'
    mux.protective_stop = False
    mux.hardware_fault = None
    mux.ee_command = {'base_joint': 0.0}
    mux.ee_stamp = object()
    mux.gripper_command = 0.5
    mux.gripper_stamp = object()

    mux.on_joy(Joy(buttons=[0, 0, 0, 1]))

    assert mux.sequence_kind == 'hold'
    assert mux.home_phase == 'hold_pose'
    assert mux.home_phase_target == {
        name: value for name, value in mux.hold_values.items()
        if name != 'base_joint'}
    assert mux.home_hold_target[-1] == 0.5
    assert mux.ee_command is None
    assert mux.gripper_command is None
    assert mux.manual_ee_ready_pub.messages[-1].data is False


def test_manual_ee_ready_publication_uses_boolean_message():
    mux = make_mux()
    mux.manual_ee_ready_pub = Publisher()

    mux.publish_manual_ee_ready(True)
    mux.publish_manual_ee_ready(False)

    assert [message.data for message in mux.manual_ee_ready_pub.messages] == [
        True, False]
