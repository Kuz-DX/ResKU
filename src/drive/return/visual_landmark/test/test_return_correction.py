"""Controller consumers agree on corrections, including arrival and mission reset."""
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ClockType
from rclpy.time import Time
from return_navigation.se2 import Pose2D, apply_correction
from return_navigation.return_path_follower_node import ReturnPathFollowerNode
from return_navigation.return_state_machine_node import ReturnStateMachineNode


def fake_node():
    now = Time(seconds=100, clock_type=ClockType.ROS_TIME)
    return SimpleNamespace(
        _origin=Pose2D(), _visual_correction=None, _auto_follow_after=None,
        _origin_received_at=Time(seconds=90, clock_type=ClockType.ROS_TIME),
        get_clock=lambda: SimpleNamespace(now=lambda: now))


@pytest.mark.parametrize('cls', [ReturnPathFollowerNode, ReturnStateMachineNode])
def test_consumer_accepts_fresh_transform_and_rejects_stale_previous_mission(cls):
    node = fake_node()
    msg = PoseStamped()
    msg.header.frame_id = 'mission'
    msg.header.stamp = Time(seconds=99, clock_type=ClockType.ROS_TIME).to_msg()
    msg.pose.position.x = -2.0
    msg.pose.orientation.w = 1.0
    cls._on_visual_correction(node, msg)
    assert apply_correction(Pose2D(5, 0, 0), node._visual_correction).x == 3.0
    accepted = node._visual_correction
    msg.pose.position.x = 100.0
    msg.header.stamp = Time(seconds=89, clock_type=ClockType.ROS_TIME).to_msg()
    cls._on_visual_correction(node, msg)
    assert node._visual_correction is accepted
    new_origin = Pose2D(1.0, 0.0, 0.0).to_ros_pose()
    cls._on_origin(node, new_origin)
    assert node._visual_correction is None


def test_arrival_uses_corrected_position_instead_of_drifting_odom():
    node = fake_node()
    now = node.get_clock().now()
    transitions = []
    node._follow_cmd_time = now
    node.follow_relay_timeout = 1.0
    node._follow_cmd = None
    node.cmd_vel_return_pub = SimpleNamespace(publish=lambda msg: None)
    node._return_path = [Pose2D()]
    node._odom_pose = Pose2D(2, 0, 0)
    node.goal_tolerance_m = 0.3
    node._go_to = transitions.append
    ReturnStateMachineNode._tick_follow_return_path(node)
    assert not transitions
    node._visual_correction = Pose2D(-2, 0, 0)
    ReturnStateMachineNode._tick_follow_return_path(node)
    assert transitions == ['FINISHED']
