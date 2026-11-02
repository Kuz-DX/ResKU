"""Real timer/clock regression without ROS nodes, DDS, or robot hardware."""
from contextlib import ExitStack
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from rclpy.clock import ClockType, ROSClock
from rclpy.context import Context
from rclpy.node import Node
from rclpy.time import Time
from rclpy.timer import Timer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from arm_control_mode_manager import ControlModeManager
from arm_control_mode_manager_test import TestControlModeManager as TimedManager


@pytest.mark.parametrize('manager_type', [ControlModeManager, TimedManager])
def test_control_timer_continues_through_ros_clock_jumps(manager_type):
    context = Context()
    context.init(args=[])
    ros_clock = ROSClock()
    ros_clock._set_ros_time_is_active(True)
    ros_clock.set_ros_time_override(Time(seconds=1000))
    timers = []

    def create_timer(node, period, callback, *, clock=None):
        # Use the same default-clock selection as Node.create_timer, while
        # constructing a real RCL timer without creating any ROS graph entities.
        timer = Timer(callback, None, int(period*1e9),
                      node._clock if clock is None else clock, context=context)
        timers.append(timer)
        return timer

    try:
        with ExitStack() as stack:
            stack.enter_context(patch.object(Node, '__init__',
                lambda node, name: setattr(node, '_clock', ros_clock)))
            stack.enter_context(patch.object(Node, 'declare_parameter',
                lambda node, name, default: SimpleNamespace(value=default)))
            for name in ('create_publisher', 'create_subscription', 'create_service', 'create_client'):
                stack.enter_context(patch.object(Node, name, Mock()))
            stack.enter_context(patch.object(Node, 'create_timer', create_timer))
            stack.enter_context(patch.object(ControlModeManager, 'publish_status'))
            stack.enter_context(patch('arm_control_mode_manager.get_package_share_directory',
                                     return_value=str(ROOT)))
            manager_type()

        assert len(timers) == 1
        timer = timers[0]
        assert timer.timer_period_ns == 10_000_000
        assert timer.clock.clock_type == ClockType.STEADY_TIME
        # A backwards jump and then a forwards jump must not pause the control
        # timer, even though the node's ROS clock remains completely frozen.
        for seconds in (10, 2000):
            timer.reset()
            ros_clock.set_ros_time_override(Time(seconds=seconds))
            deadline = time.monotonic()+1.
            while not timer.is_ready() and time.monotonic() < deadline:
                time.sleep(.001)
            assert timer.is_ready()
            assert ros_clock.now().nanoseconds == seconds*1_000_000_000
    finally:
        for timer in timers:
            timer.destroy()
        context.shutdown()
