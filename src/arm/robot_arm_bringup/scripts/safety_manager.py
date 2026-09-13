#!/usr/bin/env python3
"""Operator mode selection and latched controlled protective stop."""

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


def next_mode(current, control_toggle=False, auto_toggle=False,
              manual_100_toggle=False, manual_ee_pause_toggle=False,
              auto_return='MANUAL_EE',
              manual_100_return='MANUAL_EE'):
    """Return the operator mode after one set of rising-edge inputs."""
    if current == 'ESTOP_LATCHED':
        return current
    if control_toggle:
        if current == 'OFF':
            # TEMPORARY CALIBRATION MODE: enter MANUAL_100 first so the
            # operator can place the arm and record encoder positions without
            # triggering the MANUAL_EE home-pose sequence.
            return 'MANUAL_100'
        if current in ('MANUAL_EE', 'MANUAL_EE_PAUSED', 'MANUAL_100'):
            return 'OFF'
    if manual_ee_pause_toggle and current == 'MANUAL_EE':
        return 'MANUAL_EE_PAUSED'
    if manual_ee_pause_toggle and current == 'MANUAL_EE_PAUSED':
        return 'MANUAL_EE'
    if auto_toggle and current in ('MANUAL_EE', 'MANUAL_100'):
        return 'AUTO'
    if auto_toggle and current == 'AUTO':
        return auto_return
    if manual_100_toggle and current in ('MANUAL_EE', 'AUTO'):
        return 'MANUAL_100'
    if manual_100_toggle and current == 'MANUAL_100':
        return manual_100_return
    return current


def controller_switch_for_mode(requested):
    if requested == 'AUTO':
        return (['arm_controller', 'gripper_controller'], ['position_controller'])
    return (['position_controller'], ['arm_controller', 'gripper_controller'])


class SafetyManager(Node):
    OFF = 'OFF'
    AUTO = 'AUTO'
    MANUAL_EE = 'MANUAL_EE'
    MANUAL_EE_PAUSED = 'MANUAL_EE_PAUSED'
    MANUAL_100 = 'MANUAL_100'
    ESTOP_LATCHED = 'ESTOP_LATCHED'

    def __init__(self):
        super().__init__('safety_manager')
        self.declare_parameter('control_toggle_button', 9)
        # AUTO selection is disabled in the manual mission. Share toggles
        # MANUAL_EE/MANUAL_100 and PS pauses MANUAL_EE.
        self.declare_parameter('auto_mode_button', -1)
        self.declare_parameter('manual_100_mode_button', 8)
        self.declare_parameter('manual_ee_pause_button', 10)
        # Button 10 (PS) is the MANUAL_EE pause toggle.
        self.control_button = int(self.get_parameter('control_toggle_button').value)
        self.auto_button = int(
            self.get_parameter('auto_mode_button').value)
        self.manual_100_button = int(
            self.get_parameter('manual_100_mode_button').value)
        self.manual_ee_pause_button = int(
            self.get_parameter('manual_ee_pause_button').value)
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('switch_timeout_sec', 2)
        self.switch_timeout_sec = int(
            self.get_parameter('switch_timeout_sec').value)

        self.mode = self.OFF
        self.auto_return_mode = self.MANUAL_EE
        self.manual_100_return_mode = self.MANUAL_EE
        self.last_buttons = []
        self.pending_mode = None
        manager = str(self.get_parameter('controller_manager').value).rstrip('/')
        self.switch_client = self.create_client(
            SwitchController, manager + '/switch_controller')

        latched_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/control/mode', latched_qos)
        self.control_enabled_pub = self.create_publisher(Bool, '/control/enabled', latched_qos)
        self.manual_100_enabled_pub = self.create_publisher(
            Bool, '/control/manual_100_enabled', latched_qos)
        self.auto_enabled_pub = self.create_publisher(
            Bool, '/control/auto_enabled', latched_qos)
        self.manual_ee_enabled_pub = self.create_publisher(
            Bool, '/control/manual_ee_enabled', latched_qos)
        # [하림 수정] drive와 arm이 조이스틱 하나를 토글로 공유하기 위한 포커스 상태.
        # control_toggle_button(9번, arm OFF<->manual 토글)을 누르면
        # 이 값도 같이 바뀐다 - arm이 켜지면 'arm', 꺼지면 'drive'.
        self.active_target_pub = self.create_publisher(String, '/control/active_target', latched_qos)
        self.protective_stop_pub = self.create_publisher(
            Bool, '/control/protective_stop', latched_qos)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', latched_qos)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_subscription(
            String, '/control/hardware_fault', self.on_hardware_fault,
            latched_qos)
        self.estop_pub.publish(Bool(data=False))
        self.protective_stop_pub.publish(Bool(data=False))
        self.publish_mode()

    def pressed(self, msg, index):
        return 0 <= index < len(msg.buttons) and msg.buttons[index] != 0

    def rising(self, msg, index):
        previous = 0 <= index < len(self.last_buttons) and self.last_buttons[index] != 0
        return self.pressed(msg, index) and not previous

    def on_joy(self, msg):
        control_toggle = self.rising(msg, self.control_button)
        auto_toggle = self.rising(msg, self.auto_button)
        manual_100_toggle = self.rising(msg, self.manual_100_button)
        manual_ee_pause_toggle = self.rising(
            msg, self.manual_ee_pause_button)
        if auto_toggle and self.mode in (self.MANUAL_EE, self.MANUAL_100):
            self.auto_return_mode = self.mode
        if (manual_100_toggle and
                self.mode in (self.MANUAL_EE, self.AUTO)):
            self.manual_100_return_mode = self.mode
        requested = next_mode(
            self.mode,
            control_toggle=control_toggle,
            auto_toggle=auto_toggle,
            manual_100_toggle=manual_100_toggle,
            manual_ee_pause_toggle=manual_ee_pause_toggle,
            auto_return=self.auto_return_mode,
            manual_100_return=self.manual_100_return_mode,
        )
        if requested != self.mode:
            self.request_mode(requested)
        self.last_buttons = list(msg.buttons)

    def request_mode(self, requested):
        if self.pending_mode is not None:
            self.get_logger().warn('Controller switch already in progress.')
            return
        old_auto = self.mode == self.AUTO
        new_auto = requested == self.AUTO
        if old_auto == new_auto:
            self.mode = requested
            self.publish_mode()
            return
        if not self.switch_client.service_is_ready():
            self.get_logger().error(
                'controller_manager switch service is not ready; mode unchanged.')
            return
        # Gate the autonomous planner before releasing its controllers.
        if old_auto:
            self.auto_enabled_pub.publish(Bool(data=False))
            self.manual_ee_enabled_pub.publish(Bool(data=False))
        request = SwitchController.Request()
        activate, deactivate = controller_switch_for_mode(requested)
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = self.switch_timeout_sec
        self.pending_mode = requested
        future = self.switch_client.call_async(request)
        future.add_done_callback(self.on_switch_complete)

    def on_switch_complete(self, future):
        requested = self.pending_mode
        self.pending_mode = None
        try:
            success = bool(future.result().ok)
        except Exception as exc:
            self.get_logger().error(f'Controller switch call failed: {exc}')
            success = False
        if not success:
            self.get_logger().error(
                f'Controller switch to {requested} failed; mode remains {self.mode}.')
            if self.mode == self.AUTO:
                self.auto_enabled_pub.publish(Bool(data=True))
            return
        self.mode = requested
        self.publish_mode()

    def publish_mode(self):
        enabled = self.mode in (
            self.AUTO, self.MANUAL_EE, self.MANUAL_EE_PAUSED,
            self.MANUAL_100)
        self.mode_pub.publish(String(data=self.mode))
        self.control_enabled_pub.publish(Bool(data=enabled))
        self.manual_100_enabled_pub.publish(
            Bool(data=self.mode == self.MANUAL_100))
        self.auto_enabled_pub.publish(Bool(data=self.mode == self.AUTO))
        self.manual_ee_enabled_pub.publish(
            Bool(data=self.mode == self.MANUAL_EE))
        # [하림 수정] arm이 활성 상태면 조이스틱 포커스를 arm으로,
        # OFF/ESTOP_LATCHED면 drive로 넘긴다.
        self.active_target_pub.publish(String(data='arm' if enabled else 'drive'))
        self.get_logger().info(f'Operator control mode: {self.mode}')

    def on_hardware_fault(self, msg):
        reason = msg.data.strip() or 'unspecified hardware fault'
        self.trigger_estop(reason)

    def trigger_estop(self, reason='operator emergency stop'):
        if self.mode == self.ESTOP_LATCHED:
            return
        self.mode = self.ESTOP_LATCHED
        self.publish_mode()
        self.protective_stop_pub.publish(Bool(data=True))
        self.estop_pub.publish(Bool(data=True))
        self.get_logger().fatal(
            'PROTECTIVE E-STOP LATCHED: blocking operator commands and holding position '
            f'with motor torque enabled. Reason: {reason}. Restart bringup to recover.')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
