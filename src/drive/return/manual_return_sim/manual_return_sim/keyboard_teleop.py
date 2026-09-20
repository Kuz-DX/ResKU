"""
keyboard_teleop

Pre-flight virtual test tool (NOT for production). Drive the simulated robot
by hand from the keyboard so you can draw arbitrary paths, then trigger
RETURN and watch the robot retrace them in RViz.

Publishes:
    /motor_speed_cmd_manual  (std_msgs/Float32MultiArray, [left_dps, right_dps])
    /mission/return/trigger  (std_msgs/Bool)  -- on 'r'
Subscribes:
    /mission/return/state    (std_msgs/String) -- shown in the status line

Latching keys (the command keeps running until you press another key):
    w / s   forward / backward
    a / d   turn in place left / right
    q / e   curve forward-left / forward-right
    z / c   curve backward-left / backward-right
    space   stop
    + / -   speed up / down (10 dps steps)
    r       RETURN (stops first, then triggers the return sequence)
    x       stop and quit

Must be run in a real terminal (needs a TTY for raw key input):
    ros2 run manual_return_sim keyboard_teleop
"""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

HELP = """
  w/s  forward/back      a/d  turn in place (left/right)
  q/e  curve fwd L/R     z/c  curve back L/R
  space stop             +/-  speed up/down
  r    RETURN            x    quit
"""

CURVE_RATIO = 0.55


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('speed_dps', 150.0)
        self.declare_parameter('speed_step_dps', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.speed = float(self.get_parameter('speed_dps').value)
        self.step = float(self.get_parameter('speed_step_dps').value)
        self.period = 1.0 / float(self.get_parameter('publish_rate_hz').value)

        self.cmd_pub = self.create_publisher(Float32MultiArray, '/motor_speed_cmd_manual', 10)
        self.trigger_pub = self.create_publisher(Bool, '/mission/return/trigger', 10)
        self.create_subscription(String, '/mission/return/state', self._on_state, 10)

        self.left = 0.0
        self.right = 0.0
        self._trigger_off_at = None
        self.label = 'stop'
        self.state = '?'

    def _on_state(self, msg: String):
        self.state = msg.data

    def _publish(self):
        if self._trigger_off_at is not None and time.monotonic() >= self._trigger_off_at:
            # release the trigger so the next press is a fresh False->True edge
            self.trigger_pub.publish(Bool(data=False))
            self._trigger_off_at = None
        msg = Float32MultiArray()
        msg.data = [float(self.left), float(self.right)]
        self.cmd_pub.publish(msg)

    def _set(self, left_factor: float, right_factor: float, label: str):
        self.left = left_factor * self.speed
        self.right = right_factor * self.speed
        self.label = label

    def handle_key(self, key: str) -> bool:
        """Returns False when the user asked to quit."""
        if key == 'w':
            self._set(1, 1, 'forward')
        elif key == 's':
            self._set(-1, -1, 'backward')
        elif key == 'a':
            self._set(-1, 1, 'turn left')
        elif key == 'd':
            self._set(1, -1, 'turn right')
        elif key == 'q':
            self._set(CURVE_RATIO, 1, 'curve fwd-left')
        elif key == 'e':
            self._set(1, CURVE_RATIO, 'curve fwd-right')
        elif key == 'z':
            self._set(-CURVE_RATIO, -1, 'curve back-left')
        elif key == 'c':
            self._set(-1, -CURVE_RATIO, 'curve back-right')
        elif key == ' ':
            self._set(0, 0, 'stop')
        elif key in ('+', '='):
            self._rescale(self.speed + self.step)
        elif key in ('-', '_'):
            self._rescale(max(self.step, self.speed - self.step))
        elif key == 'r':
            self._set(0, 0, 'stop')
            self._publish()
            self.trigger_pub.publish(Bool(data=True))
            self._trigger_off_at = time.monotonic() + 0.3
            self.label = 'RETURN sent'
        elif key == 'x' or key == '\x03':
            return False
        return True

    def _rescale(self, new_speed: float):
        old = self.speed
        self.speed = new_speed
        if old > 1e-6:
            self.left *= new_speed / old
            self.right *= new_speed / old

    def status_line(self) -> str:
        return (f'\r[{self.state:<20}] {self.label:<16} '
                f'L={self.left:6.1f} R={self.right:6.1f} dps  speed={self.speed:5.1f}   ')


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop needs a real terminal (TTY). Run it directly in a terminal:\n'
              '  ros2 run manual_return_sim keyboard_teleop')
        return 1

    rclpy.init(args=args)
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        next_pub = time.monotonic()
        running = True
        while running and rclpy.ok():
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key = sys.stdin.read(1)
                running = node.handle_key(key.lower() if key.isalpha() else key)
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now >= next_pub:
                node._publish()
                sys.stdout.write(node.status_line())
                sys.stdout.flush()
                next_pub = now + node.period
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.left = node.right = 0.0
        node._publish()
        print('\nstopped.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
