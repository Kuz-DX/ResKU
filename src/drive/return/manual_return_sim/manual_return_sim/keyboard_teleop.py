"""
keyboard_teleop

Pre-flight virtual test tool (NOT for production). Drive the simulated robot
by hand from the keyboard so you can draw arbitrary paths, then trigger
RETURN and watch the robot retrace them in RViz.

Publishes:
    /motor_speed_cmd_manual  (std_msgs/Float32MultiArray, [left_dps, right_dps])
    /path/record              (std_msgs/Bool)  -- on 'g'
    /path/return               (std_msgs/Bool)  -- on 'r'
Subscribes:
    /mission/return/state    (std_msgs/String) -- shown in the status line

[UI boolean interface, 2026-09] /path/record and /path/return replace the
old /mission/return/trigger. return_state_machine_node no longer starts
recording automatically the instant odometry appears -- 'g' marks the
current position as the (0,0) origin and starts recording explicitly,
matching what the UI team's spec does. Both keys pulse True then False
0.3s later (see _fire_trigger/_publish below), mirroring the "resets to
default after the action runs" behaviour the UI is expected to have --
but return_state_machine_node's own correctness never depends on that
pulse actually arriving (it only acts on a False->True edge, see that
node's docstring).

Latching keys (the command keeps running until you press another key):
    w / s   forward / backward
    a / d   turn in place left / right
    q / e   curve forward-left / forward-right
    z / c   curve backward-left / backward-right
    space   stop
    + / -   speed up / down (10 dps steps)
    g       start recording (marks current position as origin)
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
  g    start recording   r    RETURN            x    quit
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
        self.record_pub = self.create_publisher(Bool, '/path/record', 10)
        self.return_pub = self.create_publisher(Bool, '/path/return', 10)
        self.create_subscription(String, '/mission/return/state', self._on_state, 10)

        self.left = 0.0
        self.right = 0.0
        # each entry: [publisher, monotonic time to send the False release]
        self._pending_releases = []
        self.label = 'stop'
        self.state = '?'

    def _on_state(self, msg: String):
        self.state = msg.data

    def _fire_trigger(self, pub, label: str):
        """False right before True, then True, then False again 0.3s later.

        The leading False is defensive, not load-bearing here (nothing
        else on this process writes to these topics, so the value is
        already False from this node's own previous release) -- but it's
        the pattern documented for the UI team, since a UI CAN'T assume
        that: another client might have left a stray True on the topic,
        or this might be the very first press this session. Sending False
        immediately before True makes the resulting edge unconditional
        regardless of prior state. The trailing False afterward is
        separate: it's what makes the topic's resting value satisfy the
        UI spec's "resets to default after the action runs", for anyone
        else reading the topic -- return_state_machine_node itself never
        needs it (it only acts on the edge, see that node's docstring)."""
        pub.publish(Bool(data=False))
        pub.publish(Bool(data=True))
        self._pending_releases.append([pub, time.monotonic() + 0.3])
        self.label = label

    def _publish(self):
        now = time.monotonic()
        still_pending = []
        for pub, off_at in self._pending_releases:
            if now >= off_at:
                pub.publish(Bool(data=False))
            else:
                still_pending.append([pub, off_at])
        self._pending_releases = still_pending
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
        elif key == 'g':
            self._fire_trigger(self.record_pub, 'RECORD sent')
        elif key == 'r':
            self._set(0, 0, 'stop')
            self._publish()
            self._fire_trigger(self.return_pub, 'RETURN sent')
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
