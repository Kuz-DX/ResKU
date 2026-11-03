#!/usr/bin/env python3
"""Keep standalone manual controls while the manager owns motor output."""
import signal

import rclpy

from managed_manual import ManagedManual
from manual_control_common import ManualController


class StandaloneManagedManual(ManagedManual):
    local_tool_paths = True

    def poll_controller(self, now):
        super().poll_controller(now)
        if not self.controller_ready:
            if self.docking is not None and self.docking.selected:
                self.docking.cancel('manual ownership changed')
        else:
            self.startup_goal = self.manager_status.get('startup_goal_rad')

    def on_joy(self, message):
        # Options/focus remains available during automation. Other manual
        # buttons cannot start a competing local path, alignment or latch pulse.
        if self.manager_status.get('mode') != 'manual' or not self.controller_ready:
            ManualController.on_joy(self, message)
            self.path_combos = [False]*3
            self.previous_stop_pose = True
            self.previous_docking_buttons = (True, True)
            return
        super().on_joy(message)


def main():
    rclpy.init()
    node = StandaloneManagedManual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_controller()
        if node.docking is not None:
            node.docking.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
