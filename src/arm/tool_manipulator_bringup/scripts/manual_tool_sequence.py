#!/usr/bin/env python3
"""Topic-triggered managed docking with automatic return to manual ownership."""
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from std_srvs.srv import Trigger

from tool_docking_coordinator import ToolDockingCoordinator
from tool_ids import TOOL_IDS


class ManualToolSequence(ToolDockingCoordinator):
    def __init__(self):
        self.pending_selection = None
        self.handoff_future = None
        self.return_to_manual = False
        self.resume_pending = False
        super().__init__()
        self.manual_mode = self.create_client(Trigger, '/arm/set_mode/manual')

    def on_joy(self, message):
        # This producer accepts topic commands only. Manual keeps its mappings.
        pass

    def on_selected_tool(self, message):
        if self.stage != 'idle' or self.pending_selection is not None or self.resume_pending:
            self.report('ignored', 'tool sequence or ownership handoff already active')
            return
        if message.data not in TOOL_IDS or message.data not in self.paths:
            self.report('blocked', 'tool id is not execution-enabled')
            return
        target = self.pre_dock_targets.get(message.data, {})
        if target.get('blocked_reason'):
            self.report('blocked', target['blocked_reason'])
            return
        now = time.monotonic()
        s = self.status
        if (now-self.status_time >= .3 or now-self.state_time >= .3
                or self.full_state is None or not s.get('ready')
                or not s.get('startup_complete') or not s.get('entry_alignment_complete')
                or s.get('focus') != 'arm' or s.get('transitioning')
                or s.get('mode') != 'manual' or s.get('requested') != 'manual'):
            self.report('blocked', 'fresh ready manual arm focus is required')
            return
        if not self.cancel.service_is_ready() or not self.manual_mode.service_is_ready():
            self.report('blocked', 'mode services unavailable')
            return
        self.pending_selection = message
        self.handoff_deadline = now+5.
        self.return_to_manual = True
        self.handoff_future = self.cancel.call_async(Trigger.Request())
        self.report('staging', 'stopping manual input before HOME -> BACK -> tool')

    def report(self, state, reason):
        super().report(state, reason)
        if state == 'completed' and self.return_to_manual:
            self.resume_pending = True
            self.resume_epoch = self.status.get('epoch')
            self.handoff_future = None
            self.handoff_deadline = time.monotonic()+5.
        elif state in ('failed', 'blocked'):
            self.pending_selection = None
            self.return_to_manual = self.resume_pending = False
            self.handoff_future = None

    def fail(self, reason):
        if (self.pending_selection is not None or self.resume_pending) and self.cancel.service_is_ready():
            self.cancel.call_async(Trigger.Request())
        super().fail(reason)

    def tick(self):
        if self.pending_selection is not None:
            now = time.monotonic()
            try:
                if now > self.handoff_deadline:
                    raise RuntimeError('manual-to-HOLD handoff timeout')
                if self.status.get('focus') != 'arm' or self.status.get('mode') == 'fault':
                    raise RuntimeError('manual-to-HOLD handoff cancelled')
                if self.handoff_future.done():
                    response = self.handoff_future.result()
                    if response is None or not response.success:
                        raise RuntimeError('HOLD request rejected')
                    try:
                        self.healthy_hold(now)
                    except RuntimeError:
                        return
                    message, self.pending_selection = self.pending_selection, None
                    self.handoff_future = None
                    super().on_selected_tool(message)
            except Exception as exc:
                self.fail(str(exc))
            return
        super().tick()
        if self.resume_pending and self.stage == 'idle':
            try:
                now = time.monotonic()
                if now > self.handoff_deadline:
                    raise RuntimeError('automatic return to manual timed out; remaining in HOLD')
                if self.status.get('focus') != 'arm' or self.status.get('mode') == 'fault':
                    raise RuntimeError('automatic return to manual cancelled')
                if self.handoff_future is None:
                    if self.status.get('epoch') != self.resume_epoch:
                        raise RuntimeError('HOLD ownership changed; automatic resume cancelled')
                    try:
                        self.healthy_hold(now)
                    except RuntimeError:
                        return
                    if not self.manual_mode.service_is_ready():
                        raise RuntimeError('manual mode service unavailable')
                    self.handoff_future = self.manual_mode.call_async(Trigger.Request())
                elif self.handoff_future.done():
                    response = self.handoff_future.result()
                    if response is None or not response.success:
                        raise RuntimeError('manual mode request rejected')
                    if (now-self.status_time < .3 and self.status.get('mode') == 'manual'
                            and not self.status.get('transitioning')):
                        self.resume_pending = self.return_to_manual = False
                        self.handoff_future = None
                        self.report('manual', 'tool sequence complete; manual controls restored')
            except Exception as exc:
                self.fail(str(exc))

    def on_cancel(self, request, response):
        if self.pending_selection is not None or self.resume_pending:
            self.fail('operator cancelled ownership handoff')
            response.success, response.message = True, 'handoff cancelled; no automatic resume'
            return response
        return super().on_cancel(request, response)


def main():
    rclpy.init()
    node = ManualToolSequence()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node.stage != 'idle' or node.pending_selection is not None or node.resume_pending:
            node.fail('manual tool sequence shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
