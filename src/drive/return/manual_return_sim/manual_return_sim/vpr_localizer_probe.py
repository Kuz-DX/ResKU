"""Simulation telemetry only; matching and acceptance remain production methods."""
import rclpy
from visual_landmark.visual_localization_node import VisualLocalizationNode


class Probe(VisualLocalizationNode):
    def __init__(self):
        super().__init__()
        self.create_timer(2.,self.report)

    def report(self):
        if self.state in {'IDLE','FINISHED'}:return
        self.get_logger().info(f'SIM LOCALIZER state={self.state} refs={len(self.landmarks)} check={self.check} frames={self.check_frames}')

    def event(self,kind,**fields):
        self.get_logger().info(f'SIM LOCALIZER EVENT {kind} {fields.get("reason", "")}')
        super().event(kind,**fields)


def main(args=None):
    rclpy.init(args=args);n=Probe()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
