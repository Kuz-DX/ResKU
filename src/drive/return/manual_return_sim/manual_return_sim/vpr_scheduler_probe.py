"""Read-only telemetry around the unchanged production scheduler."""
import json
import rclpy
from std_msgs.msg import String
from visual_landmark.visual_check_scheduler_node import VisualCheckScheduler


class Probe(VisualCheckScheduler):
    def __init__(self):
        super().__init__()
        self.probe=self.create_publisher(String,'/sim/vpr/watchdog',10)
        self.create_timer(1.,self.report)

    def report(self):
        data=dict(state=self.state,distance=self.watch.distance,pending=self.watch.pending,
                  odom_age=None if self.odom_time is None else self.now()-self.odom_time,
                  enabled=self.p('enable_adaptive_visual_check'),start_check=self.start_check)
        self.probe.publish(String(data=json.dumps(data)))
        if self.state=='FOLLOW_RETURN_PATH':self.get_logger().info('SIM WATCHDOG '+json.dumps(data))


def main(args=None):
    rclpy.init(args=args);n=Probe()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
