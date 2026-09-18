"""Small 2D (SE2) pose-math helpers shared by the manual-recording/return
mission nodes. /odometry/filtered only has an observable [x, y, yaw, vx, wz]
state (see reduced_odom's 5-state EKF) -- roll/pitch are carried in the
output quaternion for display only, so yaw must be extracted with the full
quaternion formula (not a naive atan2(qz, qw), which is only valid for a
pure-yaw quaternion) to correctly ignore any roll/pitch tilt.
"""

import math


def yaw_from_quaternion(q) -> float:
    """q: an object with .x, .y, .z, .w (e.g. geometry_msgs/Quaternion)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float):
    """Returns (x, y, z, w) for a yaw-only rotation (roll=pitch=0)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Pose2D:
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    @classmethod
    def from_ros_pose(cls, pose) -> 'Pose2D':
        return cls(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def to_ros_pose(self):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        return p

    def to_mission_frame(self, origin: 'Pose2D') -> 'Pose2D':
        """T_mission = inverse(origin) o self, both expressed in the same
        (odom) frame -- origin is the mission-frame's own pose in odom,
        captured once when recording starts (see manual_path_recorder_node).
        """
        dx = self.x - origin.x
        dy = self.y - origin.y
        cos0 = math.cos(origin.yaw)
        sin0 = math.sin(origin.yaw)
        x_m = dx * cos0 + dy * sin0
        y_m = -dx * sin0 + dy * cos0
        yaw_m = normalize_angle(self.yaw - origin.yaw)
        return Pose2D(x_m, y_m, yaw_m)

    def distance_to(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)
