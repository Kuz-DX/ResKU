"""ROS-independent route checkpoints and bounded yaw search."""
import math
from return_navigation.se2 import normalize_angle


def project_route(path, point, lo=0, hi=None):
    """Return (arc length from start, lateral distance), preserving path order."""
    arc, best = 0., (0., float('inf'))
    hi = len(path)-1 if hi is None else hi
    for i, (a, b) in enumerate(zip(path, path[1:])):
        dx, dy = b.x-a.x, b.y-a.y
        length = math.hypot(dx, dy)
        if length > 1e-9 and lo <= i < hi:
            t = max(0., min(1., ((point.x-a.x)*dx+(point.y-a.y)*dy)/(length*length)))
            distance = math.hypot(point.x-a.x-t*dx, point.y-a.y-t*dy)
            if distance < best[1]:
                best = (arc+t*length, distance)
        arc += length
    return best


class LandmarkWatch:
    def __init__(self):
        self.references = {}
        self.seen = set()
        self.attempted = set()
        self.near_since = {}

    def missed(self, path, current, now, lo, hi, window=1., wait_s=4., goal_margin=.4):
        if len(path) < 2:
            return None
        progress, _ = project_route(path, current, lo, hi)
        total = sum(a.distance_to(b) for a, b in zip(path, path[1:]))
        ordered = []
        for lid, point in self.references.items():
            if lid in self.seen or lid in self.attempted:
                continue
            checkpoint, _ = project_route(path, point)
            ordered.append((checkpoint, lid))
        for checkpoint, lid in sorted(ordered):
            near = progress >= max(0., checkpoint-window)
            if not near:
                self.near_since.pop(lid, None)
                continue
            self.near_since.setdefault(lid, now)
            passed = progress >= min(checkpoint+window, max(0., total-goal_margin))
            timed_out = now-self.near_since[lid] >= wait_s
            if passed or timed_out:
                self.attempted.add(lid)  # one automatic sweep per checkpoint per mission
                return lid
        return None


class YawSearch:
    """Stop and dwell at 0,+angle,-angle,0; feedback yaw, never open-loop timing."""
    def __init__(self, yaw, now, angle=math.pi/6, speed=.15, dwell=3., timeout=60.):
        if not (0 < angle <= math.pi/2 and 0 < speed <= .5 and dwell >= 1 and timeout > dwell):
            raise ValueError('invalid search angle, speed, dwell or timeout')
        self.start_yaw, self.started = yaw, now
        self.targets = [0., angle, -angle, 0.]
        self.index = 0
        self.speed, self.dwell, self.timeout = speed, dwell, timeout
        self.hold_since = None
        self.candidate_target = None
        self.done = False

    def candidate(self, yaw):
        if self.hold_since is None and self.candidate_target is None:
            self.candidate_target = yaw

    def tick(self, yaw, vx, wz, now):
        if now-self.started >= self.timeout:
            return 0., 'timeout'
        target = self.candidate_target
        if target is None:
            target = self.start_yaw+self.targets[self.index]
        error = normalize_angle(target-yaw)
        if abs(error) > math.radians(2.):
            self.hold_since = None
            # A feedback overshoot outside the search envelope is a failure, not a new turn.
            if abs(normalize_angle(yaw-self.start_yaw)) > max(abs(t) for t in self.targets)+math.radians(5.):
                return 0., 'angle_limit'
            return max(-self.speed, min(self.speed, 1.5*error)), 'turning'
        if abs(vx) > .03 or abs(wz) > .03:
            self.hold_since = None
            return 0., 'settling'
        if self.hold_since is None:
            self.hold_since = now
        if now-self.hold_since < self.dwell:
            return 0., 'observing'
        if self.candidate_target is not None:
            self.candidate_target = None
        elif self.index == len(self.targets)-1:
            self.done = True
            return 0., 'not_found'
        else:
            self.index += 1
        self.hold_since = None
        return 0., 'settling'
