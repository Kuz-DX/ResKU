"""ROS-independent scheduling. Distances and disagreement are NOT true drift."""
import math
from dataclasses import dataclass

RECORDING = {'MANUAL_RECORDING', 'WAIT_RETURN_COMMAND'}
RETURN = {'STOP_BEFORE_TURN', 'TURN_180', 'FOLLOW_RETURN_PATH', 'RETURN_MANUAL',
          'RETURN_RESUME_STOP', 'RETURN_INTERVENTION', 'RETURN_LANDMARK_SEARCH'}


def fresh(now, stamp, timeout):
    return stamp is not None and 0 <= now-stamp <= timeout


@dataclass
class AutoKeyframes:
    distance: float = 1.0
    yaw_deg: float = 12.
    interval: float = 2.
    min_motion: float = .5
    distance_only: bool = True
    time_only: bool = True
    anchor: tuple = None

    def reset(self):
        self.anchor = None

    def accept(self, now, distance, yaw):
        self.anchor = (now, distance, yaw)

    def reason(self, now, distance, yaw):
        if self.anchor is None:
            return 'initial'
        t, s, a = self.anchor
        if self.time_only:
            return 'interval' if now-t >= self.interval else None
        if distance-s >= self.distance:
            return 'distance'
        if self.distance_only:
            return None
        if abs(math.atan2(math.sin(yaw-a), math.cos(yaw-a))) >= math.radians(self.yaw_deg)-1e-12:
            return 'yaw'
        if now-t >= self.interval and distance-s >= self.min_motion:
            return 'timeout'
        return None


class CheckWatchdog:
    """Single outstanding check, bounded timeout, matching-token result only."""
    def __init__(self, distance=8., cooldown=5., timeout=5., deadband=.15,
                 threshold=.35, decay=.02):
        self.limit, self.cooldown, self.timeout = distance, cooldown, timeout
        self.deadband, self.threshold, self.decay = deadband, threshold, decay
        self.reset()

    def reset(self):
        self.distance = self.score = 0.
        self.previous = None
        self.risk_time = None
        self.pending = None
        self.next_try = -math.inf

    def odom(self, x, y):
        if not all(map(math.isfinite, [x, y])):
            self.previous = None
            return
        if self.previous is not None:
            step = math.hypot(x-self.previous[0], y-self.previous[1])
            if step <= 1.:  # reject discontinuities, never reset odometry
                self.distance += step
        self.previous = (x, y)

    def risk(self, now, wheel, gyro, healthy):
        dt = 0. if self.risk_time is None else now-self.risk_time
        self.risk_time = now
        if not healthy or not 0 < dt <= .5 or not all(map(math.isfinite, [wheel, gyro])):
            # No bridging an outage with an old risk score.
            self.score = 0.
            return
        self.score = max(0., self.score + (max(0., abs(wheel-gyro)-self.deadband)-self.decay)*dt)

    def due(self, now, slip=True):
        if self.pending is not None or now < self.next_try:
            return None
        if self.distance >= self.limit:
            return 'distance'
        if slip and self.score >= self.threshold:
            return 'slip'
        return None

    def start(self, token, now):
        self.pending = (token, now)

    def finish(self, token, now, success):
        if self.pending is None or token != self.pending[0]:
            return False
        if success and not 0 <= now-self.pending[1] < self.timeout:
            return False
        self.pending = None
        self.next_try = now+self.cooldown
        if success:
            self.distance = self.score = 0.
        return True

    def expire(self, now):
        if self.pending and now-self.pending[1] >= self.timeout:
            token = self.pending[0]
            self.finish(token, now, False)
            return token
        return None


def candidate_window(arcs, progress, last_arc=None, backtrack=2., limit=3, expansion=0):
    """Arcs run along RETURN path, not outbound ID order. Cheap bounded worklist.

    Equal-position references (adjacent photos) remain distinct for ambiguity
    verification. A geometry-derived path window, not global XY nearest, sets
    progress. Previously traversed branches are excluded by arc monotonicity.
    """
    eligible = [(lid, arc) for lid, arc in arcs.items()
                if last_arc is None or arc >= last_arc-backtrack]
    ranked = sorted(eligible, key=lambda item: (abs(item[1]-progress), item[1], item[0]))
    # Expansion adds at most two neighbours, and is never a global brute force.
    count = min(7, max(1, int(limit))+min(4, max(0, int(expansion))))
    return ranked[:count]


def reference_arcs(path, metadata):
    """Project using recorded outbound s to disambiguate crossing/parallel branches.

    Return path is PN..P0. Distances are approximate after path simplification;
    within 25cm of the best spatial projection prefer the expected outbound arc.
    Older records without s fall back to geometry and ordered progression.
    """
    total = sum(a.distance_to(b) for a, b in zip(path, path[1:]))
    result, floor = {}, 0.
    for lid, meta in sorted(metadata.items(), key=lambda item: item[1].get('order', 0), reverse=True):
        expected = max(0., min(total, total-float(meta.get('s_along_path', total-floor))))
        x, y = meta['mission_pose']['x'], meta['mission_pose']['y']
        projections, arc = [], 0.
        for a, b in zip(path, path[1:]):
            dx, dy = b.x-a.x, b.y-a.y
            length = math.hypot(dx, dy)
            if length > 1e-9:
                t = max(0., min(1., ((x-a.x)*dx+(y-a.y)*dy)/(length*length)))
                value = arc+t*length
                if value >= floor-1e-6:
                    projections.append((math.hypot(x-a.x-t*dx, y-a.y-t*dy), value))
            arc += length
        if not projections:
            continue
        best_distance = min(d for d, _ in projections)
        near = [(d, s) for d, s in projections if d <= best_distance+.25]
        _, selected = min(near, key=lambda pair: (abs(pair[1]-expected), pair[0]))
        result[lid] = selected
        floor = max(floor, selected)
    return result
