"""Ordered return checkpoints; arrival uses controller route arc, never global XY."""
import math


class CheckpointSequence:
    def __init__(self, approach=.35, passed=.75):
        self.approach, self.passed = approach, passed
        self.points = []
        self.done = set()
        self.active = None

    def reset(self):
        self.points, self.done, self.active = [], set(), None

    def load(self, points):
        parsed = []
        for point in points:
            lid, arc, order = str(point['id']), float(point['arc']), int(point['order'])
            if not lid or not math.isfinite(arc) or arc < 0:
                raise ValueError('invalid checkpoint')
            parsed.append(dict(id=lid, arc=arc, order=order))
        if len({p['id'] for p in parsed}) != len(parsed):
            raise ValueError('duplicate checkpoint')
        parsed.sort(key=lambda p: (-p['order'], p['id']))
        if any(b['arc'] < a['arc']-1e-6 for a,b in zip(parsed, parsed[1:])):
            raise ValueError('checkpoint arcs disagree with reversed order')
        self.points = parsed

    def next(self):
        return next((p for p in self.points if p['id'] not in self.done), None)

    def due(self, progress):
        """Return (due point, missed ids); each point receives one bounded check."""
        missed = []
        if self.active is not None or not math.isfinite(progress):
            return None, missed
        while (point := self.next()) is not None:
            if progress > point['arc']+self.passed:
                self.done.add(point['id'])
                missed.append(point['id'])
            elif progress >= point['arc']-self.approach:
                return point, missed
            else:
                break
        return None, missed

    def finish(self):
        if self.active is not None:
            self.done.add(self.active)
            self.active = None
