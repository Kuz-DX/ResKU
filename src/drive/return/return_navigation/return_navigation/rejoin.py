"""Conservative rejoin check on the remaining segment before the next corner."""
import math


def distance_to_remaining_segment(path, current, floor, hi):
    if len(path) < 2 or floor >= hi:
        return float('inf')
    best = float('inf')
    for i in range(max(0, floor), min(hi, len(path)-1)):
        a, b = path[i], path[i+1]
        dx, dy = b.x-a.x, b.y-a.y
        length2 = dx*dx+dy*dy
        if length2 < 1e-12:
            continue
        t = max(0., min(1., ((current.x-a.x)*dx+(current.y-a.y)*dy)/length2))
        best = min(best, math.hypot(current.x-a.x-t*dx, current.y-a.y-t*dy))
    return best
