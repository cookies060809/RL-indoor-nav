"""Footprint-aware optional path carrots; fixed lookahead remains the default.

The selector uses the same known map as the global planner. It is not an extra
ground-truth input to the learned policy.
"""
import math
import numpy as np
from numba import njit


@njit(cache=True)
def chord_clear(start, end, rectangles, radius):
    """Conservatively sample the circular footprint along a straight chord."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(length / 0.025)))
    guard = radius + length / steps / 2
    for index in range(steps + 1):
        fraction = index / steps
        x = start[0] + fraction * (end[0] - start[0])
        y = start[1] + fraction * (end[1] - start[1])
        for obstacle in range(len(rectangles)):
            dx = max(rectangles[obstacle, 0] - x, 0.0, x - rectangles[obstacle, 2])
            dy = max(rectangles[obstacle, 1] - y, 0.0, y - rectangles[obstacle, 3])
            if dx * dx + dy * dy <= guard * guard:
                return False
    return True


def choose_local_goal(pose, path, arc, index, rectangles, cfg):
    environment = cfg['environment']
    target = min(float(arc[-1]), float(arc[index]) + environment['lookahead'])
    farthest = min(len(path) - 1, int(np.searchsorted(arc, target)))
    mode = environment.get('subgoal_mode', 'fixed')
    if mode == 'fixed':
        return path[farthest].copy(), farthest
    if mode != 'visible':
        raise ValueError('subgoal_mode must be fixed or visible')
    radius = cfg['robot']['radius'] + environment.get('visibility_margin', 0.04)
    for candidate in range(farthest, index, -2):
        if chord_clear(pose[:2], path[candidate], rectangles, radius):
            return path[candidate].copy(), candidate
    candidate = min(index + 1, len(path) - 1)
    if chord_clear(pose[:2], path[candidate], rectangles, radius):
        return path[candidate].copy(), candidate
    # No safe carrot exists: a zero-distance target permits an explicit stop.
    return pose[:2].copy(), index
