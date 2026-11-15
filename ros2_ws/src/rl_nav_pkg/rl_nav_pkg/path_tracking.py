"""ROS-independent path validity, visible carrots, and final-goal settling."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


def chord_is_free(start, end, field, origin, resolution, radius, yaw=0.0):
    count = max(2, int(math.ceil(np.linalg.norm(np.asarray(end) - start) / (resolution / 2))) + 1)
    points = np.linspace(start, end, count) - origin
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c * points[:, 0] + s * points[:, 1], -s * points[:, 0] + c * points[:, 1]
    guard = radius + resolution * (math.sqrt(2) / 2 + 0.25)
    boundary = np.minimum.reduce([x, y, field.shape[1] * resolution - x,
                                  field.shape[0] * resolution - y])
    # The supplied home world spawns on the map edge. Permit an inward escape
    # only when overlap never grows and the selected endpoint is fully inside.
    # Occupancy/unknown clearance below is never relaxed.
    required = min(guard, float(boundary[0]))
    if (required <= 0 or np.any(boundary < required - 1e-8)
            or boundary[-1] < guard):
        return False
    cols, rows = np.floor(x / resolution).astype(int), np.floor(y / resolution).astype(int)
    return bool(np.all(field[rows, cols] * resolution > guard))


class PathTracker:
    def __init__(self, lookahead=1.2, radius=0.226, margin=0.04, mode='fixed',
                 goal_radius=0.3, hold_seconds=1.0, path_timeout=5.0):
        if mode not in ('fixed', 'visible') or min(lookahead, radius, goal_radius, hold_seconds, path_timeout) <= 0:
            raise ValueError('Invalid path tracking configuration')
        self.lookahead, self.radius, self.margin = lookahead, radius, margin
        self.mode, self.goal_radius = mode, goal_radius
        self.hold_seconds, self.path_timeout = hold_seconds, path_timeout
        self.generation, self.index = 0, 0
        self.path = self.received = self.within_since = self.last_sim = None
        self.field = self.origin = self.resolution = None
        self.origin_yaw = 0.0
        self.fresh_path = False

    def clear(self):
        self.generation += 1
        self.path = self.received = self.within_since = None
        self.index = 0

    def set_map(self, grid, origin, resolution, origin_yaw=0.0):
        if resolution <= 0 or not math.isfinite(resolution):
            raise ValueError('Invalid map resolution')
        blocked = (np.asarray(grid) < 0) | (np.asarray(grid) >= 50)
        self.field = distance_transform_edt(~blocked) if np.any(blocked) else np.full(blocked.shape, np.inf)
        self.origin = np.asarray(origin, dtype=float)
        self.resolution, self.origin_yaw = resolution, origin_yaw

    def set_path(self, points, now_steady):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1:] != (2,) or len(points) == 0 or not np.all(np.isfinite(points)):
            self.clear()
            return False
        same_goal = self.path is not None and np.linalg.norm(self.path[-1] - points[-1]) <= 0.02
        if not same_goal:
            self.generation += 1
            self.within_since = None
        self.path = points.copy()
        self.arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        self.index, self.received, self.fresh_path = 0, now_steady, True
        return True

    def invalid_status(self, reason):
        return {'ready': False, 'reached': False, 'reason': reason,
                'goal_generation': self.generation, 'final_distance': None}

    def update(self, position, now_sim, now_steady):
        if self.last_sim is not None and now_sim < self.last_sim - 0.5:
            self.clear()
            self.last_sim = now_sim
            return None, self.invalid_status('clock_reset')
        self.last_sim = now_sim
        if self.path is None:
            return None, self.invalid_status('no_path')
        if self.received is None or now_steady - self.received > self.path_timeout:
            self.within_since = None
            return None, self.invalid_status('path_stale')
        position = np.asarray(position, dtype=float)
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            return None, self.invalid_status('invalid_pose')
        distance = float(np.linalg.norm(self.path[-1] - position))
        status = {'ready': True, 'reached': False, 'reason': 'tracking',
                  'goal_generation': self.generation, 'final_distance': distance}
        if distance <= self.goal_radius:
            if self.within_since is None:
                self.within_since = now_sim
            reached = now_sim - self.within_since >= self.hold_seconds - 1e-8
            status.update(ready=False, reached=reached, reason='goal_reached' if reached else 'goal_hold',
                          within_for=max(0.0, now_sim - self.within_since))
            return self.path[-1].copy(), status
        self.within_since = None
        lo = 0 if self.fresh_path else max(0, self.index - 4)
        hi = len(self.path) if self.fresh_path else min(len(self.path), self.index + 50)
        self.index = lo + int(np.argmin(np.linalg.norm(self.path[lo:hi] - position, axis=1)))
        self.fresh_path = False
        target = min(float(self.arc[-1]), float(self.arc[self.index]) + self.lookahead)
        far = min(len(self.path) - 1, int(np.searchsorted(self.arc, target)))
        if self.mode == 'fixed':
            return self.path[far].copy(), status
        if self.field is None:
            return None, self.invalid_status('no_map')
        # A stop can leave the footprint inside the visibility margin; retry
        # with the bare radius so the robot can leave such a pose without
        # relaxing the obstacle clearance of the selected path point.
        for guard in (self.radius + self.margin, self.radius):
            for j in range(far, self.index, -2):
                if chord_is_free(position, self.path[j], self.field, self.origin, self.resolution,
                                 guard, self.origin_yaw):
                    return self.path[j].copy(), status
            j = min(self.index + 1, len(self.path) - 1)
            if chord_is_free(position, self.path[j], self.field, self.origin, self.resolution,
                             guard, self.origin_yaw):
                return self.path[j].copy(), status
        # AMCL drift can leave the mapped footprint touching an obstacle, so
        # every chord fails at the start cell. Escape along the planner's path
        # with a short carrot; full visibility returns once a chord is clear.
        # A clear pose that merely has no visible chord stays stopped.
        column = int(math.floor((position[0] - self.origin[0]) / self.resolution))
        row = int(math.floor((position[1] - self.origin[1]) / self.resolution))
        if 0 <= row < self.field.shape[0] and 0 <= column < self.field.shape[1]:
            min_guard = self.radius + self.resolution * (math.sqrt(2) / 2 + 0.25)
            if self.field[row, column] * self.resolution <= min_guard:
                escape = min(float(self.arc[-1]), float(self.arc[self.index]) + 0.4)
                j = min(len(self.path) - 1, int(np.searchsorted(self.arc, escape)))
                if j > self.index:
                    status = dict(status, reason='tracking_escape')
                    return self.path[j].copy(), status
        return None, self.invalid_status('no_visible_carrot')


class GoalEpoch:
    def __init__(self):
        self.current = 0

    def replace(self):
        self.current += 1
        return self.current

    def accepts(self, epoch):
        return epoch == self.current
