"""Local-goal selection shared by training and deployment references.

The adaptive rule uses only the reference path and robot pose. It never queries
obstacle ground truth, so a deployment route manager can apply the same rule to
the global path it already owns.
"""
import math
import numpy as np


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _index_at_arc(arc, index, distance):
    target = arc[index] + distance
    return min(len(arc) - 1, int(np.searchsorted(arc, target)))


def path_turn_angle(path, arc, index, scan_distance):
    """Maximum direction change inside the next scan_distance of the path."""
    end = _index_at_arc(arc, index, scan_distance)
    if end <= index:
        return 0.0
    segment = np.diff(path[index:end + 1], axis=0)
    lengths = np.linalg.norm(segment, axis=1)
    keep = lengths > 1e-6
    if np.count_nonzero(keep) < 2:
        return 0.0
    directions = np.arctan2(segment[keep, 1], segment[keep, 0])
    unwrapped = np.unwrap(directions)
    return float(np.max(unwrapped) - np.min(unwrapped))


def chord_distance_to_path(start, target, path, index, end, corridor):
    """Maximum distance from a straight chord to the local reference path."""
    samples = np.linspace(start, target, max(2, int(np.linalg.norm(target - start) / 0.05) + 1))
    local = path[max(0, index - 2):min(len(path), end + 3)]
    if len(local) == 0:
        return math.inf
    distances = np.linalg.norm(samples[:, None, :] - local[None, :, :], axis=2)
    return float(np.max(np.min(distances, axis=1)))


def adaptive_lookahead(path, arc, pose, index, cfg):
    selection = cfg['environment']['goal_selection']
    base = float(cfg['environment']['lookahead'])
    minimum = float(selection['min_lookahead'])
    threshold = float(selection['turn_angle_threshold'])
    turn = path_turn_angle(path, arc, index, float(selection['turn_scan_distance']))
    if turn <= threshold:
        return base, turn
    span = max(1e-6, math.pi / 2 - threshold)
    fraction = min(1.0, (turn - threshold) / span)
    lookahead = base - fraction * (base - minimum)
    return max(minimum, lookahead), turn


def select_local_goal(path, arc, pose, cfg, path_index):
    """Return (path_index, point) for the current local goal.

    ``fixed`` reproduces the original constant lookahead. ``adaptive`` shortens
    the lookahead at turns and backs off to the farthest preceding route point
    whose straight chord remains inside the path corridor.
    """
    path = np.asarray(path, dtype=float)
    pose = np.asarray(pose, dtype=float)
    index = int(np.clip(path_index, 0, len(path) - 1))
    mode = cfg['environment']['goal_selection']['mode']
    if mode == 'fixed':
        target = _index_at_arc(arc, index, float(cfg['environment']['lookahead']))
        return target, path[target]
    if mode != 'adaptive':
        raise ValueError('goal_selection.mode must be fixed or adaptive')
    lookahead, _ = adaptive_lookahead(path, arc, pose, index, cfg)
    end = _index_at_arc(arc, index, lookahead)
    corridor = float(cfg['environment']['goal_selection']['direct_corridor'])
    candidate = end
    minimum_index = _index_at_arc(arc, index, float(cfg['environment']['goal_selection']['min_lookahead']) * 0.5)
    while candidate > minimum_index:
        distance = chord_distance_to_path(pose[:2], path[candidate], path, index, candidate, corridor)
        if distance <= corridor:
            return candidate, path[candidate]
        candidate -= 1
    return candidate, path[candidate]
