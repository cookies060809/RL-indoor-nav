import math
import numpy as np
from .geometry import integrate
from .scan import classify_scan, scan_angles


def scale_action(action, cfg):
    action = np.asarray(action, dtype=float)
    if action.shape != (2,) or not np.all(np.isfinite(action)):
        raise ValueError('Action must have two finite components')
    clipped = np.clip(action, -1, 1)
    robot = cfg['robot']
    return np.array([(clipped[0] + 1) * 0.5 * robot['v_max'], clipped[1] * robot['w_max']])


def limit_rate(target, previous, cfg):
    robot = cfg['robot']
    delta = np.array([robot['acceleration'], robot['angular_acceleration']]) * robot['dt']
    return np.asarray(previous) + np.clip(np.asarray(target) - previous, -delta, delta)


def _sector_has_unknown(angles, unknown, center, half_width):
    offset = (angles - center + math.pi) % (2 * math.pi) - math.pi
    return bool(np.any(unknown & (np.abs(offset) <= half_width + 1e-9)))


def scan_coverage(velocity, ranges, cfg):
    """Return (linear reason, rotation blocked) for the commanded motion.

    Translation needs the swept half-plane in the direction of motion plus any
    rotation accumulated while braking. Rotation sweeps all bearings, so a
    single unknown ray blocks it. Unknown readings are never replaced by valid
    rays from another direction.
    """
    safety = cfg['safety']
    sensor = cfg['sensor']
    values, hit, free, unknown = classify_scan(ranges, sensor)
    if np.count_nonzero(hit) == 0 and np.count_nonzero(free) == 0:
        return 'no_valid_scan', True
    angles = scan_angles(len(values), sensor)
    v, w = velocity
    linear_reason = None
    if abs(v) > 1e-9:
        brake_time = safety['reaction_time'] + abs(v) / safety['brake_deceleration']
        half_width = min(math.pi, math.pi / 2 + abs(w) * brake_time)
        center = 0.0 if v > 0 else math.pi
        if _sector_has_unknown(angles, unknown, center, half_width):
            linear_reason = 'scan_unknown_front' if v > 0 else 'scan_unknown_rear'
    rotation_blocked = bool(abs(w) > 1e-9 and np.any(unknown))
    return linear_reason, rotation_blocked


def scan_coverage_reason(velocity, ranges, cfg):
    linear_reason, rotation_blocked = scan_coverage(velocity, ranges, cfg)
    if linear_reason:
        return linear_reason
    return 'scan_unknown_rotation' if rotation_blocked else None


def filter_velocity(velocity, ranges, cfg, enabled=None):
    """Scan-only circular-footprint braking filter; does not use map truth."""
    safety = cfg['safety']
    robot = cfg['robot']
    sensor = cfg['sensor']
    enabled = safety['enabled'] if enabled is None else enabled
    v, w = np.asarray(velocity, dtype=float)
    if not enabled:
        return np.array([v, w]), False, 'none'
    if abs(v) < 1e-12 and abs(w) < 1e-12:
        return np.zeros(2), False, 'none'

    coverage, rotation_blocked = scan_coverage([v, w], ranges, cfg)
    if coverage == 'no_valid_scan':
        return np.zeros(2), True, coverage
    if coverage == 'scan_unknown_front' and v > 0:
        v = 0.0
    if coverage == 'scan_unknown_rear' and v < 0:
        v = 0.0
    if rotation_blocked:
        w = 0.0
    if coverage is None and rotation_blocked:
        coverage = 'scan_unknown_rotation'

    values, hit, free, unknown = classify_scan(ranges, sensor)
    angles = scan_angles(len(values), sensor)
    points = np.column_stack([
        values[hit] * np.cos(angles[hit]) + sensor['x'],
        values[hit] * np.sin(angles[hit]) + sensor['y'],
    ])
    if len(points):
        radius = robot['radius'] + safety['margin']
        if np.min(np.linalg.norm(points, axis=1)) < radius:
            return np.zeros(2), True, 'footprint_overlap'
        if abs(v) > 1e-9:
            stopping = abs(v) * safety['reaction_time'] + v * v / (2 * safety['brake_deceleration'])
            duration = stopping / max(abs(v), 1e-8)
            for fraction in np.linspace(0.15, 1, 7):
                future = integrate(np.zeros(3), v, w, duration * fraction)
                if np.any(np.linalg.norm(points - future[:2], axis=1) < radius):
                    v = 0.0
                    coverage = coverage or 'braking_sweep'
                    break
    executed = np.array([v, w])
    intervened = bool(coverage is not None or np.any(executed != np.asarray(velocity, dtype=float)))
    return executed, intervened, coverage or ('filtered' if intervened else 'none')
