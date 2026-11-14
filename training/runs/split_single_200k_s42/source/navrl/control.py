import math
import numpy as np
from .geometry import integrate

def scale_action(action, cfg):
    action=np.asarray(action,dtype=float)
    if action.shape!=(2,) or not np.all(np.isfinite(action)):
        raise ValueError('Action must have two finite components')
    a=np.clip(action,-1,1); r=cfg['robot']
    return np.array([(a[0]+1)*0.5*r['v_max'],a[1]*r['w_max']])

def limit_rate(target, previous, cfg):
    r=cfg['robot']; delta=np.array([r['acceleration'],r['angular_acceleration']])*r['dt']
    return np.asarray(previous)+np.clip(np.asarray(target)-previous,-delta,delta)

def filter_velocity(velocity, ranges, cfg, enabled=None):
    """Scan-only circular-footprint braking filter; does not use map truth.

    Rectangular-footprint sweep and ROS watchdogs still require VM validation.
    """
    p=cfg['safety']; r=cfg['robot']; sensor=cfg['sensor']
    enabled=p['enabled'] if enabled is None else enabled
    v,w=velocity
    if not enabled: return np.array([v,w]),False
    values=np.asarray(ranges); valid=np.isfinite(values)&(values>=sensor['range_min'])&(values<=sensor['range_max'])
    if np.count_nonzero(valid)==0 and not np.any(np.isposinf(values)):
        return np.zeros(2),True
    angles=-np.pi+np.arange(len(values))*2*np.pi/len(values)+sensor['yaw']
    points=np.column_stack([values[valid]*np.cos(angles[valid])+sensor['x'],values[valid]*np.sin(angles[valid])+sensor['y']])
    if len(points)==0: return np.array([v,w]),False
    radius=r['radius']+p['margin']
    # Circle rotation is invariant. Stop both components if already unsafe.
    if np.min(np.linalg.norm(points,axis=1))<radius:
        return np.zeros(2),True
    stopping=v*p['reaction_time']+v*v/(2*p['brake_deceleration'])
    duration=stopping/max(v,1e-8)
    for fraction in np.linspace(0.15,1,7):
        future=integrate(np.zeros(3),v,w,duration*fraction)
        if np.any(np.linalg.norm(points-future[:2],axis=1)<radius):
            return np.array([0.0,w]),True
    return np.array([v,w]),False
