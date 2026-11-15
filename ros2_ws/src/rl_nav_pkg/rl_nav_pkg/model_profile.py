"""Load model-owned deployment parameters without importing ROS or PyTorch."""
from pathlib import Path
import math
import re
import yaml


def load_model_profile(project_root, model_id):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', model_id):
        raise ValueError('model_id must be a directory name under models/')
    folder = Path(project_root).resolve() / 'models' / model_id
    if not (folder / 'policy.zip').is_file() or not (folder / 'config.yaml').is_file():
        raise ValueError(f'Model bundle not found: {folder}')
    with (folder / 'config.yaml').open(encoding='utf-8-sig') as handle:
        cfg = yaml.safe_load(handle)
    if cfg.get('spec_version') not in ('bev_nav_v1', 'bev_nav_v2'):
        raise ValueError('Unsupported model observation specification')
    if cfg['observation']['size'] not in (32, 48):
        raise ValueError('Unsupported BEV shape')
    for section, names in [('robot', ('radius', 'v_max', 'w_max', 'dt', 'acceleration', 'angular_acceleration')),
                           ('observation', ('extent', 'distance_clip', 'goal_distance_scale')),
                           ('sensor', ('range_min', 'range_max'))]:
        for name in names:
            value = cfg[section][name]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid model parameter {section}.{name}')
    return cfg


def runtime_parameters(cfg, navigation=False):
    o, r, s, e = cfg['observation'], cfg['robot'], cfg['sensor'], cfg['environment']
    bev = {'use_sim_time': True, 'n': o['size'], 'extent': o['extent'], 'beams': s['beams'],
           'spec_version': cfg['spec_version'], 'robot_radius': r['radius'],
           'distance_clip': o['distance_clip'], 'goal_distance_scale': o['goal_distance_scale'],
           'range_min': s['range_min'], 'range_max': s['range_max'], 'v_max': r['v_max'], 'w_max': r['w_max'],
           'require_local_goal': navigation, 'local_goal_timeout': 1.0}
    safety = {'use_sim_time': True, 'candidate_topic': '/cmd_vel_rl', 'v_max': r['v_max'], 'w_max': r['w_max'],
              'acceleration': r['acceleration'], 'angular_acceleration': r['angular_acceleration'],
              'control_dt': 0.05, 'stale_timeout': 0.5, 'sensor_timeout': 0.5,
              'robot_radius': r['radius'], 'safety_margin': cfg['safety']['margin'],
              'reaction_time': cfg['safety']['reaction_time'], 'brake_deceleration': cfg['safety']['brake_deceleration'],
              'strict_scan_coverage': cfg['spec_version'] == 'bev_nav_v2', 'canonical_beams': s['beams'],
              'range_min': s['range_min'], 'range_max': s['range_max'],
              'sensor_future_tolerance': 0.15}
    local = {'use_sim_time': True, 'lookahead': e['lookahead'], 'publish_period': r['dt'],
             'subgoal_mode': e.get('subgoal_mode', 'fixed'), 'robot_radius': r['radius'],
             'visibility_margin': e.get('visibility_margin', cfg['safety']['margin']),
             'goal_radius': e['goal_radius'], 'goal_hold_seconds': e['goal_hold_seconds'], 'path_timeout': 5.0}
    return bev, safety, local
