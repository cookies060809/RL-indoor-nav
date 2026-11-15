from pathlib import Path
import copy
import hashlib
import json
import math
import yaml

ROOT = Path(__file__).resolve().parents[3]
CURRENT_SPEC = 'bev_nav_v2'
STAGES = ('empty', 'single', 'turn', 'clutter')


def load_config(path=None, strict=True):
    path = Path(path) if path else ROOT / 'config/base.yaml'
    with path.open(encoding='utf-8-sig') as handle:
        cfg = yaml.safe_load(handle)
    if strict:
        validate_config(cfg)
    return cfg


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _positive(cfg, section, key):
    value = cfg[section][key]
    _require(isinstance(value, (int, float)) and math.isfinite(value) and value > 0,
             f'{section}.{key} must be a positive finite number')
    return float(value)


def _nonnegative(cfg, section, key):
    value = cfg[section][key]
    _require(isinstance(value, (int, float)) and math.isfinite(value) and value >= 0,
             f'{section}.{key} must be a non-negative finite number')
    return float(value)


def validate_config(cfg, require_current=True):
    _require(isinstance(cfg, dict), 'Config must be a mapping')
    for key in ('spec_version', 'seed', 'observation', 'robot', 'sensor', 'safety',
                'environment', 'reward', 'ppo', 'training'):
        _require(key in cfg, f'Missing config section: {key}')
    if require_current:
        _require(cfg['spec_version'] == CURRENT_SPEC,
                 f'Config spec_version must be {CURRENT_SPEC}; create a new config instead of editing an old snapshot')

    observation = cfg['observation']
    _require(observation['size'] in (32, 48), 'BEV size must be 32 or 48; model must be retrained when changed')
    _positive(cfg, 'observation', 'extent')
    _positive(cfg, 'observation', 'distance_clip')
    _positive(cfg, 'observation', 'goal_distance_scale')

    _positive(cfg, 'robot', 'radius')
    _positive(cfg, 'robot', 'v_max')
    _positive(cfg, 'robot', 'w_max')
    _positive(cfg, 'robot', 'acceleration')
    _positive(cfg, 'robot', 'angular_acceleration')
    _positive(cfg, 'robot', 'dt')
    _require(observation['extent'] > 2 * cfg['robot']['radius'],
             'BEV extent must contain the robot footprint')

    sensor = cfg['sensor']
    _require(isinstance(sensor['beams'], int) and sensor['beams'] >= 8, 'sensor.beams must be an integer >= 8')
    _positive(cfg, 'sensor', 'range_min')
    _positive(cfg, 'sensor', 'range_max')
    _require(sensor['range_min'] < sensor['range_max'], 'sensor.range_min must be below range_max')
    _nonnegative(cfg, 'sensor', 'noise_std')
    for key in ('x', 'y', 'yaw'):
        _require(isinstance(sensor[key], (int, float)) and math.isfinite(sensor[key]),
                 f'sensor.{key} must be finite')

    _nonnegative(cfg, 'safety', 'margin')
    _nonnegative(cfg, 'safety', 'reaction_time')
    _positive(cfg, 'safety', 'brake_deceleration')

    environment = cfg['environment']
    _require(environment['stage'] in ('mixed',) + STAGES, f'Unknown curriculum stage: {environment["stage"]}')
    _positive(cfg, 'environment', 'room_half_size')
    _require(environment['room_half_size'] > cfg['robot']['radius'], 'room_half_size must exceed robot radius')
    _positive(cfg, 'environment', 'planner_resolution')
    _nonnegative(cfg, 'environment', 'planner_margin')
    _positive(cfg, 'environment', 'lookahead')
    _positive(cfg, 'environment', 'max_steps')
    _require(environment['timeout_factor'] >= 1.0, 'environment.timeout_factor must be >= 1')
    _positive(cfg, 'environment', 'goal_radius')
    _nonnegative(cfg, 'environment', 'goal_hold_seconds')
    _positive(cfg, 'environment', 'stuck_seconds')
    _nonnegative(cfg, 'environment', 'turn_grace_seconds')
    _nonnegative(cfg, 'environment', 'turn_rate_threshold')
    _nonnegative(cfg, 'environment', 'progress_epsilon')
    _positive(cfg, 'environment', 'min_goal_distance')
    _positive(cfg, 'environment', 'max_goal_distance')
    _require(environment['min_goal_distance'] <= environment['max_goal_distance'],
             'environment.min_goal_distance must not exceed max_goal_distance')
    _require(environment['max_goal_distance'] <= 2 * environment['room_half_size'] * math.sqrt(2),
             'environment.max_goal_distance exceeds the room diagonal')

    reward = cfg['reward']
    _positive(cfg, 'reward', 'progress')
    _positive(cfg, 'reward', 'clearance_threshold')
    _require(reward['success'] > 0, 'reward.success must be positive')
    _require(reward['collision'] < 0, 'reward.collision must be negative')

    ppo = cfg['ppo']
    _positive(cfg, 'ppo', 'learning_rate')
    _require(isinstance(ppo['n_steps'], int) and ppo['n_steps'] > 0, 'ppo.n_steps must be positive')
    _require(isinstance(ppo['batch_size'], int) and ppo['batch_size'] > 0, 'ppo.batch_size must be positive')
    _require(isinstance(ppo['n_epochs'], int) and ppo['n_epochs'] > 0, 'ppo.n_epochs must be positive')
    _require(isinstance(ppo['net_arch'], list) and ppo['net_arch']
             and all(isinstance(v, int) and v > 0 for v in ppo['net_arch']),
             'ppo.net_arch must be a non-empty list of positive integers')
    _require(ppo.get('feature_extractor', 'flat') in ('flat', 'split_mlp'),
             'ppo.feature_extractor must be flat or split_mlp')
    _require(0 < ppo['gamma'] <= 1 and 0 < ppo['gae_lambda'] <= 1,
             'ppo gamma and gae_lambda must be in (0, 1]')
    _positive(cfg, 'ppo', 'clip_range')

    training = cfg['training']
    _require(isinstance(training['total_timesteps'], int) and training['total_timesteps'] > 0,
             'training.total_timesteps must be positive')
    _require(isinstance(training['n_envs'], int) and training['n_envs'] > 0,
             'training.n_envs must be positive')
    _require(training['vec_env'] in ('dummy', 'subproc'), 'training.vec_env must be dummy or subproc')
    _require(isinstance(training['torch_threads'], int) and training['torch_threads'] > 0,
             'training.torch_threads must be positive')
    _require(isinstance(training['eval_every'], int) and training['eval_every'] > 0,
             'training.eval_every must be positive')
    _require(isinstance(training['eval_episodes'], int) and training['eval_episodes'] > 0,
             'training.eval_episodes must be positive')
    _require(isinstance(training['checkpoint_every'], int) and training['checkpoint_every'] > 0,
             'training.checkpoint_every must be positive')
    stages = training['validation_stages']
    _require(isinstance(stages, list) and stages and all(stage in STAGES for stage in stages),
             'training.validation_stages must be a non-empty subset of known stages')
    _require(training['n_envs'] * ppo['n_steps'] % ppo['batch_size'] == 0,
             'batch_size must divide n_steps*n_envs')


def spec(cfg):
    return {key: copy.deepcopy(cfg[key]) for key in ('spec_version', 'observation', 'robot', 'sensor', 'safety')}


def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def save_yaml(data, path):
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
