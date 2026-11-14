from pathlib import Path
import copy
import hashlib
import json
import yaml

ROOT = Path(__file__).resolve().parents[3]

def load_config(path=None):
    path = Path(path) if path else ROOT / 'config/base.yaml'
    with path.open(encoding='utf-8-sig') as f:
        cfg = yaml.safe_load(f)
    validate_config(cfg)
    return cfg

def validate_config(cfg):
    if cfg['observation']['size'] not in (32, 48):
        raise ValueError('BEV size must be 32 or 48; model must be retrained when changed')
    if cfg['robot']['dt'] <= 0 or cfg['robot']['radius'] <= 0:
        raise ValueError('dt and radius must be positive')
    if cfg['environment']['stage'] not in ('empty', 'single', 'turn', 'clutter', 'mixed'):
        raise ValueError('Unknown curriculum stage')
    if cfg['training']['n_envs'] * cfg['ppo']['n_steps'] % cfg['ppo']['batch_size']:
        raise ValueError('batch_size must divide n_steps*n_envs')

def spec(cfg):
    return {k: copy.deepcopy(cfg[k]) for k in ('spec_version','observation','robot','sensor','safety')}

def digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()

def save_yaml(data, path):
    with Path(path).open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
