import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
import numpy as np
import pytest
from stable_baselines3 import PPO
from navrl.config import load_config, save_yaml, spec
from navrl.env import IndoorNavEnv

ROOT = Path(__file__).resolve().parents[3]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_run(tmp_path, split_mlp):
    cfg = load_config()
    cfg['training']['n_steps'] = 8
    cfg['training']['batch_size'] = 8
    cfg['training']['n_envs'] = 1
    cfg['training']['validation_stages'] = ['empty']
    cfg['ppo']['net_arch'] = [8, 8]
    if split_mlp:
        cfg['ppo']['feature_extractor'] = 'split_mlp'
    run = tmp_path / 'run'
    run.mkdir()
    save_yaml(cfg, run / 'config.yaml')
    save_yaml(spec(cfg), run / 'obs_spec.yaml')
    (run / 'metadata.json').write_text('{}', encoding='utf-8')
    (run / 'training_result.json').write_text('{"model_timesteps": 8}', encoding='utf-8')
    shutil.copytree(ROOT / 'training/envs/navrl', run / 'source/navrl')
    env = IndoorNavEnv(cfg)
    env.reset(seed=1)
    kwargs = {'net_arch': [8, 8]}
    if split_mlp:
        from navrl.features import BevStateExtractor
        kwargs['features_extractor_class'] = BevStateExtractor
    model = PPO('MlpPolicy', env, n_steps=8, batch_size=8, seed=1, device='cpu', policy_kwargs=kwargs)
    model.save(run / 'final_model')
    env.close()
    return run, cfg


@pytest.mark.parametrize('split_mlp', [False, True])
def test_export_uses_only_run_snapshot_and_binds_weights(tmp_path, split_mlp):
    run, cfg = make_run(tmp_path, split_mlp)
    (run / 'best_validation.json').write_text(json.dumps({'success_rate': 1.0, 'weights_sha256': 'mismatch'}),
                                              encoding='utf-8')
    poison = tmp_path / 'poison/navrl'
    poison.mkdir(parents=True)
    (poison / '__init__.py').write_text('raise RuntimeError("live source imported")\n', encoding='utf-8')
    output = tmp_path / 'bundle'
    environment = os.environ.copy()
    environment['PYTHONPATH'] = str(tmp_path / 'poison')
    environment['TEMP'] = str(ROOT / '.tmp')
    environment['TMP'] = str(ROOT / '.tmp')
    subprocess.run([
        sys.executable, str(ROOT / 'training/scripts/export_bundle.py'),
        '--run', str(run), '--model', str(run / 'final_model.zip'), '--output', str(output),
    ], check=True, cwd=str(ROOT), env=environment)
    manifest = json.loads((output / 'export_manifest.json').read_text(encoding='utf-8'))
    hashes = json.loads((output / 'sha256.json').read_text(encoding='utf-8'))
    assert not any('__pycache__' in name or name.endswith(('.pyc', '.nbc', '.nbi')) for name in hashes)
    assert not any('__pycache__' in name for name in manifest['source_snapshot']['files'])
    assert manifest['selected_weights']['kind'] == 'final'
    assert manifest['selected_weights']['sha256'] == sha256(output / 'policy.zip')
    assert manifest['score_binding']['bound'] is False
    assert not (output / 'best_validation.json').exists()
    for relative, expected in manifest['source_snapshot']['files'].items():
        assert sha256(output / 'navrl' / relative) == expected
    subprocess.run([sys.executable, str(output / 'deployment/verify_bundle.py')],
                   check=True, cwd=str(output), env=environment)


def test_export_best_binds_only_matching_score(tmp_path):
    run, _ = make_run(tmp_path, False)
    shutil.copy2(run / 'final_model.zip', run / 'best_model.zip')
    weight_hash = sha256(run / 'best_model.zip')
    (run / 'best_validation.json').write_text(json.dumps({
        'success_rate': 0.5, 'weights_sha256': weight_hash, 'num_timesteps': 8,
    }), encoding='utf-8')
    output = tmp_path / 'bundle'
    subprocess.run([
        sys.executable, str(ROOT / 'training/scripts/export_bundle.py'),
        '--run', str(run), '--model', str(run / 'best_model.zip'), '--output', str(output),
    ], check=True, cwd=str(ROOT))
    manifest = json.loads((output / 'export_manifest.json').read_text(encoding='utf-8'))
    assert manifest['score_binding']['bound'] is True
    assert (output / 'best_validation.json').exists()
