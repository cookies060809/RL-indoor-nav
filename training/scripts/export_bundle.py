"""Build a model bundle in an isolated directory from a training snapshot.

This script is launched by ``run.py export`` in a clean subprocess. It never
imports the live ``training/envs/navrl`` package: the only ``navrl`` on
``sys.path`` is the copy stored next to the training run.
"""
import argparse
import copy
import hashlib
import json
import os
import platform
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def copy_requirements(destination, run_folder):
    snapshot_configs = run_folder / 'source/configs'
    for name in ('requirements.txt', 'requirements-core-lock.txt', 'requirements-win-lock.txt'):
        for base in (snapshot_configs, ROOT / 'config'):
            candidate = base / name
            if candidate.is_file():
                shutil.copy2(candidate, destination / name)
                break


def aggregate_hash(folder):
    hashes = {
        str(path.relative_to(folder)).replace('\\', '/'): sha256_file(path)
        for path in Path(folder).rglob('*') if path.is_file()
    }
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    return digest, hashes


def build(run_folder, model_path, destination):
    run_folder = Path(run_folder).resolve()
    model_path = Path(model_path).resolve()
    destination = Path(destination).resolve()
    if not model_path.is_relative_to(run_folder):
        raise RuntimeError('Selected weights must be inside the training run directory')
    source_navrl = run_folder / 'source/navrl'
    if not source_navrl.is_dir():
        raise RuntimeError('Run has no source/navrl snapshot; export would mix current source with old weights')
    config_path = run_folder / 'config.yaml'
    if not config_path.is_file():
        raise RuntimeError('Run has no config.yaml')
    destination.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source_navrl, destination / 'navrl',
                    ignore=shutil.ignore_patterns('__pycache__', '*.nbc', '*.nbi'))
    shutil.copytree(ROOT / 'training/scripts/deployment', destination / 'deployment')
    for name in ('config.yaml', 'obs_spec.yaml', 'metadata.json', 'training_result.json'):
        if (run_folder / name).is_file():
            shutil.copy2(run_folder / name, destination / name)
    if (run_folder / 'source/configs').is_dir():
        shutil.copytree(run_folder / 'source/configs', destination / 'configs')
    copy_requirements(destination, run_folder)
    shutil.copy2(model_path, destination / 'policy.zip')

    sys.path.insert(0, str(destination))
    import torch
    from stable_baselines3 import PPO
    from navrl.fixtures import fixture_cases, write_fixtures

    with config_path.open(encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    torch.set_num_threads(2)
    write_fixtures(config, destination / 'fixtures')
    observations = np.stack([case['observation'] for case in fixture_cases(config)])
    model = PPO.load(destination / 'policy.zip', device='cpu')
    actions, _ = model.predict(observations, deterministic=True)
    np.savez_compressed(destination / 'inference_reference.npz',
                        observations=observations, actions=actions)
    timings = []
    for _ in range(10):
        model.predict(observations[0], deterministic=True)
    for _ in range(100):
        start = time.perf_counter()
        model.predict(observations[0], deterministic=True)
        timings.append((time.perf_counter() - start) * 1000)
    with (destination / 'inference_latency.json').open('w', encoding='utf-8') as handle:
        json.dump({
            'p50_ms': float(np.percentile(timings, 50)),
            'p95_ms': float(np.percentile(timings, 95)),
            'note': 'Policy-only, excludes ROS/TF/BEV communication',
        }, handle, indent=2)

    kind = 'best' if model_path.stem == 'best_model' else 'final' if model_path.stem == 'final_model' else 'checkpoint'
    score_binding = {'kind': kind, 'bound': False, 'reason': 'selected weights are not a score-bearing file'}
    if kind == 'best':
        validation_path = run_folder / 'best_validation.json'
        if validation_path.is_file():
            validation = json.loads(validation_path.read_text(encoding='utf-8'))
            recorded = validation.get('weights_sha256')
            actual = sha256_file(destination / 'policy.zip')
            if recorded and recorded == actual:
                shutil.copy2(validation_path, destination / 'best_validation.json')
                score_binding = {'kind': kind, 'bound': True, 'weights_sha256': actual}
            else:
                score_binding = {
                    'kind': kind,
                    'bound': False,
                    'reason': 'legacy or mismatched score: no verified weight hash in best_validation.json',
                    'recorded_weights_sha256': recorded,
                }
    elif kind == 'final':
        final_path = run_folder / 'final_validation.json'
        if final_path.is_file():
            final = json.loads(final_path.read_text(encoding='utf-8'))
            actual = sha256_file(destination / 'policy.zip')
            if final.get('weights_sha256') == actual:
                shutil.copy2(final_path, destination / 'final_validation.json')
                score_binding = {'kind': kind, 'bound': True, 'weights_sha256': actual}
        if (destination / 'best_validation.json').exists():
            raise RuntimeError('Final export must not contain best-model scores')

    snapshot_hash, snapshot_files = aggregate_hash(destination / 'navrl')
    manifest = {
        'export_format_version': 'bundle_v2',
        'created': datetime.now().astimezone().isoformat(),
        'run': str(run_folder),
        'selected_weights': {
            'file': str(model_path.relative_to(run_folder)).replace('\\', '/'),
            'sha256': sha256_file(destination / 'policy.zip'),
            'num_timesteps': int(model.num_timesteps),
            'kind': kind,
        },
        'source_snapshot': {
            'path': 'navrl',
            'sha256': snapshot_hash,
            'files': snapshot_files,
        },
        'spec_version': config.get('spec_version'),
        'score_binding': score_binding,
        'python': sys.version,
        'platform': platform.platform(),
    }
    with (destination / 'export_manifest.json').open('w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2, ensure_ascii=False)
    hashes = {
        str(path.relative_to(destination)).replace('\\', '/'): sha256_file(path)
        for path in destination.rglob('*') if path.is_file() and path.name != 'sha256.json'
    }
    (destination / 'sha256.json').write_text(json.dumps(hashes, indent=2), encoding='utf-8')
    print('BUILT', destination)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    build(args.run, args.model, args.output)


if __name__ == '__main__':
    main()
