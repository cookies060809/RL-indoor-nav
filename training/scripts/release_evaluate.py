"""Freeze validation-only model selection before a new 96-episode release test.

Use `freeze` after all four training runs complete, then `evaluate CASE`.
Each evaluation runs in a fresh interpreter and imports its recorded environment
snapshot. Test results never change the selected model.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'experiments/ppo_v2'
MANIFEST = RELEASE / 'selection.json'
STAGES = ['empty', 'single', 'turn', 'clutter']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    return Path(path).relative_to(ROOT).as_posix()


def source_hashes(folder):
    return {relative(p): sha(p) for p in sorted(Path(folder).rglob('*.py'))}


def freeze():
    if MANIFEST.exists():
        raise RuntimeError('Selection is already frozen; do not overwrite after testing')
    import yaml
    candidates = []
    for name in ['ppo_v2_visible_s42', 'ppo_v2_visible_s43', 'ppo_v2_visible_s44', 'ppo_v2_scratch_s42']:
        folder = RELEASE / 'runs' / name
        if not (folder / 'training_result.json').exists():
            raise RuntimeError(f'Training incomplete: {name}')
        result = json.loads((folder / 'best_validation.json').read_text(encoding='utf-8'))
        weights = folder / 'best_model.zip'
        if sha(weights) != result['weights_sha256']:
            raise RuntimeError(f'Validation weights mismatch: {name}')
        candidates.append({'run': name, 'model': relative(weights), 'sha256': sha(weights),
                           'config': relative(folder / 'config.yaml'),
                           'config_sha256': sha(folder / 'config.yaml'),
                           'source': relative(folder / 'source'),
                           'source_hashes': source_hashes(folder / 'source/navrl'),
                           'validation': result})
    def score(candidate):
        v = candidate['validation']
        return (v['success_rate'], -v['collision_rate'], -v['mean_intervention_duration'], v['mean_reward'])
    selected = max(candidates, key=score)
    original = ROOT / 'models/split_mixed_pretrain_v1'
    with (ROOT / selected['config']).open(encoding='utf-8') as handle:
        v2cfg = yaml.safe_load(handle)
    with (original / 'config.yaml').open(encoding='utf-8') as handle:
        v1cfg = yaml.safe_load(handle)
    variants = {
        'recommended': {'model': selected['model'], 'source': selected['source'], 'cfg': v2cfg},
        'v2_initializer': {'model': relative(original / 'policy.zip'), 'source': selected['source'], 'cfg': v2cfg},
        'v1_original': {'model': relative(original / 'policy.zip'), 'source': relative(original), 'cfg': v1cfg},
    }
    for case in variants.values():
        case['sha256'] = sha(ROOT / case['model'])
        case['source_hashes'] = source_hashes(ROOT / case['source'] / 'navrl')
    payload = {'frozen_utc': datetime.now(timezone.utc).isoformat(),
               'selection_rule': 'maximize validation (success_rate, -collision_rate, -mean_intervention_duration, mean_reward)',
               'selected_run': selected['run'], 'selected_sha256': selected['sha256'],
               'test_seed_base': 610000, 'episodes_per_stage': 24, 'stages': STAGES,
               'note': 'Warm seeds share one v1 initializer; scratch is one independent 200k control. Test is never used to select.',
               'candidates': candidates, 'variants': variants}
    MANIFEST.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding='utf-8')
    print('Frozen:', selected['run'], selected['sha256'])


def evaluate(case_name):
    import numpy as np
    import torch
    from stable_baselines3 import PPO
    manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
    case = manifest['variants'][case_name]
    if sha(ROOT / case['model']) != case['sha256']:
        raise RuntimeError('Frozen policy changed')
    for file, expected in case['source_hashes'].items():
        if sha(ROOT / file) != expected:
            raise RuntimeError('Frozen environment changed: ' + file)
    output = RELEASE / 'holdout' / case_name
    output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(ROOT / case['source']))
    from navrl.env import IndoorNavEnv
    torch.set_num_threads(2)
    model = PPO.load(ROOT / case['model'], device='cpu')
    rows = []
    for stage_index, stage in enumerate(STAGES):
        env = IndoorNavEnv(case['cfg'], stage=stage)
        for episode in range(manifest['episodes_per_stage']):
            seed = manifest['test_seed_base'] + stage_index * 1000 + episode
            obs, _ = env.reset(seed=seed)
            actions, requested, executed = [], [], []
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, done, truncated, info = env.step(action)
                actions.append(action)
                requested.append(info.get('requested_velocity', [np.nan, np.nan]))
                executed.append(info.get('executed_velocity', [np.nan, np.nan]))
                if done or truncated:
                    break
            rows.append({'episode': episode, 'layout_seed': seed, **info['summary']})
            np.savez_compressed(output / f'{stage}_{episode:02d}.npz', trajectory=env.trajectory,
                                path=env.path, rectangles=env.rectangles, goal=env.goal,
                                actions=actions, requested=requested, executed=executed)
        env.close()
    def summarize(group):
        success = [r for r in group if r['success']]
        from collections import Counter
        return {'episodes': len(group), 'successes': len(success), 'success_rate': len(success) / len(group),
                'collision_rate': sum(r['collision'] for r in group) / len(group),
                'mean_success_time': float(np.mean([r['sim_time'] for r in success])) if success else None,
                'mean_intervention_duration': float(np.mean([r['intervention_duration'] for r in group])),
                'failure_reasons': dict(Counter(r['reason'] for r in group if not r['success']))}
    result = summarize(rows)
    result.update(case=case_name, model=case['model'], weights_sha256=case['sha256'],
                  spec_version=case['cfg']['spec_version'], seed_base=manifest['test_seed_base'],
                  selection_sha256=sha(MANIFEST),
                  per_scenario={stage: summarize([r for r in rows if r['stage'] == stage]) for stage in STAGES},
                  note='Windows 2D simulator; not a ROS/Gazebo or real-robot result. Different v1/v2 safety and stuck semantics are recorded in snapshots.')
    with (output / 'episodes.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / 'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['freeze', 'evaluate'])
    parser.add_argument('case', nargs='?', choices=['recommended', 'v2_initializer', 'v1_original'])
    args = parser.parse_args()
    if args.command == 'freeze':
        freeze()
    elif args.case:
        evaluate(args.case)
    else:
        parser.error('evaluate requires a case')
