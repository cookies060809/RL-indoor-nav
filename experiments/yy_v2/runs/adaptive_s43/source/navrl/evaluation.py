import copy
import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np
from .config import digest
from .env import IndoorNavEnv

SPLITS = {'validation': 100000, 'test': 200000, 'diagnostic': 300000}
REASONS = {
    'none': 0,
    'scan_unknown_front': 1,
    'scan_unknown_rear': 2,
    'scan_unknown_rotation': 3,
    'footprint_overlap': 4,
    'braking_sweep': 5,
    'no_valid_scan': 6,
    'filtered': 7,
}
REASON_NAMES = {value: key for key, value in REASONS.items()}


def _mean(values):
    return float(np.mean(values)) if values else None


def _scenario_summary(rows):
    successes = [row for row in rows if row['success']]
    intervention_reasons = Counter()
    for row in rows:
        for reason, count in row.get('intervention_reasons', {}).items():
            intervention_reasons[reason] += count
    return {
        'episodes': len(rows),
        'successes': len(successes),
        'success_rate': len(successes) / len(rows) if rows else 0.0,
        'collision_rate': sum(row['collision'] for row in rows) / len(rows) if rows else 0.0,
        'mean_success_time': _mean([row['sim_time'] for row in successes]),
        'mean_intervention_duration': _mean([row['intervention_duration'] for row in rows]),
        'mean_requested_speed': _mean([row['mean_requested_speed'] for row in rows]),
        'mean_executed_speed': _mean([row['mean_executed_speed'] for row in rows]),
        'failure_reasons': dict(Counter(row['reason'] for row in rows if row['reason'] != 'success')),
        'intervention_reasons': dict(intervention_reasons),
    }


def evaluate(model, cfg, episodes=8, split='validation', output=None, safety=None,
             stages=None, metadata=None):
    """Evaluate fixed seeds with per-scenario metrics and step-level records.

    The test split is never used by training callbacks. Callers that select
    models must use the validation split; the diagnostic split is for
    development only.
    """
    actual_cfg = copy.deepcopy(cfg)
    if safety is not None:
        actual_cfg['safety']['enabled'] = bool(safety)
    requested_stages = list(stages) if stages else [actual_cfg['environment']['stage']]
    base_seed = SPLITS[split]
    rows = []
    traces = []
    for stage_index, stage in enumerate(requested_stages):
        env = IndoorNavEnv(actual_cfg, stage=stage)
        for episode in range(episodes):
            if len(requested_stages) == 1:
                seed = base_seed + episode
            else:
                seed = base_seed + stage_index * 1000 + episode
            obs, _ = env.reset(seed=seed)
            raw_actions = []
            requested_velocities = []
            executed_velocities = []
            interventions = []
            reason_codes = []
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, done, truncated, info = env.step(action)
                raw_actions.append(np.asarray(info['raw_action'], dtype=np.float32))
                requested_velocities.append(np.asarray(info['requested_velocity'], dtype=np.float32))
                executed_velocities.append(np.asarray(info['executed_velocity'], dtype=np.float32))
                interventions.append(bool(info['intervening']))
                reason_codes.append(REASONS.get(info['intervention_reason'], REASONS['filtered']))
                if done or truncated:
                    break
            summary = dict(info['summary'])
            summary['episode'] = episode
            summary['layout_seed'] = seed
            summary['intervention_reason_codes'] = dict(Counter([
                REASON_NAMES[code] for code in reason_codes if code != REASONS['none']
            ]))
            rows.append(summary)
            traces.append({
                'trajectory': np.asarray(env.trajectory),
                'path': env.path.copy(),
                'rectangles': env.rectangles.copy(),
                'goal': env.goal.copy(),
                'raw_action': np.asarray(raw_actions),
                'requested_velocity': np.asarray(requested_velocities),
                'executed_velocity': np.asarray(executed_velocities),
                'intervention': np.asarray(interventions, dtype=np.int8),
                'reason_code': np.asarray(reason_codes, dtype=np.int8),
            })
        env.close()

    successes = [row for row in rows if row['success']]
    per_scenario = {}
    for stage in requested_stages:
        stage_rows = [row for row in rows if row['stage'] == stage]
        if stage_rows:
            per_scenario[stage] = _scenario_summary(stage_rows)
    intervention_reasons = Counter()
    for row in rows:
        for reason, count in row.get('intervention_reasons', {}).items():
            intervention_reasons[reason] += count
    failure_reasons = Counter(row['reason'] for row in rows if row['reason'] != 'success')
    summary = {
        'split': split,
        'stages': requested_stages,
        'episodes': len(rows),
        'successes': len(successes),
        'success_rate': len(successes) / len(rows),
        'collision_rate': sum(row['collision'] for row in rows) / len(rows),
        'mean_success_time': _mean([row['sim_time'] for row in successes]),
        'mean_intervention_duration': _mean([row['intervention_duration'] for row in rows]),
        'mean_requested_speed': _mean([row['mean_requested_speed'] for row in rows]),
        'mean_executed_speed': _mean([row['mean_executed_speed'] for row in rows]),
        'mean_reward': _mean([
            sum(value for key, value in row.items() if key.startswith('reward_')) for row in rows
        ]),
        'failure_reasons': dict(failure_reasons),
        'intervention_reasons': dict(intervention_reasons),
        'per_scenario': per_scenario,
        'effective_safety': bool(actual_cfg['safety']['enabled']),
        'spec_version': actual_cfg['spec_version'],
        'config_hash': digest(actual_cfg),
        'seed_base': base_seed,
        'seed_offsets': 'stage_index*1000+episode' if len(requested_stages) > 1 else 'episode',
    }
    if output:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        with (output / 'episodes.csv').open('w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
        if metadata:
            (output / 'evaluation_metadata.json').write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8')
        for index, trace in enumerate(traces):
            np.savez_compressed(output / f'trajectory_{index:03d}.npz', **trace)
    return summary, rows, traces
