"""Aggregate v2 comparison runs, evaluations and generate report figures."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

VARIANTS = ('fixed', 'adaptive')
METRICS = ('success_rate', 'collision_rate', 'mean_intervention_duration',
           'mean_requested_speed', 'mean_executed_speed')


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def variant_of(name):
    for variant in VARIANTS:
        if name.startswith(variant + '_'):
            return variant
    return name.split('_')[0]


def seed_of(name):
    try:
        return int(name.rsplit('_s', 1)[1])
    except (IndexError, ValueError):
        return None


def aggregate(summaries):
    if not summaries:
        return None
    result = {'runs': len(summaries)}
    for key in ('success_rate', 'collision_rate', 'mean_intervention_duration',
                'mean_requested_speed', 'mean_executed_speed'):
        values = [summary[key] for summary in summaries if summary.get(key) is not None]
        result[key] = float(np.mean(values)) if values else None
        result[key + '_std'] = float(np.std(values)) if values else None
    stages = sorted({stage for summary in summaries for stage in summary.get('per_scenario', {})})
    result['per_scenario'] = {}
    for stage in stages:
        stage_values = [summary['per_scenario'][stage] for summary in summaries
                        if stage in summary.get('per_scenario', {})]
        if not stage_values:
            continue
        stage_result = {'runs': len(stage_values)}
        for key in ('success_rate', 'collision_rate', 'mean_intervention_duration',
                    'mean_requested_speed', 'mean_executed_speed'):
            values = [value[key] for value in stage_values if value.get(key) is not None]
            stage_result[key] = float(np.mean(values)) if values else None
            stage_result[key + '_std'] = float(np.std(values)) if values else None
        failures = Counter()
        interventions = Counter()
        for value in stage_values:
            failures.update(value.get('failure_reasons', {}))
            interventions.update(value.get('intervention_reasons', {}))
        stage_result['failure_reasons'] = dict(failures)
        stage_result['intervention_reasons'] = dict(interventions)
        result['per_scenario'][stage] = stage_result
    return result


def collect(run_root, eval_root):
    runs = {}
    for run_dir in sorted(Path(run_root).iterdir()):
        if not run_dir.is_dir() or not (run_dir / 'config.yaml').exists():
            continue
        name = run_dir.name
        runs[name] = {'variant': variant_of(name), 'seed': seed_of(name), 'path': str(run_dir)}
        for split in ('validation', 'test', 'diagnostic'):
            summary_path = Path(eval_root) / split / name / 'summary.json'
            if summary_path.exists():
                runs[name][split] = load_json(summary_path)
    return runs


def plot_scenario_comparison(aggregate_by_variant, split, output):
    stages = sorted({stage for variant in aggregate_by_variant.values()
                     for stage in (variant.get(split, {}) or {}).get('per_scenario', {})})
    if not stages:
        return
    x = np.arange(len(stages))
    width = 0.36
    figure, axes = plt.subplots(1, 3, figsize=(16, 4.4))
    for index, variant in enumerate(VARIANTS):
        data = aggregate_by_variant.get(variant, {}).get(split)
        if not data:
            continue
        offset = (index - 0.5) * width
        success = [data['per_scenario'][stage]['success_rate'] for stage in stages]
        success_std = [data['per_scenario'][stage]['success_rate_std'] for stage in stages]
        collision = [data['per_scenario'][stage]['collision_rate'] for stage in stages]
        collision_std = [data['per_scenario'][stage]['collision_rate_std'] for stage in stages]
        duration = [data['per_scenario'][stage]['mean_intervention_duration'] or 0.0 for stage in stages]
        duration_std = [data['per_scenario'][stage]['mean_intervention_duration_std'] or 0.0 for stage in stages]
        axes[0].bar(x + offset, success, width, yerr=success_std, capsize=3, label=variant)
        axes[1].bar(x + offset, collision, width, yerr=collision_std, capsize=3, label=variant)
        axes[2].bar(x + offset, duration, width, yerr=duration_std, capsize=3, label=variant)
    axes[0].set_title('Success rate')
    axes[1].set_title('Collision rate')
    axes[2].set_title('Intervention duration (s)')
    for axis in axes:
        axis.set_xticks(x)
        axis.set_xticklabels(stages, rotation=30, ha='right')
        axis.grid(axis='y', alpha=0.3)
        axis.legend()
    axes[0].set_ylim(0, 1.05)
    axes[1].set_ylim(0, 1.05)
    figure.suptitle(split)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_failure_reasons(aggregate_by_variant, split, output):
    stages = sorted({stage for variant in aggregate_by_variant.values()
                     for stage in (variant.get(split, {}) or {}).get('per_scenario', {})})
    if not stages:
        return
    reasons = sorted({reason for variant in aggregate_by_variant.values()
                      for stage_data in (variant.get(split, {}) or {}).get('per_scenario', {}).values()
                      for reason in stage_data.get('failure_reasons', {})})
    if not reasons:
        return
    figure, axes = plt.subplots(1, len(VARIANTS), figsize=(14, 4.5), squeeze=False)
    x = np.arange(len(stages))
    for axis, variant in zip(axes[0], VARIANTS):
        data = aggregate_by_variant.get(variant, {}).get(split)
        if not data:
            continue
        bottom = np.zeros(len(stages))
        for reason in reasons:
            values = [data['per_scenario'][stage].get('failure_reasons', {}).get(reason, 0) for stage in stages]
            axis.bar(x, values, bottom=bottom, label=reason)
            bottom += np.asarray(values)
        axis.set_title(variant)
        axis.set_xticks(x)
        axis.set_xticklabels(stages, rotation=30, ha='right')
        axis.set_ylabel('failed episodes')
        axis.legend(fontsize=7)
    figure.suptitle(f'Failure reasons: {split}')
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def plot_training_curves(run_root, output):
    figure, axes = plt.subplots(1, 2, figsize=(12, 4))
    for variant in VARIANTS:
        histories = []
        for path in sorted(Path(run_root).glob(f'{variant}_s*/validation_history.jsonl')):
            histories.append([json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()])
        if not histories:
            continue
        length = min(len(history) for history in histories)
        x = [record['timesteps'] for record in histories[0][:length]]
        for metric, axis in zip(('success_rate', 'collision_rate'), axes):
            curves = np.asarray([[history[index][metric] for index in range(length)] for history in histories])
            axis.plot(x, curves.mean(axis=0), marker='o', label=variant)
            axis.fill_between(x, curves.min(axis=0), curves.max(axis=0), alpha=0.15)
    axes[0].set_title('Validation success rate (mean, min-max)')
    axes[1].set_title('Validation collision rate (mean, min-max)')
    for axis in axes:
        axis.set_xlabel('Environment steps')
        axis.grid(alpha=0.3)
        axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def resample(values, points=100):
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        return np.full(points, np.nan)
    source = np.linspace(0, 1, len(values))
    target = np.linspace(0, 1, points)
    return np.interp(target, source, values)


def plot_speed_comparison(eval_root, runs, output):
    representative = {}
    for name, info in runs.items():
        if info.get('seed') == 42 and info['variant'] in VARIANTS and 'test' in info:
            representative[info['variant']] = Path(eval_root) / 'test' / name
    if not representative:
        return
    stages = load_json(next(iter(representative.values())) / 'summary.json')['stages']
    figure, axes = plt.subplots(len(stages), 2, figsize=(12, 2.4 * len(stages)), squeeze=False)
    for row, stage in enumerate(stages):
        for variant, folder in representative.items():
            summary = load_json(folder / 'summary.json')
            per_stage = summary['episodes'] // max(1, len(summary['stages']))
            index = summary['stages'].index(stage) * per_stage
            path = folder / f'trajectory_{index:03d}.npz'
            if not path.exists():
                continue
            data = np.load(path)
            requested = resample(data['requested_velocity'][:, 0])
            executed = resample(data['executed_velocity'][:, 0])
            time_axis = np.linspace(0, 1, len(requested))
            axes[row, 0].plot(time_axis, requested, '--', alpha=0.5, label=f'{variant} request')
            axes[row, 0].plot(time_axis, executed, label=f'{variant} execute')
            axes[row, 1].plot(time_axis, resample(data['requested_velocity'][:, 1]), '--', alpha=0.5, label=f'{variant} request')
            axes[row, 1].plot(time_axis, resample(data['executed_velocity'][:, 1]), label=f'{variant} execute')
        axes[row, 0].set_title(stage)
        axes[row, 0].set_ylabel('linear m/s')
        axes[row, 1].set_ylabel('angular rad/s')
        for axis in axes[row]:
            axis.grid(alpha=0.3)
            axis.legend(fontsize=6)
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs-root', default='experiments/yy_v2/runs')
    parser.add_argument('--evaluations-root', default='experiments/yy_v2/evaluations')
    parser.add_argument('--output-root', default='experiments/yy_v2/summary')
    args = parser.parse_args()
    output = Path(args.output_root)
    output.mkdir(parents=True, exist_ok=True)
    runs = collect(args.runs_root, args.evaluations_root)
    aggregate_by_variant = {}
    for variant in VARIANTS:
        aggregate_by_variant[variant] = {}
        variant_runs = [info for info in runs.values() if info['variant'] == variant]
        for split in ('validation', 'test', 'diagnostic'):
            summaries = [info[split] for info in variant_runs if split in info]
            if summaries:
                aggregate_by_variant[variant][split] = aggregate(summaries)
    (output / 'summary.json').write_text(json.dumps({
        'runs': runs,
        'variants': aggregate_by_variant,
    }, indent=2, ensure_ascii=False), encoding='utf-8')
    reward_runs = {
        name: info for name, info in runs.items()
        if name.startswith('reward_') or name.startswith('adaptive30k')
    }
    (output / 'reward_summary.json').write_text(
        json.dumps(reward_runs, indent=2, ensure_ascii=False), encoding='utf-8')
    rows = []
    for name, info in runs.items():
        for split in ('validation', 'test'):
            summary = info.get(split)
            if not summary:
                continue
            for stage, values in summary.get('per_scenario', {}).items():
                rows.append({
                    'run': name,
                    'variant': info['variant'],
                    'seed': info.get('seed'),
                    'split': split,
                    'stage': stage,
                    'success_rate': values['success_rate'],
                    'collision_rate': values['collision_rate'],
                    'mean_intervention_duration': values['mean_intervention_duration'],
                    'mean_requested_speed': values['mean_requested_speed'],
                    'mean_executed_speed': values['mean_executed_speed'],
                    'failure_reasons': json.dumps(values['failure_reasons'], ensure_ascii=False),
                })
    if rows:
        with (output / 'per_scenario.csv').open('w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    for split in ('validation', 'test'):
        plot_scenario_comparison(aggregate_by_variant, split, output / f'comparison_{split}.png')
        plot_failure_reasons(aggregate_by_variant, split, output / f'failure_reasons_{split}.png')
    plot_training_curves(args.runs_root, output / 'training_curves.png')
    plot_speed_comparison(args.evaluations_root, runs, output / 'speed_comparison.png')
    print(json.dumps({variant: {split: {
        key: value.get(key) for key in ('runs', 'success_rate', 'collision_rate', 'mean_intervention_duration')
    } for split, value in data.items()} for variant, data in aggregate_by_variant.items()},
        indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
