"""Collect existing results only; does not choose models from the test set."""
import argparse
from pathlib import Path
import json
import csv
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training/envs'))
from navrl.config import ROOT,load_config

parser=argparse.ArgumentParser()
parser.add_argument('--runs-root',default=str(ROOT/'training/runs'))
parser.add_argument('--output-dir',default=str(ROOT/'docs/reports/yy_v2'))
args=parser.parse_args()
rows=[]
for folder in sorted(Path(args.runs_root).iterdir()):
    if not (folder/'best_validation.json').exists():
        continue
    cfg=load_config(folder/'config.yaml',strict=False)
    validation=json.loads((folder/'best_validation.json').read_text(encoding='utf-8'))
    result=json.loads((folder/'training_result.json').read_text(encoding='utf-8')) if (folder/'training_result.json').exists() else {}
    test_path=folder/'test_24/summary.json'
    test=json.loads(test_path.read_text(encoding='utf-8')) if test_path.exists() else {}
    rows.append({
        'run':folder.name,
        'stage':cfg['environment']['stage'],
        'extractor':cfg['ppo'].get('feature_extractor','flat'),
        'seed':cfg['seed'],
        'spec_version':cfg.get('spec_version','legacy'),
        'training_final_steps':result.get('model_timesteps'),
        'selected_model_steps':validation.get('timesteps'),
        'selected_weights_sha256':validation.get('weights_sha256'),
        'validation_successes':validation.get('successes'),
        'validation_episodes':validation.get('episodes'),
        'test_successes':test.get('successes'),
        'test_episodes':test.get('episodes'),
        'test_collision_rate':test.get('collision_rate'),
        'test_success_time':test.get('mean_success_time'),
        'training_wall_seconds':result.get('elapsed_seconds'),
    })
if not rows:
    raise SystemExit('No runs with best_validation.json found')
output=Path(args.output_dir)
output.mkdir(parents=True,exist_ok=True)
with (output/'model_summary.csv').open('w',newline='',encoding='utf-8-sig') as handle:
    writer=csv.DictWriter(handle,fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
(output/'model_summary.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(rows,indent=2,ensure_ascii=False))
