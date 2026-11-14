"""Collect existing results only; does not choose models from the test set."""
from pathlib import Path
import json
import csv
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training/envs'))
from navrl.config import ROOT,load_config

rows=[]
for folder in sorted((ROOT/'training/runs').iterdir()):
    if not (folder/'best_validation.json').exists(): continue
    cfg=load_config(folder/'config.yaml')
    validation=json.loads((folder/'best_validation.json').read_text())
    result=json.loads((folder/'training_result.json').read_text()) if (folder/'training_result.json').exists() else {}
    test=json.loads((folder/'test_24/summary.json').read_text()) if (folder/'test_24/summary.json').exists() else {}
    rows.append({'run':folder.name,'stage':cfg['environment']['stage'],'extractor':cfg['ppo'].get('feature_extractor','flat'),
                 'seed':cfg['seed'],'training_final_steps':result.get('model_timesteps'),
                 'selected_model_steps':validation['timesteps'],'validation_successes':validation['successes'],
                 'validation_episodes':validation['episodes'],'test_successes':test.get('successes'),
                 'test_episodes':test.get('episodes'),'test_collision_rate':test.get('collision_rate'),
                 'test_success_time':test.get('mean_success_time'),'training_wall_seconds':result.get('elapsed_seconds')})
target=ROOT/'docs/reports/model_summary.csv'
with target.open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
(ROOT/'docs/reports/model_summary.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(rows,indent=2,ensure_ascii=False))
