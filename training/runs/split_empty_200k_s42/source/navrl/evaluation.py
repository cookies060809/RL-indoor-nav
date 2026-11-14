import csv
import json
from pathlib import Path
import numpy as np
from .env import IndoorNavEnv

SPLITS={'validation':100000,'test':200000,'diagnostic':300000}

def evaluate(model,cfg,episodes=8,split='validation',output=None,safety=None):
    """Fixed seeds; test split is never used by training callbacks."""
    env=IndoorNavEnv(cfg,safety=safety); rows=[]; traces=[]
    for i in range(episodes):
        seed=SPLITS[split]+i
        obs,_=env.reset(seed=seed)
        while True:
            action,_=model.predict(obs,deterministic=True)
            obs,_,done,trunc,info=env.step(action)
            if done or trunc: break
        row={'episode':i,'layout_seed':seed,**info['summary']}
        rows.append(row)
        traces.append({'trajectory':np.asarray(env.trajectory),'path':env.path.copy(),'rectangles':env.rectangles.copy(),'goal':env.goal.copy()})
    env.close()
    successes=[r for r in rows if r['success']]
    summary={
        'split':split,'episodes':episodes,'successes':len(successes),
        'success_rate':len(successes)/episodes,
        'collision_rate':sum(r['collision'] for r in rows)/episodes,
        'mean_success_time':float(np.mean([r['sim_time'] for r in successes])) if successes else None,
        'mean_intervention_duration':float(np.mean([r['intervention_duration'] for r in rows])),
        'mean_reward':float(np.mean([sum(v for k,v in r.items() if k.startswith('reward_')) for r in rows])),
    }
    if output:
        output=Path(output); output.mkdir(parents=True,exist_ok=True)
        with (output/'episodes.csv').open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        for i,trace in enumerate(traces): np.savez_compressed(output/f'trajectory_{i:03d}.npz',**trace)
    return summary,rows,traces
