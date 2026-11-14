"""Non-learning path follower sanity check, never presented as a trained policy."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training/envs'))
import math
import json
import numpy as np
from navrl.config import load_config,ROOT
from navrl.env import IndoorNavEnv
cfg=load_config(); rows=[]
for stage in ('empty','single','turn','clutter'):
    for seed in (301,302,303):
        env=IndoorNavEnv(cfg,stage=stage); obs,_=env.reset(seed=seed)
        while True:
            dx,dy=env.local_goal-env.pose[:2]
            error=(math.atan2(dy,dx)-env.pose[2]+math.pi)%(2*math.pi)-math.pi
            v=cfg['robot']['v_max']*max(0,math.cos(error))
            if np.linalg.norm(env.goal-env.pose[:2])<0.20: v=0.0
            w=np.clip(2*error,-1,1)
            action=np.array([2*v/cfg['robot']['v_max']-1,w])
            obs,_,done,trunc,info=env.step(action)
            if done or trunc:
                rows.append({'stage':stage,'seed':seed,**info['summary']}); break
        env.close()
print(json.dumps(rows,indent=2))
(ROOT/'docs/reports/environment_path_follower.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
