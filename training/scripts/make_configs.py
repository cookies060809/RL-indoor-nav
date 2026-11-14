"""Generate complete independent configs; never overwrite manually edited ones."""
from pathlib import Path
import copy
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'training/envs'))
from navrl.config import ROOT,load_config,save_yaml
base=load_config()
for stage in ('empty','single','turn','mixed'):
    cfg=copy.deepcopy(base); cfg['environment']['stage']=stage
    if stage!='empty':
        cfg['environment']['min_goal_distance']=1.5
        cfg['environment']['max_goal_distance']=4.0
    target=ROOT/'config'/f'{stage}32.yaml'
    if not target.exists(): save_yaml(cfg,target)
cfg=copy.deepcopy(base); cfg['observation']['size']=48; cfg['environment']['stage']='mixed'
cfg['environment']['max_goal_distance']=4.0
target=ROOT/'config/formal48.yaml'
if not target.exists(): save_yaml(cfg,target)
for stage in ('empty','single','turn','mixed'):
    cfg=load_config(ROOT/'config'/f'{stage}32.yaml')
    cfg['ppo']['feature_extractor']='split_mlp'
    cfg['training']['n_envs']=4; cfg['training']['vec_env']='subproc'
    target=ROOT/'config'/f'split_{stage}32.yaml'
    if not target.exists(): save_yaml(cfg,target)
