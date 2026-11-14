"""Single Windows/Linux entry point. Run from anywhere via an absolute path."""
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.cache/matplotlib'))
os.environ.setdefault('NUMBA_CACHE_DIR',str(ROOT/'.cache/numba'))
os.environ.setdefault('TEMP',str(ROOT/'.tmp'))
os.environ.setdefault('TMP',str(ROOT/'.tmp'))
import argparse
import copy
import csv
import hashlib
import importlib.metadata
import json
import platform
import shutil
import sys
import time
from datetime import datetime
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv,SubprocVecEnv
from navrl.config import load_config,save_yaml,spec,digest,validate_config
from navrl.env import IndoorNavEnv
from navrl.evaluation import evaluate

def versions():
    packages=['torch','stable-baselines3','gymnasium','numpy','scipy','numba','PyYAML','tensorboard','psutil']
    return {k:importlib.metadata.version(k) for k in packages}

def make_vec(cfg):
    factories=[lambda:Monitor(IndoorNavEnv(cfg)) for _ in range(cfg['training']['n_envs'])]
    return (SubprocVecEnv(factories,start_method='spawn') if cfg['training']['vec_env']=='subproc' else DummyVecEnv(factories))

def make_model(cfg,env):
    p=copy.deepcopy(cfg['ppo']); arch=p.pop('net_arch')
    extractor=p.pop('feature_extractor','flat')
    policy_kwargs={'net_arch':arch}
    if extractor=='split_mlp':
        from navrl.features import BevStateExtractor
        policy_kwargs['features_extractor_class']=BevStateExtractor
    elif extractor!='flat':
        raise ValueError('feature_extractor must be flat or split_mlp')
    return PPO('MlpPolicy',env,device='cpu',seed=cfg['seed'],verbose=1,
               policy_kwargs=policy_kwargs,**p)

def save_json(value,path):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8')

def snapshot(cfg,folder):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    save_yaml(cfg,folder/'config.yaml'); save_yaml(spec(cfg),folder/'obs_spec.yaml')
    sources={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
             for pattern in ('navrl/*.py','run.py','tests/*.py','configs/*.yaml') for p in ROOT.glob(pattern)}
    save_json({'python':sys.version,'platform':platform.platform(),'versions':versions(),'source_hashes':sources,
               'config_hash':digest(cfg),'spec_hash':digest(spec(cfg)),'created':datetime.now().astimezone().isoformat(),
               'seed':cfg['seed'],'normalization':'fixed specification; no VecNormalize'},folder/'metadata.json')
    source=folder/'source'; source.mkdir()
    shutil.copytree(ROOT/'navrl',source/'navrl',ignore=shutil.ignore_patterns('__pycache__','*.nbc','*.nbi'))
    shutil.copytree(ROOT/'configs',source/'configs')
    shutil.copy2(ROOT/'run.py',source/'run.py')

class ValidationCallback(BaseCallback):
    def __init__(self,cfg,folder):
        super().__init__(); self.cfg=cfg; self.folder=Path(folder); self.best=None
        self.last_eval=0; self.last_checkpoint=0; self.started=time.perf_counter()
    def _on_step(self):
        for info in self.locals.get('infos',[]):
            if 'summary' in info:
                s=info['summary']
                for k,v in s.items():
                    if isinstance(v,(float,int)): self.logger.record_mean('task/'+k,v)
        if self.num_timesteps-self.last_checkpoint>=self.cfg['training']['checkpoint_every']:
            self.model.save(self.folder/f'checkpoint_{self.num_timesteps}')
            self.last_checkpoint=self.num_timesteps
        if self.num_timesteps-self.last_eval>=self.cfg['training']['eval_every']:
            self.validate()
        return True
    def validate(self):
        self.last_eval=self.num_timesteps
        result,_,_=evaluate(self.model,self.cfg,self.cfg['training']['eval_episodes'])
        result['timesteps']=self.num_timesteps
        result['wall_seconds']=time.perf_counter()-self.started
        with (self.folder/'validation_history.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(result)+'\n')
        for k,v in result.items():
            if isinstance(v,(int,float)): self.logger.record('validation/'+k,v)
        score=(result['success_rate'],-result['collision_rate'],result['mean_reward'])
        if self.best is None or score>self.best:
            self.best=score; self.model.save(self.folder/'best_model')
            save_json(result,self.folder/'best_validation.json')
        print('VALIDATION',json.dumps(result),flush=True)

def cmd_train(args,cfg):
    folder=ROOT/'runs'/args.name
    if folder.exists(): raise ValueError(f'Run already exists: {folder}; choose a new --name')
    validate_config(cfg); snapshot(cfg,folder)
    torch.set_num_threads(cfg['training']['torch_threads'])
    env=make_vec(cfg)
    try:
        if args.resume:
            source=Path(args.resume).resolve()
            old=load_config(source.parent/'config.yaml')
            if digest(spec(old))!=digest(spec(cfg)): raise ValueError('Resume observation/action/sensor/safety spec differs; retrain or explicitly build a new experiment')
            if old['ppo']['net_arch']!=cfg['ppo']['net_arch']: raise ValueError('Network architecture changed; start a new model without --resume')
            if old['ppo'].get('feature_extractor','flat')!=cfg['ppo'].get('feature_extractor','flat'): raise ValueError('Feature extractor changed; start a new model without --resume')
            # Keep trained weights/optimizer; apply current tunable hyperparameters explicitly.
            p=cfg['ppo']
            model=PPO.load(source,env=env,device='cpu',seed=cfg['seed'],learning_rate=p['learning_rate'],
                ent_coef=p['ent_coef'],n_steps=p['n_steps'],batch_size=p['batch_size'],n_epochs=p['n_epochs'],
                gamma=p['gamma'],gae_lambda=p['gae_lambda'],clip_range=p['clip_range'])
            save_json({'source':str(source),'additional_steps':cfg['training']['total_timesteps']},folder/'resume.json')
        else: model=make_model(cfg,env)
        model.set_logger(__import__('stable_baselines3.common.logger',fromlist=['configure']).configure(str(folder/'logs'),['stdout','csv','tensorboard']))
        callback=ValidationCallback(cfg,folder)
        if args.resume:
            callback.last_eval=model.num_timesteps; callback.last_checkpoint=model.num_timesteps
        started=time.perf_counter()
        try:
            model.learn(total_timesteps=cfg['training']['total_timesteps'],callback=callback,reset_num_timesteps=not bool(args.resume))
        except KeyboardInterrupt:
            model.save(folder/'interrupted_model'); print('Interrupted checkpoint saved.'); return
        model.save(folder/'final_model'); callback.validate()
        elapsed=time.perf_counter()-started
        save_json({'elapsed_seconds':elapsed,'model_timesteps':model.num_timesteps,'requested_additional_steps':cfg['training']['total_timesteps']},folder/'training_result.json')
        print(f'RUN COMPLETE: {folder}',flush=True)
    finally: env.close()

def cmd_doctor(args,cfg):
    import psutil
    env=IndoorNavEnv(cfg); check_env(env,warn=True)
    obs,_=env.reset(seed=99)
    model=make_model(cfg,DummyVecEnv([lambda:Monitor(IndoorNavEnv(cfg))]))
    action,_=model.predict(obs,deterministic=True)
    result={'python':sys.version,'executable':sys.executable,'versions':versions(),'cpu':platform.processor(),
            'physical_cores':psutil.cpu_count(logical=False),'logical_cores':psutil.cpu_count(),
            'ram_gb':psutil.virtual_memory().total/1024**3,'cuda':torch.cuda.is_available(),
            'obs_shape':list(obs.shape),'action':action.tolist(),'gym_check':'passed','spec_hash':digest(spec(cfg))}
    save_json(result,ROOT/'reports/environment.json'); print(json.dumps(result,indent=2)); env.close(); model.env.close()

def cmd_benchmark(args,cfg):
    import psutil
    rows=[]
    for n in args.envs:
        c=copy.deepcopy(cfg); c['training']['n_envs']=n
        c['training']['vec_env']=args.vec_env; validate_config(c)
        torch.set_num_threads(c['training']['torch_threads'])
        # Warm JIT in parent before wall-clock; child startup is reported separately.
        warm=IndoorNavEnv(c); warm.reset(seed=1); warm.close()
        start=time.perf_counter(); env=make_vec(c); startup=time.perf_counter()-start
        try:
            env.seed(c['seed']); obs=env.reset()
            start=time.perf_counter(); process=psutil.Process(); cpu_before=process.cpu_times()
            for _ in range(max(1,args.steps//n)):
                obs,_,_,_=env.step(np.tile(np.array([0.2,0.1],dtype=np.float32),(n,1)))
            sampling=time.perf_counter()-start
            model=make_model(c,env); start=time.perf_counter()
            model.learn(total_timesteps=args.steps)
            elapsed=time.perf_counter()-start
            row={'n_envs':n,'vec_env':args.vec_env,'torch_threads':c['training']['torch_threads'],
                 'startup_seconds':startup,'sample_steps':max(1,args.steps//n)*n,'sample_seconds':sampling,
                 'sample_fps':max(1,args.steps//n)*n/sampling,'ppo_actual_steps':model.num_timesteps,
                 'ppo_seconds':elapsed,'ppo_fps':model.num_timesteps/elapsed,
                 'ram_gb':process.memory_info().rss/1024**3,'system_cpu_percent':psutil.cpu_percent(interval=0.1)}
            rows.append(row); print('BENCHMARK',json.dumps(row),flush=True)
        finally: env.close()
    save_json(rows,ROOT/'reports'/f'benchmark_{args.vec_env}.json')

def cmd_evaluate(args,cfg):
    model_path=Path(args.model).resolve(); cfg=load_config(model_path.parent/'config.yaml')
    if args.stage: cfg['environment']['stage']=args.stage
    output=Path(args.output) if args.output else model_path.parent/f'{args.split}_{args.stage or cfg["environment"]["stage"]}_{"unfiltered" if args.no_safety else "safe"}'
    if output.exists(): raise ValueError('Evaluation output already exists; use a new --output to preserve prior results')
    model=PPO.load(model_path,device='cpu'); torch.set_num_threads(2)
    summary,_,_=evaluate(model,cfg,args.episodes,args.split,output,safety=not args.no_safety)
    save_yaml(cfg,output/'evaluation_config.yaml'); plot_traces(output)
    print(json.dumps(summary,indent=2),flush=True)

def plot_traces(folder):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    folder=Path(folder); paths=list(folder.glob('trajectory_*.npz'))
    if not paths: return
    cols=min(3,len(paths)); rows=int(np.ceil(min(6,len(paths))/cols))
    fig,axes=plt.subplots(rows,cols,figsize=(cols*4,rows*4),squeeze=False)
    for ax,path in zip(axes.flat,paths[:6]):
        z=np.load(path)
        for x0,y0,x1,y1 in z['rectangles']: ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='gray'))
        ax.plot(*z['path'].T,'--',color='silver',label='A* reference')
        ax.plot(*z['trajectory'][:,:2].T,color='#1565c0',label='Policy')
        ax.scatter(*z['trajectory'][0,:2],c='green'); ax.scatter(*z['goal'],c='red',marker='*')
        ax.set_aspect('equal'); ax.set_xlim(-3.3,3.3); ax.set_ylim(-3.3,3.3); ax.set_title(path.stem); ax.legend(fontsize=7)
    for ax in list(axes.flat)[min(6,len(paths)):]: ax.set_visible(False)
    fig.tight_layout(); fig.savefig(folder/'trajectories.png',dpi=160); plt.close(fig)

def cmd_plot(args,cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(args.run)
    history=folder/'validation_history.jsonl'
    if not history.exists(): raise ValueError('No validation history yet')
    records=[json.loads(line) for line in history.read_text().splitlines()]
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for key in ('success_rate','collision_rate'): axes[0].plot([r['timesteps'] for r in records],[r[key] for r in records],marker='o',label=key)
    axes[0].set_ylim(-0.05,1.05); axes[0].legend(); axes[0].set_xlabel('Environment steps')
    axes[1].plot([r['timesteps'] for r in records],[r['mean_reward'] for r in records]); axes[1].set_title('Fixed validation mean return')
    fig.tight_layout(); fig.savefig(folder/'learning_curves.png',dpi=160); plt.close(fig)
    print(folder/'learning_curves.png')

def cmd_export(args,cfg):
    model_path=Path(args.model).resolve(); folder=model_path.parent
    cfg=load_config(folder/'config.yaml')
    torch.set_num_threads(cfg['training']['torch_threads'])
    name=args.name or folder.name; destination=ROOT/'exports'/name
    if destination.exists(): raise ValueError('Export exists; choose a new name')
    destination.mkdir(parents=True)
    for name_in in ('config.yaml','obs_spec.yaml','metadata.json','best_validation.json','training_result.json'):
        if (folder/name_in).exists(): shutil.copy2(folder/name_in,destination/name_in)
    shutil.copy2(model_path,destination/'policy.zip')
    for name_in in ('requirements.txt','requirements-core-lock.txt','requirements-win-lock.txt'):
        if (ROOT/name_in).exists(): shutil.copy2(ROOT/name_in,destination/name_in)
    source_nav=folder/'source/navrl'
    shutil.copytree(source_nav if source_nav.exists() else ROOT/'navrl',destination/'navrl',ignore=shutil.ignore_patterns('__pycache__','*.nbc','*.nbi'))
    shutil.copytree(ROOT/'deployment',destination/'deployment')
    from navrl.fixtures import write_fixtures
    write_fixtures(cfg,destination/'fixtures')
    from navrl.fixtures import fixture_cases
    observations=np.stack([case['observation'] for case in fixture_cases(cfg)])
    model=PPO.load(model_path,device='cpu'); actions,_=model.predict(observations,deterministic=True)
    np.savez_compressed(destination/'inference_reference.npz',observations=observations,actions=actions)
    timings=[]
    for _ in range(10): model.predict(observations[0],deterministic=True)
    for _ in range(100):
        start=time.perf_counter(); model.predict(observations[0],deterministic=True); timings.append((time.perf_counter()-start)*1000)
    save_json({'p50_ms':float(np.percentile(timings,50)),'p95_ms':float(np.percentile(timings,95)),
               'note':'Policy-only, excludes ROS/TF/BEV communication'},destination/'inference_latency.json')
    manifest={str(p.relative_to(destination)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.rglob('*') if p.is_file()}
    save_json(manifest,destination/'sha256.json')
    archive=shutil.make_archive(str(destination),'zip',destination)
    print(f'EXPORTED: {archive}',flush=True)

def main():
    parser=argparse.ArgumentParser(description='Windows CPU BEV/PPO toolkit')
    parser.add_argument('--config',default=str(ROOT/'configs/base.yaml'))
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('doctor')
    train=sub.add_parser('train'); train.add_argument('--name',required=True); train.add_argument('--steps',type=int); train.add_argument('--stage'); train.add_argument('--seed',type=int); train.add_argument('--envs',type=int); train.add_argument('--vec-env',choices=['dummy','subproc']); train.add_argument('--resume'); train.add_argument('--eval-every',type=int)
    bench=sub.add_parser('benchmark'); bench.add_argument('--steps',type=int,default=10000); bench.add_argument('--envs',nargs='+',type=int,default=[1,2,4]); bench.add_argument('--vec-env',choices=['dummy','subproc'],default='dummy')
    ev=sub.add_parser('evaluate'); ev.add_argument('--model',required=True); ev.add_argument('--episodes',type=int,default=12); ev.add_argument('--split',choices=['validation','test','diagnostic'],default='test'); ev.add_argument('--stage'); ev.add_argument('--no-safety',action='store_true'); ev.add_argument('--output')
    plot=sub.add_parser('plot'); plot.add_argument('--run',required=True)
    export=sub.add_parser('export'); export.add_argument('--model',required=True); export.add_argument('--name')
    sub.add_parser('fixtures')
    args=parser.parse_args(); cfg=load_config(args.config)
    for arg,section,key in [('steps','training','total_timesteps'),('stage','environment','stage'),('seed',None,'seed'),('envs','training','n_envs'),('vec_env','training','vec_env'),('eval_every','training','eval_every')]:
        value=getattr(args,arg,None)
        if value is not None and not isinstance(value,list):
            if section: cfg[section][key]=value
            else: cfg[key]=value
    if args.command=='fixtures':
        from navrl.fixtures import write_fixtures
        write_fixtures(cfg,ROOT/'fixtures'); return
    globals()['cmd_'+args.command](args,cfg)

if __name__=='__main__':
    # Required for Windows spawn. Never create environments on module import.
    main()
