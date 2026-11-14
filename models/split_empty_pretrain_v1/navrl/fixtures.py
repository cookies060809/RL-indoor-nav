import json
from pathlib import Path
import copy
import numpy as np
from .observation import make_observation
from .config import spec,digest,save_yaml

def fixture_cases(cfg):
    n=cfg['sensor']['beams']; angles=-np.pi+np.arange(n)*2*np.pi/n
    empty=np.full(n,np.inf,dtype=np.float32)
    front=np.full(n,np.inf,dtype=np.float32)
    valid=np.cos(angles)>0.05; front[valid]=1.0/np.cos(angles[valid])
    left=np.full(n,np.inf,dtype=np.float32)
    valid=np.sin(angles)>0.05; left[valid]=1.0/np.sin(angles[valid])
    corner=np.minimum(front,left)
    invalid=empty.copy(); invalid[:20]=np.nan; invalid[20]=0.1; invalid[21]=-np.inf
    boundary=empty.copy(); boundary[n//2]=cfg['observation']['extent']/2
    minimum=empty.copy(); minimum[n//2]=cfg['sensor']['range_min']
    cases=[('empty',empty,[1,0]),('front_wall',front,[1,0]),('left_wall',left,[0,1]),('right_wall',left[::-1].copy(),[0,-1]),('corner',corner,[1,1]),('invalid',invalid,[1,0]),('boundary',boundary,[1,0]),('minimum',minimum,[1,0]),('zero_goal',empty,[0,0])]
    results=[]
    for name,ranges,goal in cases:
        c=copy.deepcopy(cfg)
        results.append({'name':name,'ranges':ranges,'goal':np.asarray(goal,dtype=np.float32),'config':c,
                        'observation':make_observation(ranges,goal,[0.1,0.2],[0.05,-0.1],c)})
    c=copy.deepcopy(cfg); c['sensor']['x']=0.12; c['sensor']['y']=-0.04; c['sensor']['yaw']=0.15
    results.append({'name':'offset_sensor','ranges':corner,'goal':np.array([1,0],dtype=np.float32),'config':c,
                    'observation':make_observation(corner,[1,0],[0.1,0.2],[0.05,-0.1],c)})
    return results

def write_fixtures(cfg,folder):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for case in fixture_cases(cfg):
        name=case['name']
        np.savez_compressed(folder/f'{name}.npz',ranges=case['ranges'],goal=case['goal'],velocity=np.array([0.1,0.2]),previous_executed=np.array([0.05,-0.1]),observation=case['observation'])
        save_yaml(case['config'],folder/f'{name}.yaml')
        # JSON transport avoids requiring C++ to read numpy compressed files.
        ranges=[float(x) if np.isfinite(x) else 'inf' if np.isposinf(x) else '-inf' if np.isneginf(x) else 'nan' for x in case['ranges']]
        value={'name':name,'ranges':ranges,'angle_min':-np.pi,'angle_increment':2*np.pi/len(ranges),
               'goal':case['goal'].tolist(),'velocity':[0.1,0.2],'previous_executed':[0.05,-0.1],
               'expected_observation':case['observation'].tolist(),'config':case['config']}
        (folder/f'{name}.json').write_text(json.dumps(value),encoding='utf-8')
        manifest.append({'case':name,'spec_hash':digest(spec(case['config']))})
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(f'{len(manifest)} fixtures written: {folder}')
