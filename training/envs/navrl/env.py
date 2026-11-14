"""Gymnasium task with geometric collision, A* route and moving local goal."""
import copy
import math
from collections import deque
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from .geometry import raycast, clearance_at, plan_path, integrate
from .observation import make_observation
from .control import scale_action, limit_rate, filter_velocity

class IndoorNavEnv(gym.Env):
    metadata={'render_modes':[]}

    def __init__(self, cfg, stage=None, safety=None):
        super().__init__()
        self.cfg=copy.deepcopy(cfg)
        if stage: self.cfg['environment']['stage']=stage
        if safety is not None: self.cfg['safety']['enabled']=safety
        n=self.cfg['observation']['size']
        low=np.concatenate([np.zeros(2*n*n),np.array([-1,-1,0,-1,0,-1,0,-1])]).astype(np.float32)
        high=np.ones_like(low)
        self.observation_space=spaces.Box(low,high,dtype=np.float32)
        self.action_space=spaces.Box(-1,1,(2,),dtype=np.float32)

    def _make_layout(self,stage):
        h=self.cfg['environment']['room_half_size']
        rect=[[-h-0.2,-h-0.2,-h,h+0.2],[h,-h-0.2,h+0.2,h+0.2],[-h,-h-0.2,h,-h],[-h,h,h,h+0.2]]
        if stage=='mixed': stage=self.np_random.choice(['empty','single','turn','clutter'])
        if stage=='turn':
            rect.extend([[-0.2,-1.8,0.2,0.7]])
        else:
            count={'empty':0,'single':1,'clutter':5}[stage]
            for _ in range(count):
                center=self.np_random.uniform(-1.8,1.8,2)
                width=self.np_random.uniform(0.25,0.65,2)
                rect.append([*(center-width/2),*(center+width/2)])
        self.actual_stage=str(stage)
        return np.asarray(rect,dtype=float)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        e=self.cfg['environment']; r=self.cfg['robot']
        for _ in range(120):
            self.rectangles=self._make_layout(e['stage'])
            h=e['room_half_size']-r['radius']-0.3
            start=self.np_random.uniform(-h,h,2)
            angle=self.np_random.uniform(-np.pi,np.pi)
            distance=self.np_random.uniform(e['min_goal_distance'],e['max_goal_distance'])
            goal=start+distance*np.array([np.cos(angle),np.sin(angle)])
            if np.any(np.abs(goal)>h): continue
            if min(clearance_at(start,self.rectangles,r['radius']),clearance_at(goal,self.rectangles,r['radius']))<e['planner_margin']+0.05: continue
            path=plan_path(start,goal,self.rectangles,self.cfg)
            if path is not None: break
        else: raise RuntimeError('Could not generate reachable task; inspect task distances/layout bounds')
        self.goal=goal; self.path=path
        self.arc=np.concatenate([[0.0],np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))])
        self.reference_length=float(self.arc[-1])
        # Task heading is randomized rather than privileged goal-aligned.
        self.pose=np.array([*start,self.np_random.uniform(-math.pi,math.pi)])
        self.velocity=np.zeros(2); self.previous_executed=np.zeros(2)
        self.steps=0; self.progress=0.0; self.path_index=0; self.hold=0.0
        self.trajectory=[self.pose.copy()]; self.path_length=0.0
        self.interventions=0; self.intervention_steps=0; self.was_intervening=False
        self.reward_sums={}; self.recent_progress=deque(maxlen=max(2,int(e['stuck_seconds']/r['dt'])))
        self.min_clearance=clearance_at(self.pose,self.rectangles,r['radius'])
        # No task may time out before the reference route is physically traversable.
        self.max_steps=max(e['max_steps'],int(math.ceil((e['timeout_factor']*self.reference_length/r['v_max']+15)/r['dt'])))
        self.ranges=self._scan()
        self.local_goal=self.path[min(len(self.path)-1,np.searchsorted(self.arc,e['lookahead']))]
        return self._observation(),{'stage':self.actual_stage,'reference_length':self.reference_length}

    def _scan(self):
        s=self.cfg['sensor']
        ranges=raycast(self.pose,self.rectangles,s['beams'],s['range_min'],s['range_max'],s['x'],s['y'],s['yaw'])
        if s['noise_std']:
            valid=np.isfinite(ranges)
            ranges[valid]+=self.np_random.normal(0,s['noise_std'],np.count_nonzero(valid)).astype(np.float32)
            ranges[valid]=np.clip(ranges[valid],s['range_min'],s['range_max'])
        return ranges

    def _update_route(self):
        """Bounded nearest-point search prevents jumps at crossing/folded routes."""
        lo=max(0,self.path_index-4); hi=min(len(self.path),self.path_index+35)
        self.path_index=lo+int(np.argmin(np.linalg.norm(self.path[lo:hi]-self.pose[:2],axis=1)))
        raw=float(self.arc[self.path_index])
        # Discrete projection can advance one grid sample; cap reward separately.
        self.progress=raw
        target=min(self.reference_length,raw+self.cfg['environment']['lookahead'])
        j=min(len(self.path)-1,int(np.searchsorted(self.arc,target)))
        self.local_goal=self.path[j]

    def _observation(self):
        delta=self.local_goal-self.pose[:2]; c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
        local=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
        return make_observation(self.ranges,local,self.velocity,self.previous_executed,self.cfg)

    def _compute_reward(self,delta_progress,clearance,success,collision,old_w):
        """Primary tuning point: every component is exposed in episode summaries."""
        r=self.cfg['reward']
        terms={
            'progress':r['progress']*delta_progress,
            'time':r['time'],
            'clearance':r['clearance']*max(0,(r['clearance_threshold']-clearance)/r['clearance_threshold']),
            'smoothness':r['smoothness']*abs(self.velocity[1]-old_w),
            'success':r['success'] if success else 0.0,
            'collision':r['collision'] if collision else 0.0,
        }
        for k,v in terms.items(): self.reward_sums[k]=self.reward_sums.get(k,0)+float(v)
        return float(sum(terms.values())),terms

    def step(self,action):
        r,e=self.cfg['robot'],self.cfg['environment']
        old_pose=self.pose.copy(); old_progress=self.progress; old_velocity=self.velocity.copy()
        target=scale_action(action,self.cfg)
        rate_limited=limit_rate(target,self.velocity,self.cfg)
        self.velocity,intervening=filter_velocity(rate_limited,self.ranges,self.cfg)
        self.interventions+=int(intervening and not self.was_intervening)
        self.intervention_steps+=int(intervening); self.was_intervening=intervening
        collision=False
        # Substep geometric contact prevents tunnelling through thin objects.
        for _ in range(4):
            self.pose=integrate(self.pose,*self.velocity,r['dt']/4)
            clearance=clearance_at(self.pose,self.rectangles,r['radius'])
            self.min_clearance=min(self.min_clearance,clearance)
            if clearance<=0:
                collision=True; break
        self.previous_executed=self.velocity.copy()
        self.steps+=1; self.path_length+=float(np.linalg.norm(self.pose[:2]-old_pose[:2]))
        self.trajectory.append(self.pose.copy()); self._update_route()
        # Do not clip positive and negative progress asymmetrically: that can reward cycling.
        delta_progress=self.progress-old_progress
        within=np.linalg.norm(self.goal-self.pose[:2])<=e['goal_radius']
        self.hold=self.hold+r['dt'] if within else 0.0
        success=self.hold>=e['goal_hold_seconds']-1e-8 and not collision
        self.recent_progress.append(self.progress)
        stuck=len(self.recent_progress)==self.recent_progress.maxlen and max(self.recent_progress)-min(self.recent_progress)<0.05 and abs(self.velocity[1])<0.1
        terminated=bool(collision or success or stuck)
        truncated=bool(self.steps>=self.max_steps and not terminated)
        reason='collision' if collision else 'success' if success else 'stuck' if stuck else 'timeout' if truncated else ''
        reward,terms=self._compute_reward(delta_progress,clearance,success,collision,old_velocity[1])
        self.ranges=self._scan()
        info={'reward_terms':terms,'success':bool(success),'collision':bool(collision),'reason':reason,'intervening':bool(intervening)}
        if terminated or truncated:
            info['summary']={
                'success':bool(success),'collision':bool(collision),'reason':reason,'stage':self.actual_stage,
                'steps':self.steps,'sim_time':self.steps*r['dt'],'path_length':self.path_length,
                'reference_length':self.reference_length,'min_clearance':self.min_clearance,
                'interventions':self.interventions,'intervention_duration':self.intervention_steps*r['dt'],
                **{'reward_'+k:v for k,v in self.reward_sums.items()},
            }
        return self._observation(),reward,terminated,truncated,info
