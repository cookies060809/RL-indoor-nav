import copy
import math
import numpy as np
import pytest
from navrl.config import load_config
from navrl.control import scale_action,limit_rate,filter_velocity
from navrl.geometry import raycast,integrate,clearance_at,plan_path
from navrl.observation import scan_to_bev,goal_features,canonicalize_scan
from navrl.env import IndoorNavEnv

@pytest.fixture
def cfg():
    return load_config()

def test_action_and_rate_contract(cfg):
    np.testing.assert_allclose(scale_action([-1,-1],cfg),[0,-1])
    np.testing.assert_allclose(scale_action([1,1],cfg),[0.26,1])
    np.testing.assert_allclose(limit_rate([0.26,1],np.zeros(2),cfg),[0.05,0.2])
    with pytest.raises(ValueError): scale_action([np.nan,0],cfg)

def test_bev_no_return_unknown_and_edge(cfg):
    n=cfg['sensor']['beams']; empty=np.full(n,np.inf)
    bev=scan_to_bev(empty,cfg)
    assert np.count_nonzero(bev[0]==0)>0.96*cfg['observation']['size']**2
    invalid=scan_to_bev(np.full(n,np.nan),cfg)
    assert invalid[0,0,0]==0.5 and invalid[1,0,0]==0
    edge=empty.copy(); edge[n//2]=2.4
    assert not np.any(scan_to_bev(edge,cfg)[0]==1)

def test_occupancy_priority_and_directions(cfg):
    n=cfg['sensor']['beams']; ranges=np.full(n,np.inf)
    ranges[n//2]=1.0
    bev=scan_to_bev(ranges,cfg)
    size=cfg['observation']['size']; c=int((1+2.4)/(4.8/size))
    assert bev[0,size//2,c]==1
    np.testing.assert_allclose(goal_features([0,1],cfg),[0,1,0.2,0.5])
    np.testing.assert_allclose(goal_features([0,0],cfg),np.zeros(4))

def test_distance_euclidean(cfg):
    from scipy.ndimage import distance_transform_edt
    free=np.ones((5,5),bool); free[2,2]=False
    assert distance_transform_edt(free)[3,3]==pytest.approx(math.sqrt(2))

def test_scan_angle_canonicalization():
    source=np.arange(360,dtype=np.float32)
    actual=canonicalize_scan(source,0,2*np.pi/360)
    assert actual[180]==0 and actual[0]==180
    partial=canonicalize_scan(np.full(181,2,dtype=np.float32),-np.pi/2,np.pi/180)
    assert partial[180]==2 and np.isnan(partial[0])

def test_raycast_and_circular_contact(cfg):
    rectangles=np.array([[1,-3,1.2,3]],dtype=float)
    ranges=raycast(np.zeros(3),rectangles,360,0.24,10,0,0,0)
    assert ranges[180]==pytest.approx(1)
    assert np.isinf(ranges[0])
    assert clearance_at([0.8,0],rectangles,0.226)<0
    p=integrate(np.zeros(3),0.2,1,0.5)
    np.testing.assert_allclose(p[:2],[0.2*math.sin(.5),0.2*(1-math.cos(.5))])

def test_safety_near_and_side(cfg):
    scan=np.full(360,np.inf); scan[180]=0.28
    command,hit=filter_velocity([0.26,0],scan,cfg)
    assert hit and command[0]==0
    scan=np.full(360,np.inf); scan[270]=0.35
    command,hit=filter_velocity([0.26,0],scan,cfg)
    assert not hit and command[0]>0
    command,hit=filter_velocity([.2,0],np.full(360,np.nan),cfg)
    assert hit and np.all(command==0)

@pytest.mark.parametrize('stage',['empty','single','turn','clutter'])
def test_seeded_reachable_tasks(cfg,stage):
    a=IndoorNavEnv(cfg,stage=stage); b=IndoorNavEnv(cfg,stage=stage)
    oa,_=a.reset(seed=123); ob,_=b.reset(seed=123)
    np.testing.assert_array_equal(oa,ob)
    assert a.observation_space.contains(oa)
    for i in range(30):
        action=np.array([0.3,0.4],dtype=np.float32)
        oa,ra,da,ta,ia=a.step(action); ob,rb,db,tb,ib=b.step(action)
        np.testing.assert_array_equal(oa,ob)
        assert ra==rb and a.observation_space.contains(oa)
        if da or ta: break

def test_timeout_and_termination_separate(cfg):
    env=IndoorNavEnv(cfg); env.reset(seed=42)
    env.max_steps=1
    _,_,done,trunc,info=env.step(np.array([-1,0]))
    assert not done and trunc and info['reason']=='timeout'
    env.reset(seed=42); env.goal=env.pose[:2].copy(); env.hold=0.9
    _,_,done,trunc,info=env.step(np.array([-1,0]))
    assert done and not trunc and info['success']

def test_geometry_contact_overrides_success(cfg):
    env=IndoorNavEnv(cfg); env.reset(seed=42)
    env.pose[:2]=[2.85,0]; env.goal=env.pose[:2].copy(); env.hold=1
    _,_,done,trunc,info=env.step(np.array([-1,0]))
    assert done and info['collision'] and not info['success']
