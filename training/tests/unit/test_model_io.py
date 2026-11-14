import numpy as np
import pytest
from stable_baselines3 import PPO
from navrl.config import load_config
from navrl.env import IndoorNavEnv

@pytest.mark.parametrize('split',[False,True])
def test_policy_save_load_preserves_deterministic_action(tmp_path,split):
    cfg=load_config(); env=IndoorNavEnv(cfg); obs,_=env.reset(seed=5)
    kwargs={'net_arch':[16,16]}
    if split:
        from navrl.features import BevStateExtractor
        kwargs['features_extractor_class']=BevStateExtractor
    model=PPO('MlpPolicy',env,n_steps=8,batch_size=8,seed=8,device='cpu',policy_kwargs=kwargs)
    expected,_=model.predict(obs,deterministic=True)
    path=tmp_path/'policy.zip'; model.save(path)
    loaded=PPO.load(path,device='cpu'); actual,_=loaded.predict(obs,deterministic=True)
    np.testing.assert_allclose(actual,expected,atol=1e-7)

@pytest.mark.parametrize('size',[32,48])
def test_policy_input_dimension_matches_bev_contract(size):
    cfg=load_config(); cfg['observation']['size']=size
    env=IndoorNavEnv(cfg); obs,_=env.reset(seed=8)
    assert obs.shape==(2*size*size+8,)
    assert env.observation_space.contains(obs)
