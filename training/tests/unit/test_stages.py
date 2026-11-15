import copy
from collections import Counter
import numpy as np
from navrl.config import load_config
from navrl.env import STAGE_NAMES, IndoorNavEnv


def test_stage_weights_control_sampling():
    cfg = copy.deepcopy(load_config())
    cfg['environment']['stage'] = 'mixed'
    env = IndoorNavEnv(cfg)
    env.reset(seed=7)
    samples = Counter(env._choose_stage() for _ in range(3000))
    weights = cfg['environment']['stage_weights']
    total = sum(weights.values())
    for stage, weight in weights.items():
        expected = weight / total
        assert abs(samples[stage] / 3000 - expected) < 0.035, (stage, samples[stage])


def test_stage_is_fixed_before_task_resampling():
    cfg = copy.deepcopy(load_config())
    cfg['environment']['stage'] = 'mixed'
    cfg['environment']['stage_weights'] = {'protective': 1.0}
    env = IndoorNavEnv(cfg)
    for seed in range(5):
        env.reset(seed=seed)
        assert env.actual_stage == 'protective'
        assert 'protective' in STAGE_NAMES


def test_new_hard_stages_generate_reachable_tasks():
    cfg = copy.deepcopy(load_config())
    for stage in ('dense', 'tight_turn', 'protective'):
        env = IndoorNavEnv(cfg, stage=stage)
        for seed in range(3):
            obs, info = env.reset(seed=seed)
            assert info['stage'] == stage
            assert env.path is not None and len(env.path) > 1
            assert env.observation_space.contains(obs)
        env.close()
