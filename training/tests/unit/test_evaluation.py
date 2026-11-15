import json
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
from navrl.config import load_config
from navrl.env import IndoorNavEnv
from navrl.evaluation import evaluate


def test_evaluation_writes_step_level_actions_and_velocities(tmp_path):
    cfg = load_config()
    env = IndoorNavEnv(cfg)
    env.reset(seed=2)
    model = PPO('MlpPolicy', env, n_steps=8, batch_size=8, seed=2, device='cpu',
                policy_kwargs={'net_arch': [8, 8]})
    summary, rows, _ = evaluate(model, cfg, episodes=1, split='diagnostic',
                                output=tmp_path, stages=['empty'],
                                metadata={'test': True})
    assert summary['episodes'] == 1
    assert summary['stages'] == ['empty']
    assert summary['effective_safety'] is True
    assert (tmp_path / 'summary.json').exists()
    assert (tmp_path / 'evaluation_metadata.json').exists()
    trace = np.load(tmp_path / 'trajectory_000.npz')
    for key in ('raw_action', 'requested_velocity', 'executed_velocity', 'intervention', 'reason_code'):
        assert key in trace
    assert trace['raw_action'].shape[1] == 2
    assert trace['requested_velocity'].shape == trace['executed_velocity'].shape
    assert rows[0]['stage'] == 'empty'
    env.close()
