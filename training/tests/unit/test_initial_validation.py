import hashlib
import json
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
from navrl.config import load_config
from navrl.env import IndoorNavEnv
import run as runner


def test_initializer_is_saved_before_first_policy_update(tmp_path, monkeypatch):
    cfg = load_config()
    cfg['training']['validation_stages'] = ['empty']
    env = IndoorNavEnv(cfg)
    model = PPO('MlpPolicy', env, n_steps=8, batch_size=8, device='cpu',
                policy_kwargs={'net_arch': [8, 8]})
    model.set_logger(configure(str(tmp_path / 'logs'), []))
    expected = {
        'success_rate': 1.0, 'collision_rate': 0.0,
        'mean_intervention_duration': 0.0, 'mean_reward': 20.0,
        'successes': 1, 'episodes': 1,
    }
    monkeypatch.setattr(runner, 'evaluate', lambda *args, **kwargs: (dict(expected), [], []))
    callback = runner.ValidationCallback(cfg, tmp_path)
    callback.init_callback(model)
    callback._on_training_start()
    assert callback.best_timesteps == 0
    model_path = tmp_path / 'best_model.zip'
    saved = json.loads((tmp_path / 'best_validation.json').read_text())
    assert saved['weights_sha256'] == hashlib.sha256(model_path.read_bytes()).hexdigest()
    assert saved['num_timesteps'] == 0
    env.close()
