import math
import numpy as np
from navrl.config import load_config
from navrl.env import IndoorNavEnv


def align_heading_error(env, error):
    delta = env.local_goal - env.pose[:2]
    heading = math.atan2(delta[1], delta[0])
    env.pose[2] = (heading + error + math.pi) % (2 * math.pi) - math.pi
    env.previous_heading_error = env._heading_error()


def run_until_done(env, action, limit=220):
    info = {}
    for _ in range(limit):
        _, _, done, truncated, info = env.step(action)
        if done or truncated:
            return info
    return info


def test_continuous_spin_is_stuck_despite_angular_velocity():
    env = IndoorNavEnv(load_config(), stage='empty')
    env.reset(seed=42)
    align_heading_error(env, math.pi)
    info = run_until_done(env, np.array([-1.0, 1.0], dtype=np.float32))
    assert info['reason'] == 'stuck'
    assert info['summary']['turn_grace_used'] <= env.cfg['environment']['turn_grace_seconds']


def test_occasional_turn_is_stuck_after_finite_grace():
    env = IndoorNavEnv(load_config(), stage='empty')
    env.reset(seed=42)
    align_heading_error(env, math.pi)
    env.step(np.array([-1.0, 1.0], dtype=np.float32))
    info = run_until_done(env, np.array([-1.0, 0.0], dtype=np.float32))
    assert info['reason'] == 'stuck'


def test_normal_large_turn_then_forward_is_not_stuck():
    env = IndoorNavEnv(load_config(), stage='empty')
    env.reset(seed=42)
    align_heading_error(env, math.pi)
    for _ in range(35):
        _, _, done, truncated, info = env.step(np.array([-1.0, 1.0], dtype=np.float32))
        assert not done and not truncated
    for _ in range(60):
        _, _, done, truncated, info = env.step(np.array([1.0, 0.0], dtype=np.float32))
        if done or truncated:
            break
    assert info['reason'] != 'stuck'


def test_positive_progress_resets_no_progress_timer():
    env = IndoorNavEnv(load_config(), stage='empty')
    env.reset(seed=42)
    env.no_progress_time = 5.0
    _, _, _, _, info = env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert env.no_progress_time == 0.0 or info['reason'] != 'stuck'
