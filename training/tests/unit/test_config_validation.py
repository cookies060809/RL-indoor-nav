import copy
import pytest
from navrl.config import CURRENT_SPEC, load_config, validate_config


@pytest.fixture
def cfg():
    return load_config()


def test_current_config_is_valid(cfg):
    assert cfg['spec_version'] == CURRENT_SPEC
    validate_config(cfg)


def test_old_spec_is_rejected_for_new_training(cfg):
    old = copy.deepcopy(cfg)
    old['spec_version'] = 'bev_nav_v1'
    with pytest.raises(ValueError, match='spec_version'):
        validate_config(old)
    validate_config(old, require_current=False)


@pytest.mark.parametrize('mutate', [
    lambda cfg: cfg['sensor'].__setitem__('range_min', cfg['sensor']['range_max']),
    lambda cfg: cfg['observation'].__setitem__('size', 64),
    lambda cfg: cfg['environment'].__setitem__('lookahead', 0),
    lambda cfg: cfg['environment']['goal_selection'].__setitem__('min_lookahead', 2.0),
    lambda cfg: cfg['environment']['stage_weights'].__setitem__('empty', -1),
    lambda cfg: cfg['training'].__setitem__('n_envs', 3) or cfg['ppo'].__setitem__('n_steps', 100),
    lambda cfg: cfg['ppo'].__setitem__('net_arch', []),
    lambda cfg: cfg['environment'].__setitem__('min_goal_distance', 9.0),
    lambda cfg: cfg['training'].__setitem__('validation_stages', ['unknown']),
])
def test_invalid_ranges_are_rejected(cfg, mutate):
    invalid = copy.deepcopy(cfg)
    mutate(invalid)
    with pytest.raises(ValueError):
        validate_config(invalid)
