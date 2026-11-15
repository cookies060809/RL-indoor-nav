import copy
import pytest
from navrl.config import load_config
from run import resolve_evaluation_config


@pytest.fixture
def cfg():
    return load_config()


@pytest.mark.parametrize('model_safety', [True, False])
@pytest.mark.parametrize('no_safety', [False, True])
def test_effective_config_matches_execution_switch(cfg, model_safety, no_safety):
    model_cfg = copy.deepcopy(cfg)
    model_cfg['safety']['enabled'] = model_safety
    effective = resolve_evaluation_config(
        model_cfg, stage='single', no_safety=no_safety,
        split='diagnostic', episodes=4)
    expected = model_safety and not no_safety
    assert effective['safety']['enabled'] is expected
    assert effective['evaluation']['safety_enabled'] is expected
    assert effective['environment']['stage'] == 'single'
    assert effective['evaluation']['stages'] == ['single']
    assert effective['evaluation']['split'] == 'diagnostic'
    assert effective['evaluation']['episodes_per_stage'] == 4


def test_stage_and_stages_are_mutually_exclusive(cfg):
    with pytest.raises(ValueError):
        resolve_evaluation_config(cfg, stage='single', stages=['turn'])


def test_stages_override_is_recorded_without_losing_model_stage(cfg):
    model_cfg = copy.deepcopy(cfg)
    model_cfg['environment']['stage'] = 'mixed'
    effective = resolve_evaluation_config(model_cfg, stages=['empty', 'dense'])
    assert effective['environment']['stage'] == 'mixed'
    assert effective['evaluation']['stages'] == ['empty', 'dense']
