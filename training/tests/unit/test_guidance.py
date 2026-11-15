import numpy as np
import pytest
from navrl.config import load_config, validate_config
from navrl.guidance import choose_local_goal, chord_clear


def test_visible_goal_rejects_a_chord_through_the_wall():
    cfg = load_config()
    cfg['environment']['subgoal_mode'] = 'visible'
    cfg['environment']['visibility_margin'] = 0.04
    cfg['environment']['lookahead'] = 2.0
    pose = np.array([-0.6, 0.0, 0.0])
    path = np.array([[-.6, 0.], [-.6, .4], [-.6, .8], [0., .8], [.6, .8]])
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    rectangles = np.array([[-.2, -1., .2, .35]])
    goal, index = choose_local_goal(pose, path, arc, 0, rectangles, cfg)
    radius = cfg['robot']['radius'] + .04
    assert not chord_clear(pose[:2], path[-1], rectangles, radius)
    assert chord_clear(pose[:2], goal, rectangles, radius)
    assert index < len(path) - 1


def test_fixed_goal_preserves_original_lookahead():
    cfg = load_config()
    path = np.column_stack([np.linspace(0, 2, 51), np.zeros(51)])
    arc = np.linspace(0, 2, 51)
    goal, index = choose_local_goal(np.zeros(3), path, arc, 0, np.empty((0, 4)), cfg)
    assert index == np.searchsorted(arc, cfg['environment']['lookahead'])
    np.testing.assert_array_equal(goal, path[index])


def test_no_safe_goal_reports_robot_position():
    cfg = load_config()
    cfg['environment']['subgoal_mode'] = 'visible'
    pose = np.array([0., 0., 0.])
    path = np.array([[0., 0.], [1., 0.]])
    goal, index = choose_local_goal(pose, path, np.array([0., 1.]), 0,
                                    np.array([[-.1, -.1, .1, .1]]), cfg)
    np.testing.assert_array_equal(goal, pose[:2])
    assert index == 0


@pytest.mark.parametrize('margin', [-1, float('nan'), float('inf')])
def test_invalid_visibility_margin_is_rejected(margin):
    cfg = load_config()
    cfg['environment']['visibility_margin'] = margin
    with pytest.raises(ValueError, match='visibility_margin'):
        validate_config(cfg)
