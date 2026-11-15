import copy
import math
import sys
from pathlib import Path
import numpy as np
import pytest
from navrl.config import load_config
from navrl.goal import adaptive_lookahead, chord_distance_to_path, select_local_goal


@pytest.fixture
def cfg():
    return load_config()


def make_path(points):
    path = []
    for start, end in zip(points[:-1], points[1:]):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        count = max(2, int(np.linalg.norm(end - start) / 0.04) + 1)
        segment = np.linspace(start, end, count, endpoint=False)
        path.extend(segment)
    path.append(np.asarray(points[-1], dtype=float))
    path = np.asarray(path)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))])
    return path, arc


def test_fixed_mode_matches_original_lookahead(cfg):
    path, arc = make_path([(0, 0), (1, 0), (1, 1)])
    index = 15
    goal_index, point = select_local_goal(path, arc, np.array([*path[index], 0.0]), cfg, index)
    expected = min(len(path) - 1, int(np.searchsorted(arc, arc[index] + cfg['environment']['lookahead'])))
    assert goal_index == expected
    np.testing.assert_allclose(point, path[expected])


def test_adaptive_shortens_lookahead_at_corner(cfg):
    adaptive = copy.deepcopy(cfg)
    adaptive['environment']['goal_selection']['mode'] = 'adaptive'
    path, arc = make_path([(0, 0), (1, 0), (1, 1)])
    index = 15
    lookahead, turn = adaptive_lookahead(path, arc, np.array([*path[index], 0.0]), index, adaptive)
    assert turn == pytest.approx(math.pi / 2)
    assert lookahead < adaptive['environment']['lookahead']
    goal_index, point = select_local_goal(path, arc, np.array([*path[index], 0.0]), adaptive, index)
    assert goal_index <= index + int(round((adaptive['environment']['lookahead']) / 0.04)) + 1


def test_adaptive_rejects_chord_outside_path_corridor(cfg):
    adaptive = copy.deepcopy(cfg)
    adaptive['environment']['lookahead'] = 3.0
    adaptive['environment']['goal_selection']['mode'] = 'adaptive'
    path, arc = make_path([(0, 0), (2, 0), (2, 1)])
    index = 0
    corridor = adaptive['environment']['goal_selection']['direct_corridor']
    assert chord_distance_to_path(path[0], path[-1], path, 0, len(path) - 1, corridor) > corridor
    goal_index, point = select_local_goal(path, arc, np.array([*path[index], 0.0]), adaptive, index)
    distance = chord_distance_to_path(path[index], point, path, index, goal_index, corridor)
    assert distance <= corridor


def test_deployment_reference_uses_same_function():
    deployment = Path(__file__).resolve().parents[2] / 'scripts/deployment'
    sys.path.insert(0, str(deployment))
    try:
        import route_goal
        assert route_goal.select_local_goal is select_local_goal
    finally:
        sys.path.remove(str(deployment))
        sys.modules.pop('route_goal', None)
