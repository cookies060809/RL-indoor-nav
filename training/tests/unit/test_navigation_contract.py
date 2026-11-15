"""Mission-level contracts independent of ROS/Gazebo installation."""
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rl_nav_pkg'))
from rl_nav_pkg.path_tracking import PathTracker, GoalEpoch, chord_is_free
from rl_nav_pkg.model_profile import load_model_profile, runtime_parameters


def test_carrot_is_not_final_goal():
    tracker = PathTracker(lookahead=0.1)
    tracker.set_path([[0, 0], [0.1, 0], [2, 0]], 0)
    goal, status = tracker.update([0, 0], 0, 0)
    assert np.linalg.norm(goal) < 0.3
    assert status['ready'] and not status['reached']
    assert status['final_distance'] == 2


def test_arrival_requires_full_sim_time_hold():
    tracker = PathTracker()
    tracker.set_path([[0, 0], [1, 0]], 0)
    assert tracker.update([1, 0], 1, 0)[1]['reason'] == 'goal_hold'
    assert not tracker.update([1, 0], 1, 2)[1]['reached']
    assert not tracker.update([1, 0], 1.9, 3)[1]['reached']
    assert tracker.update([1, 0], 2, 4)[1]['reached']


def test_departing_goal_restarts_hold_and_replan_preserves_hold():
    tracker = PathTracker()
    tracker.set_path([[0, 0], [1, 0]], 0)
    tracker.update([1, 0], 1, 0)
    tracker.update([0, 0], 1.5, 0.5)
    assert not tracker.update([1, 0], 2, 1)[1]['reached']
    tracker.set_path([[0.5, 0], [1, 0]], 1.5)
    assert tracker.update([1, 0], 3, 2)[1]['reached']


def test_stale_empty_and_clock_reset_invalidate_path():
    tracker = PathTracker(path_timeout=1)
    tracker.set_path([[0, 0], [2, 0]], 0)
    assert tracker.update([0, 0], 10, 2)[1]['reason'] == 'path_stale'
    tracker.set_path([[0, 0], [2, 0]], 2)
    assert tracker.update([0, 0], 0.1, 2)[1]['reason'] == 'clock_reset'
    assert tracker.path is None
    assert not tracker.set_path([], 3)
    assert tracker.update([0, 0], 0.2, 3)[0] is None


def test_replanned_path_projects_to_current_position():
    tracker = PathTracker(lookahead=1)
    tracker.set_path([[x / 10, 0] for x in range(101)], 0)
    goal, status = tracker.update([8, 0], 1, 0)
    assert tracker.index == 80 and goal[0] >= 9 and status['ready']


def test_visibility_blocks_unknown_and_allows_only_inward_map_edge_escape():
    field = np.full((100, 100), np.inf)
    assert chord_is_free([0.1, 2], [1, 2], field, [0, 0], 0.1, 0.26)
    assert not chord_is_free([0.1, 2], [0.05, 3], field, [0, 0], 0.1, 0.26)
    assert not chord_is_free([0.1, 2], [0.1, 3], field, [0, 0], 0.1, 0.26)
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[:, 30] = -1
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[2, 2], [4, 2]], 0)
    assert tracker.update([2, 2], 0, 0)[1]['reason'] == 'no_visible_carrot'


def test_visibility_retries_with_bare_radius_inside_margin():
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[40, :] = 100
    tracker.set_map(grid, [0, 0], 0.03)
    tracker.set_path([[1.5, 1.47], [2.5, 1.47]], 0)
    goal, status = tracker.update([1.5, 1.47], 0, 0)
    assert status['ready'] and status['reason'] == 'tracking'
    assert goal[0] > 1.5


def test_visibility_escapes_when_footprint_overlaps_the_map():
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[15:19, 17:22] = 100
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[1.7, 2.2], [4.0, 2.2]], 0)
    goal, status = tracker.update([1.7, 2.1], 0, 0)
    assert status['ready'] and status['reason'] == 'tracking_escape'
    assert goal[0] > 1.7
    assert np.linalg.norm(goal - [1.7, 2.1]) <= 0.4 + 1e-8


def test_escape_handles_coarse_visibility_guard():
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[18, :] = 100
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[1.7, 2.2], [4.0, 2.2]], 0)
    goal, status = tracker.update([1.7, 2.19], 0, 0)
    assert status['reason'] == 'tracking_escape' and np.linalg.norm(goal - [1.7, 2.19]) <= 0.4 + 1e-8


def test_escape_uses_rotated_map_origin():
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[15:19, 17:22] = 100
    origin = np.array([5.0, -3.0])
    rotation = np.array([[0.0, -1.0], [1.0, 0.0]])
    position = rotation @ [1.7, 2.1] + origin
    tracker.set_map(grid, origin, 0.1, np.pi / 2)
    tracker.set_path(np.array([[1.7, 2.2], [4.0, 2.2]]) @ rotation.T + origin, 0)
    goal, status = tracker.update(position, 0, 0)
    assert status['reason'] == 'tracking_escape'
    assert np.linalg.norm(goal - position) <= 0.4 + 1e-8


@pytest.mark.parametrize('blocked', [100, -1])
def test_escape_cannot_cross_wall_or_unknown_cells(blocked):
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[:, 20] = blocked
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[1.8, 2.0], [3.0, 2.0]], 0)
    goal, status = tracker.update([1.8, 2.0], 0, 0)
    assert goal is None and not status['ready']


def test_escape_cannot_move_closer_to_wall_even_if_centre_stays_free():
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[:, 20] = 100
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[1.8, 2.0], [1.95, 3.0]], 0)
    goal, status = tracker.update([1.8, 2.0], 0, 0)
    assert goal is None and not status['ready']


@pytest.mark.parametrize('blocked', [100, -1])
def test_escape_does_not_start_from_occupied_or_unknown_centre(blocked):
    tracker = PathTracker(mode='visible')
    grid = np.zeros((100, 100), dtype=int)
    grid[21, 17] = blocked
    tracker.set_map(grid, [0, 0], 0.1)
    tracker.set_path([[1.7, 2.2], [4.0, 2.2]], 0)
    goal, status = tracker.update([1.7, 2.1], 0, 0)
    assert goal is None and not status['ready']


def test_old_plan_generation_never_reappears():
    epoch = GoalEpoch()
    old = epoch.replace()
    new = epoch.replace()
    assert epoch.accepts(new) and not epoch.accepts(old)


def test_legacy_profile_keeps_legacy_sensor_contract():
    cfg = load_model_profile(ROOT, 'split_mixed_pretrain_v1')
    bev, safety, local = runtime_parameters(cfg, navigation=True)
    assert bev['spec_version'] == 'bev_nav_v1'
    assert bev['require_local_goal'] and not safety['strict_scan_coverage']
    assert local['subgoal_mode'] == 'fixed'
    cfg['spec_version'] = 'bev_nav_v2'
    cfg['environment']['subgoal_mode'] = 'visible'
    bev, safety, local = runtime_parameters(cfg, navigation=True)
    assert bev['spec_version'] == 'bev_nav_v2' and safety['strict_scan_coverage']
    assert local['subgoal_mode'] == 'visible'


@pytest.mark.parametrize('model_id', ['../outside', 'x/y', '', 'a;sh'])
def test_profile_rejects_non_bundle_name(model_id):
    with pytest.raises(ValueError):
        load_model_profile(ROOT, model_id)
