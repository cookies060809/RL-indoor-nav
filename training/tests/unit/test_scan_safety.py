import numpy as np
import pytest
from navrl.config import load_config
from navrl.control import filter_velocity, scan_coverage
from navrl.observation import scan_to_bev
from navrl.scan import FREE, HIT, UNKNOWN, classify_scan, normalize_for_raster


@pytest.fixture
def cfg():
    return load_config()


def test_scan_classification_is_single_source(cfg):
    ranges = np.array([np.inf, 0.5, 12.0, np.nan, -np.inf, 0.1], dtype=np.float32)
    values, hit, free, unknown = classify_scan(ranges, cfg['sensor'])
    assert np.count_nonzero(hit) == 1
    assert np.count_nonzero(free) == 1
    assert np.count_nonzero(unknown) == 4
    normalized = normalize_for_raster(ranges, cfg['sensor'])
    assert np.isposinf(normalized[0])
    assert normalized[1] == pytest.approx(0.5)
    assert np.all(np.isnan(normalized[2:]))


def test_above_max_is_unknown_not_free(cfg):
    ranges = np.full(cfg['sensor']['beams'], np.inf, dtype=np.float32)
    ranges[170:191] = cfg['sensor']['range_max'] * 1.5
    bev = scan_to_bev(ranges, cfg)
    assert np.count_nonzero(bev[0] == 0.5) > 20
    command, hit, reason = filter_velocity([0.26, 0], ranges, cfg)
    assert hit and command[0] == 0 and reason == 'scan_unknown_front'


def test_front_missing_does_not_allow_forward_but_allows_reverse(cfg):
    beams = cfg['sensor']['beams']
    scan = np.full(beams, np.inf, dtype=np.float32)
    scan[150:211] = np.nan
    forward, hit, reason = filter_velocity([0.26, 0], scan, cfg)
    assert hit and forward[0] == 0 and reason == 'scan_unknown_front'
    reverse, hit, reason = filter_velocity([-0.26, 0], scan, cfg)
    assert not hit and reverse[0] < 0 and reason == 'none'


def test_rear_missing_does_not_allow_reverse_but_allows_forward(cfg):
    beams = cfg['sensor']['beams']
    scan = np.full(beams, np.inf, dtype=np.float32)
    scan[330:] = np.nan
    scan[:31] = np.nan
    reverse, hit, reason = filter_velocity([-0.26, 0], scan, cfg)
    assert hit and reverse[0] == 0 and reason == 'scan_unknown_rear'
    forward, hit, reason = filter_velocity([0.26, 0], scan, cfg)
    assert not hit and forward[0] > 0 and reason == 'none'


def test_all_no_return_is_known_free(cfg):
    scan = np.full(cfg['sensor']['beams'], np.inf, dtype=np.float32)
    command, hit, reason = filter_velocity([0.26, 0.2], scan, cfg)
    assert not hit and command[0] > 0 and command[1] == pytest.approx(0.2)


def test_all_nan_is_no_valid_scan(cfg):
    scan = np.full(cfg['sensor']['beams'], np.nan, dtype=np.float32)
    command, hit, reason = filter_velocity([0.26, 0], scan, cfg)
    assert hit and np.all(command == 0) and reason == 'no_valid_scan'


def test_rotation_requires_full_coverage(cfg):
    scan = np.full(cfg['sensor']['beams'], np.inf, dtype=np.float32)
    scan[270] = np.nan
    command, hit, reason = filter_velocity([0, 0.8], scan, cfg)
    assert hit and command[1] == 0 and reason == 'scan_unknown_rotation'


def test_below_min_is_unknown(cfg):
    scan = np.full(cfg['sensor']['beams'], np.inf, dtype=np.float32)
    scan[180] = cfg['sensor']['range_min'] * 0.5
    command, hit, reason = filter_velocity([0.26, 0], scan, cfg)
    assert hit and command[0] == 0 and reason == 'scan_unknown_front'
