import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts/deployment'))
from watchdog import ObservationGate


def test_duplicate_and_old_observations_are_rejected():
    gate = ObservationGate(timeout=1.0)
    token, error = gate.begin(100, 0.0, 1_000_000_000)
    assert token == 1 and error is None
    duplicate, error = gate.begin(100, 0.1, 1_100_000_000)
    assert duplicate is None and error == 'duplicate_or_old_stamp'
    older, error = gate.begin(99, 0.2, 1_200_000_000)
    assert older is None and error == 'duplicate_or_old_stamp'


def test_simulation_pause_does_not_extend_watchdog():
    gate = ObservationGate(timeout=1.0)
    gate.begin(100, 0.0, 1_000_000_000)
    assert not gate.watchdog_expired(0.9)
    assert gate.watchdog_expired(1.1)


def test_simulation_backwards_resets_state():
    gate = ObservationGate(timeout=1.0)
    gate.begin(100, 0.0, 10_000_000_000)
    token, error = gate.begin(200, 0.1, 5_000_000_000)
    assert token is None and error == 'sim_time_backwards'
    assert gate.last_receive is None
    assert gate.watchdog_expired(0.1)


def test_inference_timeout_and_newer_observation_invalidate_result():
    gate = ObservationGate(timeout=1.0, max_inference_seconds=0.5)
    token, _ = gate.begin(100, 0.0, 1_000_000_000)
    assert gate.can_publish(token, 0.0, 0.2)
    assert not gate.can_publish(token, 0.0, 0.6)
    newer, _ = gate.begin(200, 0.1, 1_100_000_000)
    assert newer != token
    assert not gate.can_publish(token, 0.1, 0.2)


def test_reset_clears_old_state():
    gate = ObservationGate(timeout=1.0)
    gate.begin(100, 0.0, 1_000_000_000)
    gate.reset()
    assert gate.last_receive is None and gate.last_stamp_ns is None
    token, error = gate.begin(100, 0.1, 1_000_000_000)
    assert token is not None and error is None
