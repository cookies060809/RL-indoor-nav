"""Runtime gates tested without rclpy, Gazebo, or wall-clock sleeps."""
from concurrent.futures import Future
import hashlib
import importlib.util
from pathlib import Path
import sys
import threading

import pytest

ROOT = Path(__file__).resolve().parents[3]
RUNTIME = ROOT / 'ros2_ws/src/rl_nav_pkg/rl_nav_pkg/watchdog.py'
spec = importlib.util.spec_from_file_location('policy_runtime_under_test', RUNTIME)
runtime = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runtime
spec.loader.exec_module(runtime)
ObservationGate = runtime.ObservationGate
NavigationGate = runtime.NavigationGate
FinalGoalHold = runtime.FinalGoalHold
LatestInference = runtime.LatestInference


def test_stamp_age_and_vm_clock_tolerance():
    gate = ObservationGate()
    token, error = gate.begin(1_050_000_000, 0.0, 1_000_000_000)
    assert token is not None and error is None  # Gazebo clock can lag scan by 50ms.
    assert gate.begin(1_200_000_000, .1, 1_000_000_000)[1] == 'stale_or_future_observation'
    assert gate.begin(1_000_000_000, .2, 10_000_000_000)[1] == 'stale_or_future_observation'


def test_reset_detection_precedes_duplicate_rejection():
    gate = ObservationGate()
    old_token, _ = gate.begin(10_000_000_000, 0.0, 10_000_000_000)
    assert gate.begin(100_000_000, .1, 100_000_000)[1] == 'sim_time_backwards'
    fresh_token, error = gate.begin(200_000_000, .2, 200_000_000)
    assert fresh_token is not None and error is None
    assert not gate.can_publish(old_token, 0.0, .2, 200_000_000)


def test_explicit_reset_requires_post_reset_observation():
    gate = ObservationGate()
    gate.begin(1_000_000_000, 0.0, 1_000_000_000)
    gate.reset(1_050_000_000)
    assert gate.begin(1_000_000_000, .1, 1_050_000_000)[1] == 'pre_reset_observation'
    assert gate.begin(1_100_000_000, .2, 1_100_000_000)[0] is not None


def test_explicit_reset_after_time_jump_drops_old_epoch_barrier():
    gate = ObservationGate()
    gate.begin(10_000_000_000, 0.0, 10_000_000_000)
    gate.reset(100_000_000)
    assert gate.begin(200_000_000, .2, 200_000_000)[0] is not None


def test_duplicate_old_and_stalled_sensor_are_not_fresh():
    gate = ObservationGate(timeout=1.0)
    gate.begin(1_000_000_000, 0.0, 1_000_000_000)
    assert gate.begin(1_000_000_000, .1, 1_000_000_000)[1] == 'duplicate_or_old_stamp'
    assert gate.begin(900_000_000, .2, 1_000_000_000)[1] == 'duplicate_or_old_stamp'
    assert not gate.watchdog_expired(.9)
    assert gate.watchdog_expired(1.1)


def test_inference_result_checks_completion_age_and_generation():
    gate = ObservationGate()
    token, _ = gate.begin(1_000_000_000, 0.0, 1_000_000_000)
    assert gate.can_publish(token, 0.0, .1, 1_100_000_000)
    assert not gate.can_publish(token, 0.0, .6, 1_100_000_000)
    assert not gate.can_publish(token, 0.0, .2, 1_700_000_000)
    gate.begin(1_200_000_000, .2, 1_200_000_000)
    assert not gate.can_publish(token, 0.0, .2, 1_200_000_000)


def test_watchdog_also_checks_command_publication_age():
    gate = ObservationGate(timeout=1.0)
    gate.begin(1_000_000_000, 0.0, 1_000_000_000)
    gate.mark_publish(0.0)
    gate.begin(1_900_000_000, .9, 1_900_000_000)
    assert gate.watchdog_expired(1.1)
    gate.invalidate()
    gate.begin(2_200_000_000, 1.2, 2_200_000_000)
    assert not gate.watchdog_expired(1.2)


def test_navigation_heartbeat_stops_for_hold_success_and_expiry():
    nav = NavigationGate(timeout=1.0)
    assert not nav.allows_motion(0.0)
    assert nav.update({'ready': True, 'reached': False, 'reason': 'tracking',
                       'final_distance': 2.0, 'goal_generation': 4}, 0.0)
    assert nav.allows_motion(.9)
    assert not nav.allows_motion(1.1)
    assert nav.update({'ready': False, 'reached': False, 'reason': 'goal_hold',
                       'final_distance': .2, 'goal_generation': 4}, 1.2)
    assert not nav.allows_motion(1.2) and not nav.reached
    assert nav.update({'ready': False, 'reached': True, 'reason': 'goal_reached',
                       'final_distance': .2, 'goal_generation': 4}, 2.2)
    assert not nav.allows_motion(2.2) and nav.reached


def test_navigation_invalid_status_accepts_null_distance_and_closes_gate():
    nav = NavigationGate()
    assert nav.update({'ready': False, 'reached': False, 'reason': 'empty_path',
                       'final_distance': None, 'goal_generation': 5}, 0.0)
    assert not nav.allows_motion(0.0)
    assert not nav.update({'ready': 1, 'reached': False}, .1)
    assert not nav.update({'ready': True, 'reached': False, 'final_distance': float('nan')}, .2)
    assert not nav.update({'ready': True, 'reached': False, 'goal_generation': '5'}, .3)


def test_final_arrival_parks_immediately_but_requires_continuous_hold():
    hold = FinalGoalHold(.3, 1.0)
    assert hold.update(.2, 1_000_000_000) == (True, False)
    assert hold.update(.2, 1_999_000_000) == (True, False)
    assert hold.update(.2, 2_000_000_000) == (True, True)
    assert hold.update(.4, 2_100_000_000) == (False, False)
    assert hold.update(.2, 2_200_000_000) == (True, False)


def test_final_hold_cannot_complete_by_clock_rollback():
    hold = FinalGoalHold(.3, 1.0)
    hold.update(.2, 10_000_000_000)
    assert hold.update(.2, 100_000_000) == (True, False)
    assert hold.update(.2, 1_100_000_000) == (True, True)


@pytest.mark.parametrize('kwargs', [{'timeout': 0}, {'max_inference_seconds': -1},
                                   {'future_tolerance': float('nan')}])
def test_invalid_runtime_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        ObservationGate(**kwargs)


class ManualExecutor:
    def __init__(self):
        self.calls = []

    def submit(self, function, value):
        future = Future()
        future.set_running_or_notify_cancel()
        self.calls.append((function, value, future))
        return future


def test_inference_queue_is_bounded_and_latest_input_replaces_old_input():
    executor = ManualExecutor()
    inference = LatestInference(lambda value: value, executor=executor)
    inference.offer(1, 'first')
    assert inference.start_latest(0.0, lambda token: True)
    for token in range(2, 100):
        inference.offer(token, f'frame-{token}')
        assert not inference.start_latest(.1, lambda token: True)
    assert len(executor.calls) == 1
    executor.calls[0][2].set_result('first-result')
    result = inference.poll()
    assert result.token == 1 and result.action == 'first-result'
    assert inference.start_latest(.2, lambda token: token == 99)
    assert len(executor.calls) == 2 and executor.calls[1][1] == 'frame-99'
    inference.close()


def test_invalidation_discards_pending_generation_without_queueing_more_work():
    executor = ManualExecutor()
    inference = LatestInference(lambda value: value, executor=executor)
    gate = ObservationGate()
    token, _ = gate.begin(1_000_000_000, 0., 1_000_000_000)
    inference.offer(token, 'frame')
    inference.start_latest(0.0, lambda current: current == gate.generation)
    gate.reset(1_100_000_000)
    inference.invalidate()
    assert inference.latest is None
    executor.calls[0][2].set_result('obsolete')
    result = inference.poll()
    assert not gate.can_publish(result.token, result.started, .2, 1_200_000_000)
    inference.close()


def test_worker_exceptions_are_reported_to_main_callback():
    executor = ManualExecutor()
    inference = LatestInference(lambda value: value, executor=executor)
    inference.offer(1, 'frame'); inference.start_latest(0., lambda token: True)
    executor.calls[0][2].set_exception(RuntimeError('bad inference'))
    result = inference.poll()
    assert isinstance(result.error, RuntimeError)
    assert inference.pending is None
    inference.close()


def test_inference_completion_only_notifies_main_dispatcher():
    executor = ManualExecutor()
    notifications = []
    inference = LatestInference(lambda value: value, executor=executor,
                                on_completed=lambda: notifications.append('wake'))
    inference.offer(1, 'frame'); inference.start_latest(0., lambda token: True)
    executor.calls[0][2].set_result('prediction')
    assert notifications == ['wake']
    # Completion notification did not consume or publish a result in the worker.
    assert inference.pending is not None
    assert inference.poll().action == 'prediction'
    inference.close()


def test_blocked_predict_does_not_block_main_watchdog():
    entered, release = threading.Event(), threading.Event()
    def predict(value):
        entered.set()
        if not release.wait(2.0):
            raise RuntimeError('test worker not released')
        return value
    inference = LatestInference(predict)
    try:
        gate = ObservationGate(timeout=1.0)
        token, _ = gate.begin(1_000_000_000, 0., 1_000_000_000)
        inference.offer(token, 'frame'); inference.start_latest(0., lambda token: True)
        assert entered.wait(1.0)
        assert inference.poll() is None
        assert inference.over_budget(.6, .5)
        assert gate.watchdog_expired(1.1)
        release.set()
        inference.pending[0].result(timeout=1.0)
        assert inference.poll().action == 'frame'
    finally:
        release.set(); inference.close()


def test_deployment_templates_match_ros_runtime_sources():
    for name in ('watchdog.py', 'policy_node.py'):
        ros = ROOT/'ros2_ws/src/rl_nav_pkg/rl_nav_pkg'/name
        template = ROOT/'training/scripts/deployment'/name
        assert hashlib.sha256(ros.read_bytes()).digest() == hashlib.sha256(template.read_bytes()).digest()
