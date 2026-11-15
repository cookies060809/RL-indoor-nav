"""ROS-independent gates and bounded inference scheduling for PPO deployment.

Receive/inference/status expiry uses caller-supplied monotonic time. Simulation
time is used only for sensor age, reset detection and final-goal hold duration.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import time


class ObservationGate:
    def __init__(self, timeout=1.0, max_inference_seconds=0.5,
                 sim_jump_seconds=0.5, max_observation_age=0.5,
                 future_tolerance=0.15):
        for name, value in [('timeout', timeout),
                            ('max_inference_seconds', max_inference_seconds),
                            ('sim_jump_seconds', sim_jump_seconds),
                            ('max_observation_age', max_observation_age)]:
            if not math.isfinite(float(value)) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not math.isfinite(float(future_tolerance)) or future_tolerance < 0:
            raise ValueError('future_tolerance must be non-negative and finite')
        self.timeout = float(timeout)
        self.max_inference_seconds = float(max_inference_seconds)
        self.sim_jump_ns = int(sim_jump_seconds * 1e9)
        self.max_observation_age = float(max_observation_age)
        self.future_tolerance = float(future_tolerance)
        self.generation = 0
        self.last_receive = None
        self.last_stamp_ns = None
        self.last_sim_ns = None
        self.last_publish = None
        self.after_stamp_ns = None

    def invalidate(self, clear_receive=True):
        self.generation += 1
        self.last_publish = None
        if clear_receive:
            self.last_receive = None

    def reset(self, now_sim_ns=None):
        previous_stamp = self.last_stamp_ns
        backwards = (now_sim_ns is not None and self.last_sim_ns is not None
                     and now_sim_ns < self.last_sim_ns - self.sim_jump_ns)
        self.invalidate()
        self.last_stamp_ns = None
        self.last_sim_ns = now_sim_ns
        if now_sim_ns is None:
            self.after_stamp_ns = None
        else:
            # A new time epoch must not retain a high stamp from the old epoch.
            self.after_stamp_ns = int(now_sim_ns) if backwards else max(
                int(now_sim_ns), previous_stamp or 0)
        return self.generation

    def stamp_is_fresh(self, stamp_ns, now_sim_ns):
        if now_sim_ns is None:
            return True
        age = (int(now_sim_ns) - int(stamp_ns)) / 1e9
        return -self.future_tolerance <= age <= self.max_observation_age

    def begin(self, stamp_ns, now_steady, now_sim_ns=None):
        if not math.isfinite(float(now_steady)):
            return None, 'invalid_steady_time'
        # Reset detection MUST precede the old/duplicate stamp check.
        if (self.last_sim_ns is not None and now_sim_ns is not None
                and now_sim_ns < self.last_sim_ns - self.sim_jump_ns):
            self.reset(now_sim_ns)
            return None, 'sim_time_backwards'
        self.last_sim_ns = now_sim_ns
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            return None, 'invalid_stamp'
        if not self.stamp_is_fresh(stamp_ns, now_sim_ns):
            return None, 'stale_or_future_observation'
        if self.after_stamp_ns is not None and stamp_ns <= self.after_stamp_ns:
            return None, 'pre_reset_observation'
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return None, 'duplicate_or_old_stamp'
        if self.last_receive is not None and now_steady - self.last_receive > self.timeout:
            self.invalidate()
        self.last_receive = float(now_steady)
        self.last_stamp_ns = stamp_ns
        self.generation += 1
        return self.generation, None

    def can_publish(self, token, inference_started, now_steady, now_sim_ns=None):
        if token != self.generation or self.last_receive is None:
            return False
        if now_steady < inference_started or now_steady < self.last_receive:
            return False
        if now_steady - self.last_receive > self.timeout:
            return False
        if now_steady - inference_started > self.max_inference_seconds:
            return False
        return self.stamp_is_fresh(self.last_stamp_ns, now_sim_ns)

    def mark_publish(self, now_steady):
        self.last_publish = float(now_steady)

    def watchdog_expired(self, now_steady=None):
        now_steady = time.monotonic() if now_steady is None else now_steady
        if self.last_receive is None or now_steady - self.last_receive > self.timeout:
            return True
        return (self.last_publish is not None
                and now_steady - self.last_publish > self.timeout)


class NavigationGate:
    def __init__(self, timeout=1.0):
        if not math.isfinite(float(timeout)) or timeout <= 0:
            raise ValueError('navigation_timeout must be positive and finite')
        self.timeout = float(timeout)
        self.last_receive = None
        self.ready = False
        self.reached = False
        self.reason = 'navigation_unavailable'
        self.goal_generation = None

    def invalidate(self, reason='navigation_invalid'):
        self.last_receive = None
        self.ready = False
        self.reached = False
        self.reason = reason

    def update(self, status, now_steady):
        if (not isinstance(status, dict)
                or type(status.get('ready')) is not bool
                or type(status.get('reached')) is not bool):
            self.invalidate()
            return False
        distance = status.get('final_distance')
        if distance is not None and (isinstance(distance, bool)
                                     or not isinstance(distance, (int, float))
                                     or not math.isfinite(distance) or distance < 0):
            self.invalidate()
            return False
        generation = status.get('goal_generation')
        if generation is not None and (type(generation) is not int or generation < 0):
            self.invalidate()
            return False
        self.ready = status['ready']
        self.reached = status['reached']
        self.reason = str(status.get('reason', 'navigation_status'))
        self.goal_generation = generation
        self.last_receive = float(now_steady)
        return True

    def is_fresh(self, now_steady):
        return (self.last_receive is not None
                and 0 <= now_steady - self.last_receive <= self.timeout)

    def allows_motion(self, now_steady):
        return self.is_fresh(now_steady) and self.ready and not self.reached


class FinalGoalHold:
    def __init__(self, radius=0.3, hold_seconds=1.0):
        if not math.isfinite(float(radius)) or not math.isfinite(float(hold_seconds)) or radius < 0 or hold_seconds < 0:
            raise ValueError('goal hold parameters must be finite and non-negative')
        self.radius = float(radius)
        self.hold_ns = int(hold_seconds * 1e9)
        self.entered_ns = None

    def reset(self):
        self.entered_ns = None

    def update(self, distance, now_sim_ns):
        if self.radius <= 0 or not math.isfinite(distance) or distance > self.radius:
            self.reset()
            return False, False
        if self.entered_ns is None or now_sim_ns < self.entered_ns:
            self.entered_ns = now_sim_ns
        return True, now_sim_ns - self.entered_ns >= self.hold_ns


@dataclass
class InferenceResult:
    token: int
    started: float
    action: object = None
    error: object = None


class LatestInference:
    """One running prediction plus one replaceable latest input; never a queue."""
    def __init__(self, predict, executor=None, on_completed=None):
        self.predict = predict
        self.executor = executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix='ppo_inference')
        self.owns_executor = executor is None
        self.latest = None
        self.pending = None
        self.closed = False
        self.on_completed = on_completed

    def offer(self, token, observation):
        if not self.closed:
            self.latest = (token, observation)

    def start_latest(self, now_steady, token_is_valid):
        if self.closed or self.pending is not None or self.latest is None:
            return False
        token, observation = self.latest
        self.latest = None
        if not token_is_valid(token):
            return False
        future = self.executor.submit(self.predict, observation)
        self.pending = (future, token, float(now_steady))
        if self.on_completed is not None:
            future.add_done_callback(lambda _future: self.on_completed())
        return True

    def poll(self):
        if self.pending is None or not self.pending[0].done():
            return None
        future, token, started = self.pending
        self.pending = None
        try:
            return InferenceResult(token=token, started=started, action=future.result())
        except Exception as error:
            return InferenceResult(token=token, started=started, error=error)

    def over_budget(self, now_steady, budget):
        return self.pending is not None and now_steady - self.pending[2] > budget

    def invalidate(self):
        self.latest = None
        if self.pending is not None and self.pending[0].cancel():
            self.pending = None

    def close(self):
        self.closed = True
        self.invalidate()
        if self.owns_executor:
            self.executor.shutdown(wait=False, cancel_futures=True)
