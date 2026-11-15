"""ROS-independent freshness gate for the policy node.

All safety timing uses a steady clock supplied by the caller. Simulation time is
only used to detect backwards jumps; a paused simulation cannot extend the
watchdog. A token invalidates an inference result if a newer observation or a
reset happened while the policy was running.
"""
import time


class ObservationGate:
    def __init__(self, timeout=1.0, max_inference_seconds=0.5, sim_jump_seconds=0.5):
        self.timeout = float(timeout)
        self.max_inference_seconds = float(max_inference_seconds)
        self.sim_jump_ns = int(sim_jump_seconds * 1e9)
        self.generation = 0
        self.last_receive = None
        self.last_stamp_ns = None
        self.last_sim_ns = None
        self.last_publish = None

    def reset(self):
        self.generation += 1
        self.last_receive = None
        self.last_stamp_ns = None
        self.last_sim_ns = None
        self.last_publish = None
        return self.generation

    def begin(self, stamp_ns, now_steady, now_sim_ns=None):
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return None, 'duplicate_or_old_stamp'
        if (self.last_sim_ns is not None and now_sim_ns is not None
                and now_sim_ns < self.last_sim_ns - self.sim_jump_ns):
            self.reset()
            return None, 'sim_time_backwards'
        if self.last_receive is not None and now_steady - self.last_receive > self.timeout:
            self.reset()
        self.last_receive = now_steady
        self.last_stamp_ns = stamp_ns
        self.last_sim_ns = now_sim_ns
        self.generation += 1
        return self.generation, None

    def can_publish(self, token, inference_started, now_steady):
        if token != self.generation:
            return False
        if self.last_receive is None:
            return False
        if now_steady - self.last_receive > self.timeout:
            return False
        if now_steady - inference_started > self.max_inference_seconds:
            return False
        return True

    def mark_publish(self, now_steady):
        self.last_publish = now_steady

    def watchdog_expired(self, now_steady=None):
        now_steady = time.monotonic() if now_steady is None else now_steady
        return self.last_receive is None or now_steady - self.last_receive > self.timeout
