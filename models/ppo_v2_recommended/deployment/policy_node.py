"""PPO inference with independent steady-time gating and final-goal semantics.

The worker only computes a prediction. ROS callbacks on the main executor
validate and publish it. Local carrots are never treated as terminal goals.
"""
from pathlib import Path
import json
import sys
import numpy as np
import torch
import yaml
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from project_msgs.msg import PolicyObservation
from stable_baselines3 import PPO
try:
    from .watchdog import ObservationGate, NavigationGate, FinalGoalHold, LatestInference
except ImportError:
    from watchdog import ObservationGate, NavigationGate, FinalGoalHold, LatestInference


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('wall_timeout', 1.0)
        self.declare_parameter('max_inference_seconds', 0.5)
        self.declare_parameter('max_observation_age', 0.5)
        self.declare_parameter('future_tolerance', 0.15)
        self.declare_parameter('reset_topic', '/policy_reset')
        self.declare_parameter('goal_mode', 'final')
        self.declare_parameter('goal_stop_radius', 0.3)
        self.declare_parameter('goal_hold_seconds', 1.0)
        self.declare_parameter('require_navigation_status', False)
        self.declare_parameter('navigation_status_topic', '/navigation/status')
        self.declare_parameter('navigation_timeout', 1.0)
        self.goal_mode = self.get_parameter('goal_mode').value
        if self.goal_mode not in ('final', 'local'):
            raise ValueError('goal_mode must be final or local')
        self.needs_navigation = bool(self.get_parameter('require_navigation_status').value) or self.goal_mode == 'local'
        path = Path(self.get_parameter('model_path').value)
        sys.path.insert(0, str(path.parent.resolve()))
        torch.set_num_threads(2)
        self.model = PPO.load(path, device='cpu')
        with (path.parent / 'config.yaml').open(encoding='utf-8') as handle:
            self.cfg = yaml.safe_load(handle)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.gate = ObservationGate(
            timeout=self.get_parameter('wall_timeout').value,
            max_inference_seconds=self.get_parameter('max_inference_seconds').value,
            max_observation_age=self.get_parameter('max_observation_age').value,
            future_tolerance=self.get_parameter('future_tolerance').value,
        )
        self.navigation = NavigationGate(timeout=self.get_parameter('navigation_timeout').value)
        self.arrival = FinalGoalHold(
            radius=self.get_parameter('goal_stop_radius').value,
            hold_seconds=self.get_parameter('goal_hold_seconds').value,
        )
        self.closing = False
        self.completion_guard = self.create_guard_condition(self.on_inference_ready)
        self.inference = LatestInference(
            lambda observation: self.model.predict(observation, deterministic=True)[0],
            on_completed=self.notify_inference_ready)
        self.goal_reached = False
        self.last_stop_reason = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel_rl', 10)
        self.create_subscription(PolicyObservation, '/policy_observation', self.on_observation, 10)
        self.create_subscription(Empty, self.get_parameter('reset_topic').value, self.on_reset, 10)
        self.create_subscription(String, self.get_parameter('navigation_status_topic').value, self.on_navigation, 10)
        self.create_timer(0.05, self.watchdog, clock=self.steady_clock)
        self.get_logger().info(
            f'policy ready: spec={self.cfg["spec_version"]}, goal_mode={self.goal_mode}, '
            f'navigation_gate={self.needs_navigation}')

    def steady_seconds(self):
        return self.steady_clock.now().nanoseconds / 1e9

    def notify_inference_ready(self):
        # Worker only wakes the executor; all ROS publication stays in callbacks.
        if not self.closing:
            try:
                self.completion_guard.trigger()
            except Exception:
                # Shutdown can destroy the guard between the check and trigger.
                pass

    def on_inference_ready(self):
        if not self.closing:
            self.watchdog()

    def stop(self, reason=None, invalidate=False, preserve_receive=False, warn=False):
        if invalidate:
            self.gate.invalidate(clear_receive=not preserve_receive)
            self.inference.invalidate()
        self.publisher.publish(Twist())
        if warn and reason != self.last_stop_reason:
            self.get_logger().warning(f'policy stopped: {reason}')
        self.last_stop_reason = reason

    def on_reset(self, _message):
        self.gate.reset(self.get_clock().now().nanoseconds)
        self.inference.invalidate()
        self.arrival.reset()
        self.goal_reached = False
        if self.needs_navigation:
            self.navigation.invalidate('policy_reset')
        self.stop('policy_reset')

    def on_navigation(self, message):
        if not self.needs_navigation:
            return
        now = self.steady_seconds()
        previous_generation = self.navigation.goal_generation
        previously_ready = self.navigation.allows_motion(now)
        try:
            status = json.loads(message.data)
        except (ValueError, TypeError):
            status = None
        if not self.navigation.update(status, now):
            self.stop('invalid_navigation_status', invalidate=True, warn=True)
            return
        generation_changed = self.navigation.goal_generation != previous_generation
        newly_ready = self.navigation.allows_motion(now) and not previously_ready
        if generation_changed or newly_ready:
            self.gate.reset(self.get_clock().now().nanoseconds)
            self.inference.invalidate()
            self.arrival.reset()
            self.goal_reached = False
        if not self.navigation.allows_motion(now):
            self.goal_reached = self.navigation.reached
            self.stop(self.navigation.reason, invalidate=True)

    def navigation_allows_motion(self, now):
        return not self.needs_navigation or self.navigation.allows_motion(now)

    def service_inference(self, now):
        result = self.inference.poll()
        now_sim = self.get_clock().now().nanoseconds
        if result is not None:
            if result.error is not None:
                self.stop(f'inference_error: {result.error}', invalidate=True, warn=True)
            elif (self.navigation_allows_motion(now)
                  and self.gate.can_publish(result.token, result.started, now, now_sim)):
                try:
                    action = np.asarray(result.action, dtype=float)
                    if action.shape != (2,) or not np.all(np.isfinite(action)):
                        raise ValueError('model returned an invalid action')
                    clipped = np.clip(action, -1, 1)
                    robot = self.cfg['robot']
                    command = Twist()
                    command.linear.x = float((clipped[0] + 1) * 0.5 * robot['v_max'])
                    command.angular.z = float(clipped[1] * robot['w_max'])
                    self.publisher.publish(command)
                    self.gate.mark_publish(now)
                    self.last_stop_reason = None
                except (TypeError, ValueError) as error:
                    self.stop(f'invalid_action: {error}', invalidate=True, warn=True)
            else:
                self.stop('obsolete_inference')
        if self.navigation_allows_motion(now):
            self.inference.start_latest(
                now, lambda token: self.gate.can_publish(token, now, now, now_sim))

    def watchdog(self):
        now = self.steady_seconds()
        if not self.navigation_allows_motion(now):
            self.stop(self.navigation.reason if self.navigation.is_fresh(now) else 'navigation_stale',
                      invalidate=True)
            return
        self.service_inference(now)
        if self.inference.over_budget(now, self.gate.max_inference_seconds):
            self.stop('inference_timeout', invalidate=True, warn=True)
        elif self.gate.watchdog_expired(now):
            self.stop('observation_or_command_timeout', invalidate=True)

    def on_observation(self, message):
        size = self.cfg['observation']['size']
        extent = self.cfg['observation']['extent']
        stamp_ns = message.header.stamp.sec * 10 ** 9 + message.header.stamp.nanosec
        valid = (
            message.spec_version == self.cfg['spec_version']
            and message.height == size and message.width == size
            and abs(message.resolution - extent / size) < 1e-6
            and len(message.bev) == 2 * size * size
            and message.header.frame_id == 'base_footprint'
        )
        observation = np.asarray([*message.bev, *message.goal, *message.motion], dtype=np.float32)
        if not valid or not np.all(np.isfinite(observation)) or not self.model.observation_space.contains(observation):
            self.stop('invalid_observation', invalidate=True, warn=True)
            return
        now = self.steady_seconds()
        now_sim = self.get_clock().now().nanoseconds
        token, error = self.gate.begin(stamp_ns, now, now_sim)
        if error is not None:
            self.arrival.reset()
            self.stop(error, invalidate=True, warn=True)
            return
        if not self.navigation_allows_motion(now):
            self.stop(self.navigation.reason if self.navigation.is_fresh(now) else 'navigation_stale',
                      invalidate=True)
            return
        if self.goal_mode == 'final':
            distance = float(message.goal[2]) * self.cfg['observation']['goal_distance_scale']
            holding, reached = self.arrival.update(distance, now_sim)
            if holding:
                if reached and not self.goal_reached:
                    self.get_logger().info(f'final goal held: {distance:.2f} m')
                self.goal_reached = reached
                self.stop('goal_reached' if reached else 'goal_hold',
                          invalidate=True, preserve_receive=True)
                return
        # Local mode uses only fresh navigation/status for arrival, not carrot distance.
        self.goal_reached = False
        self.inference.offer(token, observation.copy())
        self.service_inference(now)

    def close(self):
        self.closing = True
        self.gate.reset()
        self.inference.close()
        self.stop('shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
