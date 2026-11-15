"""Copy into ament_python package in VM. ROS imports intentionally stay here.

Requires project_msgs/PolicyObservation generated in the VM workspace.
Only publishes /cmd_vel_rl; safety_node must own /cmd_vel.

All watchdog and inference freshness decisions use a STEADY_TIME clock. A
paused simulation cannot keep a stale command alive, and a backwards clock jump
clears the gate.
"""
from pathlib import Path
import sys
import numpy as np
import torch
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from project_msgs.msg import PolicyObservation
from stable_baselines3 import PPO
import yaml
try:
    from .watchdog import ObservationGate
except ImportError:
    from watchdog import ObservationGate


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('wall_timeout', 1.0)
        self.declare_parameter('max_inference_seconds', 0.5)
        self.declare_parameter('reset_topic', '/policy_reset')
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
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel_rl', 10)
        self.create_subscription(PolicyObservation, '/policy_observation', self.on_observation, 10)
        self.create_subscription(Empty, self.get_parameter('reset_topic').value, self.on_reset, 10)
        # A simulation clock can pause; the watchdog timer must not.
        self.create_timer(0.05, self.watchdog, clock=self.steady_clock)

    def steady_seconds(self):
        return self.steady_clock.now().nanoseconds / 1e9

    def stop(self):
        self.publisher.publish(Twist())

    def on_reset(self, _message):
        self.gate.reset()
        self.stop()

    def watchdog(self):
        if self.gate.watchdog_expired(self.steady_seconds()):
            self.stop()

    def on_observation(self, message):
        size = self.cfg['observation']['size']
        extent = self.cfg['observation']['extent']
        stamp_ns = message.header.stamp.sec * 10 ** 9 + message.header.stamp.nanosec
        valid = (
            message.spec_version == self.cfg['spec_version']
            and message.height == size
            and message.width == size
            and abs(message.resolution - extent / size) < 1e-6
            and len(message.bev) == 2 * size * size
            and message.header.frame_id == 'base_footprint'
            and stamp_ns > 0
        )
        observation = np.asarray([*message.bev, *message.goal, *message.motion], dtype=np.float32)
        if not valid or not np.all(np.isfinite(observation)) or not self.model.observation_space.contains(observation):
            self.stop()
            return
        now_steady = self.steady_seconds()
        now_sim_ns = self.get_clock().now().nanoseconds
        token, error = self.gate.begin(stamp_ns, now_steady, now_sim_ns)
        if error is not None:
            self.stop()
            return
        inference_started = now_steady
        action, _ = self.model.predict(observation, deterministic=True)
        finished = self.steady_seconds()
        if not np.all(np.isfinite(action)) or not self.gate.can_publish(token, inference_started, finished):
            self.stop()
            return
        clipped = np.clip(action, -1, 1)
        robot = self.cfg['robot']
        command = Twist()
        command.linear.x = float((clipped[0] + 1) * 0.5 * robot['v_max'])
        command.angular.z = float(clipped[1] * robot['w_max'])
        self.publisher.publish(command)
        self.gate.mark_publish(finished)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
