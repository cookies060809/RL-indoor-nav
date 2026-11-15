"""Path carrots plus separate final-goal and navigation-validity heartbeat."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Empty, String
import tf2_ros
from .path_tracking import PathTracker


class LocalGoalNode(Node):
    def __init__(self):
        super().__init__('local_goal_node')
        for name, value in [('lookahead', 1.2), ('path_topic', '/global_path'), ('robot_frame', 'base_footprint'),
                            ('publish_period', 0.1), ('subgoal_mode', 'fixed'), ('robot_radius', 0.226),
                            ('visibility_margin', 0.04), ('goal_radius', 0.3), ('goal_hold_seconds', 1.0),
                            ('path_timeout', 5.0)]:
            self.declare_parameter(name, value)
        self.tracker = PathTracker(
            lookahead=self.get_parameter('lookahead').value, radius=self.get_parameter('robot_radius').value,
            margin=self.get_parameter('visibility_margin').value, mode=self.get_parameter('subgoal_mode').value,
            goal_radius=self.get_parameter('goal_radius').value, hold_seconds=self.get_parameter('goal_hold_seconds').value,
            path_timeout=self.get_parameter('path_timeout').value)
        self.publisher = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, self.get_parameter('path_topic').value, self.on_path, qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(Empty, '/policy_reset', self.on_reset, 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.get_parameter('publish_period').value, self.tick, clock=self.steady_clock)

    def publish_status(self, status):
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def on_reset(self, _message):
        self.tracker.clear()
        self.publish_status(self.tracker.invalid_status('reset'))

    def on_path(self, message):
        if not message.poses:
            self.tracker.clear()
            self.publish_status(self.tracker.invalid_status('no_path'))
            return
        now = self.get_clock().now().nanoseconds
        stamp = message.header.stamp.sec * 10 ** 9 + message.header.stamp.nanosec
        if message.header.frame_id != 'map' or stamp > now + 150_000_000 or now - stamp > 1_000_000_000:
            self.tracker.clear()
            self.publish_status(self.tracker.invalid_status('invalid_path_frame_or_stamp'))
            return
        self.tracker.set_path([[p.pose.position.x, p.pose.position.y] for p in message.poses], time.monotonic())

    def on_map(self, message):
        if message.header.frame_id != 'map':
            self.tracker.field = None
            return
        q = message.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            self.tracker.set_map(np.asarray(message.data).reshape(message.info.height, message.info.width),
                                 [message.info.origin.position.x, message.info.origin.position.y], message.info.resolution, yaw)
        except ValueError:
            self.tracker.field = None

    def tick(self):
        now_sim = self.get_clock().now().nanoseconds / 1e9
        try:
            tf = self.buffer.lookup_transform('map', self.get_parameter('robot_frame').value, Time(), timeout=Duration(seconds=0.02))
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            if not -0.15 <= now_sim - stamp <= 0.5:
                raise ValueError('stale TF')
            goal, status = self.tracker.update([tf.transform.translation.x, tf.transform.translation.y], now_sim, time.monotonic())
        except (tf2_ros.TransformException, ValueError):
            self.tracker.within_since = None
            self.publish_status(self.tracker.invalid_status('pose_unavailable'))
            return
        if goal is not None:
            message = PoseStamped()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y = float(goal[0]), float(goal[1])
            message.pose.orientation.w = 1.0
            self.publisher.publish(message)
        self.publish_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = LocalGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.on_reset(None)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
