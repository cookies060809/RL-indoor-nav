"""Latest-goal planning; failures explicitly invalidate the old path."""
import copy
import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import Empty, String
from .path_tracking import GoalEpoch


class RouteManager(Node):
    def __init__(self):
        super().__init__('route_manager')
        for name, value in [('planner_action', '/compute_path_to_pose'), ('goal_topic', '/goal_pose'),
                            ('path_topic', '/global_path'), ('server_timeout', 5.0),
                            ('planning_timeout', 10.0), ('replan_period', 2.0)]:
            self.declare_parameter(name, value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Path, self.get_parameter('path_topic').value, qos)
        self.status_pub = self.create_publisher(String, '/planning/status', 10)
        self.client = ActionClient(self, ComputePathToPose, self.get_parameter('planner_action').value)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self.on_goal, 10)
        self.create_subscription(Empty, '/policy_reset', self.on_reset, 10)
        self.epoch = GoalEpoch()
        self.pending = self.inflight = self.handle = self.active_goal = None
        self.started = self.last_plan = 0.0
        self.last_sim = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.steady_clock)
        self.clear_path('waiting_for_goal')

    def report(self, reason):
        self.status_pub.publish(String(data=json.dumps({'reason': reason, 'goal_generation': self.epoch.current})))

    def clear_path(self, reason):
        message = Path()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)
        self.report(reason)

    def on_reset(self, _message):
        self.epoch.replace()
        self.pending = self.active_goal = None
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.handle = self.inflight = None
        self.clear_path('reset')

    def on_goal(self, message):
        generation = self.epoch.replace()
        self.clear_path('new_goal')
        point = message.pose.position
        if not all(math.isfinite(x) for x in (point.x, point.y, point.z)):
            self.active_goal = self.pending = None
            self.report('invalid_goal')
            return
        self.active_goal = copy.deepcopy(message)
        if not self.active_goal.header.frame_id:
            self.active_goal.header.frame_id = 'map'
        self.pending = (generation, self.active_goal, time.monotonic())
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.try_plan()

    def try_plan(self):
        if self.pending is None or self.inflight is not None:
            return
        generation, message, queued_at = self.pending
        if not self.client.server_is_ready():
            if time.monotonic() - queued_at > self.get_parameter('server_timeout').value:
                self.pending = self.active_goal = None
                self.clear_path('planner_unavailable')
            return
        self.pending = None
        self.inflight, self.started = generation, time.monotonic()
        goal = ComputePathToPose.Goal()
        goal.goal, goal.use_start = message, False
        self.client.send_goal_async(goal).add_done_callback(lambda future: self.on_accepted(future, generation))

    def on_accepted(self, future, generation):
        try:
            handle = future.result()
        except Exception as error:
            self.finish_failure(generation, 'planner_exception: ' + str(error))
            return
        if not self.epoch.accepts(generation):
            if handle.accepted:
                handle.cancel_goal_async()
            if self.inflight == generation:
                self.inflight = None
            self.try_plan()
            return
        if not handle.accepted:
            self.finish_failure(generation, 'planner_rejected')
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.on_result(result, generation))

    def finish_failure(self, generation, reason):
        if self.inflight == generation:
            self.inflight = self.handle = None
        if self.epoch.accepts(generation):
            self.active_goal = None
            self.clear_path(reason)
        self.try_plan()

    def on_result(self, future, generation):
        if not self.epoch.accepts(generation):
            if self.inflight == generation:
                self.inflight = self.handle = None
            self.try_plan()
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
                self.finish_failure(generation, 'planner_failed_or_empty')
                return
        except Exception as error:
            self.finish_failure(generation, 'planner_exception: ' + str(error))
            return
        self.inflight = self.handle = None
        self.last_plan = time.monotonic()
        path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(path)
        self.report('path_ready')
        self.try_plan()

    def tick(self):
        now, sim = time.monotonic(), self.get_clock().now().nanoseconds
        if self.last_sim is not None and sim < self.last_sim - 500_000_000:
            self.on_reset(None)
        self.last_sim = sim
        if self.inflight is not None and now - self.started > self.get_parameter('planning_timeout').value:
            self.on_reset(None)
            self.report('planning_timeout')
        if (self.active_goal is not None and self.pending is None and self.inflight is None
                and now - self.last_plan >= self.get_parameter('replan_period').value):
            self.pending = (self.epoch.current, self.active_goal, now)
        self.try_plan()


def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    finally:
        node.on_reset(None)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
