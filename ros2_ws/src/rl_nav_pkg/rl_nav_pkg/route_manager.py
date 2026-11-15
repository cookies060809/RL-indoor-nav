"""Request a global path from the Nav2 planner and publish it on /global_path."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class RouteManager(Node):
    def __init__(self):
        super().__init__('route_manager')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('server_timeout', 2.0)

        self.publisher = self.create_publisher(Path, self.get_parameter('path_topic').value, 10)
        self.client = ActionClient(
            self, ComputePathToPose, self.get_parameter('planner_action').value)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self.on_goal, 10)
        self.goal_handle = None
        self.busy = False
        self.get_logger().info(
            f'route_manager ready: goal={self.get_parameter("goal_topic").value} '
            f'action={self.get_parameter("planner_action").value}')

    def on_goal(self, msg):
        if self.busy:
            self.get_logger().warning('planner busy, ignoring new goal')
            return
        timeout = float(self.get_parameter('server_timeout').value)
        if not self.client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('planner action server unavailable')
            return
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.use_start = False
        self.busy = True
        self.get_logger().info(
            f'planning to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) '
            f'in {msg.header.frame_id or "map"}')
        self.client.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('planning goal rejected')
            self.busy = False
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.busy = False
        self.goal_handle = None
        result = future.result().result
        path = result.path
        if not path.poses:
            self.get_logger().warning('planner returned an empty path')
            return
        self.publisher.publish(path)
        self.get_logger().info(
            f'published path with {len(path.poses)} poses in {path.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()