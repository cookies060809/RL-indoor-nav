"""Publish a lookahead point on the global path as /local_goal."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros


class LocalGoalNode(Node):
    def __init__(self):
        super().__init__('local_goal_node')
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('publish_period', 0.1)

        self.publisher = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self.on_path, 10)
        self.create_timer(self.get_parameter('publish_period').value, self.tick)
        self.path = None
        self.arc = None
        self.index = 0
        self.get_logger().info(
            f'local_goal_node ready: lookahead={self.get_parameter("lookahead").value}')

    def on_path(self, msg):
        if not msg.poses:
            return
        self.path = msg
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        arc = [0.0]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            arc.append(arc[-1] + math.hypot(x1 - x0, y1 - y0))
        self.arc = arc
        self.index = 0
        self.get_logger().info(
            f'received path: {len(points)} poses, {arc[-1]:.2f} m')

    def tick(self):
        if self.path is None:
            return
        frame = self.path.header.frame_id or 'map'
        try:
            tf = self.buffer.lookup_transform(
                frame, self.get_parameter('robot_frame').value, Time(),
                Duration(seconds=0.1))
        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f'robot TF unavailable: {ex}')
            return
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        points = [(p.pose.position.x, p.pose.position.y) for p in self.path.poses]
        lo = max(0, self.index - 5)
        hi = min(len(points), self.index + 50)
        best = lo
        best_d = float('inf')
        for i in range(lo, hi):
            d = math.hypot(points[i][0] - rx, points[i][1] - ry)
            if d < best_d:
                best_d = d
                best = i
        self.index = best

        lookahead = float(self.get_parameter('lookahead').value)
        target = self.arc[best] + lookahead
        if target >= self.arc[-1]:
            j = len(points) - 1
            x, y = points[j]
            yaw = self._yaw(points[j - 1], points[j])
        else:
            j = best
            while j + 1 < len(self.arc) and self.arc[j] < target:
                j += 1
            segment = self.arc[j] - self.arc[j - 1]
            ratio = 0.0 if segment <= 1e-9 else (target - self.arc[j - 1]) / segment
            x = points[j - 1][0] + ratio * (points[j][0] - points[j - 1][0])
            y = points[j - 1][1] + ratio * (points[j][1] - points[j - 1][1])
            yaw = self._yaw(points[j - 1], points[j])

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = frame
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(out)

    @staticmethod
    def _yaw(p0, p1):
        return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def main(args=None):
    rclpy.init(args=args)
    node = LocalGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()