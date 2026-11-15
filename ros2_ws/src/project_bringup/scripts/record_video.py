#!/usr/bin/env python3
"""Record a ROS image topic to an mp4 file with OpenCV."""

import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--output', required=True)
    parser.add_argument('--fps', type=float, default=5.0)
    parser.add_argument('--max-seconds', type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    parent = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(parent, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('demo_video_recorder')
    writer = None
    frames = 0

    def on_image(msg):
        nonlocal writer, frames
        if msg.encoding not in ('rgb8', 'bgr8'):
            node.get_logger().warning(f'unsupported encoding {msg.encoding}')
            return
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else array.copy()
        if writer is None:
            writer = cv2.VideoWriter(
                args.output, cv2.VideoWriter_fourcc(*'mp4v'), args.fps,
                (msg.width, msg.height))
            if not writer.isOpened():
                node.get_logger().error(f'cannot open {args.output}')
                return
        writer.write(frame)
        frames += 1
        if frames % 25 == 0:
            node.get_logger().info(f'recorded {frames} frames')

    node.create_subscription(Image, args.topic, on_image, 5)
    node.get_logger().info(f'recording {args.topic} to {args.output}')

    if args.max_seconds > 0.0:
        node.create_timer(args.max_seconds, rclpy.shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if writer is not None:
            writer.release()
        node.get_logger().info(f'saved {frames} frames to {args.output}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())