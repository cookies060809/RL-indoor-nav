#!/usr/bin/env python3
"""Run one fresh PPO home-map episode per goal and record the outcome.

Each goal gets its own Gazebo launch so every episode starts from the world's
fixed spawn pose. Output matches the CSV columns used by the earlier
experiments/ppo_v2/linux_home runs.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLUMNS = ['wall', 'sim', 'x', 'y', 'nav_ready', 'nav_reached', 'nav_reason',
           'nav_distance', 'nav_generation', 'safety_estop', 'safety_reason',
           'cmd_v', 'cmd_w', 'rl_v', 'rl_w', 'plan_reason']


def load_status(text):
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return {}


class Recorder(Node):
    def __init__(self):
        super().__init__('eval_recorder', parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.odom = None
        self.odom_wall = 0.0
        self.scan_wall = 0.0
        self.have_map = False
        self.nav = {}
        self.safety = {}
        self.cmd = (0.0, 0.0)
        self.rl = (0.0, 0.0)
        self.plan = ''
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, '/navigation/status', self.on_nav, 10)
        self.create_subscription(String, '/safety/status', self.on_safety, 10)
        self.create_subscription(String, '/planning/status', self.on_plan, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_rl', self.on_rl, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.reset_pub = self.create_publisher(Empty, '/policy_reset', 10)

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.odom_wall = time.monotonic()

    def on_scan(self, _m):
        self.scan_wall = time.monotonic()

    def on_map(self, _m):
        self.have_map = True

    def on_nav(self, m):
        self.nav = load_status(m.data)

    def on_safety(self, m):
        self.safety = load_status(m.data)

    def on_plan(self, m):
        self.plan = m.data

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_rl(self, m):
        self.rl = (m.linear.x, m.angular.z)

    def fresh(self, timeout=1.5):
        now = time.monotonic()
        return (self.odom is not None and now - self.odom_wall < timeout
                and self.scan_wall and now - self.scan_wall < timeout)

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(msg)
            self.spin(0.5)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def stop_stack(proc):
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 15
    while proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    for pattern in ('gzserver', 'gzclient'):
        subprocess.run(['pkill', '-f', pattern], check=False)
    time.sleep(2.0)


def run_episode(node, goal, log_path, sim_timeout, wall_timeout, model_id=None):
    env = dict(os.environ)
    env.pop('DISPLAY', None)
    log = open(log_path, 'w', encoding='utf-8')
    command = ['bash', os.path.join(ROOT, 'setup', 'run_ppo_home.sh'), 'rviz:=false']
    if model_id:
        command.append(f'model_id:={model_id}')
    proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    rows = []
    result = {'goal_map': [goal[0], goal[1]], 'reached': False, 'reason': 'launch_failed',
              'sim_seconds': 0.0, 'path_length_m': 0.0, 'final_distance_m': None,
              'safety_interventions': 0, 'intervention_samples': 0,
              'escape_samples': 0, 'goal_hold_samples': 0, 'tracking_samples': 0}
    try:
        ready_deadline = time.monotonic() + 90
        while time.monotonic() < ready_deadline:
            node.spin(0.2)
            if node.fresh() and node.have_map:
                break
            if proc.poll() is not None:
                result['reason'] = 'stack_exited'
                return result
        else:
            result['reason'] = 'stack_not_ready'
            return result
        node.spin(4.0)
        node.reset_pub.publish(Empty())
        node.spin(1.0)
        for _ in range(10):
            if node.fresh():
                break
            node.spin(0.5)
        node.send_goal(*goal)

        start_wall = time.monotonic()
        start_sim = node.get_clock().now().nanoseconds / 1e9
        last = node.odom
        path = 0.0
        first_status = True
        while True:
            node.spin(0.1)
            now_wall = time.monotonic()
            now_sim = node.get_clock().now().nanoseconds / 1e9
            if node.odom and last:
                path += math.dist(node.odom, last)
            last = node.odom
            nav = node.nav
            status = nav.get('reason', '')
            if status:
                first_status = False
            if status == 'tracking_escape':
                result['escape_samples'] += 1
            elif status == 'goal_hold':
                result['goal_hold_samples'] += 1
            elif status == 'tracking':
                result['tracking_samples'] += 1
            if node.safety.get('reason', 'ok') != 'ok':
                result['intervention_samples'] += 1
            rows.append([
                f'{now_wall - start_wall:.3f}',
                f'{now_sim - start_sim:.3f}',
                '' if not node.odom else f'{node.odom[0]:.3f}',
                '' if not node.odom else f'{node.odom[1]:.3f}',
                str(nav.get('ready', '')).lower(),
                str(nav.get('reached', '')).lower(),
                status,
                '' if nav.get('final_distance') is None else f"{nav['final_distance']:.3f}",
                nav.get('goal_generation', ''),
                str(node.safety.get('estop', '')).lower(),
                node.safety.get('reason', ''),
                f'{node.cmd[0]:.3f}', f'{node.cmd[1]:.3f}',
                f'{node.rl[0]:.3f}', f'{node.rl[1]:.3f}',
                node.plan])
            if nav.get('reached'):
                result.update(reached=True, reason='goal_reached')
                break
            if now_sim - start_sim > sim_timeout or now_wall - start_wall > wall_timeout:
                result['reason'] = 'timeout' if not first_status else 'no_navigation_status'
                break
            if proc.poll() is not None:
                result['reason'] = 'stack_exited'
                break
        result['sim_seconds'] = round(now_sim - start_sim, 2)
        result['path_length_m'] = round(path, 2)
        result['final_distance_m'] = nav.get('final_distance')
        result['safety_interventions'] = int(node.safety.get('interventions', 0) or 0)
        return result
    finally:
        with open(log_path.replace('.log', '.csv'), 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(COLUMNS)
            writer.writerows(rows)
        stop_stack(proc)
        log.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goals', default='0,0;2,3;4.5,-2;-2,2;0,-2.5;-3.5,-2;3.5,3;2.5,-3')
    parser.add_argument('--out', default='experiments/ppo_v2/linux_home/goal_sweep')
    parser.add_argument('--sim-timeout', type=float, default=150.0)
    parser.add_argument('--wall-timeout', type=float, default=240.0)
    parser.add_argument('--tag', default='develop_b3a0')
    parser.add_argument('--model-id', default=None)
    args = parser.parse_args()
    goals = []
    for item in args.goals.split(';'):
        if item.strip():
            x, y = item.split(',')
            goals.append((float(x), float(y)))
    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Recorder()
    results = []
    try:
        for index, goal in enumerate(goals):
            stem = os.path.join(out, f'goal_{index:02d}_{goal[0]:+.1f}_{goal[1]:+.1f}')
            print(f'[{index + 1}/{len(goals)}] goal ({goal[0]}, {goal[1]})', flush=True)
            result = run_episode(node, goal, stem + '.log', args.sim_timeout, args.wall_timeout,
                                 args.model_id)
            result['id'] = os.path.basename(stem)
            result['csv'] = os.path.relpath(stem + '.csv', ROOT)
            results.append(result)
            print('   ->', json.dumps({k: result[k] for k in
                                       ('reached', 'reason', 'sim_seconds', 'path_length_m',
                                        'final_distance_m', 'safety_interventions')}), flush=True)
            with open(os.path.join(out, 'results.json'), 'w', encoding='utf-8') as handle:
                json.dump({'tag': args.tag, 'goals': results}, handle, indent=2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    reached = sum(1 for r in results if r['reached'])
    summary = {
        'tag': args.tag,
        'date': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'episodes': len(results),
        'reached': reached,
        'success_rate': round(reached / len(results), 4) if results else 0.0,
        'results': results,
    }
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps({k: summary[k] for k in ('episodes', 'reached', 'success_rate')}))


if __name__ == '__main__':
    main()
