"""Copy into ament_python package in VM. ROS imports intentionally stay here.

Requires project_msgs/PolicyObservation generated in the VM workspace.
Only publishes /cmd_vel_rl; safety_node must own /cmd_vel.
"""
from pathlib import Path
import sys
import time
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from project_msgs.msg import PolicyObservation
from stable_baselines3 import PPO
import yaml

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        self.declare_parameter('model_path','')
        self.declare_parameter('wall_timeout',1.0)
        self.declare_parameter('goal_stop_radius',0.3)
        path=Path(self.get_parameter('model_path').value)
        sys.path.insert(0,str(path.parent.resolve()))
        torch.set_num_threads(2)
        self.model=PPO.load(path,device='cpu')
        with (path.parent/'config.yaml').open(encoding='utf-8') as f: self.cfg=yaml.safe_load(f)
        self.last_wall=None
        self.goal_reached=False
        self.publisher=self.create_publisher(Twist,'/cmd_vel_rl',10)
        self.create_subscription(PolicyObservation,'/policy_observation',self.on_observation,10)
        self.create_timer(0.05,self.watchdog)

    def stop(self): self.publisher.publish(Twist())

    def watchdog(self):
        if self.last_wall is None or time.monotonic()-self.last_wall>self.get_parameter('wall_timeout').value:
            self.stop()

    def on_observation(self,msg):
        n=self.cfg['observation']['size']; extent=self.cfg['observation']['extent']
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        valid=(msg.spec_version==self.cfg['spec_version'] and msg.height==n and msg.width==n
               and abs(msg.resolution-extent/n)<1e-6 and len(msg.bev)==2*n*n
               and msg.header.frame_id=='base_footprint' and -0.15<=age<=0.5)
        obs=np.asarray([*msg.bev,*msg.goal,*msg.motion],dtype=np.float32)
        if not valid or not np.all(np.isfinite(obs)) or not self.model.observation_space.contains(obs):
            self.stop(); return
        stop_radius=self.get_parameter('goal_stop_radius').value
        distance=float(msg.goal[2])*self.cfg['observation']['goal_distance_scale']
        if stop_radius>0.0 and distance<=stop_radius:
            if not self.goal_reached:
                self.get_logger().info(f'goal reached: {distance:.2f} m, holding stop')
                self.goal_reached=True
            self.stop(); self.last_wall=time.monotonic(); return
        self.goal_reached=False
        action,_=self.model.predict(obs,deterministic=True)
        if not np.all(np.isfinite(action)): self.stop(); return
        a=np.clip(action,-1,1); r=self.cfg['robot']; out=Twist()
        out.linear.x=float((a[0]+1)*0.5*r['v_max']); out.angular.z=float(a[1]*r['w_max'])
        self.publisher.publish(out); self.last_wall=time.monotonic()

def main(args=None):
    rclpy.init(args=args); node=PolicyNode()
    try: rclpy.spin(node)
    finally: node.stop(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
