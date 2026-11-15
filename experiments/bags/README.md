# Gazebo rosbag 记录

2026-11-15 在 Ubuntu 虚拟机中用 PPO 闭环演示录制的两段 bag，作为中期审查的原始数据。两段都包含 `/scan`、`/odom`、`/cmd_vel`、`/cmd_vel_rl`、`/policy_observation`、`/bev_grid`、`/safety/status`、`/tf`、`/tf_static`、`/clock`。

| 目录 | 时长 | 消息数 | 内容 |
|---|---|---|---|
| `midterm_rl_demo` | 29.45 s | 3722 | 策略到达 (0,2) 后的停止过程与安全状态 |
| `midterm_rl_motion` | 27.54 s | 3521 | 发布 `/local_goal` (0,-1.5) 后的实际运动段，y 从约 1.78 到 -1.28 |

回放：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play experiments/bags/midterm_rl_motion
```

查看：

```bash
ros2 bag info experiments/bags/midterm_rl_motion
```

重新录制：

```bash
ros2 bag record -o experiments/bags/<name> \
  /scan /odom /cmd_vel /cmd_vel_rl /policy_observation /bev_grid /safety/status /tf /tf_static
```

bag 中的 `/cmd_vel` 由 `safety_node` 发布，`/cmd_vel_rl` 是策略候选速度。