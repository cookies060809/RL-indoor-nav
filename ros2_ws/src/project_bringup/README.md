# project_bringup

演示与集成的启动包。

## PPO 闭环

```bash
cd ~/ROS2hw
./setup/run_rl_nav.sh
```

启动内容：Gazebo `wpb_simple.world`、`bev_node`、`policy_node`、`safety_node`、RViz。

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_id` | `split_mixed_pretrain_v1` | `models/` 下的模型目录 |
| `goal_x` / `goal_y` | 0.0 / 2.0 | 固定目标 |
| `goal_frame` | `odom` | 目标坐标系，接入 AMCL 后改为 `map` |
| `rviz` | `true` | 是否启动 RViz |
| `policy` | `true` | 是否启动策略节点 |
| `project_root` | `~/ROS2hw` | 项目根目录 |

```text
/scan → bev_node → /policy_observation → policy_node → /cmd_vel_rl
                                                         ↓
/scan → safety_node ←────────────────────────────────────┘
             ↓
          /cmd_vel → Gazebo 底盘
```

## PPO 居家地图（Nav2 规划 + 前视目标）

```bash
cd ~/ROS2hw
./setup/run_ppo_home.sh
```

启动内容：`robocup_home.world`、课程地图、`map_server`、AMCL、`planner_server`、`route_manager`、`local_goal_node`、BEV、策略、`safety_node`、RViz。AMCL 初始位姿 (-5.9, -0.5)：出发点在地图边界，向图内偏 0.1 m 以避免规划器判起点越界。

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_id` | `ppo_v2_recommended` | 模型目录；旧权重可显式选 `split_mixed_pretrain_v1` |
| `rviz` | `true` | 是否启动 RViz |
| `policy` | `true` | 是否启动策略节点 |
| `initial_x` / `initial_y` / `initial_yaw` | -5.9 / -0.5 / 0.0 | AMCL 初始位姿 |

发送最终目标（RViz `2D Goal Pose` 或命令行）：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"
```

```text
/goal_pose → route_manager → /compute_path_to_pose → /global_path
/global_path + TF → local_goal_node → /local_goal (1.2 m lookahead)
/local_goal → bev_node → policy_node → /cmd_vel_rl → safety_node → /cmd_vel
```

复位当前任务、旧路径和待执行推理：

```bash
ros2 topic pub --once /policy_reset std_msgs/msg/Empty '{}'
```

2026-11-15 Ubuntu 实测（`ppo_v2_recommended`）：(0,0) 23.1 s / 5.35 m、(2,3) 38.7 s / 9.33 m，reset 后 (4.5,-2.0) 39.4 s / 9.45 m，全部到点且无安全干预。若停车位置与地图障碍重叠，前视节点会沿规划路径给出 0.4 m 脱困前视点，离开后恢复正常可见前视。数据见 `experiments/ppo_v2/linux_home/`。

## Nav2 基线（居家地图）

```bash
cd ~/ROS2hw
./setup/run_nav2_home.sh
```

启动内容：`robocup_home.world`、课程包 `maps/map.yaml`、`map_server`、`amcl`、Nav2 各服务器、`safety_node`、RViz。AMCL 初始位姿固定为 (-6.0, -0.5, 0.0)。

| 参数 | 默认 | 说明 |
|---|---|---|
| `rviz` | `true` | 是否启动 RViz |
| `safety` | `true` | 是否启动 safety（关闭后 Nav2 直接发布 `/cmd_vel`，仅用于对照） |
| `initial_x` / `initial_y` / `initial_yaw` | -6.0 / -0.5 / 0.0 | AMCL 初始位姿 |

速度链路（方案 A）：

```text
controller_server / behavior_server → /cmd_vel_nav
        → velocity_smoother → /cmd_vel_nav_smoothed
        → safety_node → /cmd_vel
```

`/cmd_vel` 只有 `safety_node` 一个发布者。safety 仅允许前进，Nav2 的倒车恢复会被截断，原地旋转恢复仍可用。

## 录制视频

VMware 下 X11 抓屏为全黑，因此使用 Gazebo 俯视相机：

```bash
ros2 launch project_bringup demo_capture.launch.py \
  output:=$HOME/ROS2hw/experiments/videos/demo_raw.mp4
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `output` | `~/ROS2hw/experiments/videos/demo.mp4` | mp4 输出路径 |
| `topic` | `/camera/image_raw` | 图像话题 |
| `fps` | `5.0` | 录制帧率，应接近相机实际帧率 |
| `camera_x` / `camera_y` / `camera_z` | 0.0 / 0.0 / 6.5 | 俯视相机位置 |

相机约 5 Hz（关闭 `gzclient` 后），可用 ffmpeg 插值到 15 fps 并叠加文字。