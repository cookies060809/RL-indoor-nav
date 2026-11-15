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