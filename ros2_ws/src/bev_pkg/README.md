# bev_pkg

C++ 激光 BEV 节点，按模型配置选择 v1/v2 观测语义；v2 与 `training/envs/navrl/observation.py` 对齐，旧模型沿用其冻结的 v1 语义。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 按消息时间戳查询 TF |
| 订阅 | `/odom` | `nav_msgs/Odometry` | 时间偏差超过 0.2 s，v1 置零，v2 停发观测 |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 上一周期实际执行速度 |
| 订阅 | `/local_goal` | `geometry_msgs/PoseStamped` | 可选，优先于固定参数目标 |
| 发布 | `/policy_observation` | `project_msgs/PolicyObservation` | BEV 2×N×N + 目标 4 + 运动 4 |
| 发布 | `/bev_grid` | `nav_msgs/OccupancyGrid` | 0 空闲、100 占用、-1 未知 |

## 关键参数

`n`（32/48）、`extent`（4.8）、`beams`（360）、`robot_radius`（0.226）、`distance_clip`（1.0）、`range_min`/`range_max`（0.24/10.0）、`goal_frame`、`goal_x`、`goal_y`、`spec_version`（bev_nav_v1）。

## 一致性验收

```bash
python3 ros2_ws/src/bev_pkg/test/export_fixtures.py training/tests/fixtures /tmp/bev_fixtures
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run bev_pkg bev_fixture_check /tmp/bev_fixtures
```

Windows 原生 C++ 检查通过旧 v1 的 10 个和发布包 v2 的 11 个固定样例，占用层、距离层与目标标量误差均为 0。新版用 `models/ppo_v2_recommended/fixtures` 替换上述样例目录；完整 ROS 节点仍需 Ubuntu 编译与闭环验收。

v2 默认要求新鲜 odom 和实际 `/cmd_vel` 反馈。居家地图模式设置 `require_local_goal=true`，目标超时 1 秒停止生成观测。雷达范围使用模型与 LaserScan 合法范围的交集；仅正无穷表示无回波，NaN、负无穷、过近及超限有限值保持未知。
