# bev_pkg

C++ 激光 BEV 节点，输出与 `training/envs/navrl/observation.py` 完全一致的策略观测。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 按消息时间戳查询 TF |
| 订阅 | `/odom` | `nav_msgs/Odometry` | 速度特征，时间偏差超过 0.2 s 置零 |
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

当前 10 个固定样例全部通过，占用层、距离层与目标标量误差为 0。
