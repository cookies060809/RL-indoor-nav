# safety_pkg

C++ 安全节点，是 `/cmd_vel` 的唯一发布者。

## 行为

- 20 Hz 输出，限幅 `v_max=0.26`、`w_max=1.0`，按 `acceleration`/`angular_acceleration` 做速率限制。
- 候选命令超过 `stale_timeout`（0.5 s）或激光超过 `sensor_timeout`（0.5 s）时停车。
- 用激光点在 `base_footprint` 下的圆形足迹（半径 + `safety_margin`）检查间距；按制动距离积分预判碰撞，必要时只保留角速度。
- `/estop_request` 为 true 时锁存；只有收到 `/safety/reset` 且条件正常才解除。

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `candidate_topic`（默认 `/cmd_vel_rl`） | `geometry_msgs/Twist` |
| 订阅 | `/scan` | `sensor_msgs/LaserScan` |
| 订阅 | `/estop_request` | `std_msgs/Bool` |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` |
| 发布 | `/estop_state` | `std_msgs/Bool` |
| 发布 | `/safety/status` | `std_msgs/String`（JSON 状态与干预计数） |
| 服务 | `/safety/reset` | `std_srvs/Trigger` |

## 已验证

在 `wpb_simple.world` 中直行冲向 x=3 的书架：x=2.32 m 处 `/safety/status` 报告 `reason=clearance`，`/cmd_vel` 归零；停止发布候选速度 0.5 s 后报告 `command_stale`。
