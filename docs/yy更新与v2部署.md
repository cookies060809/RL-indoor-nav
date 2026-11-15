# yy 更新、重训结果与虚拟机启动

交付日期：2026-11-15。默认居家地图模型：`models/ppo_v2_recommended`。

## 这次 yy 比刚才多了什么

上一轮查看的是 `9849ce9084d07fc05bb901edf4a35ef8a4c9cc9b`；本轮 yy 是 `d98da4b5dec218533bd455e9b3f65ef1be508cec`。d98 的两个父提交正是 9849 和 main 的 `e020a96`，主要把 main 的 C++ BEV、安全节点、Nav2/PPO 启动、全局路径和演示数据合回 yy，并增加 `goal_stop_radius`。这次没有新的网络架构或奖励函数。

main 接收了 yy 的 v2 扫描分类、方向性未知区保护、卡住判定、配置校验、分场景评估和隔离导出。集成时另做了以下修正：

- C++ BEV 支持 v1/v2，并为 v2 安全核增加与 Python 对照的可执行检查。模型配置决定观测版本、BEV 大小、速度与安全参数。
- 仿真时间回退先于重复时间戳检查，独立检查消息绝对年龄，防止 reset 后旧时间戳锁死或旧消息被接收。
- 推理在单个工作线程执行，最多保留一个运行任务和一个最新待处理观测；主线程用稳态时钟检查超时，过期推理不能恢复运动。
- 局部前视点不作为最终到点依据。最终目标半径内立即停车，仿真时间连续保持 1 秒后报告完成；路径、TF 或导航心跳失效时停车。
- 新目标使旧路径失效，旧规划结果按任务代次丢弃；规划失败、超时和时钟复位有明确状态。路径定期重规划，默认 2 秒。
- 加入基于已有地图和机器人圆形足迹的可见前视目标，避免前视点跨过墙角。地图未知单元视为阻挡。训练使用矩形几何，ROS 使用栅格距离场，后者存在地图分辨率带来的保守间隙。
- 导出只使用训练时的环境快照，分数与权重哈希绑定，不把 Windows Python/Numba 缓存放进跨平台哈希清单。

原来的三个 v1 模型包保持原样，仍可显式选择回退。

## 训练与新测试结果

三次迁移训练从同一个旧版混合模型初始化，优化器和步数归零，各训练 100,352 步；另外从零训练 200,704 步作对照。迁移的三个 seed 不能写成三个独立从零训练实验。

每次验证使用固定的四类场景，各 8 个任务，共 32 个。按成功率、低碰撞率、低干预时长、平均奖励依次选优。在新测试前已写入不可覆盖的 `experiments/ppo_v2/selection.json`。

| 候选 | 验证成功 | 选中步数 | 平均安全干预秒/任务 |
|---|---:|---:|---:|
| 迁移 seed 42 | 31/32 | 20,000 | 0.359375 |
| 迁移 seed 43（发布） | 31/32 | 40,000 | 0.343750 |
| 迁移 seed 44 | 31/32 | 0 | 0.362500 |
| 从零 seed 42 | 31/32 | 200,000 | 0.359375 |

seed 44 保留初始化检查点，说明训练并非每次都能改善策略。最终发布的是 seed 43 的 best，完整训练终点的 final 也保留。发布权重 SHA-256：`aefe0c94a1480d9ed5ebcff3689aae4b311bc2c8c6efc240db36baef53c29fc5`。

新的测试种子基数为 610000，四场景分别偏移 0/1000/2000/3000，每场景 24 个，共 96 个。测试后没有换模型或再次调参。

| 完整运行条件 | 成功 | 碰撞 | 平均安全干预秒/任务 |
|---|---:|---:|---:|
| 旧 main 的 v1 环境、固定前视、旧权重 | 82/96 | 0/96 | 1.455208 |
| v2 环境、可见前视、未微调的旧权重 | 95/96 | 0/96 | 0.251042 |
| v2 环境、可见前视、发布权重 | 95/96 | 0/96 | 0.141667 |

成功率提升主要来自前视目标与运行语义的改进；本次微调没有进一步提升成功数，安全干预时长下降约 43.6%。旧版与新版的扫描安全和 stuck 判定也有差异，不能把整栈差异全部归因于 PPO 学习。发布策略的一个失败发生在 turn 场景，判为 stuck。

这些是 Windows 二维仿真结果。C++ 核心已原生编译，v1 的 10 个、v2 的 11 个 BEV 样例均零误差，1,200 个安全核随机对照零差异，89 项 Python 测试通过；完整 ROS 2 编译、Gazebo 到点和暂停/reset 测试仍需在 Ubuntu 实机执行。

证据位于 `experiments/ppo_v2/holdout/*/summary.json`、`episodes.csv`、逐回合 NPZ、`runs/*` 和 `verification/*`。`models/ppo_v2_recommended/export_manifest.json` 记录选中检查点和导出来源。

## 虚拟机第一次升级

先关闭当前 Gazebo、旧 launch 和策略进程，防止两套节点同时运行。在项目目录执行：

```bash
cd ~/ROS2hw
git pull --ff-only origin main
bash setup/run_ppo_v2.sh
```

这个脚本依次安装锁定的 Linux Python 依赖，重编译消息、BEV、安全、策略和 bringup 五个包，执行模型和 C++ 一致性检查，然后启动居家地图。沿用已经装好的 ROS 2 Humble 和课程 `~/ros2_ws`，首次安装依赖需要联网。使用 `bash` 调用，不依赖脚本可执行权限。

如果 `git pull` 提示本地修改，先 `git status` 查看并保存自己的修改，再处理合并；不要用 reset 覆盖虚拟机上的工作。

安装完成后再次启动只需：

```bash
cd ~/ROS2hw
bash setup/run_ppo_home.sh
```

RViz 用 **2D Goal Pose** 指定 `map` 中的最终目标。也可以在另一终端发送目标，例如：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2hw/ros2_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}"
```

启动后等待地图、定位与规划节点就绪再发目标。默认定位初值保留前一版能工作的 `(-5.9,-0.5,0)`；课程世界实际出生点在 `(-6,-0.5)` 的地图边缘，仍需用 RViz 核对激光与地图对齐。可见目标只在已存在的地图边缘足迹重叠不增加、目标完整进入地图且沿途占用检查通过时允许向内驶离，不会放宽障碍间隙。

## 不动时按这个顺序看

先 source ROS 和项目工作空间，然后：

```bash
ros2 topic echo /planning/status
ros2 topic echo /navigation/status
ros2 topic echo /safety/status
```

每条命令在独立终端运行，或看完按 Ctrl+C 再运行下一条。

| 状态/现象 | 检查位置与处理 |
|---|---|
| `waiting_for_goal` / `no_path` | 在 RViz 发送 2D Goal Pose；确认 `/compute_path_to_pose` 服务端处于 active。 |
| `planner_failed_or_empty` / `planner_rejected` | 检查起终点是否在自由地图、AMCL 与 map→odom TF；先选近处开阔目标。 |
| `pose_unavailable` | 检查 map→base_footprint TF 和 `/clock`；不要用调奖励解决定位问题。 |
| `no_map` / `no_visible_carrot` | 检查 `/map`、机器人是否落在未知/障碍单元，以及路径周围圆形足迹间隙。 |
| `tracking_escape` | 正常脱困回退：车体压入地图安全边距时沿路径给 0.4 m 前视，离开后恢复 `tracking`；若持续不消失再查定位与地图。 |
| `navigation_stale` | 检查 local_goal_node 是否运行及 `/navigation/status` 是否持续发布。 |
| `sensor_timeout` | 检查 `/scan`、激光 TF、时间戳与频率；候选命令仍经 safety 输出。 |
| `scan_unknown_front/rotation` | 看原始 ranges 是否有 NaN/-inf/超限有限数、是否覆盖 360°；修传感器/角度配置。不要把未知读数全部改成正无穷。 |
| `observation_or_command_timeout` | 检查 BEV 的 odom、上一周期 `/cmd_vel` 及 `/local_goal` 是否新鲜。 |
| `inference_timeout` | 看 CPU 占用、推理耗时及模型形状；降低并行占用，保持 32 格模型和配置配套。 |
| `goal_hold` / `goal_reached` | 已在最终目标半径内，停车是预期行为。暂停仿真不会累加完成时间。 |

确认唯一底盘命令出口：

```bash
ros2 topic info /cmd_vel --verbose
```

发布者应只有 `safety_node`。查看策略请求用 `/cmd_vel_rl`，实际执行用 `/cmd_vel`。

复位命令会清除当前任务、旧路径和待执行推理，之后重新发送目标：

```bash
ros2 topic pub --once /policy_reset std_msgs/msg/Empty '{}'
```

保留 v1 回退入口：先关闭当前 launch，再运行 `bash setup/run_ppo_home.sh model_id:=split_mixed_pretrain_v1`。启动器会读取该模型原始 v1 配置，使用固定前视和 v1 BEV；不会把旧权重标成 v2。运行时看门狗和最终目标修正仍保留。

## Ubuntu + Gazebo 实测（2026-11-15）

以下结果在虚拟机 ROS 2 Humble + Gazebo Classic 11 的 `robocup_home` 地图测得，使用 `models/ppo_v2_recommended`；旧权重用 `model_id:=split_mixed_pretrain_v1` 对照。原始采样在 `experiments/ppo_v2/linux_home/`，汇总在 `experiments/ppo_v2/verification/linux_gazebo.json`。

| 任务 | v2 成功 | 仿真耗时 | 路径 | 到点剩余 | 安全干预 |
|---|---|---:|---:|---:|---:|
| (-5.9,-0.5) → (0,0) | 是 | 23.1 s | 5.35 m | 0.204 m | 0 |
| (-5.9,-0.5) → (2,3) | 是 | 38.7 s | 9.33 m | 0.202 m | 0 |
| v1 同目标 (0,0) | 是 | 23.2 s | 5.28 m | 0.239 m | 0 |
| v1 同目标 (2,3) | 是 | 38.3 s | 9.32 m | 0.224 m | 0 |

- 多目标：依次发送 (0,0)、(2,3)，再 `/policy_reset` 后发送 (4.5,-2.0)，三段全部到点（23.7 s / 18.2 s / 39.4 s），无安全干预。
- 运动中改目标：前进 6 s 后把目标改到身后 (-5.5,-0.5)，v2 与 v1 都在 7.3 s 内完成掉头并到点，旧路径按代次失效。
- 回归：Nav2 经安全节点的 (0,0) 目标 `SUCCEEDED`，剩余 0.247 m，恢复 0 次，`/cmd_vel` 发布者仍只有 `safety_node`；简易地图 PPO 在 (0,2) 前 0.21 m 处正常保持停车。
- 校验：91 项 Python 测试、11 个 BEV 样例、1200 组安全核随机对照全部通过，模型包哈希与推理参考一致。

这台机器上 v1/v2 权重在这些开阔居家目标上的耗时和轨迹几乎相同；发布版 v2 的 95/96 成功率来自 Windows 二维测试集，居家地图这两次到点不能替代统计成功率。

集成到 Linux 时修复了三个问题（源码已更新）：

1. `route_manager` 把动作句柄存到 `self.handle`，与 rclpy `Node.handle` 只读属性冲突，节点启动即崩溃；改为 `goal_handle`。
2. `bev_node` 用 scan 时间戳查 `map→base_footprint`，而 AMCL 的 TF 常落后约 5 ms，导致周期性丢观测；目标变换改用最新 TF。
3. 机器人停下时若地图足迹与障碍重叠（AMCL 偏差），可见前视的所有连线都会失败并永久停住；现在车体压入安全边距时先按实体半径重试，仍失败则沿规划路径给 0.4 m 短前视脱困，正常后恢复可见前视。

重复启动 Gazebo 时偶尔会出现 Nav2 生命周期节点停在 `unconfigured`，确认旧进程已退出后重启即可，不是策略问题。

## 后续需要调策略时

1. 先记录失败目标、`/scan`、TF、`/odom`、`/cmd_vel_rl`、`/cmd_vel` 和三个 status。先确认是策略绕障、规划、定位还是消息新鲜度问题。
2. 角落跨墙的目标选择改 `training/envs/navrl/guidance.py::choose_local_goal` 与 `chord_clear`；ROS 对应 `path_tracking.py::PathTracker.update` 和 `chord_is_free`。配置在 `config/ppo_v2_visible.yaml` 的 `lookahead`、`visibility_margin`；每次只改一项，训练和 ROS 保持一致。
3. 先转身却被 stuck 中断，查 `training/envs/navrl/env.py` 的进展/stuck 判定、`turn_grace_seconds`、`progress_epsilon`，保留时间上限，避免原地打转无限续命。
4. 动作抖动或微调变差，查配置的 `ppo.learning_rate`、`clip_range`、`ent_coef` 和 `n_epochs`；当前 1e-4、0.1、0.001、5。先小步降低学习率并保留初始化验证，避免仅保存最后一步。
5. 修改 BEV、雷达语义或形状时同步 Python `scan.py`/`observation.py`、C++ bev_core、安全核及 fixtures；这是观测契约变化，不能只改版本字符串。旧权重跨版本用 `--init-from`，不能 `--resume`。
6. 在 Windows 的项目目录运行新名字的实验：

```powershell
.\run.cmd --config config\ppo_v2_visible.yaml train --name my_v2_s45 --steps 100000 --seed 45 --init-from models\ppo_v2_recommended\policy.zip
```

只改 PPO 超参数且观测契约一致时也可使用 `--resume`，它会继续已有优化器和累计步数；`--init-from` 明确重置优化器与步数。实际参数和来源写入新 run，旧 run 不覆盖。先用 validation 调参；本次 610000 测试已公开，不再把它当作下一轮未见测试。最后重新导出新模型目录、验证哈希、更新默认模型 ID，再到 Gazebo 做真实闭环验收。
