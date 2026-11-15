# Windows 完成与待办清单（yy / v2）

检查日期：2026 年 11 月 15 日。项目位置：`D:\Personal\projects\RL-indoor-nav`。本清单只记录当前分支状态；历史报告和旧模型保留在 `docs/reports/windows_original/` 与 `training/runs/`，不回写。

## 1 责任边界

| 工作 | Windows 物理机 | Ubuntu 虚拟机 |
|---|---|---|
| 二维任务生成、PPO 训练、验证与导出 | 在这里执行 | 可复验 |
| ROS 通信、SLAM、AMCL、Nav2 | 不运行 | 待执行 |
| C++ BEV、安全节点、route_manager | 只提供参考与 fixtures | 待编译集成 |
| Gazebo 闭环与正式 A/B/C 实验 | 不能据此验收 | 待执行 |

## 2 当前环境

- Python 3.10.22；torch 2.8.0+cpu；SB3 2.9.2；Gymnasium 1.4.0；NumPy 2.2.6；SciPy 1.15.3；Numba 0.63.1。
- 当前 Windows 实测 14 个物理核、20 个逻辑线程、约 31.7 GB 内存、无 CUDA；以 `docs/reports/yy_v2/environment.json` 为准。
- 项目内 `.venv` 由 `setup_windows.ps1` 重建；运行入口 `run.cmd`/`python.cmd`。
- 所有新训练、评估、图表和报告写入 `experiments/yy_v2/` 与 `docs/reports/yy_v2/`。

## 3 本次完成的代码

- [x] 统一 `HIT/FREE/UNKNOWN` 扫描分类；超量程、低于量程、NaN、`-inf` 均为未知；只有 `+inf` 表示无回波。
- [x] BEV 与保护模块共用同一分类；新增 `above_max` 固定样例。
- [x] 保护按运动方向和制动扫掠角域检查覆盖；前方缺测禁止前进，后方缺测禁止倒车，纯旋转要求全周覆盖。
- [x] 评估使用 effective config，CLI 覆盖统一用于执行、目录命名和保存。
- [x] ROS 节点使用 `STEADY_TIME` 看门狗，重置清空状态，推理超时/旧 token 不发布。
- [x] 导出在隔离子进程中只使用训练快照，记录权重哈希、步数和成绩绑定；final 不携带 best 成绩。
- [x] 卡死按持续无进展判定，移除角速度豁免，增加有限转向宽限期。
- [x] 配置范围检查、续训检查前置、临时目录导出、逐步评估记录（动作、请求/执行速度、干预原因、版本）。
- [x] 新增自适应局部目标、`dense`/`tight_turn`/`protective` 场景和显式 `stage_weights`。
- [x] 默认关闭的干预/动作偏差奖励开关，支持单因素实验。

## 4 已实测

- [x] `pytest training/tests/unit`：63 项通过。
- [x] `run.py doctor`：Gymnasium `check_env` 通过，输入 2056 维。
- [x] 2,048 步 SubprocVecEnv 短训完成，四个新阶段均可生成可达任务。
- [x] 旧 run 的 `final_model` 隔离子进程导出成功，`verify_bundle.py` 通过，final 未携带 best 成绩。
- [x] 旧 v1 模型普通 `--resume` 被拒绝且未创建实验目录；`--init-from` 成功并记录来源。
- [x] 评估 CLI 的 `--no-safety` 与模型配置组合已验证：执行、保存配置和 `effective_safety` 一致。

## 5 尚未验证

- [ ] Ubuntu 上的 rclpy 实际调度、STEADY_TIME 定时器、重置话题和推理超时。
- [ ] C++ BEV 对 v2 fixtures 的真实输出对照。
- [ ] safety_node 的矩形足迹扫掠、唯一 `/cmd_vel` 发布者。
- [ ] Gazebo 接触、雷达外参、底盘响应和实体重置。
- [ ] 真实地图上的 Nav2 基线与正式 A/B/C 实验。

## 6 当前待办

1. 在 Ubuntu 编译 `project_msgs`、`rl_nav_pkg`，运行 `deployment/verify_bundle.py`。
2. 用 v2 fixtures 对照 C++ BEV；确认 `+inf`、NaN、超量程语义一致。
3. 实测策略节点在仿真暂停、时钟倒退、重复观测和推理超时下的停车行为。
4. 测量真实雷达外参、足迹、控制周期；若与配置不同，冻结新规格并重训。
5. 用冻结的验证/测试集评估 v2 模型；不要把诊断集或测试集用于调参。
6. 完成 Gazebo 闭环与正式实验，二维结果只在二维章节报告。

## 7 文件导航

| 内容 | 文件 |
|---|---|
| 统一扫描语义 | `training/envs/navrl/scan.py` |
| 保护与覆盖 | `training/envs/navrl/control.py` |
| BEV | `training/envs/navrl/observation.py` |
| 环境、场景、卡死 | `training/envs/navrl/env.py` |
| 局部目标 | `training/envs/navrl/goal.py` |
| 评估记录 | `training/envs/navrl/evaluation.py` |
| 训练/评估/导出入口 | `training/scripts/run.py` |
| 隔离导出 | `training/scripts/export_bundle.py` |
| 看门狗状态机 | `training/scripts/deployment/watchdog.py` |
| 策略节点 | `training/scripts/deployment/policy_node.py` |
| v2 报告 | `docs/reports/yy_v2/` |

历史 Windows 实测摘要仍见 `docs/reports/windows_original/验收摘要.md`；那些结果的模型、语义版本和测试集与本页不同，不能直接合并比较。
