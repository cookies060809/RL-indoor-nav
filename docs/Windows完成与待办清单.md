> 已按 project 交付目录更新路径；旧日志中的历史 Windows 路径保持原样。虚拟机入口见《虚拟机导入步骤.md》。

# Windows 部分完成与待办清单

检查日期：2026 年 11 月 14 日。项目位置：`E:\ROS2\project`。本清单区分已写好、已实测与尚未在虚拟机验证的内容；具体训练结果另见文末实测记录。

## 1 物理机与虚拟机分别负责什么

| 工作 | Windows 物理机 | Ubuntu 虚拟机 |
|---|---|---|
| Python 二维任务生成、PPO 训练 | 在这里执行 | 可作后备，当前不依赖 |
| 验证与测试、训练曲线、二维轨迹 | 在这里执行 | 可加载同一代码复验 |
| 模型打包、哈希、固定推理样例 | 在这里生成 | 接收并比对 |
| ROS 通信、SLAM、AMCL、Nav2 | 当前不在这里运行 | 在这里执行 |
| C++ BEV、安全输出、全局路径管理 | 参考接口由这里提供 | 在这里编译、集成、测试 |
| Gazebo 策略闭环及正式 A/B/C 实验 | 当前不能据此验收 | 在这里执行 |

## 2 我已完成的 Windows 配置

- [x] 读取并按《实施方案_综合版.md》实现 Windows 训练部分，保留综合版原文件。
- [x] 核实物理机为 i7-14700，20 个物理核心、28 个逻辑线程，内存约 64 GB。
- [x] 在 `.runtime` 安装项目专用 Python 3.10.22，在 `.venv` 建立独立环境。
- [x] 安装并实测 CPU PyTorch、Stable-Baselines3、Gymnasium、NumPy、SciPy、Numba、TensorBoard 等依赖。
- [x] 依赖一致性检查通过，保存 Windows 完整锁文件和 Linux 交接用核心锁文件。
- [x] 下载工具、库环境、缓存和临时文件目录均指定在 `E:\ROS2\project` 内。
- [x] 提供 `setup_windows.cmd` 重建脚本和 `run.cmd` 统一运行入口，不要求修改全局 PATH。

实际组合是 **Python 3.10.22、torch 2.8.0+cpu、SB3 2.9.2、Gymnasium 1.4.0**。综合版写的 torch 2.14.1 在本机出现 c10.dll / WinError 1114；本机系统 VC++ DLL 为 14.36，因此采用已实际跑通的 2.8 CPU 版。没有升级系统运行库或修改其他项目环境。

PyTorch 官方问题记录说明，新版构建与旧 msvcp140.dll 的组合会出现此类初始化失败；本次本机现象与其描述相符。[官方原因与修复讨论](https://github.com/pytorch/pytorch/pull/196987)。SB3 2.9.2 的文档要求 PyTorch ≥2.8，当前组合满足。[SB3 安装要求](https://stable-baselines3.readthedocs.io/en/v2.9.2/guide/install.html)

## 3 我已完成的代码

- [x] 圆形机器人足迹、矩形静态障碍与精确激光射线求交。
- [x] 独轮车运动、速度和加速度约束、子步碰撞检测。
- [x] 足迹膨胀 A*、可达起终点生成、路径简化和前视子目标。
- [x] 32/48 格 BEV、未知与无回波处理、欧氏距离变换、目标与运动标量拼接。
- [x] 真值几何碰撞、保持 1 秒的最终目标成功、卡死与超时区分。
- [x] 分项奖励累计、训练与验证记录、best/final/checkpoint 模型。
- [x] 保留展平 MLP 基线，增加 BEV 编码后拼接目标与运动状态的 split_mlp 特征提取器。
- [x] 基于扫描的圆形足迹保护过滤，原始与执行动作分离。
- [x] 固定训练种子、验证与测试种子分离、多环境及 Windows spawn 支持。
- [x] 固定任务评估、CSV 与轨迹保存、验证曲线和轨迹图。
- [x] 1、2、4 环境性能测量；DummyVecEnv 和 SubprocVecEnv 均实际运行。
- [x] 三种子顺序训练脚本，避免并发抢资源。
- [x] 模型规格、源码与配置快照、依赖、SHA256 清单和跨系统推理参考导出。
- [x] 10 个固定观测样例，包含前墙、左右墙、转角、无效数据、边界、最小量程、零目标与偏置雷达。
- [x] C++ 输出对照脚本、PolicyObservation 消息源文件、ROS Python 推理节点源文件。
- [x] 手把手训练与调参教程，逐项对应配置、文件、函数及重新训练条件。

## 4 已实际做过的验证

- [x] Gymnasium `check_env` 通过，2056 维 float32 观测与连续动作空间一致。
- [x] 观测方向、无回波与未知、边界、欧氏距离、动作缩放、扫描保护、足迹碰撞、任务复现、成功与超时测试通过。
- [x] 生成并检查 BEV 样例图 `docs/reports/bev_fixtures.png`。
- [x] 使用独立路径跟随器做 12 个环境诊断任务，完成 10 个，2 个在转角保护处卡死；这不是 PPO 成绩。
- [x] 实际运行 PPO 短训及较长训练，保存原始日志和最佳检查点。
- [x] 实际运行两种多环境模式的吞吐测量，结果在 reports 下。

当前保护是训练端的圆形近似，没有完成真实 ROS 看门狗、矩形足迹扫掠或 Gazebo 接触验证。C++ BEV 没有编译运行，Ubuntu 推理没有在这里执行。请勿把文件准备好等同于这些项目已通过。

## 5 推荐的 Windows 设置

`config/windows_recommended32.yaml` 使用 mixed 阶段、4 个 SubprocVecEnv 环境、2 个 PyTorch 线程，并采用 split_mlp 将 BEV 编码后再拼接状态。依据本机展平 MLP 的 1 万步测试，4 子进程约 2192 环境交互步/秒，4 Dummy 环境约 1495 步/秒；都是当次 PPO 总训练吞吐，不含正式训练的周期验证耗时。改用 split_mlp 后的速度以其训练日志为准，不直接照搬该数字。

这些测试没有代表完整 Gazebo 场景。首次 JIT、障碍数量、验证频率、后台程序、VMware 负载都可能改变速度。报告中的内存与 system_cpu_percent 是采样值，不是全程峰值或平均值。

正式训练时先停掉虚拟机中的重负载仿真；正式 Gazebo 计时实验时暂停 Windows 训练。没有自动关闭你的虚拟机，也没有调整电源设置、杀毒排除项或 VMware 设置。

## 6 你接下来需要做的事情

### 现在在 Windows 做的检查

1. 打开 PowerShell，`cd E:\ROS2\project`，执行 `.\run.cmd doctor`，确认能找到项目环境。
2. 打开 `docs/reports/bev_fixtures.png`、训练曲线和轨迹，确认理解 x 向前、y 向左以及未知区域。
3. 阅读调参教程；已有模型可用于验证流程或预训练起点，不直接作为课程最终模型。

### 在虚拟机必须完成

- [ ] 保存完整正式地图，关闭建图后重新加载，完成至少 5 组 Nav2 基线记录。
- [ ] 测量激光实际 angle_min、angle_increment、range_min/max、频率与平面 TF 外参。
- [ ] 核实机器人真实碰撞足迹、底盘运动模式、速度及响应，确认 vy=0。
- [ ] 建立 project_msgs，编译 C++ BEV 节点，对比真实 C++ 输出与 fixtures。
- [ ] 安装 Ubuntu Python 3.10 环境，同时验证 rclpy 与核心学习库。
- [ ] 将导出包复制到共享文件夹，再从共享文件夹复制到 Ubuntu 本地磁盘，执行 verify_bundle.py。
- [ ] 集成 route_manager 主动请求 ComputePathToPose，完成路径和子目标更新。
- [ ] 实现并验证 C++ safety_node，确保 `/cmd_vel` 只有一个发布者。
- [ ] 检查 Nav2 的 controller、恢复行为和 velocity_smoother 速度链，避免争抢输出。
- [ ] 验证实体出生位置、速度、定位和历史状态的完整重置，不只发布 initialpose。
- [ ] 完成空场景、单障碍、转角、完整路径部署，并进行丢传感器或策略故障停车测试。

### 核实部署参数后完成正式训练与实验

- [ ] 若雷达外参、足迹或动作周期与当前近似不同，先统一配置并重新训练。
- [ ] 完成障碍与混合任务训练，用验证集选择配置，最后运行三个独立种子。
- [ ] 冻结模型和任务，再用独立测试集评估；不要只靠训练奖励选模型。
- [ ] Gazebo 完成 A/B 48 次主实验和 C 12 次安全消融，保存失败与干预。
- [ ] 核对结果、截图、报告、视频、源码与成员贡献，不使用这里的二维结果冒充 Gazebo 结果。

目前需要你操作的部分主要涉及虚拟机现场状态与真实仿真参数，不是让你重新编写 Windows 训练流程。Windows 部分已有运行入口和调参教程。

## 7 文件导航

| 要找什么 | 文件 |
|---|---|
| 一键检查与所有命令入口 | `run.cmd`、`training/scripts/run.py` |
| 重建环境 | `setup_windows.cmd`、`training/scripts/setup_windows.ps1` |
| 实际软件版本与设备 | `docs/reports/environment.json` |
| 测试报告 | `docs/reports/tests.xml` |
| 性能原始数据 | `docs/reports/benchmark_dummy.json`、`benchmark_subproc.json` |
| 训练步骤与失败排查 | `docs/训练与调参手把手教程.md` |
| 模型交接 | `training/scripts/deployment/README.md` |
| 奖励函数 | `training/envs/navrl/env.py → IndoorNavEnv._compute_reward` |
| BEV 生成 | `training/envs/navrl/observation.py → rasterize / scan_to_bev` |
| 目标和路径 | `training/envs/navrl/env.py → _update_route`、`geometry.py → plan_path` |
| 速度与保护 | `training/envs/navrl/control.py → scale_action / limit_rate / filter_velocity` |
| PPO 与模型选择 | `training/scripts/run.py → make_model / ValidationCallback.validate` |

## 8 本次实际模型与验收摘要

已实际完成空场景、单障碍和混合布局的阶段训练、模型保存、另外生成的任务评测和导出。17 项自动测试通过，三个模型包均在 Windows 校验了哈希与固定输入动作。

| 模型阶段 | 测试成功 | 测试碰撞 | 可交接模型包 |
|---|---|---|---|
| 空场景 | 23/24 | 0/24 | `models/split_empty_pretrain_v1.zip` |
| 单障碍 | 22/24 | 0/24 | `models/split_single_pretrain_v1.zip` |
| 混合布局 | 19/24 | 0/24 | `models/split_mixed_pretrain_v1.zip` |

混合布局的 5 次失败均为卡死。另做同模型、同 12 个验证任务的安全过滤诊断：开启保护为 9 次成功、0 次碰撞；关闭额外保护为 9 次成功、3 次碰撞。这说明当前保护能拦下部分危险动作，但没有解决这些任务的局部决策失败。

模型均为 seed 42 的初步成果，阶段训练会从验证集选中的较早检查点续训，不能按目录名称直接相加推断最终模型步数。准确数据见 [验收摘要](reports/验收摘要.md)、`docs/reports/model_summary.csv` 及各运行目录中的配置与来源。

混合模型的策略单次推理 p95 约 0.29 ms，未包含 ROS 通信、TF 或 BEV 构建。先用空场景包验证部署接口，再用混合包做后续场景验证。全部为二维结果，不能替代 Gazebo 或三种子正式实验。
