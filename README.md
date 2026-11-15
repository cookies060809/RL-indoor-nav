# 室内移动机器人导航项目

## 直接从 GitHub 下载到虚拟机

```bash
git clone https://github.com/cookies060809/RL-indoor-nav.git ~/ROS2hw
cd ~/ROS2hw
python3 training/scripts/verify_project.py
bash training/scripts/setup_ubuntu.sh
bash run.sh doctor
```

仓库根目录对应方案中的 project。完整步骤见 [GitHub 下载与虚拟机使用](docs/GitHub下载与虚拟机使用.md)，无需 VMware 共享文件夹。原仓库的 setup、开题报告和旧方案保留。


本目录按《实施方案_综合版》第 9 章整理，作为 Windows 与 Ubuntu 之间的正式交付目录。整理日期：2026 年 11 月 15 日。

Windows 旧工程和原虚拟环境仍在 `E:\ROS2`，此前的训练记录保留。本目录中的训练、评估和模型导出入口已适配新路径；传到 Ubuntu 时只需传递本目录的压缩包。

## 先读

1. [虚拟机导入步骤](docs/虚拟机导入步骤.md)：解压、检查、安装 Linux 依赖、验证模型、编译消息与策略包。
2. [文件迁移对照](docs/文件迁移对照.md)：旧路径与新路径的对应关系。
3. [Windows 完成与待办清单](docs/Windows完成与待办清单.md)：已实测的范围及 Gazebo 后续工作。
4. [训练与调参教程](docs/训练与调参手把手教程.md)：具体参数、函数和行为排查。
5. [已有验收数据](docs/reports/windows_original/验收摘要.md)：此前 Windows 二维环境实测结果。

## 目录

```text
project/
├── README.md
├── config/                       配置及核心依赖锁文件
├── ros2_ws/src/
│   ├── wpr_simulation2/          说明目录，课程源码沿用虚拟机现有版本
│   ├── project_msgs/             消息定义及构建文件，待 Ubuntu 编译
│   ├── bev_pkg/                  C++ BEV 核心、节点与一致性校验
│   ├── rl_nav_pkg/               Python 推理节点及构建文件
│   ├── safety_pkg/               独立安全节点，唯一 /cmd_vel 发布者
│   ├── evaluation_pkg/           Gazebo 评测（待实现）
│   └── project_bringup/          PPO/Nav2 启动、俯视相机与录像
├── training/
│   ├── envs/navrl/               二维环境、BEV、奖励、控制和特征提取器
│   ├── scripts/                  训练、评估、打包与安装工具
│   ├── tests/                    单元测试及固定观测样例
│   └── runs/                     已有 best/final 模型、日志与配置快照
├── models/<model_id>/            三个可独立验证的模型包
├── maps/                         待保存正式 PGM 与 YAML 地图
├── experiments/
│   ├── manifests/               二维实验索引、配置与摘要
│   ├── csv/                     逐回合指标
│   ├── trajectories/            NPZ 轨迹
│   ├── bags/                    待保存真实 rosbag
│   └── figures/                 曲线、BEV 和轨迹图
└── docs/                         方案、教程、迁移说明与验收记录
```

训练日志中的历史 Windows 路径和源代码快照没有重写，它们用于追溯当时的实验。`training/runs` 与 `experiments` 中有部分相同数据：前者保留运行上下文，后者按方案分类便于分析；对应关系见 `experiments/manifests/file_index.json`。

## Windows 使用

```powershell
cd E:\ROS2\project
.\run.cmd doctor
.\python.cmd -m pytest training\tests\unit -q --basetemp=.tmp\pytest
.\run.cmd --config config\split_mixed32.yaml train --name my_mixed_continue --steps 100000 --resume training\runs\split_mixed_100k_s42\best_model.zip
```

本机 `python.cmd` 优先使用本目录 `.venv`，不存在时使用上一层已有的 `E:\ROS2\.venv`，无需重复安装。另一台 Windows 使用 `setup_windows.cmd` 建立本目录专用环境。

## Ubuntu 使用

```bash
cd ~/ROS2hw
bash setup/04_setup_rl_env.sh    # 首次建立 venv 与依赖
bash run.sh doctor
./setup/run_rl_nav.sh            # PPO 闭环：Gazebo + BEV + 策略 + safety
./setup/run_ppo_home.sh          # PPO + 居家地图：Nav2 全局规划 + 1.2 m 前视目标
./setup/run_nav2_home.sh         # Nav2 + 居家地图：唯一 /cmd_vel 出口仍是 safety
```

已实现 C++ BEV、独立安全节点、Nav2 经 safety 的统一速度出口，并在 `experiments/videos/` 生成两段演示视频。PPO 主动全局规划、`evaluation_pkg` 与正式 A/B/C 实验仍在进行，详见 [中期审查进展](docs/中期审查进展.md)。不要直接把 `/cmd_vel_rl` 接到底盘替代安全节点。

## 模型和既有结果

| 模型目录 | 二维测试成功数 | 用途 |
|---|---|---|
| models/split_empty_pretrain_v1 | 23/24 | 首次加载及空场景部署 |
| models/split_single_pretrain_v1 | 22/24 | 单障碍预训练 |
| models/split_mixed_pretrain_v1 | 19/24 | 混合布局初步策略 |

上述都是先前 Windows 二维环境的结果，不能当作 Ubuntu 或 Gazebo 已通过。核心库使用 torch 2.8 CPU、SB3 2.9.2、Gymnasium 1.4.0；不复制 Windows 虚拟环境到 Linux。
