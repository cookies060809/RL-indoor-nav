# 室内移动机器人导航项目

当前工作分支：`yy`。当前接口版本：`bev_nav_v2`。策略保持原有固定 1.2 m 前视、四场景课程（empty/single/turn/clutter/mixed）；本轮只保留六项正确性修复和保护机制。`main` 分支新增的 C++ BEV、独立安全节点、Nav2/PPO 启动与中期演示数据已合并到本分支。

## 环境与入口

Windows 项目目录：`D:\Personal\projects\RL-indoor-nav`。正式环境为项目内 `.venv`（Python 3.10.22、torch 2.8.0+cpu、SB3 2.9.2、Gymnasium 1.4.0）。

```powershell
.\run.cmd doctor
.\python.cmd -m pytest training\tests\unit -q --basetemp=.tmp\pytest
```

Ubuntu 首次导入：

```bash
git clone https://github.com/cookies060809/RL-indoor-nav.git ~/ROS2hw
cd ~/ROS2hw
python3 training/scripts/verify_project.py
bash training/scripts/setup_ubuntu.sh
bash run.sh doctor
```

更完整的导入说明见 [GitHub 下载与虚拟机使用](docs/GitHub下载与虚拟机使用.md) 与 [虚拟机导入步骤](docs/虚拟机导入步骤.md)。不要复制 Windows `.venv`。

## v2 正确性修复

- 扫描分类统一为：量程内有限值 = 命中，`+inf` = 无回波，其余（NaN、`-inf`、低于 `min_range`、高于 `max_range`）= 未知。未知不再被当作空闲。
- 保护过滤按运动方向和制动扫掠角域检查覆盖；前方缺测禁止前进，后方缺测禁止倒车，纯旋转要求全周已知。策略动作、请求速度、执行速度和干预原因均写入评估记录。
- 卡死按持续无进展判定，角速度大不再豁免；仅在朝局部目标转向且航向误差下降时给予有限宽限期。
- 评估使用 effective config：模型配置为默认，`--stage`/`--stages`/`--no-safety` 显式覆盖，并统一用于执行、目录命名、`evaluation_config.yaml` 和 `evaluation_metadata.json`。
- ROS 策略节点使用 `STEADY_TIME` 看门狗；重置清空状态，推理超时或旧结果不会发布。该节点尚未在 Windows 运行 ROS，需在 Ubuntu 验证。
- 导出在隔离子进程中使用运行目录的 `source/navrl` 快照生成 fixtures、加载和验证模型；记录所选权重 SHA256、步数和成绩绑定。`final_model` 不携带 best 成绩；旧运行若缺少权重哈希，best 成绩不标记为已绑定。

## 训练

沿用原有课程配置：

```powershell
.\run.cmd --config config\split_empty32.yaml train --name empty_s42 --steps 100000 --seed 42
.\run.cmd --config config\split_single32.yaml train --name single_s42 --steps 100000 --seed 42 `
  --resume training\runs\...\best_model.zip
.\run.cmd --config config\split_mixed32.yaml train --name mixed_s42 --steps 300000 --seed 42
```

v1 与 v2 的观测/保护/终止语义不同，旧权重不能普通 `--resume`；需要显式初始化时使用 `--init-from`，程序会写 `initialization.json` 记录来源、权重哈希、源步数和源/新 spec 哈希。续训兼容检查在创建实验目录之前完成。

## 评估

```powershell
.\run.cmd evaluate --model training\runs\mixed_s42\best_model.zip `
  --split validation --episodes 12 `
  --stages empty single turn clutter
```

- `diagnostic`：开发诊断任务。
- `validation`：分场景固定种子，用于选型。
- `test`：独立种子，配置冻结后才运行；`--no-safety` 只用于安全消融，默认尊重模型配置。
- 评估目录包含 `episodes.csv`、`summary.json`、逐回合 `trajectory_*.npz`（原始动作、请求/执行速度、干预标记）、分场景图、失败原因图和速度曲线。

## 导出

```powershell
.\run.cmd export --model training\runs\mixed_s42\final_model.zip --name mixed_final_s42 --output-root models
```

导出先写 `.tmp/export_*`，在隔离子进程中用训练快照生成并验证，成功后原子移动到 `models/<name>`。包内 `export_manifest.json` 记录权重、步数、源码快照哈希和成绩绑定；`deployment/verify_bundle.py` 必须通过。旧 v1 模型包不改写。

## 目录

```text
project/
├── README.md
├── config/                       配置及核心依赖锁文件
├── ros2_ws/src/
│   ├── wpr_simulation2/          说明目录，课程源码沿用虚拟机现有版本
│   ├── project_msgs/             消息定义及构建文件，待 Ubuntu 编译
│   ├── bev_pkg/                  C++ BEV 核心、节点与一致性校验
│   ├── rl_nav_pkg/               Python 推理节点、全局路径与局部目标
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
│   ├── bags/                    真实 rosbag 与中期演示数据
│   ├── videos/                  Nav2/PPO 演示视频
│   └── figures/                 曲线、BEV 和轨迹图
└── docs/                         方案、教程、迁移说明与验收记录
```

## Windows 使用

```powershell
cd D:\Personal\projects\RL-indoor-nav
.\run.cmd doctor
.\python.cmd -m pytest training\tests\unit -q --basetemp=.tmp\pytest
.\run.cmd --config config\split_mixed32.yaml train --name my_mixed_continue --steps 100000 --resume training\runs\split_mixed_100k_s42\best_model.zip
```

`python.cmd` 优先使用本目录 `.venv`；另一台 Windows 使用 `setup_windows.cmd` 建立本目录专用环境。

## Ubuntu 使用

```bash
cd ~/ROS2hw
bash setup/04_setup_rl_env.sh    # 首次建立 venv 与依赖
bash run.sh doctor
./setup/run_rl_nav.sh            # PPO 闭环：Gazebo + BEV + 策略 + safety
./setup/run_ppo_home.sh          # PPO + 居家地图：Nav2 全局规划 + 1.2 m 前视目标
./setup/run_nav2_home.sh         # Nav2 + 居家地图：唯一 /cmd_vel 出口仍是 safety
```

已实现 C++ BEV、独立安全节点、Nav2 经 safety 的统一速度出口，并在 `experiments/videos/` 生成演示视频。PPO 主动全局规划、`evaluation_pkg` 与正式 A/B/C 实验仍在进行，详见 [中期审查进展](docs/中期审查进展.md)。不要直接把 `/cmd_vel_rl` 接到底盘替代安全节点。

## 模型和既有结果

| 模型目录 | 语义版本 | 二维测试成功数 | 用途 |
|---|---|---|---|
| models/split_empty_pretrain_v1 | v1 | 23/24 | 首次加载及空场景部署 |
| models/split_single_pretrain_v1 | v1 | 22/24 | 单障碍预训练 |
| models/split_mixed_pretrain_v1 | v1 | 19/24 | 混合布局初步策略 |

这些是先前 Windows 二维结果，不能当作 Ubuntu 或 Gazebo 已通过。

## 未验证项

Windows 未运行 ROS、Gazebo、真实雷达、Ubuntu 调度和底盘；合并进来的 C++ BEV、safety_node、Nav2 启动和演示数据尚未在本机重新编译/实跑，需在 Ubuntu 验证。二维结果单独报告，不冒充 Gazebo 结果。六项修复与测试记录见 [v2 正确性修复说明](docs/yy_v2_正确性修复说明.md)。