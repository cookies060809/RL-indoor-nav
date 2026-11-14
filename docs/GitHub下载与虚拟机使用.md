# 从 GitHub 下载并导入虚拟机

仓库：https://github.com/cookies060809/RL-indoor-nav  
更新日期：2026 年 11 月 15 日

## 1 首次下载

在 Ubuntu 虚拟机打开终端，执行：

```bash
git clone https://github.com/cookies060809/RL-indoor-nav.git ~/RL-indoor-nav
cd ~/RL-indoor-nav
python3 training/scripts/verify_project.py
```

仓库根目录就是方案中的 project 目录。无需共享文件夹，也无需再进入一层 project。请使用新目录保留你已有的 ~/ros2_ws 和课程仿真代码。

如果缺少 git，在虚拟机安装 git 后重试。已有同名目录时先查看内容，选择另一个下载目录，不要直接删除已有项目。

## 2 安装 Linux 依赖并检查模型

```bash
cd ~/RL-indoor-nav
bash training/scripts/setup_ubuntu.sh
bash run.sh doctor
```

脚本使用 Python 3.10 创建本仓库专用 .venv，安装 CPU PyTorch 2.8 和锁定依赖，再验证三个模型的文件哈希与固定观测推理。下载依赖需要网络。

如果报错缺少 venv，可先安装 Ubuntu 的 python3.10-venv。不要复制 Windows 的 .venv，也不要使用 sudo pip。

后续操作见 [虚拟机详细导入步骤](虚拟机导入步骤.md) 的第 3 至 6 节，其中旧压缩包示例目录 ~/ROS2_project_import/project 在本下载方式下统一替换为 ~/RL-indoor-nav。

## 3 构建消息与策略节点

```bash
cd ~/RL-indoor-nav
bash training/scripts/build_ros_interfaces.sh
```

本命令只构建新工作空间中的 project_msgs 与 rl_nav_pkg。完整 BEV、安全节点、路径管理和 Gazebo 评测仍需按方案完成；README 说明目录不代表模块已经实现。

已有有效观测发布者和独立安全节点时，可启动策略节点：

```bash
bash training/scripts/run_policy.sh split_empty_pretrain_v1
```

该节点只发布 /cmd_vel_rl，不能绕过 safety_node 接到底盘。

## 4 后续更新

```bash
cd ~/RL-indoor-nav
git status
git pull --ff-only
```

如果你已在虚拟机修改文件，先保存自己的提交或备份，再处理更新。不要直接丢弃本地改动。完整文件校验应在首次下载后进行；本地改过配置或新生成报告后，对应哈希变化属于预期，需要区分开发修改与传输损坏。

## 5 已有内容

- 训练代码、配置、17 项测试、固定观测样例与调参教程。
- 空场景、单障碍、混合布局三个已训练的初步模型及验证工具。
- 既有二维实验数据、图表和训练记录。
- ROS 消息与策略包结构，以及待实现模块的状态说明。
- 原仓库的 setup 脚本、开题报告和旧实施方案保留。

Windows 上已完成测试与模型检查；Ubuntu 的依赖安装、colcon 构建和 Gazebo 闭环仍需在虚拟机执行。
