# 配置入口

当前接口版本为 `bev_nav_v2`。`base.yaml` 与各 `*32.yaml` 保留原有四场景课程：`empty`、`single`、`turn`、`clutter`；`mixed` 在 reset 时按原有逻辑随机选择四类场景。本轮只加入扫描/保护/终止语义版本和配置范围检查，不包含扩展场景权重、局部目标模式或额外奖励项。

- `split_*32.yaml`：split_mlp 特征提取器，4 个 SubprocVecEnv。
- `empty32.yaml`、`single32.yaml` 等：展平 MLP 对照配置。
- `formal48.yaml`：48 格配置，不能续接 32 格模型。
- `windows_recommended32.yaml`：Windows 混合任务推荐配置。

历史运行和模型包内的 `config.yaml` 保持 v1，不得用当前代码冒充其原始语义。

`requirements-core-lock.txt` 用于 Linux 核心依赖，先从官方 CPU 源单独安装 torch 2.8。`requirements-win-lock.txt` 仅用于 Windows 重建。