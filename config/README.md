# 配置入口

`ppo_v2_visible.yaml` 是本次 main 集成与重训使用的候选配置：32 格、可见子目标、4 个 CPU 子进程、2 个 PyTorch 线程。它使用 `bev_nav_v2`，不能把旧模型配置的版本字符串直接改成 v2；迁移 v1 权重必须通过 `--init-from` 明确初始化、重新训练并记录来源。

`environment.subgoal_mode` 缺省为 `fixed`，保留 yy 的原固定前视行为；设置为 `visible` 才启用地图足迹可见性选择，`visibility_margin` 是额外间隙，默认 0.04 m。训练、验证和部署的目标选择模式必须一致。

当前接口版本为 `bev_nav_v2`。`base.yaml` 与各 `*32.yaml` 保留原有四场景课程：`empty`、`single`、`turn`、`clutter`；`mixed` 在 reset 时按原有逻辑随机选择四类场景。本轮加入扫描/保护/终止语义版本、配置范围检查和可选可见前视目标，奖励函数仍保留原版。

- `split_*32.yaml`：split_mlp 特征提取器，4 个 SubprocVecEnv。
- `empty32.yaml`、`single32.yaml` 等：展平 MLP 对照配置。
- `formal48.yaml`：48 格配置，不能续接 32 格模型。
- `windows_recommended32.yaml`：Windows 混合任务推荐配置。

历史运行和模型包内的 `config.yaml` 保持 v1，不得用当前代码冒充其原始语义。

`requirements-core-lock.txt` 用于 Linux 核心依赖，先从官方 CPU 源单独安装 torch 2.8。`requirements-win-lock.txt` 仅用于 Windows 重建。
