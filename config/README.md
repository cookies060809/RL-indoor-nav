# 配置入口

当前接口版本为 `bev_nav_v2`。`base.yaml` 与各 `*32.yaml` 已写入统一的扫描分类、保护覆盖、场景权重、局部目标、卡死宽限期和奖励开关字段。历史运行目录和模型包内的 `config.yaml` 保持 v1，不得用当前代码冒充其原始语义。

- `yy_v2_fixed50k.yaml`：v2 固定 1.2 m 前视基线，混合场景，4 SubprocVecEnv，5 万步对比预算。
- `yy_v2_adaptive50k.yaml`：v2 自适应局部目标，其余与固定基线一致。
- `yy_v2_reward_intervention30k.yaml`：在自适应配置上只增加持续干预惩罚。
- `yy_v2_reward_deviation30k.yaml`：在自适应配置上只增加动作偏差惩罚。

`split_*32.yaml` 使用 split_mlp 特征提取器；`empty32.yaml` 等保留展平 MLP 对照。`formal48.yaml` 为 48 格配置，不能续接 32 格模型。

阶段配置：`empty`、`single`、`turn`、`clutter`、`dense`、`tight_turn`、`protective`。`mixed` 使用 `environment.stage_weights` 显式控制比例，先选场景再重采样可达任务。

`requirements-core-lock.txt` 用于 Linux 核心依赖，先从官方 CPU 源单独安装 torch 2.8。`requirements-win-lock.txt` 仅用于 Windows 重建。
