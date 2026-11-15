# yy / bev_nav_v2 实验目录

本目录只存放 v2 新结果。历史 v1 运行、模型和报告在 `training/runs/`、`models/`、`docs/reports/windows_original/` 保持原样。

## 目录

- `runs/`：6 个 50k 对比训练（fixed/adaptive × seed 42/43/44），以及 2 个 30k 单因素奖励实验。
- `evaluations/validation/`：分场景验证集，每场景 8 回合，用于选型。
- `evaluations/test/`：独立测试集，每场景 12 回合，配置冻结后使用。
- `evaluations/diagnostic/`：现有开发诊断集，4 个场景 × 8 回合 = 32 个任务。
- `evaluations/ablation/`：安全过滤开/关配对诊断。
- `summary/`：跨种子汇总、分场景图、失败原因和速度对比。
- `exports/`：隔离导出的 v2 模型包（如生成）。

## 种子协议

- validation：`100000 + stage_index*1000 + episode`
- test：`200000 + stage_index*1000 + episode`
- diagnostic：`300000 + stage_index*1000 + episode`

stage 顺序为 `empty, single, turn, clutter, dense, tight_turn, protective`。测试集不参与调参；诊断集不作为最终结论。

## 对比约束

fixed 与 adaptive 使用相同网络（split_mlp）、输入维度、场景权重、PPO 超参数、50k 预算和三个训练种子；唯一差异是 `environment.goal_selection.mode`。旧 v1 权重只作历史基线，不进入 v2 排名。
