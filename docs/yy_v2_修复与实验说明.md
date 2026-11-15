# yy / bev_nav_v2 修复与实验说明

日期：2026 年 11 月 15 日。分支：`yy`。当前代码提交：`6521afb`。接口版本：`bev_nav_v2`。本文只报告二维 CPU 环境结果；ROS/Gazebo 未运行。

## 1 范围与版本

- 新增实验、评估和图表只在 `experiments/yy_v2/`、`docs/reports/yy_v2/` 下；历史 v1 运行、模型包和 `docs/reports/windows_original/` 保持原样。
- v1 与 v2 的观测、保护和终止语义不同。旧权重只作历史基线，或通过 `--init-from` 显式初始化并记录来源；不能普通 `--resume`。
- 正式依赖：Python 3.10.22、torch 2.8.0+cpu、SB3 2.9.2、Gymnasium 1.4.0、NumPy 2.2.6、SciPy 1.15.3、Numba 0.63.1。
- 所有 6 个主实验运行使用相同 `env.py` 哈希 `a97ea167…`、`config.py` 哈希 `f9bae0d6…`、spec 哈希 `ab2684a1…`；训练源快照在各自 `source/`。

## 2 六项正确性修复

| 问题 | 处理 | 验证 |
|---|---|---|
| 部分激光缺失仍放行 | `scan.py` 统一分类；`control.py` 按运动方向和制动旋转扫掠角域检查未知；前/后缺测分别禁止前进/倒车，纯旋转要求全周 | 前缺后有效、后缺前有效、全无回波、NaN、超量程样例；`test_scan_safety.py` |
| 超量程被当空闲 | 只有 `+inf` 是无回波；有限值必须在 `[min,max]`；超量程/低于量程/NaN/-inf 均为未知；BEV 与保护共用 `classify_scan` | `above_max` fixture、BEV 未知栅格、保护拦截 |
| 评估配置不一致 | `resolve_evaluation_config` 一次写入 stage/stages/no-safety/seed set；执行、目录名、`evaluation_config.yaml`、`evaluation_metadata.json` 共用 effective config | 配置开/关 × `--no-safety` 单元组合；评估 CLI 实测保存 false/true 与执行一致 |
| 看门狗依赖仿真时间 | 节点和 watchdog 使用 STEADY_TIME；模拟时钟只用于倒退检测；重复/旧观测拒绝；推理 token 与耗时检查；重置清空并停车 | `test_watchdog.py` 覆盖暂停、倒退、重复、推理超时；ROS 实际调度未运行 |
| 导出混用源码和成绩 | `export_bundle.py` 在隔离子进程只使用 run 的 `source/navrl`；记录权重 SHA256、步数、快照哈希；final 不含 best，best 只在哈希匹配时绑定 | `test_export.py`；旧 run 实际导出与 `verify_bundle.py` 通过 |
| 转圈逃过卡死 | 移除角速度豁免；持续无进展计时；朝局部目标且航向误差下降时最多 `turn_grace_seconds`，进展恢复后重置 | 持续旋转、偶尔旋转、正常大角度转向、正常进展；`test_stuck.py` |

版本化：`spec_version=bev_nav_v2`；旧配置加载时若用于新训练会被拒绝。

## 3 工程质量

- `fixtures.py`、`geometry.py`、`observation.py` 等关键函数的紧凑多语句行已拆分。
- `validate_config` 覆盖观测、量程、足迹、场景权重、局部目标、奖励、PPO、训练和评估字段范围。
- 续训/初始化兼容检查在创建实验目录前完成；不兼容时不会留下空目录。
- 导出先写 `.tmp/export_*`，隔离校验成功后原子移动到目标；失败清理临时目录。
- 评估逐步记录原始动作、请求速度、执行速度、干预原因和 reward terms；逐回合保存 `trajectory_*.npz`。
- 更新了 README、配置说明、Windows 清单、训练教程、部署说明和 GitHub 导入说明；历史报告不改。

## 4 策略改进

### 4.1 局部目标

`navrl/goal.py` 提供固定前视与自适应前视。自适应按路径转角缩短前视，并在前视范围内回退到与路径走廊一致、足迹可直达的路线点。规则只使用参考路径、位姿和配置，不使用障碍真值；部署参考在 `training/scripts/deployment/route_goal.py`。

### 4.2 场景与比例

新增 `dense`、`tight_turn`、`protective`。`mixed` 使用显式 `stage_weights`，先固定场景类型再重采样可达任务；生成失败不会改变训练比例。`protective` 是 0.8 m 窄走廊。

### 4.3 奖励

默认奖励不变。`reward.intervention` 和 `reward.action_deviation` 默认 0，分别做单因素实验；一次只改变一类因素。

## 5 实验协议

| 集合 | 种子 | 用途 |
|---|---|---|
| diagnostic | `300000 + stage*1000 + i` | 现有 32 个开发诊断任务（4 场景 × 8） |
| validation | `100000 + stage*1000 + i` | 分场景选型，每场景 8 回合 |
| test | `200000 + stage*1000 + i` | 独立测试，每场景 12 回合，配置冻结后使用 |

stage 顺序：`empty, single, turn, clutter, dense, tight_turn, protective`。fixed 与 adaptive 只有 `goal_selection.mode` 不同；网络为 split_mlp、输入 2056 维，PPO 超参数、场景权重、50k 预算和种子 42/43/44 相同。主实验目录：`experiments/yy_v2/runs/`。

## 6 主结果

### 6.1 总体（三个训练种子均值 ± 标准差）

| 集合 | 方案 | 成功率 | 碰撞率 | 平均干预时长 | 请求速度 | 执行速度 |
|---|---|---|---|---|---|---|
| validation | fixed | 35.1% ± 3.0% | 0.0% | 4.97 s | 0.211 m/s | 0.185 m/s |
| validation | adaptive | 19.6% ± 1.5% | 0.0% | 4.77 s | 0.223 m/s | 0.197 m/s |
| test | fixed | 36.5% ± 7.0% | 0.0% | 5.69 s | 0.201 m/s | 0.173 m/s |
| test | adaptive | 14.3% ± 3.5% | 0.0% | 4.50 s | 0.224 m/s | 0.201 m/s |

**结论：本轮 50k 预算下，自适应局部目标没有超过固定 1.2 m 基线，反而在 validation/test 上更低。** 不能据此把 adaptive 作为改进模型；应保留 fixed 作为当前候选，并延长预算或重新设计局部目标规则后再验证。

### 6.2 分场景成功率（validation / test，三个种子均值）

| 场景 | fixed validation | adaptive validation | fixed test | adaptive test |
|---|---|---|---|---|
| empty | 46% | 21% | 44% | 19% |
| single | 54% | 33% | 61% | 14% |
| turn | 33% | 25% | 39% | 11% |
| clutter | 21% | 12% | 33% | 14% |
| dense | 25% | 8% | 28% | 3% |
| tight_turn | 42% | 38% | 39% | 36% |
| protective | 25% | 0% | 11% | 3% |

图：`experiments/yy_v2/summary/comparison_validation.png`、`comparison_test.png`。干预时长和碰撞率在同图的三栏中；安全开启时所有主实验碰撞率为 0。

### 6.3 失败原因

安全开启时失败几乎全部是 `stuck`：validation fixed 109 次、adaptive 135 次；test fixed 160 次、adaptive 216 次。没有碰撞。主要瓶颈不是碰撞，而是策略在局部目标/障碍附近无法持续产生路径进展，最终触发卡死判定。图：`failure_reasons_validation.png`、`failure_reasons_test.png`。

非学习路径跟随器在 7 个场景 × 3 个种子上 21/21 全部成功（`docs/reports/yy_v2/environment_path_follower.json`），说明任务几何和局部目标规则本身不是不可达；PPO 在 50k 预算下尚未学会稳定控制。

### 6.4 请求/执行速度

`speed_comparison.png` 显示请求线速度普遍高于执行线速度，差值与安全干预对应。adaptive 的请求/执行速度均值更高，但成功率更低，不能证明它学会了主动减速或绕行；更像是在更差的行为上保持了更高速度。

### 6.5 安全消融（seed 42，validation 56 回合）

| 模型 | 安全 | 成功率 | 碰撞率 | 干预时长 |
|---|---|---|---|---|
| fixed | 开 | 32.1% | 0.0% | 4.81 s |
| fixed | 关 | 32.1% | 53.6% | 0 s |
| adaptive | 开 | 21.4% | 0.0% | 5.63 s |
| adaptive | 关 | 19.6% | 62.5% | 0 s |

保护过滤显著降低碰撞，成功率没有因过滤而下降（fixed 持平，adaptive 略降但样本小）。这支持“保护有效”，但不等于策略本身改善；过滤后的剩余失败仍是卡死。

### 6.6 开发诊断集（32 任务，seed 42）

| 方案 | 成功 | 碰撞 | 平均干预时长 |
|---|---|---|---|
| fixed | 28.1% (9/32) | 0 | 3.20 s |
| adaptive | 18.8% (6/32) | 0 | 3.29 s |

该集合只用于诊断，不作为最终结论。

### 6.7 奖励单因素实验（30k，seed 42，validation）

| 方案 | 成功率 | 碰撞 | 干预时长 | 说明 |
|---|---|---|---|---|
| adaptive 30k 控制 | 25.0% | 0 | 4.62 s | 同预算对照 |
| + 持续干预惩罚 | 33.9% | 0 | 4.95 s | 成功率更高但干预时长未下降，且请求速度更高；单种子，不能证明主动减速 |
| + 动作偏差惩罚 | 25.0% | 0 | 6.54 s | 成功率不变、干预更长；未改善 |

单种子结果方差大，不能据此确定奖励改动有效。下一轮应使用三种子并同时检查速度曲线、碰撞与干预，不能只挑成功率。

### 6.8 训练曲线

`experiments/yy_v2/summary/training_curves.png` 显示每 10k 步固定验证的成功/碰撞率（mean 与 min-max）。50k 预算下曲线仍明显波动，固定方案在多个检查点高于 adaptive。

## 7 测试与流程记录

- `pytest training/tests/unit`：**63 passed**，耗时约 178 s（正式 Python 3.10 环境）。
- `run.py doctor`：Gymnasium `check_env` 通过，观测 2056 维，无 CUDA。
- 2,048 步 SubprocVecEnv 冒烟训练完成；7 个场景均可生成可达任务。
- 旧 v1 模型普通 `--resume` 被拒绝且未创建目录；`--init-from` 成功并写 `initialization.json`。
- 旧 run 的 `final_model` 导出成功，`verify_bundle.py` 通过；final 未携带 best 成绩。
- 评估 CLI 的 `--no-safety` 与模型配置组合实测；保存配置与 `effective_safety` 一致。
- 未运行：Ubuntu rclpy/STEADY_TIME 实际调度、C++ BEV 对照、safety_node、Gazebo 接触、真实雷达外参、底盘响应。ROS 代码只做了 Python 语法/单元级检查。

## 8 模型包

| 包 | 类型 | 权重步数 | 权重 SHA256 前缀 | 成绩绑定 |
|---|---|---|---|---|
| yy_v2_fixed_best_s42 | best | 20,000 | 68340bc364d2f445 | 是 |
| yy_v2_fixed_final_s42 | final | 51,200 | fec223fb624eb171 | final 自身验证，不含 best |
| yy_v2_adaptive_best_s42 | best | 40,000 | 8d4d660fe140680b | 是 |
| yy_v2_adaptive_final_s42 | final | 51,200 | 1d731f0c6ca62aa7 | final 自身验证，不含 best |

位置：`experiments/yy_v2/exports/`。每个包包含 `export_manifest.json`、`sha256.json`、快照 `navrl/`、fixtures、固定推理参考和看门狗/局部目标部署参考。`verify_bundle.py` 已在 Windows 通过。

## 9 限制与下一步

1. 50k 是短预算；绝对成功率低，结论只适用于该预算。下一步先冻结 v2 语义，再用更长预算（如 150k–300k）和三种子复验。
2. 自适应局部目标需要重新设计或调参；本轮 negative result 不支持替换 fixed。
3. 奖励实验只跑了单种子，不能作为结论；至少三种子后再判断。
4. 主要失败是卡死而非碰撞；后续应分场景分析局部目标、保护停车和恢复策略，而不是单纯放宽安全阈值。
5. 所有结果均为二维；虚拟机与 Gazebo 必须在 Ubuntu 重新验证，不能把本页成功率当作部署成绩。
