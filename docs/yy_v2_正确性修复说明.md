# yy / bev_nav_v2 正确性修复说明

日期：2026 年 11 月 15 日。分支：`yy`。本页只记录六项正确性修复和工程质量改动；策略保持原有固定 1.2 m 前视、四场景课程和原奖励函数。

## 1 六项修复

| 问题 | 处理 | 验证 |
|---|---|---|
| 部分激光缺失仍放行 | `scan.py` 统一分类；`control.py` 按运动方向和制动旋转扫掠角域检查未知；前/后缺测分别禁止前进/倒车，纯旋转要求全周 | 前缺后有效、后缺前有效、全无回波、NaN、超量程；`test_scan_safety.py` |
| 超量程被当空闲 | 只有 `+inf` 是无回波；有限值必须在 `[min,max]`；超量程/低于量程/NaN/-inf 均为未知；BEV 与保护共用分类 | `above_max` fixture、BEV 未知栅格、保护拦截 |
| 评估配置不一致 | `resolve_evaluation_config` 一次写入 stage/stages/no-safety/seed set；执行、目录名、保存配置和 metadata 共用 | 配置开/关 × `--no-safety` 组合；评估 CLI 实测 |
| 看门狗依赖仿真时间 | 节点和 watchdog 使用 STEADY_TIME；仿真时钟只用于倒退检测；重复/旧观测拒绝；推理 token 与耗时检查；重置清空并停车 | `test_watchdog.py`；真实 ROS 调度未运行 |
| 导出混用源码和成绩 | 隔离子进程只使用 run 的 `source/navrl`；记录权重 SHA256、步数、快照哈希；final 不含 best，best 只在哈希匹配时绑定 | `test_export.py`；旧 run 实际导出与 `verify_bundle.py` 通过 |
| 转圈逃过卡死 | 移除角速度豁免；持续无进展计时；朝局部目标且航向误差下降时最多 `turn_grace_seconds`，进展恢复后重置 | 持续旋转、偶尔旋转、正常大角度转向、正进展重置；`test_stuck.py` |

版本化：`spec_version=bev_nav_v2`。旧配置用于新训练会被拒绝；旧权重只能 `--init-from` 显式初始化并记录来源。

## 2 工程质量

- 关键函数中的多语句行已拆分。
- `validate_config` 覆盖观测、量程、足迹、任务、PPO、训练和评估字段范围。
- 续训/初始化兼容检查在创建实验目录前完成。
- 导出先写临时目录，隔离校验成功后原子移动。
- 评估逐步记录原始动作、请求速度、执行速度、干预原因和 reward terms。
- 当前操作文档、完整性清单已同步；历史报告不改。

## 3 测试

- `pytest training/tests/unit`：55 项通过。
- `verify_project.py --models`：文件哈希和旧模型包通过。
- 部署文件通过 `py_compile`。
- 未运行：Ubuntu rclpy/STEADY_TIME 实际调度、C++ BEV 对照、safety_node、Gazebo 接触、真实雷达外参和底盘响应。

所有二维结果只代表当前 CPU 环境，不能当作 Gazebo 或部署成绩。