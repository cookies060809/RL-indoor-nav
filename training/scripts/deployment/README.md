# 模型导出辅助文件

本目录保留策略节点、看门狗状态机、消息源文件与模型包校验脚本。`run.py export` 会把本目录复制到新模型包；已有 v1 模型包内的旧副本保持原样。

- `policy_node.py`：STEADY_TIME 看门狗、重置清空、推理 token/超时检查；只发布 `/cmd_vel_rl`。
- `watchdog.py`：不依赖 rclpy 的新鲜度状态机，可在 Windows 单元测试。
- `verify_bundle.py`：校验 SHA256、`export_manifest.json` 权重绑定、final 不携带 best 成绩，并比对固定推理输出。
- `PolicyObservation.msg`：观测消息源文件。

导出包内的 `export_manifest.json` 记录所选权重文件、SHA256、`num_timesteps`、源码快照哈希和成绩绑定状态。旧运行若没有权重哈希，best 成绩不会被标为已绑定。v2 观测语义为：量程内有限值 = 命中，`+inf` = 无回波，其余 = 未知；C++ 端必须一致。

当前 ROS 工作空间中的正式副本在 `ros2_ws/src/project_msgs` 和 `ros2_ws/src/rl_nav_pkg`。Ubuntu 导入和运行步骤见项目根目录 `docs/虚拟机导入步骤.md`。
