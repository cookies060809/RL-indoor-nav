# 将 Windows 模型交给 Ubuntu 虚拟机

## 1 在 Windows 导出

```powershell
cd E:\ROS2
.\run.cmd export --model runs\my_empty_s42\best_model.zip --name my_empty_s42_v1
```

得到 `exports/my_empty_s42_v1.zip`。用户要求所有工作位于 ROS2 文件夹，因此脚本不自动写入 share_file。由你把这个 zip 复制到共享文件夹，再在 Ubuntu 解压至本地目录，例如 `~/ROS2_models/my_empty_s42_v1`。不要复制 `.venv` 或 Windows Python，Linux 需要自己的环境。

导出包包含 policy.zip、完整配置、库版本、观测规格、源码、10 个观测样例、固定推理输入输出、文件 SHA256 和策略推理耗时。`inference_latency.json` 只测神经网络，不包含 TF、C++ BEV 或 ROS 通信。

## 2 在虚拟机建立 Python 环境

以下是需要你在 Ubuntu 执行的步骤，Windows 未执行：

```bash
source /opt/ros/humble/setup.bash
python3.10 -m venv --system-site-packages ~/rl_venv
source ~/rl_venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.8.0 --index-url https://download.pytorch.org/whl/cpu
cd ~/ROS2_models/my_empty_s42_v1
python -m pip install -r requirements-core-lock.txt
python -m pip check
python deployment/verify_bundle.py
```

Linux 使用 core 锁文件，不使用 Windows 完整锁文件。若系统 numpy 或其他包影响解析，记录 `pip check` 的结果再处理，不混用 sudo pip。不设置 VecNormalize，不额外除以 255，不把 BEV 当作 uint8 图像。

出现 `PASS: manifest hashes and deterministic inference reference` 表示模型包和固定观测推理通过；还需看到 rclpy 与 SB3 同时可导入。1e-5 浮点容差用于比较，不能要求不同系统逐位相同。

## 3 对齐 C++ BEV

将 `PolicyObservation.msg` 放入 ROS 工作空间中的 `project_msgs/msg`，配置 rosidl 后编译。完整消息是 `[2×H×W BEV, 4维goal, 4维motion]`，float32，共 2056 或 4616 维。

参考实现是导出包的 `navrl/observation.py` 和每个 `fixtures/<案例>.json`：JSON 中用字符串 `inf`、`-inf`、`nan` 表示非有限测距，C++ 解析时必须转换回浮点语义。每个案例同时给出外参、扫描、目标和期望观测。

让 C++ 程序对每个案例输出 `<案例>.json`，结构为 `{"observation": [浮点数组]}`，然后在 Windows 源项目执行：

```powershell
.\.venv\Scripts\python.exe scripts\check_fixture_outputs.py E:\ROS2\cpp_outputs
```

`cpp_outputs` 应先由虚拟机导出并复制至项目目录。没有真实 C++ 输出时不能标记“两端一致性通过”。如果实际 LaserScan 角度区间、端点或束数与参考 `[-π,π)` 的 360 束不同，应先重采样到冻结规格，或共同修改两端规格并重新训练；不能直接按数组长度相同就拼接。

`navrl.observation.canonicalize_scan` 提供角度重采样的 Python 参考：最近束映射，不插值 NaN/inf，未观测角域仍保留 NaN。C++ 端按相同规则实现。48 格模型应使用其导出包内的 fixtures，而不是项目根目录早期生成的 32 格样例。

## 4 策略节点与控制链

`policy_node.py` 是可移入 ament_python 包的节点源文件，需要虚拟机安装 rclpy、geometry_msgs 和已编译的 project_msgs。此文件在 Windows 只做语法检查，尚未在 ROS 运行。

节点参数 `model_path` 指向包内 `policy.zip`，`use_sim_time` 设置为 true。订阅 `/policy_observation`，仅发布 `/cmd_vel_rl`。最终 `/cmd_vel` 必须由虚拟机的 safety_node 独占。

它会检查规格版本、尺寸、分辨率、有限值、数组范围、base_footprint 坐标系和 0.5 秒仿真时间的新鲜度，并设置墙钟看门狗。低实时率条件下需要验证 `wall_timeout`；不要为掩盖真正失联而无限延长超时。

Windows 没有完成 C++ 安全节点、Nav2 速度链改造、route_manager、生命周期管理或 Gazebo 实体重置。请按综合版第 4、7、8 章在虚拟机实现并实测，这些工作仍在待办清单。

## 5 部署前必须核实的参数

- 从 TF 测量雷达的平面 x、y、yaw；高度 0.15 m 不能直接当作 x 偏移。
- 校验机器人碰撞体，圆形半径 0.226 m 只是当前训练近似。
- 确认禁止 vy 与倒车，限速为 0.26 m/s 和 1 rad/s。
- 测试底盘实际速度响应、加速度与控制频率。
- 相同输入上的 C++/Python 栅格一致后，再检查 Gazebo 实际传感器与二维扫描差异。
- 若外参、足迹、控制周期或 BEV 规格发生变化，修改配置并重新训练、生成样例和导出；不要只在虚拟机单方面改。

部署验证顺序：加载 → 固定观测推理 → C++ 观测比对 → 空场景 → 单障碍 → 转角 → 完整全局路径 → 正式对照实验。任何层失败先排查该层，不直接延长 PPO 训练。
