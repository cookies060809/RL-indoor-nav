# 策略推理包

已将既有 policy_node.py 放入 ament_python 包结构，供虚拟机编译。Windows 只验证 Python 语法和包描述，尚未执行 ROS 编译或运行。

节点只输出 `/cmd_vel_rl`。需要有效的 PolicyObservation 发布者和独立安全节点，才能连接到底盘。

package.xml 的维护者邮箱与 license 是占位值，课程提交前由小组填写实际信息。
