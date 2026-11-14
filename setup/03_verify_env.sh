#!/usr/bin/env bash
# ROS 2 环境自检：发行版、依赖包、话题通信、Gazebo
# 用法：bash setup/03_verify_env.sh
set -uo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${HOME}/ros2_ws/install/setup.bash"

fail=0
check() {
  local name="$1"; shift
  if "$@" >/dev/null 2>&1; then
    echo "[ OK ] ${name}"
  else
    echo "[FAIL] ${name}"
    fail=1
  fi
}

[[ -f "${ROS_SETUP}" ]] || { echo "未安装 ROS 2 Humble"; exit 1; }
# shellcheck disable=SC1090
source "${ROS_SETUP}"
[[ -f "${WS_SETUP}" ]] && source "${WS_SETUP}"

echo "ROS_DISTRO=${ROS_DISTRO:-<未设置>}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<未设置>}"
echo

echo "== 核心软件包 =="
for pkg in rclcpp rclpy turtlesim rviz2 nav2_bringup slam_toolbox xacro \
           gazebo_ros gazebo_ros2_control cv_bridge wpr_simulation2; do
  check "包 ${pkg}" ros2 pkg prefix "${pkg}"
done

echo
echo "== Gazebo =="
check "gazebo 可执行文件" which gazebo
gazebo --version 2>/dev/null | head -1

echo
echo "== 话题通信（talker -> listener） =="
tmp="$(mktemp -d)"
timeout 15 ros2 run demo_nodes_cpp listener >"${tmp}/listener.log" 2>&1 &
listener_pid=$!
sleep 2
timeout 8 ros2 run demo_nodes_cpp talker >"${tmp}/talker.log" 2>&1 || true
sleep 1
if grep -q "I heard: \[hello world" "${tmp}/listener.log"; then
  echo "[ OK ] talker/listener 通信正常"
else
  echo "[FAIL] talker/listener 通信失败"
  sed -n '1,20p' "${tmp}/listener.log"
  fail=1
fi
kill "${listener_pid}" 2>/dev/null || true
rm -rf "${tmp}"

echo
if [[ ${fail} -eq 0 ]]; then
  echo "环境自检通过。"
else
  echo "存在失败项，请根据上面的输出排查。"
fi
exit "${fail}"
