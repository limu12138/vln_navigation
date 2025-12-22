#!/usr/bin/env bash
# set -euo pipefail

# 修改为你系统的 ROS 发行版（如果不是 humble 请改为相应名字）
ROS_DISTRO=humble

echo "1) 清理旧构建/安装产物..."
rm -rf build install log

echo "2) 使用系统 ROS 环境..."
source /opt/ros/${ROS_DISTRO}/setup.bash

echo "3) 安装 rosdep 依赖（可能需要 sudo）..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y || echo "rosdep install finished (some optional deps may be skipped)"

echo "4) 按依赖顺序构建 navigation2 及上游依赖..."
colcon build --packages-up-to navigation2

echo "构建完成。记得 source install/setup.bash："
echo "  source install/setup.bash"
