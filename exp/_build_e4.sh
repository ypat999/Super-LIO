#!/usr/bin/env bash
# 独立目录编译 E-4 改动（不覆盖 /tmp/sl_install，避免影响正在跑的 E-2 批次）
set +u
source /opt/ros/humble/setup.bash
source /home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO
colcon build --paths src/basic src/super_lio \
  --build-base /tmp/sl_build_e4 --install-base /tmp/sl_install_e4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/e4_build.log 2>&1
echo "rc=$?"
grep -E "error|Error" /tmp/e4_build.log | head -20
tail -3 /tmp/e4_build.log
