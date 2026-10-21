#!/usr/bin/env bash
# 编译最新源码到 /tmp/sl_install（run_one.sh 默认从这里 source）。
# /tmp 重启后会丢，重跑实验前先执行一次本脚本。
set -e
WS=/home/ywj/git/xtd2_ws

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

cd "$WS"
colcon build --packages-select super_lio \
  --build-base /tmp/sl_build --install-base /tmp/sl_install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[build_ws] done -> /tmp/sl_install/setup.bash"
