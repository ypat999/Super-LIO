#!/usr/bin/env bash
# 清理上一轮实验残留（不碰其它用户进程）：只杀本仓库 exp 起的 node / bag play
set -u
pkill -f "exp/results/m40_" 2>/dev/null
pkill -f "ros2 bag play /home/ywj/slam_data/livox_record" 2>/dev/null
pkill -f "super_lio_node --ros-args --params-file /home/ywj/git/Super-LIVO" 2>/dev/null
sleep 2
pgrep -fa "super_lio_node --ros-args" | head -5
pgrep -fa "ros2 bag play" | head -5
echo "[clean] done"
