#!/usr/bin/env bash
# M.25 阶段 2：新机制 A/B/D/E（lane D，楼梯 bag，ROS_DOMAIN_ID=44）
# 用带阶段 2 代码的独立安装前缀 /tmp/sl_install2（默认全关 = 原行为）
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=44
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install2/setup.bash

# 对照：阶段 2 二进制 + 全默认，应与 m25_base 完全一致
                 ./run_one.sh s2_base     6 0.25 0.1 0.1 0.25

# 2.A 地面/重力软约束
GROUND_WEIGHT=1000.0  ./run_one.sh s2_ground1k  6 0.25 0.1 0.1 0.25
GROUND_WEIGHT=10000.0 ./run_one.sh s2_ground10k 6 0.25 0.1 0.1 0.25

# 2.B 观测噪声按距离建模
OBS_DIST_K=0.02 ./run_one.sh s2_obsd002 6 0.25 0.1 0.1 0.25
OBS_DIST_K=0.05 ./run_one.sh s2_obsd005 6 0.25 0.1 0.1 0.25

# 2.D Huber 阈值随 |omega| 自适应
THRESH_MOTION_K=0.5 ./run_one.sh s2_thm05 6 0.25 0.1 0.1 0.25

# 2.E 按点时刻分配 ΔT
DELTA_INTERP_EN=true ./run_one.sh s2_dint 6 0.25 0.1 0.1 0.25

echo "[laneD] all done"
