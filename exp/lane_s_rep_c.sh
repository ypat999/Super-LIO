#!/usr/bin/env bash
# M.34 补充格（lane C）：1f 基线复现 + 楼梯 merge 独立效应
#   rep0_1f 首轮带 odom_gap=0.40s 污染标记 ⇒ 基线闭环 2.844 不可信，必须复跑
#   rep0+merge0.25（楼梯）：分离"merge 放宽"与"rep_mode=1"各自的贡献
#     （首轮 B2 崩到 1.690 m，但没有 rep0+merge0.25 这格就无法归因）
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=63
export WS_SETUP=/tmp/sl_install/setup.bash

BAG=/home/ywj/slam_data/livox_record_drone_1f MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep0_1f_r2 6 0.25 0.01 0.1 0.25

BAG=/home/ywj/slam_data/livox_record_drone MAP_REP_MODE=0 MAP_MERGE_DIST=0.25 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep0m25_stair 6 0.25 0.3 0.1 0.25

echo "[laneS-rep-c] done"
