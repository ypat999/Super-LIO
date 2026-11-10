#!/usr/bin/env bash
# M.34 补充格（lane D）：1f rep_mode=1 复现 + 楼梯 max_points 独立效应
#   rep1_1f 首轮 2.339 vs 基线 2.844（-17.8%）；基线带 odom_gap 污染，
#   且同一 B 线历史上 run-to-run 就有 3.051/2.976（-2.5%）的波动 ⇒ 必须复跑才能判真伪
#   rep0+max_points 254（楼梯）：检验"20 点冻结"这个门限本身是不是 A 线的支柱
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=64
export WS_SETUP=/tmp/sl_install/setup.bash

BAG=/home/ywj/slam_data/livox_record_drone_1f MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep1_1f_r2 6 0.25 0.01 0.1 0.25

BAG=/home/ywj/slam_data/livox_record_drone MAP_REP_MODE=0 MAP_MAX_POINTS=254 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep0mp_stair 6 0.25 0.3 0.1 0.25

echo "[laneS-rep-d] done"
