#!/usr/bin/env bash
# M.34 地图代表点策略（rep_mode）复测 —— lane A：基线 + 只翻 rep_mode
#   B0 = rep_mode=0（增量均值，论文默认）：新代码基线锚点
#   B1 = rep_mode=1（只存真实点，取离子体素中心最近者）
# 之所以要重跑 B0：3c9cec6 把 /lio/odom 换到 base_link 系，
# 历史 results 里的 odom.csv 与当前代码不同系，md5 不可跨时代引用。
# 楼梯用 A 线参数，1f 用 B 线参数（与 m26_h20_na03 / f1_bline_nbg3 同）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=61
export WS_SETUP=/tmp/sl_install/setup.bash

STAIR_BAG=/home/ywj/slam_data/livox_record_drone
F1_BAG=/home/ywj/slam_data/livox_record_drone_1f

# ---- 楼梯（A 线）----
BAG=$STAIR_BAG MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep0_stair 6 0.25 0.3 0.1 0.25

BAG=$STAIR_BAG MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep1_stair 6 0.25 0.3 0.1 0.25

# ---- 1f（B 线）----
BAG=$F1_BAG MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep0_1f 6 0.25 0.01 0.1 0.25

BAG=$F1_BAG MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep1_1f 6 0.25 0.01 0.1 0.25

echo "[laneS-rep-a] done"
