#!/usr/bin/env bash
# M.34 地图代表点策略（rep_mode）复测 —— lane B：两个"更彻底"的变体
#   B2 = rep_mode=1 + merge_dist 0.1 -> 0.25
#        （merge 门控在 rep_mode 判断之前：0.1 m 内才允许替换，
#          放宽到体素尺寸才真正做到"邻域内挑离中心最近的真实点"）
#   B3 = rep_mode=1 + max_points 20 -> 254
#        （去掉"一个子体素只融合前 20 个点就冻结"的上限，
#          检验"陈旧代表点"假设；不能填 255，count 为 uint8_t，
#          255+1 溢出回 0 = UNINIT_MASK，会让该格被当作空重新写入）
# 与 lane A 的 B1 同一代码同一 bag，可归因。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=62
export WS_SETUP=/tmp/sl_install/setup.bash

STAIR_BAG=/home/ywj/slam_data/livox_record_drone
F1_BAG=/home/ywj/slam_data/livox_record_drone_1f

# ---- 楼梯（A 线）----
BAG=$STAIR_BAG MAP_REP_MODE=1 MAP_MERGE_DIST=0.25 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep1m25_stair 6 0.25 0.3 0.1 0.25

BAG=$STAIR_BAG MAP_REP_MODE=1 MAP_MAX_POINTS=254 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m34_rep1mp_stair 6 0.25 0.3 0.1 0.25

# ---- 1f（B 线）----
BAG=$F1_BAG MAP_REP_MODE=1 MAP_MERGE_DIST=0.25 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep1m25_1f 6 0.25 0.01 0.1 0.25

BAG=$F1_BAG MAP_REP_MODE=1 MAP_MAX_POINTS=254 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m34_rep1mp_1f 6 0.25 0.01 0.1 0.25

echo "[laneS-rep-b] done"
