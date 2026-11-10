#!/usr/bin/env bash
# M.35 K 矩阵（lane B，rep_mode=1 一侧）：K∈{5,8} × 两 bag
#   与 lane A 同一二进制，只差 MAP_REP_MODE ⇒ 可直接做 K×rep_mode 交互判读。
#   K=5 两格是等价性验收：md5 必须分别等于 exp/results/m34_rep1_stair
#   与 exp/results/m34_rep1_1f（同为 K=5、rep_mode=1、同参数线）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=76
export WS_SETUP=/tmp/sl_install/setup.bash

STAIR_BAG=/home/ywj/slam_data/livox_record_drone
F1_BAG=/home/ywj/slam_data/livox_record_drone_1f

BAG=$STAIR_BAG PLANE_KNN_K=5 MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m35_k5_rep1_stair 6 0.25 0.3 0.1 0.25

BAG=$STAIR_BAG PLANE_KNN_K=8 MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m35_k8fix_rep1_stair 6 0.25 0.3 0.1 0.25

BAG=$F1_BAG PLANE_KNN_K=5 MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m35_k5_rep1_1f 6 0.25 0.01 0.1 0.25

BAG=$F1_BAG PLANE_KNN_K=8 MAP_REP_MODE=1 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m35_k8fix_rep1_1f 6 0.25 0.01 0.1 0.25

echo "[laneU-k58-b] done"
