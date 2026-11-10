#!/usr/bin/env bash
# M.35 K 矩阵（lane A，rep_mode=0 一侧）：K∈{5,8} × 两 bag
#   代码 = 67c0acd + 两项改动：
#     (a) g_plane_knn_k 运行时化（已验收：K=8 与 m34b_rep0_stair 逐字节相同）
#     (b) 修 KNNHeap::update_worst 只扫 dist2_[0..4] 的缺陷
#         —— 该缺陷是 67c0acd 把容量 5→8 时漏改的，K=8 下堆不再维护"最近 8 个"
#   本 lane 的 m35_k5_rep0_stair / _1f 同时是 (b) 的等价性验收：
#   K=5 时索引 5..7 永不被写入（保持 memset 的 0），扫描 0..7 与旧
#   KNNHeap<5> 的 unrolled 结果逐位相同 ⇒ odom.csv md5 必须分别等于
#   exp/results/m34_rep0_stair 与 exp/results/m34_rep0_1f_r2（同为 K=5、同参数线）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=75
export WS_SETUP=/tmp/sl_install/setup.bash

STAIR_BAG=/home/ywj/slam_data/livox_record_drone
F1_BAG=/home/ywj/slam_data/livox_record_drone_1f

BAG=$STAIR_BAG PLANE_KNN_K=5 MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m35_k5_rep0_stair 6 0.25 0.3 0.1 0.25

BAG=$STAIR_BAG PLANE_KNN_K=8 MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m35_k8fix_rep0_stair 6 0.25 0.3 0.1 0.25

BAG=$F1_BAG PLANE_KNN_K=5 MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m35_k5_rep0_1f 6 0.25 0.01 0.1 0.25

BAG=$F1_BAG PLANE_KNN_K=8 MAP_REP_MODE=0 \
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m35_k8fix_rep0_1f 6 0.25 0.01 0.1 0.25

echo "[laneU-k58-a] done"
