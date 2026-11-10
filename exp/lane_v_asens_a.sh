#!/usr/bin/env bash
# M.36 楼梯 A 线重定标（lane A）：单轴敏感度，K=8 固定
#   基准格复用 M.35 的 m35_k8fix_rep0_stair（na0.3 nbg0.003 huber0.20 nw σ0.035）
#   = 闭环 #2 0.562 m；同 bag 同代码，可直接逐轴比较。
#   之所以要重扫：64c217d→当前代码让同配置从 0.023 退化到 1.030(K=5)/0.562(K=8)，
#   M.26/M.28 在旧代码上定的最优轴排序可能已经变了。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=85
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone
export PLANE_KNN_K=8 MAP_REP_MODE=0

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_as_na01 6 0.25 0.1 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_as_na001 6 0.25 0.01 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.05 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_as_h005 6 0.25 0.3 0.1 0.25

echo "[laneV-asens-a] done"
