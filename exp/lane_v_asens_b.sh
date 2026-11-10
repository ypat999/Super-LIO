#!/usr/bin/env bash
# M.36 楼梯 A 线重定标（lane B）：nw 轴与 nbg 轴，K=8 固定
#   与 lane A 同一基准（m35_k8fix_rep0_stair，0.562 m）可归因。
#   nwoff / nw σ0.05：旧代码上 σ0.035→0.05 就是 53x 退化（M.28），
#     本轮看这条"刀刃"在当前代码上是否还这么陡——若不陡了，说明
#     新代码的平衡点已经换到别处，那 A 线重定标要按新曲率重扫。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=86
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone
export PLANE_KNN_K=8 MAP_REP_MODE=0

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=false \
  ./run_one.sh m36_as_nwoff 6 0.25 0.3 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05 \
  ./run_one.sh m36_as_nw05 6 0.25 0.3 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.0001 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_as_nbg01 6 0.25 0.3 0.1 0.25

echo "[laneV-asens-b] done"
