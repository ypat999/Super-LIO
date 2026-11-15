#!/usr/bin/env bash
# M.40 E-4：Observe 槽扫描优化（占用位图 + AABB 距离上界剪枝）的守门回归。
#   同一份生产参数、同一条 noshell bag 前 330 s，只换二进制：
#     m40_e4_base = /tmp/sl_install     （未优化，14:03 前编译）
#     m40_e4_opt  = /tmp/sl_install_e4  （占用位图 + aabbDist2 剪枝）
# 判据：Observe 耗时下降，且 deg_cond 分布 / 轨迹 / 地图点数不变坏。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export BAG=/home/ywj/slam_data/livox_record_noshell
export ROS_DOMAIN_ID=84
# run_one.sh 自己不设 RMW；本机 FastDDS 跨进程发现失效（lane_e2/p2 都显式用 Cyclone），
# 漏掉这一行会让节点与播放端完全互相发现不到（imu_buf 恒 0）。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# WSL 里同时存在 enP4353p0s0 / loopback0 / docker0 等多个可组播网卡，
# Cyclone 的自动选卡会让节点与播放端落到不同接口上（表现为 imu_buf 恒 0）。
# 固定到 LAN 网卡，两个进程必然同接口。
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
# M.39 生产定稿（与 m40_e2_* 同口径，便于跨实验比较）
export SUB_DIV=4
export PLANE_KNN_K=8
export TRUST_JUMP_ROT_MAX=25.0
export DEGENERACY_DETECT_EN=true
export NORMAL_WEIGHT_EN=true
export NORMAL_SIGMA=0.035
export HUBER_BASE=0.20
export IMU_NBG=0.003
export DEBUG_LEVEL=2

./run_one.sh m40_e4_base 6 0.25 0.3 0.1 0.25 330
WS_SETUP=/tmp/sl_install_e4/setup.bash ./run_one.sh m40_e4_opt 6 0.25 0.3 0.1 0.25 330
echo "[lane_e4] all done"
