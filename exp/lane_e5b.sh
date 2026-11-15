#!/usr/bin/env bash
# M.40 E-5 补格：把 A/B 线拆成 2x2，分离 imu_na 与 normal_weight 各自对 1f 竖直漂移的贡献。
#   已跑：A = na0.3 + nw on  |  B = na0.01 + nw off
#   本脚本串行补两侧：C = na0.01 + nw on，D = na0.3 + nw off
# 串行执行（历史上并发两路会把播包饿死 / 让 LIO 停摆）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

export BAG=/home/ywj/slam_data/livox_record_drone_1f
export ROS_DOMAIN_ID=88
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
export SUB_DIV=4 PLANE_KNN_K=8 TRUST_JUMP_ROT_MAX=25.0 DEGENERACY_DETECT_EN=true
export HUBER_BASE=0.20 IMU_NBG=0.003 DEBUG_LEVEL=2

export NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035
./run_one.sh m40_e5_1f_c_na01_nwon 6 0.25 0.01 0.1 0.25 0

export NORMAL_WEIGHT_EN=false
./run_one.sh m40_e5_1f_d_na03_nwoff 6 0.25 0.3 0.1 0.25 0
