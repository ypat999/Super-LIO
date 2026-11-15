#!/usr/bin/env bash
# M.40 E-5：1f 水平残差（2.87 m）的 A/B 线参数对照。
#   1f 只有 /livox/imu + /livox/lidar，没有相机 ⇒ 视觉 A/B 线不可用，
#   这里退化为"两条生产参数线"的干净消融（同二进制、同 bag、只换参数）。
#   A 线（已跑）：na0.3 / ng0.1 / nbg0.003 / huber0.20 / nw on σ0.035
#   B 线（本脚本）：na0.01 / ng0.1 / nbg0.003 / huber0.20 / nw off
# run_one.sh 自己不设 RMW；本机 FastDDS 跨进程发现失效，必须显式用 Cyclone 并锁网卡。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

export BAG=/home/ywj/slam_data/livox_record_drone_1f
export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
export SUB_DIV=4 PLANE_KNN_K=8 TRUST_JUMP_ROT_MAX=25.0 DEGENERACY_DETECT_EN=true
export HUBER_BASE=0.20 IMU_NBG=0.003 DEBUG_LEVEL=2
export NORMAL_WEIGHT_EN=false NORMAL_SIGMA=0.05

./run_one.sh m40_e5_1f_bline 6 0.25 0.01 0.1 0.25 0
