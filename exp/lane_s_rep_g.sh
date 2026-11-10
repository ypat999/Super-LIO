#!/usr/bin/env bash
# M.34 §9 新基线复验（lane G，1f）：exp 已前进到 67c0acd（平面拟合邻域 K=5→8、
#   beam_err=0 关闭距离自适应方差），§3 的数值是在 64c217d 上取得的，
#   必须在新基线重取 B0/B1 两格，才能判断"rep_mode 在 1f 上 -23%"是否仍成立。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=69
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false

MAP_REP_MODE=0 ./run_one.sh m34b_rep0_1f 6 0.25 0.01 0.1 0.25
MAP_REP_MODE=1 ./run_one.sh m34b_rep1_1f 6 0.25 0.01 0.1 0.25

echo "[laneS-rep-g] done"
