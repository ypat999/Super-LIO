#!/usr/bin/env bash
# M.34 §9 新基线复验（lane H，楼梯 A 线）：见 lane_s_rep_g.sh 头部说明。
#   楼梯侧要验证的是更关键的判断——"K=8 本身是不是也在抑制均值抖动，
#   从而改变 A 线对地图改动的敏感度"。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=70
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone
export HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035

MAP_REP_MODE=0 ./run_one.sh m34b_rep0_stair 6 0.25 0.3 0.1 0.25
MAP_REP_MODE=1 ./run_one.sh m34b_rep1_stair 6 0.25 0.3 0.1 0.25

echo "[laneS-rep-h] done"
