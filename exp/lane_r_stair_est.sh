#!/usr/bin/env bash
# M.32 估计（楼梯，A 线参数，calib_td 开、apply 关）
# 目的：① 读 [TdCalib] 收敛/锁定过程；② 确认"只估计不应用"时 odom 与锚点
#       m36_td_anchor_stair 逐字节相同（侧车不得反馈进滤波）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=93
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=true CALIB_TD_APPLY=false \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
./run_one.sh m37_td_est_stair 6 0.25 0.3 0.1 0.25

echo "[laneR-stair-est] done"
echo "== TdCalib =="
grep -h "TdCalib" results/m37_td_est_stair/*.log 2>/dev/null | tail -12
echo "== md5（应与 m36_td_anchor_stair 相同）=="
md5sum results/m37_td_est_stair/odom.csv | cut -d' ' -f1
md5sum results/m36_td_anchor_stair/odom.csv | cut -d' ' -f1
