#!/usr/bin/env bash
# M.32 估计复验（1f，B 线，calib_td 开、apply 关）
# 目的：用修正后的锁窗口（300 帧/30 s、tol 1 ms）复验锁定行为与估计量级。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=96
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=true CALIB_TD_APPLY=false \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=false \
./run_one.sh f1_td_est2 6 0.25 0.01 0.1 0.25

echo "[laneR-1f-est2] done"
echo "== TdCalib =="
grep -h "TdCalib" results/f1_td_est2/*.log 2>/dev/null | grep -v warmup | tail -14
