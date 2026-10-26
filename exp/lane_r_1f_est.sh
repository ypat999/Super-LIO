#!/usr/bin/env bash
# M.32 估计（1f，B 线参数，calib_td 开、apply 关）
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=94
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=true CALIB_TD_APPLY=false \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=false \
./run_one.sh f1_td_est 6 0.25 0.01 0.1 0.25

echo "[laneR-1f-est] done"
echo "== TdCalib =="
grep -h "TdCalib" results/f1_td_est/*.log 2>/dev/null | tail -12
echo "== md5（应与 f1_td_anchor 相同）=="
md5sum results/f1_td_est/odom.csv | cut -d' ' -f1
md5sum results/f1_td_anchor/odom.csv | cut -d' ' -f1
