#!/usr/bin/env bash
# M.32 应用验证（1f，B 线参数，calib_td 开 + apply 开）
# 目的：与楼梯 apply 对照，看非刀刃配置下应用 ~1.7 ms 修正的影响。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=97
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=true CALIB_TD_APPLY=true \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=false \
./run_one.sh f1_td_apply 6 0.25 0.01 0.1 0.25

echo "[laneR-1f-apply] done"
grep -h "TdCalib" results/f1_td_apply/*.log 2>/dev/null | grep -v warmup | tail -6
