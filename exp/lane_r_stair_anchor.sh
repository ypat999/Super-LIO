#!/usr/bin/env bash
# M.32 锚点（楼梯，A 线参数，时延标定关闭）
# 目的：确认新增侧车代码默认关时对主链路零副作用。
# 期望 md5 与 m26_h20_na03 相同（6e01aa2dc05967f661450488dd56fc34）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=91
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=false \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
./run_one.sh m36_td_anchor_stair 6 0.25 0.3 0.1 0.25

echo "[laneR-stair-anchor] done"
md5sum results/m36_td_anchor_stair/odom.csv | cut -d' ' -f1
md5sum results/m26_h20_na03/odom.csv | cut -d' ' -f1
