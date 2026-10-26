#!/usr/bin/env bash
# M.32 锚点（1f，B 线参数，时延标定关闭）
# 期望 md5 与 f1_bline_nbg3 相同。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=92
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=false \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=false \
./run_one.sh f1_td_anchor 6 0.25 0.01 0.1 0.25

echo "[laneR-1f-anchor] done"
md5sum results/f1_td_anchor/odom.csv | cut -d' ' -f1
md5sum results/f1_bline_nbg3/odom.csv | cut -d' ' -f1
