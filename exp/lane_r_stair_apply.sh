#!/usr/bin/env bash
# M.32 应用验证（楼梯，A 线参数，calib_td 开 + apply 开）
# 目的：① 验证锁定后能写入 g_lidar_time_offset_extra 并反馈到下一帧点时间戳
#       （effective_toff 应变为锁定值）；② 量化"应用 ~1 ms 修正"对漂移的影响。
# 期望：odom 与锚点 m36_td_anchor_stair 不再相同（反馈确实生效），
#       但漂移不应显著变化（~1 ms 远小于 M.10 已扫过的 ±3 ms）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=95
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

CALIB_TD_EN=true CALIB_TD_APPLY=true \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
./run_one.sh m38_td_apply_stair 6 0.25 0.3 0.1 0.25

echo "[laneR-stair-apply] done"
echo "== TdCalib =="
grep -h "TdCalib" results/m38_td_apply_stair/*.log 2>/dev/null | tail -8
echo "== md5（应与锚点不同）=="
md5sum results/m38_td_apply_stair/odom.csv | cut -d' ' -f1
md5sum results/m36_td_anchor_stair/odom.csv | cut -d' ' -f1
