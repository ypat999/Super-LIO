#!/usr/bin/env bash
# M.31 场景自动档验收（楼梯 lane）
# 期望：窗口内 cond 呈周期振荡（永不超 223）⇒ hi_frac 远低于 0.5 ⇒ 判 stair(A)。
# 判 A 后全程参数 = 楼梯档（na0.3 / nw σ0.035 / hub0.20 / nbg3e-3），
# 与固定 A 线对照 m26_h20_na03 参数完全一致 ⇒ odom.csv 应逐字节相同。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=83
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

AUTO_SCENE_EN=true \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
./run_one.sh m35_auto_stair 6 0.25 0.3 0.1 0.25

echo "[laneQ-stair] done"
echo "== AutoScene 判定 =="
grep -h "AutoScene" results/m35_auto_stair/*.log 2>/dev/null | head -3
echo "== md5（应与 A 线对照 m26_h20_na03 相同）=="
md5sum results/m35_auto_stair/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/m26_h20_na03/odom.csv 2>/dev/null | cut -d' ' -f1
