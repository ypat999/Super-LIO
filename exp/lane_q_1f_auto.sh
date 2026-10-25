#!/usr/bin/env bash
# M.31 场景自动档验收（1f lane）
# 期望：窗口内 cond 持续高位 ⇒ hi_frac > 0.5 ⇒ 判 corridor(B)，切 na0.01 / nw 关。
# 前 120 s 仍走楼梯档（A 线），因此不会与任何固定参数 run 逐字节一致，
# 漂移需与 f1_h20_na03（A 线 4.097 m）/ f1_bline_nbg3（B 线 3.050 m）对比评估。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=84
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

AUTO_SCENE_EN=true \
HUBER_BASE=0.20 IMU_NBG=0.003 \
NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
./run_one.sh f1_auto 6 0.25 0.3 0.1 0.25

echo "[laneQ-1f] done"
echo "== AutoScene 判定 =="
grep -h "AutoScene" results/f1_auto/*.log 2>/dev/null | head -3
