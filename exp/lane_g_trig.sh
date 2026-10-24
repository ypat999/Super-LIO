#!/usr/bin/env bash
# M.25 阶段 1 补充：确认退化判据在这条 bag 上到底有没有触发（lane G，ROS_DOMAIN_ID=48）
# 不开 debug.csv（那个 declare 曾被合并弄坏），改用 degeneracy_detect_en 的
# 2 s 一条 [Degeneracy] cond=/axes=/active= WARNING 日志来判断。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=48
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install2/setup.bash

# mode 0：开检测 + 截断，看 cond 分布与 active 是否真的变 1
DEGENERACY_DETECT_EN=true DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=0 \
  ./run_one.sh m25_tr0_det 6 0.25 0.1 0.1 0.25

# mode 2：同上，看预条件后 cond 与 axes
DEGENERACY_DETECT_EN=true DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=2 \
  ./run_one.sh m25_tr2b_det 6 0.25 0.1 0.1 0.25

echo "[laneG] all done"
