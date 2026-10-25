#!/usr/bin/env bash
# M.26 P0-4：1f 园区交叉验证（lane L，ROS_DOMAIN_ID=53）
# 背景：f1_best（na0.3/nbg3e-3/nw035）出自 nw 陈旧权重 bug 之前，na=0.3 在 1f
#       从未在干净代码上单变量测过；f1_hub20 上次带 2 个 gap（污染，不进结论）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=53
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install3/setup.bash

# 1) na=0.3 单变量（其余同 f1_s2_base）
                 ./run_one.sh f1_na03     6 0.25 0.3 0.1 0.25

# 2) huber 0.20 干净复跑（对照 f1_s2_base，替代被污染的 f1_hub20）
HUBER_BASE=0.20  ./run_one.sh f1_hub20_r2 6 0.25 0.1 0.1 0.25

echo "[laneL] all done"
