#!/usr/bin/env bash
# M.27-P1：1f 侧消融（hub0.2 + na0.3 归因格）+ debug.csv 通路验证（1f lane）
# M.26 中 1f 全 h20+na0.3 最差(4.097)：本 lane 补 hub0.2+na0.3（无 nw/nbg）单格，
# 确认 1f 变差是否只由 na 引起（对照 f1_hub20_r2=2.803 / f1_na03=3.638）。
# f1_dbg 取 1f 走廊的逐帧 n_plane_fail×cond 诊断数据（走廊是细通道的主战场）。
# 预期：md5(f1_dbg/odom.csv) == md5(f1_s2_base/odom.csv)（埋点无副作用）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=62
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

# 1) 归因格：hub0.2 + na0.3（nw/nbg 保持默认关/1e-4）
HUBER_BASE=0.20 ./run_one.sh f1_hub20_na03 6 0.25 0.3 0.1 0.25

# 2) debug.csv 通路验证 + P3.1 诊断数据（1f 走廊）
DEBUG_LEVEL=2 DEGENERACY_DETECT_EN=true ./run_one.sh f1_dbg 6 0.25 0.1 0.1 0.25

echo "[laneM-1f] all done"
echo "== md5 校验（应相等）=="
md5sum results/f1_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/f1_s2_base/odom.csv 2>/dev/null | cut -d' ' -f1
