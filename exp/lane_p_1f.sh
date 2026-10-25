#!/usr/bin/env bash
# M.30 场景判别埋点验证（1f lane）
# 目的同 lane_p_stair.sh：取 1f 走廊的 deg_dir_v / deg_dir_g / deg_dir_z，
#       检验走廊段是否呈现"退化轴 ≈ 速度方向"（|u_min·v̂| → 1）。
# 参数与 f1_dbg 完全一致（纯埋点，不改滤波），预期 odom.csv md5 与 f1_dbg 相同。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=82
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

DEBUG_LEVEL=2 DEGENERACY_DETECT_EN=true ./run_one.sh f1_dirs_dbg 6 0.25 0.1 0.1 0.25

echo "[laneP-1f] all done"
echo "== md5 校验（应与 f1_dbg 相同）=="
md5sum results/f1_dirs_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/f1_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
