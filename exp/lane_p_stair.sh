#!/usr/bin/env bash
# M.30 场景判别埋点验证（楼梯 lane）
# 目的：在 debug.csv 新增平移块最弱方向三项对齐度（deg_cond_t / deg_evt_min /
#       deg_dir_v / deg_dir_g / deg_dir_z），检验"退化是否落在运动轴上"能否
#       补上 deg_cond 的重叠带（1f 有 19% 时间 cond<150 被误判为楼梯）。
# 参数与 m30_dbg 完全一致（纯埋点，不改滤波），预期 odom.csv md5 与 m30_dbg 相同。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=81
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

DEBUG_LEVEL=2 DEGENERACY_DETECT_EN=true ./run_one.sh m34_dirs_dbg 6 0.25 0.1 0.1 0.25

echo "[laneP-stair] all done"
echo "== md5 校验（应与 m30_dbg 相同）=="
md5sum results/m34_dirs_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/m30_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
