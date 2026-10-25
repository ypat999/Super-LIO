#!/usr/bin/env bash
# M.27-P1：M.26 楼梯冠军(h20+na0.3)最小消融 + debug.csv 通路验证（楼梯 lane）
# 消融对象：nw σ 0.035→0.05 / nbg 3e-3→1e-4，各 1 格，确认冠军不依赖细调。
# 另含 DEBUG_LEVEL=2 一格：验证 debug.csv 通路（M.25-D 下一步 4）+ 取 P3.1 诊断数据
# （逐帧 n_plane_fail 与 deg_cond 的相关性）。DEBUG_LEVEL 只加埋点、不改行为，
# 其 odom.csv 应与 m26_base 完全一致（md5 校验），本身即"埋点无副作用"的验收。
#
# 前置：colcon build --packages-select super_lio --build-base /tmp/sl_build4 \
#         --install-base /tmp/sl_install4 --cmake-args -DCMAKE_BUILD_TYPE=Release
# 判据：loop_error.py --bag $BAG --loops --win 12:21,392:420,440:465 --health <runs>
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=61
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash

# 1) 消融 A：σ 0.035 → 0.05（其余同冠军）
env NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05 IMU_NBG=0.003 HUBER_BASE=0.20 \
  ./run_one.sh m29_h20_s05 6 0.25 0.3 0.1 0.25

# 2) 消融 B：nbg 3e-3 → 1e-4（其余同冠军）
env NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.0001 HUBER_BASE=0.20 \
  ./run_one.sh m29_h20_nbg1e4 6 0.25 0.3 0.1 0.25

# 3) debug.csv 通路验证 + P3.1 诊断数据（埋点无行为副作用）
#    预期：md5(m30_dbg/odom.csv) == md5(m26_base/odom.csv)；
#    debug.csv 落在 m30_dbg/ 下，用 fine_diag.py 分析。
DEBUG_LEVEL=2 DEGENERACY_DETECT_EN=true ./run_one.sh m30_dbg 6 0.25 0.1 0.1 0.25

echo "[laneM-stair] all done"
echo "== md5 校验（应相等）=="
md5sum results/m30_dbg/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/m26_base/odom.csv 2>/dev/null | cut -d' ' -f1
