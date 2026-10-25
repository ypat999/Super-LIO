#!/usr/bin/env bash
# M.26 P0-1：imu_na 响应曲线收口（lane J，楼梯 bag，ROS_DOMAIN_ID=51）
# 背景：na=0.3 与 na=0.01 两个"谷"都远好于 0.1，但 0.3 一侧只有 bug 前代码
#       （e98dc1f，nw 陈旧权重未修）的 deg_h20_dbg=0.025，需要在 b 系列代码上补点。
# 本 lane：基座复跑（比对 m25_base 是否仍逐字节一致）+ na 0.3 / 0.03 两个新点。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=51
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install3/setup.bash

# 0) 代码一致性锚点：应与 m25_base 的 odom.csv md5 相同
                 ./run_one.sh m26_base    6 0.25 0.1  0.1 0.25

# 1) na 曲线高端：0.3（"更不信任 IMU"一侧的谷）
                 ./run_one.sh m26_na03    6 0.25 0.3  0.1 0.25

# 2) na 曲线中段：0.03（判断 0.01 是真谷底还是斜坡上的点）
                 ./run_one.sh m26_na0030  6 0.25 0.03 0.1 0.25

echo "[laneJ] all done"
