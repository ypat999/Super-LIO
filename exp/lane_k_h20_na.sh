#!/usr/bin/env bash
# M.26 P0-2/3：h20 基座上的 na 归因（lane K，楼梯 bag，ROS_DOMAIN_ID=52）
# 背景：deg_h20_dbg（h20 + na0.3，脏代码）闭环 0.025 m，但同名义配置在干净代码上
#       以 na=0.1 复测只有 1.398 m（m25_h20）。两者差两个混淆变量（bug × na），
#       这里在干净代码上分别补 na=0.3（复现裁决）与 na=0.01（解绑 ng 混淆）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=52
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install3/setup.bash
H20_ENV="NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.003 HUBER_BASE=0.20"

# 1) 复现裁决：h20 基座 + na=0.3（= deg_h20_dbg 原配置，代码已修）
env $H20_ENV ./run_one.sh m26_h20_na03  6 0.25 0.3  0.1 0.25

# 2) 解绑混淆：h20 基座 + na=0.01，imu_ng 保持 0.1
#    （m25_h20_imu01 是 na01+ng01 一起降，而 ng01 单独有害，故该格不等于纯 na01）
env $H20_ENV ./run_one.sh m26_h20_na01  6 0.25 0.01 0.1 0.25

echo "[laneK] all done"
