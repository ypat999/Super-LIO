#!/usr/bin/env bash
# M.25 阶段 1：1.2 h20 组合复测（lane B，ROS_DOMAIN_ID=42）
# 基座 = nw035 + nbg 3e-3 + huber 0.20（deg_h20_dbg 冠军，0.025 m）
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=42
export BAG=/home/ywj/slam_data/livox_record_drone
H20_ENV="NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.003 HUBER_BASE=0.20"

env $H20_ENV ./run_one.sh m25_h20 6 0.25 0.1 0.1 0.25

env $H20_ENV FREEZE_GRAVITY=true \
  ./run_one.sh m25_h20_gfz 6 0.25 0.1 0.1 0.25

env $H20_ENV DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 \
  ./run_one.sh m25_h20_tr2 6 0.25 0.1 0.1 0.25

env $H20_ENV \
  ./run_one.sh m25_h20_imu01 6 0.25 0.01 0.01 0.25

env $H20_ENV PLANE_FIT_THR=0.05 \
  ./run_one.sh m25_h20_pft05 6 0.25 0.1 0.1 0.25
echo "[laneB] all done"
