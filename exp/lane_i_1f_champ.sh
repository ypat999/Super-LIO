#!/usr/bin/env bash
# M.25 阶段 1 冠军项跨场景验证（lane I，园区 1f bag，ROS_DOMAIN_ID=50）
# 楼梯上 imu_na=0.01（0.55 m）与 obs_weight=300（0.91 m）都优于基线 1.599 m，
# 这里在同一套 1f bag 上验证是否同样有效，避免只在楼梯 bag 上过拟合。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=50
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install2/setup.bash

                 ./run_one.sh f1_na01     6 0.25 0.01 0.1 0.25
OBS_WEIGHT=300.0 ./run_one.sh f1_obs300   6 0.25 0.1 0.1 0.25
HUBER_BASE=0.20  ./run_one.sh f1_hub20    6 0.25 0.1 0.1 0.25

echo "[laneI] all done"
