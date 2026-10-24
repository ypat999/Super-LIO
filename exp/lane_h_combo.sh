#!/usr/bin/env bash
# M.25 阶段 1 收尾：冠军项复现 + 组合（lane H，ROS_DOMAIN_ID=49）
# 依据：1.3 单变量里 imu_na=0.01（0.55 m）、obs_weight=300（0.91 m）、
#      1.1 里 mode0 截断（1.06 m）三者各自都优于基线 1.599 m，这里做复现与两两组合。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=49
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install2/setup.bash
H20_ENV="NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.003 HUBER_BASE=0.20"
TR="DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=0"

# 1) na=0.01 复现（判断是参数效应还是单跑噪声）
                 ./run_one.sh m25_na01_r2 6 0.25 0.01 0.1 0.25

# 2) na=0.01 + mode0 截断
env $TR ./run_one.sh m25_na01_tr0 6 0.25 0.01 0.1 0.25

# 3) h20 基座 + na=0.01 + 截断
env $H20_ENV $TR ./run_one.sh m25_h20_na01_tr0 6 0.25 0.01 0.1 0.25

# 4) na=0.01 + obs_weight=300
OBS_WEIGHT=300.0 ./run_one.sh m25_na01_obs300 6 0.25 0.01 0.1 0.25

echo "[laneH] all done"
