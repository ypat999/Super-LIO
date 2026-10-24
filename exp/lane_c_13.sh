#!/usr/bin/env bash
# M.25 阶段 1：1.3 参数欠账（lane C，ROS_DOMAIN_ID=43）
# 单变量叠加在基线（deg_base）之上
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=43
export BAG=/home/ywj/slam_data/livox_record_drone

KF_MAX_ITER=8    ./run_one.sh m25_kf8   6 0.25 0.1 0.1 0.25
OBS_WEIGHT=300   ./run_one.sh m25_obs300 6 0.25 0.1 0.1 0.25
OBS_WEIGHT=3000  ./run_one.sh m25_obs3000 6 0.25 0.1 0.1 0.25
                 ./run_one.sh m25_na01 6 0.25 0.01 0.1 0.25
                 ./run_one.sh m25_ng01 6 0.25 0.1 0.01 0.25
HUBER_SCALE=0.001 ./run_one.sh m25_hs001 6 0.25 0.1 0.1 0.25
HUBER_SCALE=0.006 ./run_one.sh m25_hs006 6 0.25 0.1 0.1 0.25

DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-2 DEGENERACY_THRESHOLD=50.0 \
  ./run_one.sh m25_t50_l2 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-4 DEGENERACY_THRESHOLD=50.0 \
  ./run_one.sh m25_t50_l4 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-2 DEGENERACY_THRESHOLD=200.0 \
  ./run_one.sh m25_t200_l2 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-4 DEGENERACY_THRESHOLD=200.0 \
  ./run_one.sh m25_t200_l4 6 0.25 0.1 0.1 0.25
echo "[laneC] all done"
